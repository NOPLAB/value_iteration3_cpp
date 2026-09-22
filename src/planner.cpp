// SPDX-FileCopyrightText: 2026 nop and CIT autonomous robot laboratory
// SPDX-License-Identifier: BSD-3-Clause

#include "value_iteration3/planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <utility>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace value_iteration3 {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::uint64_t kUnreached = ~std::uint64_t{0};
constexpr std::uint64_t kHitPenalty = 2048ull << Scale::prob_base_bit;
constexpr double kLocalRangeM = 1.0;
constexpr std::uint32_t kMaxRounds = 100000000u;
constexpr int kSampleBit = 6;

int countr_zero(std::uint64_t bits) {
#if defined(_MSC_VER)
  unsigned long index = 0;
  _BitScanForward64(&index, bits);
  return static_cast<int>(index);
#else
  return __builtin_ctzll(bits);
#endif
}

std::uint64_t heading_mask(int nt) {
  return nt >= 64 ? kUnreached : ((std::uint64_t{1} << nt) - 1);
}

// Expand a heading bitmask by k steps on a circle of length nt.
std::uint64_t dilate_headings(std::uint64_t mask, int k, int nt) {
  const std::uint64_t full = heading_mask(nt);
  std::uint64_t acc = mask & full;
  std::uint64_t left = acc;
  std::uint64_t right = acc;
  for (int i = 0; i < k; ++i) {
    left = ((left << 1) | (left >> (nt - 1))) & full;
    right = ((right >> 1) | (right << (nt - 1))) & full;
    acc |= left | right;
  }
  return acc;
}

void cell_delta(double x, double y, double t, double resolution, double t_resolution,
                int &ix, int &iy, int &it) {
  ix = static_cast<int>(std::floor(std::fabs(x) / resolution));
  if (x < 0.0) {
    ix = -ix - 1;
  }
  iy = static_cast<int>(std::floor(std::fabs(y) / resolution));
  if (y < 0.0) {
    iy = -iy - 1;
  }
  it = static_cast<int>(std::floor(t / t_resolution));
}

void no_noise(double forward, double rotate_deg, double from_x, double from_y, double from_t,
              double &to_x, double &to_y, double &to_t) {
  const double ang = from_t / 180.0 * kPi;
  to_x = from_x + forward * std::cos(ang);
  to_y = from_y + forward * std::sin(ang);
  to_t = from_t + rotate_deg;
  while (to_t < 0.0) {
    to_t += 360.0;
  }
}

struct DeltaKey {
  int dix;
  int diy;
  int dit;
  bool operator==(const DeltaKey &other) const {
    return dix == other.dix && diy == other.diy && dit == other.dit;
  }
};

struct DeltaHash {
  std::size_t operator()(const DeltaKey &key) const {
    const auto h = static_cast<std::uint32_t>(key.dix) * 73856093u ^
                   static_cast<std::uint32_t>(key.diy) * 19349663u ^
                   static_cast<std::uint32_t>(key.dit) * 83492791u;
    return h;
  }
};

struct RawDelta {
  int dix;
  int diy;
  int dit;
  std::uint32_t prob;
};

class PhaseBarrier {
 public:
  explicit PhaseBarrier(int parties) : parties_(parties) {}

  void arrive() {
    std::unique_lock<std::mutex> lock(mu_);
    const int generation = generation_;
    if (++waiting_ == parties_) {
      waiting_ = 0;
      ++generation_;
      cv_.notify_all();
    } else {
      cv_.wait(lock, [&] { return generation_ != generation; });
    }
  }

 private:
  int parties_;
  int waiting_ = 0;
  int generation_ = 0;
  std::mutex mu_;
  std::condition_variable cv_;
};

}  // namespace

template <class F>
void Planner::for_rows(F fn) const {
  int parties = thread_num_;
  if (parties > ny_) {
    parties = ny_;
  }
  if (parties <= 1) {
    fn(0, ny_);
    return;
  }
  std::vector<std::thread> threads;
  threads.reserve(static_cast<std::size_t>(parties));
  const int base = ny_ / parties;
  const int extra = ny_ % parties;
  int y = 0;
  for (int i = 0; i < parties; ++i) {
    const int height = base + (i < extra ? 1 : 0);
    const int y0 = y;
    const int y1 = y + height;
    y = y1;
    threads.emplace_back([fn, y0, y1] { fn(y0, y1); });
  }
  for (auto &thread : threads) {
    thread.join();
  }
}

Planner::Planner(int thread_num) {
  set_thread_num(thread_num);
  actions_.push_back({"forward", 0.3, 0.0});
  actions_.push_back({"back", -0.2, 0.0});
  actions_.push_back({"right", 0.0, -20.0});
  actions_.push_back({"rightfw", 0.2, -20.0});
  actions_.push_back({"left", 0.0, 20.0});
  actions_.push_back({"leftfw", 0.2, 20.0});
}

void Planner::set_thread_num(int thread_num) {
  if (thread_num <= 0) {
    thread_num = static_cast<int>(std::thread::hardware_concurrency());
  }
  if (thread_num < 1) {
    thread_num = 1;
  }
  thread_num_ = thread_num;
}

void Planner::set_logger(std::function<void(const std::string &)> logger) {
  logger_ = std::move(logger);
}

void Planner::log(const std::string &text) const {
  if (logger_) {
    logger_(text);
  }
}

std::int64_t Planner::pad_col(int ix, int iy) const {
  return static_cast<std::int64_t>(ix + mx_) * nt_ +
         static_cast<std::int64_t>(iy + my_) * row_stride_;
}

std::size_t Planner::index_of(int ix, int iy, int it) const {
  return static_cast<std::size_t>(it) + static_cast<std::size_t>(ix) * static_cast<std::size_t>(nt_) +
         static_cast<std::size_t>(iy) * static_cast<std::size_t>(nt_) * static_cast<std::size_t>(nx_);
}

std::size_t Planner::mask_index(int ix, int iy) const {
  return static_cast<std::size_t>(ix + mx_) +
         static_cast<std::size_t>(iy + my_) * static_cast<std::size_t>(mask_w_);
}

bool Planner::cancelled(const std::atomic<std::uint32_t> &epoch, std::uint32_t ticket) const {
  return epoch.load(std::memory_order_relaxed) != ticket;
}

bool Planner::in_window(int ix, int iy) const {
  return ix >= win_x0_ && ix <= win_x1_ && iy >= win_y0_ && iy <= win_y1_;
}

bool Planner::heading_is_goal(int ix, int iy, int it) const {
  const std::size_t xy = static_cast<std::size_t>(ix) + static_cast<std::size_t>(iy) * nx_;
  if (!free_[xy]) {
    return false;
  }
  const double x0 = ix * resolution_ + origin_x_;
  const double y0 = iy * resolution_ + origin_y_;
  const double r0 = (x0 - goal_x_) * (x0 - goal_x_) + (y0 - goal_y_) * (y0 - goal_y_);
  const double x1 = x0 + resolution_;
  const double y1 = y0 + resolution_;
  const double r1 = (x1 - goal_x_) * (x1 - goal_x_) + (y1 - goal_y_) * (y1 - goal_y_);
  const double radius2 = goal_margin_radius_ * goal_margin_radius_;
  if (!(r0 < radius2 && r1 < radius2)) {
    return false;
  }
  const int t0 = it * t_resolution_;
  const int t1 = (it + 1) * t_resolution_;
  const int yaw_wrap = goal_yaw_ > 180 ? goal_yaw_ - 360 : goal_yaw_ + 360;
  const bool direct = goal_yaw_ - goal_margin_theta_ <= t0 && t1 <= goal_yaw_ + goal_margin_theta_;
  const bool wrapped =
      yaw_wrap - goal_margin_theta_ <= t0 && t1 <= yaw_wrap + goal_margin_theta_;
  return direct || wrapped;
}

void Planner::fill_cp(std::uint64_t value) {
  const std::size_t n = n_pad_;
  int parties = std::min(thread_num_, static_cast<int>(n > 0 ? n : 1));
  if (parties < 1) {
    parties = 1;
  }
  std::vector<std::thread> threads;
  threads.reserve(static_cast<std::size_t>(parties));
  const std::size_t chunk = (n + static_cast<std::size_t>(parties) - 1) / parties;
  for (int i = 0; i < parties; ++i) {
    const std::size_t begin = std::min(n, chunk * static_cast<std::size_t>(i));
    const std::size_t end = std::min(n, begin + chunk);
    threads.emplace_back([this, begin, end, value] {
      for (std::size_t k = begin; k < end; ++k) {
        cp_[k].store(value, std::memory_order_relaxed);
      }
    });
  }
  for (auto &thread : threads) {
    thread.join();
  }
}

void Planner::zero_masks() {
  const std::size_t n = mask_n_;
  int parties = std::min(thread_num_, static_cast<int>(n > 0 ? n : 1));
  if (parties < 1) {
    parties = 1;
  }
  std::vector<std::thread> threads;
  threads.reserve(static_cast<std::size_t>(parties));
  const std::size_t chunk = (n + static_cast<std::size_t>(parties) - 1) / parties;
  for (int i = 0; i < parties; ++i) {
    const std::size_t begin = std::min(n, chunk * static_cast<std::size_t>(i));
    const std::size_t end = std::min(n, begin + chunk);
    threads.emplace_back([this, begin, end] {
      for (std::size_t k = begin; k < end; ++k) {
        mask_[k].store(0, std::memory_order_relaxed);
      }
    });
  }
  for (auto &thread : threads) {
    thread.join();
  }
}

void Planner::build_transitions() {
  const int sample_n = 1 << kSampleBit;
  const double xy_step = resolution_ / sample_n;
  const double t_step = static_cast<double>(t_resolution_) / sample_n;
  const int action_n = static_cast<int>(actions_.size());

  std::vector<std::vector<std::vector<RawDelta>>> raw(
      actions_.size(), std::vector<std::vector<RawDelta>>(static_cast<std::size_t>(nt_)));
  std::vector<int> mx(static_cast<std::size_t>(nt_), 0);
  std::vector<int> my(static_cast<std::size_t>(nt_), 0);
  std::vector<int> mt(static_cast<std::size_t>(nt_), 0);

  int parties = std::min(thread_num_, nt_);
  std::vector<std::thread> threads;
  threads.reserve(static_cast<std::size_t>(parties));
  for (int worker = 0; worker < parties; ++worker) {
    threads.emplace_back([&, worker] {
      for (int it = worker; it < nt_; it += parties) {
        int local_mx = 0;
        int local_my = 0;
        int local_mt = 0;
        const double theta_origin = it * static_cast<double>(t_resolution_);
        for (int ai = 0; ai < action_n; ++ai) {
          std::unordered_map<DeltaKey, std::uint32_t, DeltaHash> hist;
          hist.reserve(64);
          for (double oy = 0.5 * xy_step; oy < resolution_; oy += xy_step) {
            for (double ox = 0.5 * xy_step; ox < resolution_; ox += xy_step) {
              for (double ot = 0.5 * t_step; ot < t_resolution_; ot += t_step) {
                double dx = 0;
                double dy = 0;
                double dt = 0;
                no_noise(actions_[ai].forward_m, actions_[ai].rotate_deg, ox, oy,
                         ot + theta_origin, dx, dy, dt);
                int dix = 0;
                int diy = 0;
                int dit = 0;
                cell_delta(dx, dy, dt, resolution_, t_resolution_, dix, diy, dit);
                hist[DeltaKey{dix, diy, dit}] += 1;
                local_mx = std::max(local_mx, std::abs(dix));
                local_my = std::max(local_my, std::abs(diy));
                int raw = dit - it;
                raw %= nt_;
                if (raw < 0) {
                  raw += nt_;
                }
                const int circ = std::min(raw, nt_ - raw);
                local_mt = std::max(local_mt, circ);
              }
            }
          }
          auto &dst = raw[ai][it];
          dst.reserve(hist.size());
          for (const auto &item : hist) {
            dst.push_back(RawDelta{item.first.dix, item.first.diy, item.first.dit, item.second});
          }
        }
        mx[it] = local_mx;
        my[it] = local_my;
        mt[it] = local_mt;
      }
    });
  }
  for (auto &thread : threads) {
    thread.join();
  }

  int reach_x = 1;
  int reach_y = 1;
  int reach_t = 0;
  for (int it = 0; it < nt_; ++it) {
    reach_x = std::max(reach_x, mx[it]);
    reach_y = std::max(reach_y, my[it]);
    reach_t = std::max(reach_t, mt[it]);
  }
  mx_ = reach_x;
  my_ = reach_y;
  mt_ = reach_t;
  row_stride_ = static_cast<std::int64_t>(nt_) * (nx_ + 2 * mx_);

  transitions_.assign(actions_.size(),
                      std::vector<std::vector<Bucket>>(static_cast<std::size_t>(nt_)));
  for (int ai = 0; ai < action_n; ++ai) {
    for (int it = 0; it < nt_; ++it) {
      auto &dst = transitions_[ai][it];
      dst.reserve(raw[ai][it].size());
      for (const RawDelta &delta : raw[ai][it]) {
        int nit = delta.dit % nt_;
        if (nit < 0) {
          nit += nt_;
        }
        Bucket bucket;
        bucket.offset = static_cast<std::int64_t>(delta.dix) * nt_ +
                        static_cast<std::int64_t>(delta.diy) * row_stride_ + nit;
        bucket.prob = delta.prob;
        dst.push_back(bucket);
      }
    }
  }
}

void Planner::allocate_field() {
  const std::size_t cells = static_cast<std::size_t>(nx_) * ny_;
  const std::size_t states = cells * static_cast<std::size_t>(nt_);
  local_.assign(cells, 0);
  dirty_.assign(cells, 0);
  final_.assign(states, 0);
  n_pad_ = static_cast<std::size_t>(row_stride_ * (ny_ + 2 * my_));
  cp_.reset(new std::atomic<std::uint64_t>[n_pad_]);
  fill_cp(kUnreached);
  pen_.assign(n_pad_, 0);
  eval_ok_.assign(n_pad_, 0);
  mask_w_ = nx_ + 2 * mx_;
  mask_n_ = static_cast<std::size_t>(mask_w_) * static_cast<std::size_t>(ny_ + 2 * my_);
  mask_.reset(new std::atomic<std::uint64_t>[mask_n_]);
  zero_masks();
}

bool Planner::load_map(int width, int height, double resolution, double origin_x, double origin_y,
                       double qx, double qy, double qz, double qw,
                       const std::vector<std::int8_t> &occupancy, int theta_cells,
                       double safety_radius, double safety_penalty, double goal_margin_radius,
                       int goal_margin_theta_deg) {
  if (width <= 0 || height <= 0 || resolution < 0.0001 || theta_cells <= 0 || theta_cells > 64) {
    return false;
  }
  if (static_cast<int>(occupancy.size()) != width * height) {
    return false;
  }
  if (360 / theta_cells <= 0) {
    return false;
  }

  nx_ = width;
  ny_ = height;
  nt_ = theta_cells;
  t_resolution_ = 360 / nt_;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  origin_qx_ = qx;
  origin_qy_ = qy;
  origin_qz_ = qz;
  origin_qw_ = qw;
  goal_margin_radius_ = goal_margin_radius;
  goal_margin_theta_ = goal_margin_theta_deg;
  map_ready_ = false;
  goal_set_ = false;
  field_ready_ = false;

  const std::size_t cells = static_cast<std::size_t>(nx_) * ny_;
  free_.assign(cells, 0);
  static_pen_.assign(cells, 0);
  const int margin = static_cast<int>(std::ceil(safety_radius / resolution_));
  for (int y = 0; y < ny_; ++y) {
    for (int x = 0; x < nx_; ++x) {
      const std::size_t xy = static_cast<std::size_t>(x) + static_cast<std::size_t>(y) * nx_;
      if (occupancy[xy] != 0) {
        continue;
      }
      free_[xy] = 1;
      std::uint64_t penalty = Scale::prob_base;
      for (int ix = x - margin; ix <= x + margin; ++ix) {
        for (int iy = y - margin; iy <= y + margin; ++iy) {
          const long pos = static_cast<long>(iy) * nx_ + ix;
          if (pos >= 0 && pos < static_cast<long>(occupancy.size()) && occupancy[pos] != 0) {
            penalty = static_cast<std::uint64_t>(safety_penalty * Scale::prob_base) + Scale::prob_base;
          }
        }
      }
      static_pen_[xy] = penalty;
    }
  }

  build_transitions();
  allocate_field();
  map_ready_ = true;
  std::ostringstream text;
  text << "map " << nx_ << "x" << ny_ << "x" << nt_ << " threads " << thread_num_;
  log(text.str());
  return true;
}

void Planner::prepare_goal(double x, double y, int yaw_deg) {
  if (!map_ready_) {
    return;
  }
  while (yaw_deg < 0) {
    yaw_deg += 360;
  }
  while (yaw_deg >= 360) {
    yaw_deg -= 360;
  }
  goal_x_ = x;
  goal_y_ = y;
  goal_yaw_ = yaw_deg;
  std::fill(local_.begin(), local_.end(), 0);
  std::fill(dirty_.begin(), dirty_.end(), 0);
  {
    std::lock_guard<std::mutex> lock(view_mu_);
    view_.reset();
  }
  field_ready_ = false;
  goal_set_ = true;

  for_rows([this](int y0, int y1) {
    for (int iy = y0; iy < y1; ++iy) {
      for (int ix = 0; ix < nx_; ++ix) {
        const std::size_t xy = static_cast<std::size_t>(ix) + static_cast<std::size_t>(iy) * nx_;
        const std::uint64_t penalty = free_[xy] ? static_pen_[xy] : 0;
        const std::int64_t column = pad_col(ix, iy);
        for (int it = 0; it < nt_; ++it) {
          const bool final_state = heading_is_goal(ix, iy, it);
          const std::size_t orig = index_of(ix, iy, it);
          const std::size_t pad = static_cast<std::size_t>(column + it);
          final_[orig] = final_state ? 1 : 0;
          pen_[pad] = penalty;
          eval_ok_[pad] = free_[xy] && !final_state;
          cp_[pad].store(free_[xy] && final_state ? penalty : kUnreached, std::memory_order_relaxed);
        }
      }
    }
  });

  std::size_t goals = 0;
  for (std::uint8_t flag : final_) {
    goals += flag ? 1 : 0;
  }
  std::ostringstream text;
  text << "goal (" << goal_x_ << ", " << goal_y_ << ", " << goal_yaw_ << " deg) cells " << goals;
  log(text.str());
}

std::vector<std::pair<int, int>> Planner::dilate(
    const std::vector<std::pair<int, int>> &cells) const {
  std::vector<std::uint8_t> seen(static_cast<std::size_t>(nx_) * ny_, 0);
  std::vector<std::pair<int, int>> out;
  out.reserve(cells.size());
  for (const auto &cell : cells) {
    const int x0 = std::max(0, cell.first - mx_);
    const int x1 = std::min(nx_ - 1, cell.first + mx_);
    const int y0 = std::max(0, cell.second - my_);
    const int y1 = std::min(ny_ - 1, cell.second + my_);
    for (int iy = y0; iy <= y1; ++iy) {
      for (int ix = x0; ix <= x1; ++ix) {
        const std::size_t xy = static_cast<std::size_t>(ix) + static_cast<std::size_t>(iy) * nx_;
        if (seen[xy]) {
          continue;
        }
        seen[xy] = 1;
        out.emplace_back(ix, iy);
      }
    }
  }
  return out;
}

bool Planner::write_local(int ix, int iy, std::uint64_t neu) {
  const std::size_t xy = static_cast<std::size_t>(ix) + static_cast<std::size_t>(iy) * nx_;
  if (local_[xy] == neu) {
    return false;
  }
  local_[xy] = neu;
  const std::uint64_t penalty = free_[xy] ? static_pen_[xy] + neu : 0;
  bool changed = false;
  const std::int64_t column = pad_col(ix, iy);
  for (int it = 0; it < nt_; ++it) {
    const std::size_t pad = static_cast<std::size_t>(column + it);
    const std::uint64_t previous = pen_[pad];
    if (previous == penalty) {
      continue;
    }
    pen_[pad] = penalty;
    const std::uint64_t value = cp_[pad].load(std::memory_order_relaxed);
    if (value != kUnreached) {
      cp_[pad].store(value - previous + penalty, std::memory_order_relaxed);
    }
    changed = true;
  }
  if (changed) {
    dirty_[xy] = 1;
  }
  return changed;
}

void Planner::set_window(double x, double y) {
  if (!map_ready_) {
    return;
  }
  const int range = static_cast<int>(kLocalRangeM / resolution_);
  const int ix = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  const int iy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  win_x0_ = std::max(0, ix - range);
  win_y0_ = std::max(0, iy - range);
  win_x1_ = std::min(nx_ - 1, ix + range);
  win_y1_ = std::min(ny_ - 1, iy + range);
}

bool Planner::apply_scan(const std::vector<float> &ranges, float angle_min, float angle_increment,
                         float range_min, float range_max, double x, double y, double yaw) {
  if (!map_ready_ || win_x1_ < win_x0_) {
    return false;
  }
  bool any = false;
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const float range = ranges[i];
    if (!std::isfinite(range) || range <= 0.0f || range < range_min) {
      continue;
    }
    if (range_max > 0.0f && range > range_max) {
      continue;
    }
    const double angle = yaw + static_cast<double>(angle_increment) * static_cast<double>(i) +
                         static_cast<double>(angle_min);
    const double hit_x = x + static_cast<double>(range) * std::cos(angle);
    const double hit_y = y + static_cast<double>(range) * std::sin(angle);
    const int hit_ix = static_cast<int>(std::floor((hit_x - origin_x_) / resolution_));
    const int hit_iy = static_cast<int>(std::floor((hit_y - origin_y_) / resolution_));

    for (double fraction = 0.1; fraction <= 0.9; fraction += 0.1) {
      const double sx = x + static_cast<double>(range) * std::cos(angle) * fraction;
      const double sy = y + static_cast<double>(range) * std::sin(angle) * fraction;
      const int ix = static_cast<int>(std::floor((sx - origin_x_) / resolution_));
      const int iy = static_cast<int>(std::floor((sy - origin_y_) / resolution_));
      if (!in_window(ix, iy)) {
        continue;
      }
      const std::size_t xy = static_cast<std::size_t>(ix) + static_cast<std::size_t>(iy) * nx_;
      any = write_local(ix, iy, local_[xy] / 2) || any;
    }

    for (int dx = -2; dx <= 2; ++dx) {
      for (int dy = -2; dy <= 2; ++dy) {
        const int ix = hit_ix + dx;
        const int iy = hit_iy + dy;
        if (!in_window(ix, iy)) {
          continue;
        }
        any = write_local(ix, iy, kHitPenalty) || any;
      }
    }
  }
  return any;
}

std::uint64_t Planner::action_cost(int action, int theta, std::int64_t column) const {
  std::uint64_t cost = 0;
  for (const Bucket &bucket : transitions_[action][theta]) {
    const std::uint64_t value =
        cp_[static_cast<std::size_t>(column + bucket.offset)].load(std::memory_order_relaxed);
    if (value == kUnreached) {
      return Scale::max_cost;
    }
    cost += value * bucket.prob;
  }
  return cost >> Scale::prob_base_bit;
}

std::uint64_t Planner::compute_phase(int worker, bool all_headings) {
  const std::size_t n = candidates_.size();
  auto &mine = changed_[static_cast<std::size_t>(worker)];
  mine.clear();
  std::uint64_t my_updates = 0;
  const std::uint64_t full = heading_mask(nt_);
  constexpr std::size_t kBlock = 16;
  const int action_n = static_cast<int>(actions_.size());

  while (true) {
    const std::size_t begin = cursor_.fetch_add(kBlock, std::memory_order_relaxed);
    if (begin >= n) {
      break;
    }
    const std::size_t end = std::min(begin + kBlock, n);
    for (std::size_t j = begin; j < end; ++j) {
      const int ix = candidates_[j].first;
      const int iy = candidates_[j].second;
      std::uint64_t eval_mask = full;
      if (!all_headings) {
        std::uint64_t gathered = 0;
        for (int dy = -my_; dy <= my_; ++dy) {
          const std::size_t row = mask_index(ix - mx_, iy + dy);
          for (int k = 0; k <= 2 * mx_; ++k) {
            gathered |= mask_[row + static_cast<std::size_t>(k)].load(std::memory_order_relaxed);
          }
        }
        if (gathered == 0) {
          continue;
        }
        eval_mask = dilate_headings(gathered, mt_, nt_);
      }

      const std::int64_t column = pad_col(ix, iy);
      std::uint64_t changed_mask = 0;
      std::uint64_t bits = eval_mask;
      while (bits != 0) {
        const int it = countr_zero(bits);
        bits &= bits - 1;
        const std::size_t pad = static_cast<std::size_t>(column + it);
        if (!eval_ok_[pad]) {
          continue;
        }
        const std::uint64_t stored = cp_[pad].load(std::memory_order_relaxed);
        const std::uint64_t before =
            stored == kUnreached ? Scale::max_cost : stored - pen_[pad];
        std::uint64_t best = Scale::max_cost;
        for (int action = 0; action < action_n; ++action) {
          const std::uint64_t cost = action_cost(action, it, column);
          if (cost < best) {
            best = cost;
          }
        }
        // Penalties injected while driving can raise a value, so any move propagates.
        if (best != before) {
          cp_[pad].store(best + pen_[pad], std::memory_order_relaxed);
          ++my_updates;
          changed_mask |= std::uint64_t{1} << it;
        }
      }
      if (changed_mask != 0) {
        mine.emplace_back(ix, iy, changed_mask);
      }
    }
  }
  return my_updates;
}

void Planner::publish() {
  auto view = std::make_shared<View>();
  view->nx = nx_;
  view->ny = ny_;
  view->nt = nt_;
  view->t_resolution_deg = t_resolution_;
  view->resolution = resolution_;
  view->origin_x = origin_x_;
  view->origin_y = origin_y_;
  view->origin_qx = origin_qx_;
  view->origin_qy = origin_qy_;
  view->origin_qz = origin_qz_;
  view->origin_qw = origin_qw_;
  const std::size_t states =
      static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_) * static_cast<std::size_t>(nt_);
  view->cost.assign(states, Scale::max_cost);
  view->action.assign(states, -1);
  view->final_state = final_;

  for_rows([this, &view](int y0, int y1) {
    const int action_n = static_cast<int>(actions_.size());
    for (int iy = y0; iy < y1; ++iy) {
      for (int ix = 0; ix < nx_; ++ix) {
        const std::int64_t column = pad_col(ix, iy);
        for (int it = 0; it < nt_; ++it) {
          const std::size_t pad = static_cast<std::size_t>(column + it);
          const std::size_t orig = index_of(ix, iy, it);
          const std::uint64_t stored = cp_[pad].load(std::memory_order_relaxed);
          if (stored == kUnreached) {
            continue;
          }
          view->cost[orig] = stored - pen_[pad];
          if (!eval_ok_[pad]) {
            continue;
          }
          std::uint64_t best = Scale::max_cost;
          int chosen = -1;
          for (int action = 0; action < action_n; ++action) {
            const std::uint64_t cost = action_cost(action, it, column);
            if (cost < best) {
              best = cost;
              chosen = action;
            }
          }
          if (chosen >= 0 && best < Scale::max_cost) {
            view->action[orig] = static_cast<std::int8_t>(chosen);
          }
        }
      }
    }
  });

  std::lock_guard<std::mutex> lock(view_mu_);
  view_ = std::move(view);
}

SolveResult Planner::run(bool all_headings_first, const std::atomic<std::uint32_t> &epoch,
                         std::uint32_t ticket) {
  changed_.assign(static_cast<std::size_t>(thread_num_), {});
  cursor_.store(0, std::memory_order_relaxed);
  updates_.store(0, std::memory_order_relaxed);

  PhaseBarrier barrier(thread_num_);
  std::atomic<bool> done{false};
  std::vector<std::thread> workers;
  workers.reserve(static_cast<std::size_t>(std::max(0, thread_num_ - 1)));
  for (int worker = 1; worker < thread_num_; ++worker) {
    workers.emplace_back([&, worker] {
      int round = 0;
      for (;;) {
        const std::uint64_t ups = compute_phase(worker, all_headings_first && round == 0);
        if (ups > 0) {
          updates_.fetch_add(ups, std::memory_order_relaxed);
        }
        ++round;
        barrier.arrive();
        barrier.arrive();
        if (done.load(std::memory_order_relaxed)) {
          break;
        }
      }
    });
  }

  SolveResult result;
  const auto started = std::chrono::steady_clock::now();
  auto next_log = started + std::chrono::seconds(2);
  std::uint32_t iter = 0;
  for (;;) {
    const std::uint64_t ups = compute_phase(0, all_headings_first && iter == 0);
    if (ups > 0) {
      updates_.fetch_add(ups, std::memory_order_relaxed);
    }
    barrier.arrive();

    ++iter;
    for (const auto &cell : prev_cells_) {
      mask_[mask_index(cell.first, cell.second)].store(0, std::memory_order_relaxed);
    }
    prev_cells_.clear();

    std::vector<std::pair<int, int>> touched;
    for (int worker = 0; worker < thread_num_; ++worker) {
      for (const auto &item : changed_[static_cast<std::size_t>(worker)]) {
        const int ix = std::get<0>(item);
        const int iy = std::get<1>(item);
        const std::uint64_t bits = std::get<2>(item);
        mask_[mask_index(ix, iy)].store(bits, std::memory_order_relaxed);
        touched.emplace_back(ix, iy);
      }
    }
    prev_cells_ = touched;
    const bool any = !touched.empty();
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_log) {
      std::ostringstream text;
      text << "round " << iter << " updates " << updates_.load(std::memory_order_relaxed);
      log(text.str());
      next_log = now + std::chrono::seconds(2);
    }

    const bool stop = cancelled(epoch, ticket) || !any || iter >= kMaxRounds;
    if (!stop) {
      candidates_ = dilate(touched);
      if (iter % 2 == 1) {
        std::reverse(candidates_.begin(), candidates_.end());
      }
      cursor_.store(0, std::memory_order_relaxed);
    } else {
      result.rounds = iter;
      result.updates = updates_.load(std::memory_order_relaxed);
      result.cancelled = cancelled(epoch, ticket);
      result.converged = !result.cancelled && !any;
      done.store(true, std::memory_order_relaxed);
    }
    barrier.arrive();
    if (done.load(std::memory_order_relaxed)) {
      break;
    }
  }

  for (auto &worker : workers) {
    worker.join();
  }

  if (!result.cancelled) {
    publish();
    field_ready_ = true;
  }
  const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  std::ostringstream text;
  text << (result.cancelled ? "cancelled" : result.converged ? "converged" : "stopped")
       << " rounds " << result.rounds << " updates " << result.updates << " in " << seconds << " s";
  log(text.str());
  return result;
}

SolveResult Planner::solve(const std::atomic<std::uint32_t> &epoch, std::uint32_t ticket) {
  if (!goal_set_) {
    return {};
  }
  zero_masks();
  prev_cells_.clear();
  std::vector<std::pair<int, int>> seeds;
  for (int iy = 0; iy < ny_; ++iy) {
    for (int ix = 0; ix < nx_; ++ix) {
      for (int it = 0; it < nt_; ++it) {
        if (final_[index_of(ix, iy, it)]) {
          seeds.emplace_back(ix, iy);
          break;
        }
      }
    }
  }
  candidates_ = dilate(seeds);
  const SolveResult result = run(true, epoch, ticket);
  if (result.converged) {
    warn_unreached();
  }
  return result;
}

void Planner::warn_unreached() const {
  const auto field = view();
  if (!field) {
    return;
  }
  std::size_t free_cells = 0;
  std::size_t bare = 0;
  for (int iy = 0; iy < ny_; ++iy) {
    for (int ix = 0; ix < nx_; ++ix) {
      const std::size_t xy = static_cast<std::size_t>(ix) + static_cast<std::size_t>(iy) * nx_;
      if (!free_[xy]) {
        continue;
      }
      ++free_cells;
      bool finite = false;
      for (int it = 0; it < nt_; ++it) {
        if (field->cost[index_of(ix, iy, it)] < Scale::max_cost) {
          finite = true;
          break;
        }
      }
      if (!finite) {
        ++bare;
      }
    }
  }
  if (free_cells > 0 && bare * 2 > free_cells) {
    log("most free cells have no finite cost; raise goal_margin_radius");
  }
}

SolveResult Planner::propagate(const std::atomic<std::uint32_t> &epoch, std::uint32_t ticket) {
  if (!field_ready_) {
    return {};
  }
  std::vector<std::pair<int, int>> seeds;
  const std::size_t cells = static_cast<std::size_t>(nx_) * ny_;
  for (std::size_t xy = 0; xy < cells; ++xy) {
    if (!dirty_[xy]) {
      continue;
    }
    dirty_[xy] = 0;
    seeds.emplace_back(static_cast<int>(xy % static_cast<std::size_t>(nx_)),
                       static_cast<int>(xy / static_cast<std::size_t>(nx_)));
  }
  if (seeds.empty()) {
    return SolveResult{true, false, 0, 0};
  }
  zero_masks();
  const std::uint64_t full = heading_mask(nt_);
  for (const auto &cell : seeds) {
    mask_[mask_index(cell.first, cell.second)].store(full, std::memory_order_relaxed);
  }
  prev_cells_ = seeds;
  candidates_ = dilate(seeds);
  return run(false, epoch, ticket);
}

std::shared_ptr<const View> Planner::view() const {
  std::lock_guard<std::mutex> lock(view_mu_);
  return view_;
}

std::string Planner::audit() const {
  if (!view_ || transitions_.empty() || transitions_[0].empty()) {
    return "no field";
  }
  for (std::size_t action = 0; action < transitions_.size(); ++action) {
    for (int theta = 0; theta < nt_; ++theta) {
      std::uint64_t sum = 0;
      for (const Bucket &bucket : transitions_[action][static_cast<std::size_t>(theta)]) {
        sum += bucket.prob;
      }
      if (sum != Scale::prob_base) {
        return "transition weight sum " + std::to_string(sum);
      }
    }
  }
  const int action_n = static_cast<int>(actions_.size());
  for (int iy = 0; iy < ny_; ++iy) {
    for (int ix = 0; ix < nx_; ++ix) {
      const std::int64_t column = pad_col(ix, iy);
      for (int it = 0; it < nt_; ++it) {
        const std::size_t pad = static_cast<std::size_t>(column + it);
        const std::size_t cell = index_of(ix, iy, it);
        if (!eval_ok_[pad]) {
          if (final_[cell]) {
            if (view_->cost[cell] != 0) {
              return "goal cost";
            }
          } else if (view_->cost[cell] != Scale::max_cost) {
            return "blocked cost";
          }
          continue;
        }
        std::uint64_t best = Scale::max_cost;
        int chosen = -1;
        for (int action = 0; action < action_n; ++action) {
          const std::uint64_t cost = action_cost(action, it, column);
          if (cost < best) {
            best = cost;
            chosen = action;
          }
        }
        if (view_->cost[cell] != best) {
          return "backup";
        }
        const std::int8_t expected =
            chosen >= 0 && best < Scale::max_cost ? static_cast<std::int8_t>(chosen) : -1;
        if (view_->action[cell] != expected) {
          return "action";
        }
      }
    }
  }
  return {};
}

}  // namespace value_iteration3
