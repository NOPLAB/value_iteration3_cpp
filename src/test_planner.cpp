// SPDX-FileCopyrightText: 2026 nop and CIT autonomous robot laboratory
// SPDX-License-Identifier: BSD-3-Clause

#include "value_iteration3/planner.hpp"

#include <atomic>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace value_iteration3 {
namespace {

constexpr int kWidth = 32;
constexpr int kHeight = 24;
constexpr int kPocketX = 20;
constexpr int kPocketY = 12;

int failures = 0;

void expect(bool condition, const std::string &text) {
  if (!condition) {
    std::cerr << "FAIL " << text << "\n";
    ++failures;
  }
}

std::vector<std::int8_t> make_map() {
  std::vector<std::int8_t> occupancy(kWidth * kHeight, 0);
  for (int y = kPocketY - 8; y <= kPocketY + 8; ++y) {
    for (int x = kPocketX - 8; x <= kPocketX + 8; ++x) {
      if (x == kPocketX && y == kPocketY) {
        continue;
      }
      occupancy[static_cast<std::size_t>(x) + static_cast<std::size_t>(y) * kWidth] = 100;
    }
  }
  return occupancy;
}

bool load(Planner &planner, int threads) {
  planner.set_thread_num(threads);
  return planner.load_map(kWidth, kHeight, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, make_map(), 60, 0.2,
                          30.0, 0.3, 15);
}

}  // namespace

int planner_self_test() {
  Planner one(1);
  Planner two(2);
  expect(load(one, 1), "load 1 thread");
  expect(load(two, 2), "load 2 threads");
  if (failures > 0) {
    return 1;
  }

  std::atomic<std::uint32_t> epoch{1};
  one.prepare_goal(0.10, 0.10, 0);
  two.prepare_goal(0.10, 0.10, 0);
  const SolveResult first = one.solve(epoch, 1);
  const SolveResult second = two.solve(epoch, 1);
  expect(first.converged, "1 thread converged");
  expect(second.converged, "2 threads converged");
  expect(one.audit().empty(), "1 thread " + one.audit());
  expect(two.audit().empty(), "2 threads " + two.audit());

  const auto left = one.view();
  const auto right = two.view();
  expect(left != nullptr && right != nullptr, "views");
  if (left && right) {
    expect(left->cost == right->cost, "thread counts agree on cost");
    expect(left->action == right->action, "thread counts agree on action");
    const std::size_t corner = left->index(kWidth - 1, kHeight - 1, 0);
    expect(left->cost[corner] < Scale::max_cost, "far corner is reached");
    bool pocket_blocked = true;
    for (int it = 0; it < left->nt; ++it) {
      if (left->cost[left->index(kPocketX, kPocketY, it)] != Scale::max_cost) {
        pocket_blocked = false;
      }
    }
    expect(pocket_blocked, "sealed cell stays blocked");
  }

  const std::vector<std::uint64_t> before = left ? left->cost : std::vector<std::uint64_t>{};
  one.set_window(0.4, 0.1);
  const bool painted = one.apply_scan({0.35f}, 0.0f, 0.0f, 0.0f, 10.0f, 0.4, 0.1, 0.0);
  expect(painted, "scan changes a penalty");
  const SolveResult repaired = one.propagate(epoch, 1);
  expect(repaired.converged, "penalty update converged");
  expect(one.audit().empty(), "after scan " + one.audit());
  const auto after = one.view();
  if (after && !before.empty()) {
    bool raised = false;
    bool decreased = false;
    for (std::size_t i = 0; i < before.size(); ++i) {
      if (after->cost[i] > before[i]) {
        raised = true;
      }
      if (after->cost[i] < before[i]) {
        decreased = true;
      }
    }
    expect(raised, "a penalty raises some cost");
    expect(!decreased, "a penalty does not lower cost");
  }

  epoch.store(2);
  const SolveResult cancelled = one.solve(epoch, 1);
  expect(cancelled.cancelled, "a new request cancels the running solve");

  if (failures == 0) {
    std::cout << "planner_test ok rounds " << first.rounds << " " << second.rounds << "\n";
  }
  return failures == 0 ? 0 : 1;
}

}  // namespace value_iteration3

int main() { return value_iteration3::planner_self_test(); }
