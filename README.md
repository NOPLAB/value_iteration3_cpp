# value_iteration3

[![Humble CI](https://img.shields.io/github/check-runs/NOPLAB/value_iteration3_cpp/main?nameFilter=ROS%202%20humble%20build%20and%20test%20%28ubuntu-24.04%29&label=Humble)](https://github.com/NOPLAB/value_iteration3_cpp/actions/workflows/ci.yaml?query=branch%3Amain)
[![Jazzy CI](https://img.shields.io/github/check-runs/NOPLAB/value_iteration3_cpp/main?nameFilter=ROS%202%20jazzy%20build%20and%20test%20%28ubuntu-24.04%29&label=Jazzy)](https://github.com/NOPLAB/value_iteration3_cpp/actions/workflows/ci.yaml?query=branch%3Amain)
[![Lyrical CI](https://img.shields.io/github/check-runs/NOPLAB/value_iteration3_cpp/main?nameFilter=ROS%202%20lyrical%20build%20and%20test%20%28ubuntu-24.04%29&label=Lyrical)](https://github.com/NOPLAB/value_iteration3_cpp/actions/workflows/ci.yaml?query=branch%3Amain)

## Overview

ROS 2 navigation by value iteration. The node reads a grid map, computes the cost
of reaching a goal from each free cell and heading, and publishes the selected
discrete motion at the robot's current pose.

It supports occupancy maps and raw cost grids, configurable motion actions,
parallel global solving, and local obstacle updates from laser scans.

## Requirements

CI covers these ROS 2 distributions on x86_64 and ARM64:

| ROS 2 | Container OS |
| --- | --- |
| Humble | Ubuntu 22.04 |
| Jazzy | Ubuntu 24.04 |
| Lyrical | Ubuntu 26.04 |

Install ROS 2, `colcon`, and `rosdep`, and initialize rosdep before building.
Package dependencies are declared in [package.xml](package.xml).

Navigation also needs a map server, a `map -> base_link` transform, a laser scan,
and a base that accepts the command format described under
[ROS interfaces](#ros-interfaces).

## Build

The examples use Humble. Replace `humble` with `jazzy` or `lyrical` to use another
supported distribution.

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/NOPLAB/value_iteration3_cpp.git
cd ~/ros2_ws
rosdep update --rosdistro "$ROS_DISTRO"
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y
colcon build --packages-select value_iteration3 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

The repository is named `value_iteration3_cpp`; the ROS package is
`value_iteration3`.

## Run

From the workspace root:

```bash
source install/setup.bash
ros2 launch value_iteration3 vi.launch.py
```

The launch file loads [config/params.yaml](config/params.yaml). Send a goal in the
`map` frame on `/goal_pose` (`geometry_msgs/msg/PoseStamped`). Solving starts when
a goal arrives and can take seconds on a large map. The robot waits until a
policy is available.

To use a raw cost grid without the launch file:

```bash
ros2 run value_iteration3 vi_node --ros-args \
  -p map_type:=cost_replace -p online:=true
```

Set `online: false` to load the map without subscribing to goals or scans or
publishing navigation results. Direct execution defaults to `false`; the shipped
launch configuration sets it to `true`.

## ROS interfaces

| Name | Type | Role |
| --- | --- | --- |
| `/map_server/map` | `nav_msgs/srv/GetMap` | Occupancy map service; read once at startup |
| `/cost_map` | `nav_msgs/srv/GetMap` | Raw cost map service; used in either cost mode |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Goal position and yaw in `map` |
| `/scan` | `sensor_msgs/msg/LaserScan` | Local obstacle updates; sensor-data QoS |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Selected discrete action, repeated at 10 Hz |
| `/value_function` | `nav_msgs/msg/OccupancyGrid` | Cost for the robot's current heading, about every 1.5 s |
| `map -> base_link` | TF | Current robot pose |

**`/cmd_vel` carries discrete motion steps, not smooth velocities.**
`linear.x` is the forward step in metres and `angular.z` is the rotation in
radians. At the goal, or when no action exists at the current cell, the node
publishes zeros.

`/value_function` is `0` at the goal and saturates at `100` when the cost reaches
`cost_drawing_threshold` seconds.

## Configuration

### Map modes

Set `map_type` in [config/params.yaml](config/params.yaml):

| `map_type` | Immediate cost of a free cell |
| --- | --- |
| `occupancy` | 1 second, plus occupancy safety penalties; only map value `0` is free |
| `cost_add` | 1 second plus the raw cell value |
| `cost_replace` | The raw cell value alone; a value of `0` costs nothing |

Both cost modes read `/cost_map` and interpret each byte as unsigned: `0` through
`254` are costs in seconds, and `255` (signed `-1`) is blocked. Signed `-2`
therefore represents cost `254`. Cost maps supply their own inflation;
`safety_radius` and `safety_radius_penalty` apply only in occupancy mode.

Unknown modes, including the older ambiguous `cost`, are rejected. Use
`cost_replace` for the raw-cost behavior of `value_iteration`.

### Parameters

Defaults below come from the shipped YAML. Without a parameter file,
`online` defaults to `false` and `global_thread_num` to `1`.

| Parameter | Default | Meaning |
|---|---|---|
| `map_type` | `occupancy` | `occupancy` selects `/map_server/map`; `cost_add` and `cost_replace` select `/cost_map` |
| `online` | `true` | Subscribe to the goal and the scan, and publish `/cmd_vel` |
| `global_thread_num` | `2` | Threads used while solving. `0` uses all logical cores |
| `theta_cell_num` | `60` | Number of heading bins |
| `safety_radius` | `0.2` | Meters around an occupied cell that receive the extra penalty |
| `safety_radius_penalty` | `30.0` | Extra seconds to enter a cell near an obstacle |
| `goal_margin_radius` | `0.2` | Meters. Both checked corners of a goal cell must lie inside this radius |
| `goal_margin_theta` | `10` | Degrees of heading that still count as the goal |
| `cost_drawing_threshold` | `60` | Seconds at which `/value_function` saturates at 100 |
| `local_xy_range` | `1.0` | Meters. Half-width of the square updated from the laser |
| `config_file` | set by the launch file | Path of the YAML file that contains `action_list` |

### Motion actions

Define `action_list` in [config/params.yaml](config/params.yaml), alongside the
`/**` parameter section. ROS 2 cannot store a list of maps as a parameter, so the
node reads this list directly from the file. Add or remove entries to change
the action set; each entry needs all three fields:

```yaml
action_list:
  - name: forward
    onestep_forward_m: 0.3
    onestep_rotation_deg: 0.0
  - name: back
    onestep_forward_m: -0.2
    onestep_rotation_deg: 0.0
  - name: right
    onestep_forward_m: 0.0
    onestep_rotation_deg: -20.0
  - name: rightfw
    onestep_forward_m: 0.2
    onestep_rotation_deg: -20.0
  - name: left
    onestep_forward_m: 0.0
    onestep_rotation_deg: 20.0
  - name: leftfw
    onestep_forward_m: 0.2
    onestep_rotation_deg: 20.0
```

`onestep_forward_m` is the forward step in metres; negative means backward.
`onestep_rotation_deg` is the rotation in degrees; negative means clockwise when
viewed from above. The list supports at most 127 actions.

The shipped list has six motions: 0.3 m forward, 0.2 m backward, a 20 degree
turn in either direction, and either turn combined with 0.2 m forward.

## How it works

### Goal regions and solving

A cell belongs to the goal region when both checked corners lie within
`goal_margin_radius` and its heading bin lies within `goal_margin_theta` degrees
of the goal yaw. Goal cells start at cost `0`; other free cells start unreachable.

`theta_cell_num` divides a full turn into 1 through 64 equal heading bins:
60 bins give 6 degrees per bin. Global solving runs until the goal's field stops
changing. In occupancy mode, cells within `safety_radius` of an obstacle add
`safety_radius_penalty` seconds to the 1 second base cost.

If most free cells remain unreachable, the goal region may be too small for the
motion steps. For a 5 cm map, try increasing `goal_margin_radius` to 0.3 m.

### Laser updates

Each scan updates a square with half-width `local_xy_range` around the robot
(default 1 m). Along 10% through 90% of each valid return, the local penalty is
halved. Cells within two cells of the hit receive a 2048 second penalty.

Non-finite and out-of-range returns are ignored. Changed cells start an update
that may propagate beyond the local window and stops when the change settles.
A new goal clears the local penalties.

### Comparison with earlier versions

`value_iteration` is the ROS 1 planner; `value_iteration2` is its ROS 2 port.
This package keeps the occupancy-map, pose, laser-scan, and discrete-command
model, while making solving respond to goals and changed scan penalties.

| Area | `value_iteration` | `value_iteration2` | This package |
| --- | --- | --- | --- |
| ROS | ROS 1 | ROS 2 | ROS 2 |
| Goal input | `value_iteration/ViAction` action | `/goal_pose` | `/goal_pose` |
| Value output | `policy` and `value` services | `/value_function`, every 1.5 s | `/value_function`, every 1.5 s |
| Motion actions | Configurable parameter list | Six actions fixed in source | YAML list, up to 127 actions |
| Global threads | `thread_num`, default 4 | `global_thread_num`, default 1 in code or 2 in YAML | `global_thread_num`, default 1 in code or 2 in YAML; 0 uses all cores |
| Global solving | Continuous sweeps, even before a goal | Continuous sweeps, even before a goal | Starts on a goal and stops when settled |
| Local updates | Continuous sweeps inside a fixed 1 m window | Continuous sweeps inside a fixed 1 m window | Starts on changed scan penalties and may propagate beyond the configurable window |
| No available action | No command published | No command, unless idling | Zero command published |

The ROS 1 map services are `static_map` and `cost_map`. This package uses
`/map_server/map` for occupancy maps and implements `/cost_map` with ROS 2
clients; the equivalent cost-service branch in `value_iteration2` is commented
out. `cost_replace` preserves its raw-cost conversion, while `cost_add` adds the
1 second base cost.

Legacy YAML keys such as `gthread_num`, `gsafety_*`, `ggoal_*`, `gmap_type`, and
`local_thread_num` are not used here. Use the names under
[Parameters](#parameters).

## Testing

CI builds and runs the solver, ROS integration tests, and ament lint checks on
Humble, Jazzy, and Lyrical, on both x86_64 and ARM64. The badges at the top track
each distribution's x86_64 build-and-test job on `main`; follow them to view the
complete matrix.

From the workspace root after building:

```bash
source install/setup.bash
colcon test --packages-select value_iteration3 \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
```

The solver tests cover action parsing, thread-count agreement, blocked cells,
raw cost decoding, and laser penalties. Integration tests run the installed node
against real `nav_msgs/srv/GetMap` services and verify routing, map metadata,
and the distinction between raw costs `254` and `255`. Lint runs cppcheck,
flake8, lint_cmake, and xmllint. CI uploads the test result files.

### Standalone solver test

To test without ROS, use a C++17 compiler and pthreads from the repository root:

```bash
c++ -std=c++17 -O2 -pthread -I include \
  src/planner.cpp src/actions.cpp src/global_planner.cpp src/local_planner.cpp \
  src/test_planner.cpp -o /tmp/planner_test
/tmp/planner_test
```

The test prints `planner_test ok` on success.

## License and attribution

Author: nop (noplab90@gmail.com).

[BSD-3-Clause](LICENSE.txt). Copyright (c) 2026, nop and CIT autonomous robot laboratory.

This package refers to two earlier BSD-3-Clause programs by Ryuichi Ueda:

- `value_iteration`, Copyright (c) 2021, CIT autonomous robot laboratory
- `value_iteration2`, Copyright (c) 2024, Ryuichi Ueda and CIT autonomous robot laboratory
