# value_iteration3

A ROS 2 node that drives a robot to a goal by value iteration. It reads an occupancy grid, computes the cost of reaching the goal from every free cell and heading, and publishes the discrete action at the robot's current pose.

Author: nop (noplab90@gmail.com). License: BSD-3-Clause. Copyright: nop and CIT autonomous robot laboratory. See [License](#license).

## What you need running

Before the node can drive:

- A map server that serves `nav_msgs/GetMap` on `/map_server/map`. Cell value `0` is free. Any other value is occupied.
- A transform from `map` to `base_link`.
- A `sensor_msgs/LaserScan` on `/scan`.
- A base that accepts `geometry_msgs/Twist` on `/cmd_vel`.

The command is not a smooth velocity. While an action is selected, the node republishes that action at 10 Hz: `linear.x` is the action's forward step in meters, and `angular.z` is its rotation in radians. At the goal, and whenever the current cell has no action, it publishes zeros.

## Build and run

Place this directory in a workspace `src` folder, then:

```bash
colcon build --packages-select value_iteration3
source install/setup.bash
ros2 launch value_iteration3 vi.launch.py
```

Send a goal in the `map` frame on `/goal_pose` (`geometry_msgs/PoseStamped`). The node solves from that pose, then starts publishing `/cmd_vel`. Solving can take seconds on a large map. The robot holds still until a policy exists.

`/value_function` (`nav_msgs/OccupancyGrid`) shows the cost for the robot's current heading. The value is `0` at the goal and `100` at `cost_drawing_threshold` seconds or above. It is published about every 1.5 s.

## Topics and the map service

| Name | Type | Role |
|---|---|---|
| `/map_server/map` | `nav_msgs/GetMap` | Occupancy grid, read once at startup |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Goal position and yaw in `map` |
| `/scan` | `sensor_msgs/LaserScan` | Local obstacle update. Sensor data QoS |
| `/cmd_vel` | `geometry_msgs/Twist` | Selected action, 10 Hz |
| `/value_function` | `nav_msgs/OccupancyGrid` | Cost of the current heading |
| `map` → `base_link` | TF | Robot pose |

Set `online` to `false` to load the map and publish nothing else. The launch file leaves it `true`.

## How a goal is stored

A cell is a goal only when both of its checked corners lie inside `goal_margin_radius` of the goal position, and its heading bin lies inside `goal_margin_theta` degrees of the goal yaw. Goal cells have cost 0. Every other free cell starts unreachable.

Headings are `theta_cell_num` equal bins around the circle. With 60 bins, each bin is 6 degrees. The value must be from 1 to 64.

Every free cell already costs 1 second to enter. Cells within `safety_radius` of an occupied cell cost an extra `safety_radius_penalty` seconds.

If the log says that most free cells have no finite cost, the goal region is too small for the action steps. Increase `goal_margin_radius` (0.3 m is a useful next try on a 5 cm map).

## Laser update

`/scan` changes costs inside a square of radius `local_xy_range` around the robot (default 1 m).

Along each return, from 10% to 90% of the range, the local penalty is halved. The cells within two cells of the hit are then given a local penalty of 2048 seconds. Returns that are non-finite or outside the scan's range are ignored. The field is then updated from those cells until the change settles. A new goal clears local penalties.

## Parameters

Numeric defaults below are the values in `config/params.yaml`, which the launch file loads. If you run `vi_node` with no parameter file, `online` is `false` and `global_thread_num` is `1`.

| Parameter | Default | Meaning |
|---|---|---|
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

## Actions

Actions live in `config/params.yaml` under `action_list`, next to `ros__parameters`. ROS 2 cannot store a list of maps as a parameter, so the node reads this list from the file. The number of entries is the number of actions. Add or remove entries to change the set. Each entry needs all three fields:

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

`onestep_forward_m` is the forward step in meters. Negative is backward. `onestep_rotation_deg` is the rotation in degrees. Negative is clockwise when viewed from above. At most 127 actions.

The shipped list is six motions: 0.3 m forward, 0.2 m back, a turn of 20 degrees either way, and the same turn combined with 0.2 m forward.

## Solver test

This checks the solver without ROS. It needs a C++17 compiler and pthreads.

```bash
c++ -std=c++17 -O2 -pthread -I include \
  src/planner.cpp src/actions.cpp src/global_planner.cpp src/local_planner.cpp \
  src/test_planner.cpp -o /tmp/planner_test
/tmp/planner_test
```

`planner_test ok` means the action list parsed, two thread counts agreed, a far cell reached the goal, a walled-in cell stayed blocked, and a laser hit raised costs without lowering any.

## Differences from value_iteration and value_iteration2

`value_iteration` is the ROS 1 planner. `value_iteration2` is its ROS 2 port. This package keeps the same kind of goal, scan, and velocity command, and changes when the field is solved and which inputs are accepted.

Shared with both of them:

- An occupancy grid, a pose in `map`, and a laser scan.
- A goal cell must lie inside `goal_margin_radius` and `goal_margin_theta`. The node defaults are 0.2 m and 10 degrees. Many `value_iteration` launch files override those to 0.3 m and 15 degrees.
- The same six motions are shipped: 0.3 m forward, 0.2 m back, ±20 degrees, and ±20 degrees with 0.2 m forward.
- Free cells cost 1 second to enter. Cells within `safety_radius` (0.2 m) cost an extra `safety_radius_penalty` (30 seconds).
- A laser hit inside 1 m adds a local penalty of 2048 seconds, and cells along the beam have that penalty halved.
- `/cmd_vel` repeats the chosen step. It is not a smooth velocity.

Different in this package:

| | `value_iteration` | `value_iteration2` | this package |
|---|---|---|---|
| ROS | ROS 1 | ROS 2 | ROS 2 |
| Goal input | Action `value_iteration/ViAction` | `/goal_pose` | `/goal_pose` |
| Map service | `static_map`, or `cost_map` when `map_type` is `cost` | `/map_server/map`, or `/cost_map` for a cost grid | `/map_server/map` only. Cell `0` is free |
| Value output | `policy` and `value` services (`grid_map_msgs/GetGridMap`) | `/value_function` occupancy grid, every 1.5 s | `/value_function` occupancy grid, every 1.5 s |
| Actions | `action_list` parameter. The count can change | Six actions fixed in the source. The `action_list` in its YAML is commented out | `action_list` in `config/params.yaml`. The count can change, up to 127 |
| Threads | `thread_num`, default 4 | `global_thread_num`, default 1 in code and 2 in its YAML. `local_thread_num` is in the YAML and is never read | `global_thread_num`, default 1 without a file and 2 in `params.yaml`. `0` uses every logical core. There is no `local_thread_num` |
| When solving starts | The field is swept continuously in the background, including before a goal | Same continuous sweep. Threads start toward the origin before any goal | Solving starts when `/goal_pose` arrives, and runs until that goal's field stops changing |
| Laser update | A separate thread keeps sweeping only the 1 m window, even when no new scan arrives. `local_xy_range` is fixed at 1 m | Same one local thread and the same fixed 1 m window | A scan that changes a penalty starts an update from those cells. The update may leave the window, and it stops when that change settles. `local_xy_range` is a parameter |
| Scan samples | Every range in the message | Every range in the message | Drops non-finite ranges and ranges outside the scan's reported limits. Subscription uses sensor-data QoS |
| No action at the current cell | `/cmd_vel` is published only while `online` is true and an action exists | A zero command is published while idling. A cell with no action publishes nothing, so the previous command can remain | A zero command is published, so the robot waits |
| YAML keys `gthread_num`, `gsafety_*`, `ggoal_*`, `gmap_type` | Not used | Present in `params.yaml` and not read by the node | Not used. The names this node reads are listed under [Parameters](#parameters) |

`map_type: cost` exists only in the earlier packages. A cost grid gives each free cell its own immediate cost, and cells with value 255 are blocked. This node does not read that grid.

## License

BSD-3-Clause. Copyright (c) 2026, nop and CIT autonomous robot laboratory.

This package refers to two earlier BSD-3-Clause programs by Ryuichi Ueda:

- `value_iteration`, Copyright (c) 2021, CIT autonomous robot laboratory
- `value_iteration2`, Copyright (c) 2024, Ryuichi Ueda and CIT autonomous robot laboratory
