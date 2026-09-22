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

## License

BSD-3-Clause. Copyright (c) 2026, nop and CIT autonomous robot laboratory.

This package refers to two earlier BSD-3-Clause programs by Ryuichi Ueda:

- `value_iteration`, Copyright (c) 2021, CIT autonomous robot laboratory
- `value_iteration2`, Copyright (c) 2024, Ryuichi Ueda and CIT autonomous robot laboratory
