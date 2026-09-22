# value_iteration3

ROS 2 向けの価値反復ナビゲータです。`value_iteration2` と同じ使い方で、地図全体の求解が速くなっています。

製作者は nop（noplab90@gmail.com）です。ライセンスは BSD-3-Clause で、著作権者は nop と CIT autonomous robot laboratory です。

Ryuichi Ueda による次のソフトウェアを参考にしています。いずれも BSD-3-Clause です。

- `value_iteration` — Copyright (c) 2021, CIT autonomous robot laboratory
- `value_iteration2` — Copyright (c) 2024, Ryuichi Ueda and CIT autonomous robot laboratory

## 起動

このディレクトリをワークスペースの `src` に置き、地図サーバと `map` → `base_link` の TF、`/scan` がある状態で:

```bash
colcon build --packages-select value_iteration3
source install/setup.bash
ros2 launch value_iteration3 vi.launch.py
```

`/goal_pose` に目標を出すと価値関数を計算し、`/cmd_vel` に速度を出し続けます。`/scan` はロボット周囲 1 m の即時代金を更新します。`/value_function` は現在の向きのコストです。地図は `/map_server/map` から読みます。

## パラメータ

`config/params.yaml` を参照。`global_thread_num: 0` は論理コア数です。行動は次の 6 つです。

| 名前 | 前進 [m] | 回転 [deg] |
|---|---:|---:|
| forward | 0.3 | 0 |
| back | -0.2 | 0 |
| right | 0 | -20 |
| rightfw | 0.2 | -20 |
| left | 0 | 20 |
| leftfw | 0.2 | 20 |

## テスト

ROS なしでソルバだけ確認できます。

```bash
c++ -std=c++17 -O2 -pthread -I include src/planner.cpp src/test_planner.cpp -o /tmp/planner_test
/tmp/planner_test
```
