# SPDX-FileCopyrightText: 2026 nop and CIT autonomous robot laboratory
# SPDX-License-Identifier: BSD-3-Clause

from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import pytest
import rclpy


@pytest.mark.parametrize('mode', ['occupancy', 'cost'])
def test_map_service(mode):
    """Exercise the installed node against both actual ROS 2 GetMap services."""
    rclpy.init()
    node = rclpy.create_node('map_service_test')
    calls = []
    grids = []

    def respond(name):
        def callback(request, response):
            calls.append(name)
            grid = response.map
            grid.info.width = 8
            grid.info.height = 8
            grid.info.resolution = 0.1
            grid.info.origin.position.x = 2.0
            grid.info.origin.position.y = 3.0
            grid.info.origin.orientation.w = 1.0
            grid.data = [1 if name == '/cost_map' else 0] * 64
            grid.data[63] = -1
            grid.data[36] = -2  # unsigned 254: free only in cost mode
            return response
        return callback

    services = [node.create_service(GetMap, name, respond(name))
                for name in ['/map_server/map', '/cost_map']]
    subscription = node.create_subscription(
        OccupancyGrid, '/value_function', grids.append, 10)
    publisher = node.create_publisher(PoseStamped, '/goal_pose', 10)
    executable = Path(get_package_prefix('value_iteration3')) / 'lib/value_iteration3/vi_node'
    process = subprocess.Popen([
        str(executable), '--ros-args', '-p', 'online:=true',
        '-p', f'map_type:={mode}', '-p', 'theta_cell_num:=60',
        '-p', 'goal_margin_theta:=30', '-p', 'goal_margin_radius:=0.3',
        '-p', 'global_thread_num:=1', '-p', 'cost_drawing_threshold:=10000',
    ])
    try:
        deadline = time.monotonic() + 30
        goal_sent = False
        while time.monotonic() < deadline and not grids:
            rclpy.spin_once(node, timeout_sec=0.1)
            assert process.poll() is None, 'vi_node exited before publishing'
            if calls and publisher.get_subscription_count() and not goal_sent:
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.pose.position.x = 2.15
                goal.pose.position.y = 3.15
                goal.pose.orientation.w = 1.0
                publisher.publish(goal)
                goal_sent = True
        assert calls == ['/cost_map' if mode == 'cost' else '/map_server/map']
        assert grids, 'no solved value function received'
        grid = grids[0]
        assert grid.info.width == 8 and grid.info.height == 8
        assert grid.info.origin.position.x == 2.0
        assert grid.info.origin.position.y == 3.0
        assert grid.data[63] == 100, '255 must stay blocked'
        if mode == 'cost':
            assert grid.data[36] < 100, '254 must be traversable'
        else:
            assert grid.data[36] == 100, 'nonzero occupancy must stay blocked'
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        node.destroy_subscription(subscription)
        for service in services:
            node.destroy_service(service)
        node.destroy_node()
        rclpy.shutdown()
