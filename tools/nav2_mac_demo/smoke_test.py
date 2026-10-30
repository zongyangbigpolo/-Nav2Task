#!/usr/bin/env python3
"""Verify the Jazzy TB3 sandbox demo; run with an outer wall-clock timeout."""

import argparse
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--initial-x', type=float, default=-2.0)
    parser.add_argument('--initial-y', type=float, default=-0.5)
    parser.add_argument('--goal-x', type=float, default=-0.5)
    parser.add_argument('--goal-y', type=float, default=-0.5)
    parser.add_argument('--output', default='/tmp/nav2-demo/result.json')
    args = parser.parse_args()

    Path(args.output).unlink(missing_ok=True)
    rclpy.init()
    navigator = BasicNavigator(node_name='first_demo_verifier')
    navigator.set_parameters([Parameter('use_sim_time', value=True)])
    tf_buffer = Buffer()
    _tf_listener = TransformListener(tf_buffer, navigator)
    counts = dict(clock=0, scan=0, odom=0, plan=0, moving_cmd=0)
    odom_positions = []
    distance = 0.0
    valid_scan = False
    odom_speed = math.inf

    def clock_callback(msg):
        if msg.clock.sec > 0 or msg.clock.nanosec > 0:
            counts['clock'] += 1

    def scan_callback(msg):
        nonlocal valid_scan
        counts['scan'] += 1
        valid_scan = valid_scan or any(
            math.isfinite(r) and msg.range_min <= r <= msg.range_max
            for r in msg.ranges
        )

    def odom_callback(msg):
        nonlocal distance, odom_speed
        position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        velocity = msg.twist.twist
        odom_speed = max(math.hypot(velocity.linear.x, velocity.linear.y),
                         abs(velocity.angular.z))
        if odom_positions:
            distance += math.dist(odom_positions[-1], position)
        odom_positions.append(position)
        counts['odom'] += 1

    def plan_callback(msg):
        if len(msg.poses) > 1:
            counts['plan'] += 1

    def velocity_callback(msg):
        if math.hypot(msg.linear.x, msg.linear.y) > 0.01 or abs(msg.angular.z) > 0.01:
            counts['moving_cmd'] += 1

    subscriptions = [
        navigator.create_subscription(Clock, '/clock', clock_callback, qos_profile_sensor_data),
        navigator.create_subscription(LaserScan, '/scan', scan_callback, qos_profile_sensor_data),
        navigator.create_subscription(Odometry, '/odom', odom_callback, qos_profile_sensor_data),
        navigator.create_subscription(NavPath, '/plan', plan_callback, qos_profile_sensor_data),
        navigator.create_subscription(Twist, '/cmd_vel', velocity_callback, qos_profile_sensor_data),
    ]

    def pose(x, y):
        result = PoseStamped()
        result.header.frame_id = 'map'
        result.header.stamp = navigator.get_clock().now().to_msg()
        result.pose.position.x = x
        result.pose.position.y = y
        result.pose.orientation.w = 1.0
        return result

    def spin_until(predicate, seconds, description):
        deadline = time.monotonic() + seconds
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError(description)
            rclpy.spin_once(navigator, timeout_sec=0.1)

    goal_started = False
    try:
        spin_until(
            lambda: counts['clock'] > 1 and counts['odom'] > 1 and valid_scan,
            60, 'No advancing simulation clock, odometry, or valid laser scan',
        )
        navigator.setInitialPose(pose(args.initial_x, args.initial_y))
        # BasicNavigator republishes the initial pose until AMCL responds.
        # The outer GNU timeout also bounds its lifecycle/service waits.
        navigator.waitUntilNav2Active()
        start = pose(args.initial_x, args.initial_y)
        goal = pose(args.goal_x, args.goal_y)
        path = navigator.getPath(start, goal)
        if path is None or len(path.poses) < 2:
            raise RuntimeError('Planner did not return a non-empty path')
        initial_odom = odom_positions[-1]
        initial_distance = distance
        started = time.monotonic()
        if not navigator.goToPose(goal):
            raise RuntimeError('NavigateToPose goal was rejected')
        goal_started = True
        while not navigator.isTaskComplete():
            if time.monotonic() - started > 120:
                raise TimeoutError('Navigation exceeded 120 wall-clock seconds')
        task_result = navigator.getResult()
        goal_started = False
        if task_result != TaskResult.SUCCEEDED:
            raise RuntimeError(f'Navigation failed: {task_result}')
        completed = time.monotonic()
        spin_until(
            lambda: time.monotonic() - completed >= 1.0 and odom_speed < 0.01,
            10, 'Robot did not settle after navigation',
        )
        spin_until(
            lambda: tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()),
            5, 'No map -> base_link transform',
        )
        transform = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        position = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1 - 2 * (rotation.y ** 2 + rotation.z ** 2),
        )
        error = math.hypot(position.x - args.goal_x, position.y - args.goal_y)
        displacement = math.dist(initial_odom, odom_positions[-1])
        report = {
            'result': 'SUCCEEDED',
            'initial_xy': [args.initial_x, args.initial_y],
            'goal_xy': [args.goal_x, args.goal_y],
            'final_map_xy': [position.x, position.y],
            'position_error_m': error,
            'yaw_error_rad': abs(yaw),
            'odom_displacement_m': displacement,
            'odom_travel_m': distance - initial_distance,
            'planned_poses': len(path.poses),
            'wall_seconds': completed - started,
            'messages_received': counts,
            'valid_laser_scan': valid_scan,
        }
        if not (
            error <= 0.25 and abs(yaw) <= 0.25 and displacement >= 0.5
            and counts['moving_cmd'] > 0 and counts['plan'] > 0
        ):
            raise RuntimeError(
                'Action succeeded but independent motion/tolerance checks failed: '
                + json.dumps(report)
            )
        Path(args.output).write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(report, indent=2), flush=True)
    finally:
        if goal_started:
            navigator.cancelTask()
        for subscription in subscriptions:
            navigator.destroy_subscription(subscription)
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
