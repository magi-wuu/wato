#!/usr/bin/env python3
"""Exercise the four real C++ nodes with synthetic ROS input in an isolated domain.

This is an integration check, not a substitute for a Gazebo navigation demo.
Run with the robot image and /opt/watonomous/setup.bash sourced.
"""
import math
import os
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster


def main():
    rclpy.init()
    node = rclpy.create_node('navigation_smoke_test')
    children = []
    logs = []
    received = {}
    subscriptions = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for topic, message_type in [('/map', OccupancyGrid), ('/path', Path),
                                ('/costmap', OccupancyGrid), ('/cmd_vel', Twist)]:
        subscriptions.append(node.create_subscription(
            message_type, topic, lambda message, key=topic: received.update({key: message}),
            qos if topic in ('/map', '/path') else 10))
    scan_pub = node.create_publisher(LaserScan, '/lidar', 10)
    odom_pub = node.create_publisher(Odometry, '/odom/filtered', 10)
    goal_pub = node.create_publisher(PointStamped, '/goal_point', 10)
    tf = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = 'sim_world'
    transform.child_frame_id = 'robot/chassis/lidar'
    transform.transform.rotation.w = 1.0
    transform.header.stamp = node.get_clock().now().to_msg()
    tf.sendTransform(transform)

    def inputs():
        stamp = node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'sim_world'
        odom.child_frame_id = 'robot/chassis/lidar'
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = odom.child_frame_id
        scan.range_min, scan.range_max = 0.1, 20.0
        scan.angle_min = -math.pi
        scan.angle_increment = 2 * math.pi / 256
        scan.ranges = [float('inf')] * 256
        # An obstacle off the desired route tests marking without blocking the goal.
        scan.ranges[192] = 5.0
        scan_pub.publish(scan)

    def wait_for(predicate, timeout=12, publish=True):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if publish:
                inputs()
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
            time.sleep(0.04)
        raise AssertionError('Timed out waiting for expected ROS behavior')

    try:
        for package in ('costmap', 'map_memory', 'planner', 'control'):
            log = open('/tmp/' + package + '_smoke.log', 'w+')
            logs.append(log)
            children.append(subprocess.Popen(
                ['ros2', 'run', package, package + '_node'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        wait_for(lambda: '/map' in received, publish=False)
        assert all(value == -1 for value in received['/map'].data)
        print('PASS: initial unknown map published before sensor input', flush=True)
        wait_for(lambda: '/costmap' in received and 100 in received['/map'].data)
        print('PASS: scan -> inflated costmap -> transformed persistent map', flush=True)
        goal = PointStamped()
        goal.header.frame_id = 'sim_world'
        goal.point.x = 3.0
        goal_pub.publish(goal)
        wait_for(lambda: '/path' in received and len(received['/path'].poses) > 0
                 and '/cmd_vel' in received and received['/cmd_vel'].linear.x > 0)
        assert received['/path'].header.frame_id == 'sim_world'
        print('PASS: goal -> A* path -> forward velocity', flush=True)
        # A blocked goal must invalidate the previous path and command zero speed.
        goal.point.x, goal.point.y = 0.0, 5.0
        goal_pub.publish(goal)
        wait_for(lambda: not received['/path'].poses
                 and received['/cmd_vel'].linear.x == 0
                 and received['/cmd_vel'].angular.z == 0)
        print('PASS: unreachable goal cancels motion', flush=True)
        goal.point.x, goal.point.y = 3.0, 0.0
        goal_pub.publish(goal)
        wait_for(lambda: received['/cmd_vel'].linear.x > 0)
        wait_for(lambda: received['/cmd_vel'].linear.x == 0
                 and received['/cmd_vel'].angular.z == 0, timeout=3, publish=False)
        print('PASS: stale odometry stops motion', flush=True)
    finally:
        for child in children:
            os.killpg(child.pid, signal.SIGINT)
        for child in children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
        for log in logs:
            log.seek(0)
            print(log.read()[-2000:])
            log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
