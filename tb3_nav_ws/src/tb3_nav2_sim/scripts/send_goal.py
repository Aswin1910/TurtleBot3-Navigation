#!/usr/bin/env python3
"""
Send TurtleBot3 to a goal with Nav2 and report progress.

    ros2 run tb3_nav2_sim send_goal.py                 # default goal: (4.0, 4.0)
    ros2 run tb3_nav2_sim send_goal.py 3.5 -3.0 90     # x y yaw_deg
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

START = (-4.0, -4.0, 0.0)   # must match spawn pose in the launch file


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def main():
    gx, gy, gyaw = 4.0, 4.0, 0.0
    argv = [a for a in sys.argv[1:] if not a.startswith('--')]
    if len(argv) >= 2:
        gx, gy = float(argv[0]), float(argv[1])
    if len(argv) >= 3:
        gyaw = math.radians(float(argv[2]))

    rclpy.init()
    nav = BasicNavigator()

    # Make sure AMCL is localised (harmless if params already set it)
    nav.setInitialPose(make_pose(nav, *START))
    nav.waitUntilNav2Active()

    goal = make_pose(nav, gx, gy, gyaw)
    nav.get_logger().info(f'Navigating to x={gx:.2f}, y={gy:.2f}, yaw={math.degrees(gyaw):.0f} deg')
    nav.goToPose(goal)

    i = 0
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        i += 1
        if fb and i % 10 == 0:
            nav.get_logger().info(
                f'distance remaining: {fb.distance_remaining:.2f} m | '
                f'recoveries: {fb.number_of_recoveries}')

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        nav.get_logger().info('Goal reached!')
    elif result == TaskResult.CANCELED:
        nav.get_logger().warn('Goal was canceled.')
    else:
        nav.get_logger().error('Goal failed.')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
