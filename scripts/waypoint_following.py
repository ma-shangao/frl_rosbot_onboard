#! /usr/bin/env python3
# Copyright 2023 MA Song, developed based on: 
# https://github.com/ros-planning/navigation2/issues/2283
#
import sys

import rclpy

from multi-ugv_client.basic_navigator import BasicNavigator

def main(argv=sys.argv[1:]):
    rclpy.init()
    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = Pose()
    initial_pose.position.x = 3.45
    initial_pose.position.y = 2.15
    initial_pose.orientation.z = 1.0
    initial_pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -2.0
    goal_pose.pose.position.y = -0.5
    goal_pose.pose.orientation.w = 1.0
    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isNavComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelNav()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == GoalStatus.STATUS_SUCCEEDED:
        print('Goal succeeded!')
    elif result == GoalStatus.STATUS_CANCELED:
        print('Goal was canceled!')
    elif result == GoalStatus.STATUS_ABORTED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    exit(0)


if __name__ == '__main__':
    main()
