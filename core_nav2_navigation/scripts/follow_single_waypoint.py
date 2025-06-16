#! /usr/bin/env python3

import rclpy
from core_nav2_navigation.multi_task_navigator import MultiTaskNavigator


def main():

    rclpy.init(args=None)
    navigator = MultiTaskNavigator()
    namespace = navigator.get_namespace()

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active(localizer=f"{namespace}/planner_server")
    first_call = True
    while rclpy.ok():
        if first_call:
            i = 0
            navigator.previous_task_number = navigator.task_number
            task_points = navigator.update_goal()
            if len(task_points) != 1:
                navigator.get_logger().error(
                    'Expected exactly one goal point, but got {}'.format(len(task_points)))
                exit(1)
            first_call = False
        navigator.goToPose(task_points[0])

        while not navigator.isTaskComplete():
            if navigator.new_task_requested():
                task_points = navigator.update_goal()
            i = i + 1

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
