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
    task_points = []
    while rclpy.ok():
        if first_call:
            i = 0
            navigator.previous_task_number = navigator.task_number
            task_points = navigator.update_goal()
            first_call = False
        navigator.goThroughPoses(task_points)

        while not navigator.isTaskComplete():
            if navigator.new_task_requested():
                task_points = navigator.update_goal()

                i = i + 1

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
