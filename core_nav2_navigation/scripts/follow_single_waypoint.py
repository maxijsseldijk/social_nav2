#! /usr/bin/env python3

import rclpy
from core_nav2_navigation.multi_task_navigator import MultiTaskNavigator, TaskResult


def main():

    rclpy.init(args=None)
    navigator = MultiTaskNavigator()
    namespace = navigator.get_namespace()

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active(localizer=f"{namespace}/planner_server")

    while rclpy.ok():
        i = 0
        navigator.previous_task_number = navigator.task_number
        goal_update = navigator.update_goal()
        if len(goal_update) != 1:
            navigator.get_logger().error(
                'Expected exactly one goal point, but got {}'.format(len(goal_update)))
            exit(1)
        task_points = goal_update[0]
        navigator.goToPose(task_points)

        while not navigator.isTaskComplete():
            if navigator.new_task_requested():
                new_points = navigator.update_goal()
                navigator.goToPose(new_points[0])

            i = i + 1

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            pass

        elif result == TaskResult.CANCELED:
            exit(1)
        elif result == TaskResult.FAILED:
            navigator.get_logger().info('Task failed! Returning to start...')

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
