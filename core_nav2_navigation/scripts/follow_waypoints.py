#! /usr/bin/env python3

import rclpy
import time
from core_nav2_navigation.multi_task_navigator import MultiTaskNavigator


def main():
    rclpy.init(args=None)
    navigator = MultiTaskNavigator()
    namespace = navigator.get_namespace()

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active(localizer=f"{namespace}/planner_server")

    first_call = True
    task_points = []
    task_check_interval = 0.1

    navigator.get_logger().info(
        "Starting waypoint following with enhanced task handling")

    while rclpy.ok():
        if first_call:
            navigator.get_logger().info(
                f"Initial task number: {navigator.task_number}")
            task_points = navigator.updateGoal()
            first_call = False

        if task_points:
            navigator.goThroughPoses(task_points)

        while not navigator.isTaskComplete():
            if navigator.newTaskRequested():

                navigator.cancelTask()
                task_points = navigator.updateGoal()

                status = navigator.getTaskUpdateStatus()
                navigator.get_logger().info(
                    f"Task update status: {status}")

                if task_points:
                    navigator.goThroughPoses(task_points)
                break

            time.sleep(task_check_interval)

        if navigator.isTaskComplete() and not navigator.newTaskRequested():
            navigator.get_logger().info(
                f"Task {navigator.task_number} completed successfully", throttle_duration_sec=10.0)

    navigator.get_logger().info("Navigation loop ended")
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
