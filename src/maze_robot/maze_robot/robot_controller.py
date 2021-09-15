#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
from geometry_msgs.msg import PoseStamped
from custom_interfaces.srv import Start
from custom_interfaces.srv import UpdateMap

#from nav2_msgs.srv import LoadMap
from nav_msgs.msg import Path

import time

from threading import Event
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__("robot_controller")

        self.future_done_event = Event()
        self.callback_group = ReentrantCallbackGroup()

        self.goal_x = 10.5
        self.goal_y = 8.0

        self.path_ = Path()

        self.start_server_ = self.create_service(
            Start, "start", self.callback_start_server, callback_group=self.callback_group)
        self.goal_pose_pub_ = self.create_publisher(
            PoseStamped, 'goal_pose', 10)
        self.get_logger().info("Robot controller node has been started")

        
    # Start server
    def callback_start_server(self, request, response):

        self.goal_x = request.x
        self.goal_y = request.y

        time.sleep(1.0)
        if self.call_update_map():

            time.sleep(1.0)
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = self.goal_x
            goal.pose.position.y = self.goal_y
            self.goal_pose_pub_.publish(goal)

            response.success = True
            self.get_logger().info("Goal pose sent successfully")
        else:
            response.success = False
            self.get_logger().error("Failed to send goal pose")

        return response

    # update map client
    def call_update_map(self):
        client = self.create_client(
            UpdateMap, "update_map", callback_group=self.callback_group)
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for UpdateMap server...")

        request = UpdateMap.Request()

        self.future_done_event.clear()

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_update_map))

        self.future_done_event.wait()

        return future.result().success

    def callback_update_map(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Map updated successfully")
            else:
                self.get_logger().info("Failed to update map")
            self.future_done_event.set()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
