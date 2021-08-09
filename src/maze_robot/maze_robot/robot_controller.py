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

#TODO start response after reaching goal


class RobotControllerNode(Node):
    def __init__(self):
        super().__init__("robot_controller")

        self.goal_x = 10.5
        self.goal_y = 8.0

        self.path_ = Path()

        self.start_server_ = self.create_service(
            Start, "start", self.callback_start_server)
        self.goal_pose_pub_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.get_logger().info("Robot controller node has been started")

        #self.call_update_map()

    # Start server
    def callback_start_server(self, request, response):

        self.goal_x = request.x
        self.goal_y = request.y

        self.call_update_map()
      
        time.sleep(5.0)

        goal = PoseStamped()
        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        self.goal_pose_pub_.publish(goal)

        #self.send_goal_navigate(self.goal_x, self.goal_y)  

        response.success = True
        return response

    # update map client
    def call_update_map(self):
        client=self.create_client(UpdateMap, "update_map")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for UpdateMap server...")

        request=UpdateMap.Request()

        future=client.call_async(request)
        future.add_done_callback(
            partial(self.callback_update_map))

        #return future

    def callback_update_map(self, future):
        try:
            response=future.result()
            if response.success:
                self.get_logger().info("Map updated successfully")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
