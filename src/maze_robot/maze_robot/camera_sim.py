#!/usr/bin/env python3
from os import truncate
from cv_bridge.core import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv_bridge
from cv_bridge import CvBridgeError
from functools import partial
from nav2_msgs.srv import LoadMap
from ament_index_python.packages import get_package_share_directory

from custom_interfaces.srv import UpdateMap
from custom_interfaces.srv import GetImage

import cv2
import numpy as np
import time
import os

from threading import Thread


# TODO Parameters for map scaling

def empty_callback():
    pass

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera")

        self.cv_map_image_ = None
        self.image_loaded_ = False
        self.goal_x = 0
        self.goal_y = 0
        self.bridge = CvBridge()

        self.save_map_url_ = os.path.join(get_package_share_directory('maze_bringup'), 'config/maze_cam.png')
        self.map_yaml_url_ = os.path.join(get_package_share_directory('maze_bringup'), 'config/map_sim.yaml')

        self.height_ = 0
        self.width_ = 0

        self.declare_parameter('camera_topic','camera_map/image_raw')
        self.declare_parameter('lower_hsv',[80, 0, 173])
        self.declare_parameter('upper_hsv',[179, 255, 255])

        self.camera_topic_ = self.get_parameter('camera_topic').value
        self.lower_hsv = np.array(self.get_parameter('lower_hsv').value)
        self.upper_hsv = np.array(self.get_parameter('upper_hsv').value)

        self.map_sub_ = self.create_subscription(
            Image, self.camera_topic_, self.map_sub_callback, 10)
        self.update_map_server_ = self.create_service(
            UpdateMap, 'update_map', self.update_map_server)

        self.get_logger().info("Camera node has been started")

        # self.convert_map(self.cv_map_image_)
        # self.call_load_map(self.map_yaml_url_)

    # update map server
    def update_map_server(self, request, response):
        try:
            self.convert_map(self.cv_map_image_)
            self.call_load_map(self.map_yaml_url_)
            response.success = True
        except:
            response.success = False
            self.get_logger().error("Failed to update map")
    
        return response

    # camera image subscriber callback
    def map_sub_callback(self, msg):
        try:
            self.cv_map_image_ = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

    def convert_map(self, img):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # lower_goal = np.array([50, 0, 0])
        # upper_goal = np.array([110, 255, 255])       

        # goal_mask = cv2.inRange(hsv_img, lower_goal, upper_goal)
        # goal = np.nonzero(goal_mask)
        # # depends on camera resolution 
        # self.goal_x = goal[1][0]/26.666
        # self.goal_y = (len(goal_mask) - goal[0][0])/26.666
        # #print(str(self.goal_y) + '   ' + str((self.goal_x)))

        mask = cv2.inRange(hsv_img, self.lower_hsv, self.upper_hsv)
        maze = cv2.bitwise_not(mask)
        # maze = cv2.rotate(maze, cv2.ROTATE_180) #use if image flipped

        cv2.imwrite(self.save_map_url_, maze)

    # load map client
    def call_load_map(self, map_url):
        client = self.create_client(LoadMap, "map_server/load_map")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for map server...")

        request = LoadMap.Request()
        request.map_url = map_url

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_load_map, map_url=map_url))

    def callback_load_map(self, future, map_url):
        try:
            response = future.result()
            if response.RESULT_SUCCESS == 1:
                self.get_logger().info("Map loaded successfully")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
