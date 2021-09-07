#!/usr/bin/env python3
from cv_bridge.core import CvBridge
import rclpy
from rclpy.node import Node
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

from threading import Event
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# TODO Parameters for map scaling

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera")

        self.future_done_event = Event()
        self.callback_group = ReentrantCallbackGroup()

        self.cv_map_image_ = None
        self.goal_x = 0
        self.goal_y = 0
        self.bridge = CvBridge()

        self.save_map_url_ = os.path.join(get_package_share_directory('maze_bringup'), 'config/maze_cam.png')
        self.map_yaml_url_ = os.path.join(get_package_share_directory('maze_bringup'), 'config/map.yaml')

        self.declare_parameter('camera_topic','image_raw')
        self.declare_parameter('lower_hsv',[80, 0, 173])
        self.declare_parameter('upper_hsv',[179, 255, 255])

        self.camera_topic_ = self.get_parameter('camera_topic').value
        self.lower_hsv = np.array(self.get_parameter('lower_hsv').value)
        self.upper_hsv = np.array(self.get_parameter('upper_hsv').value)

        self.update_map_server_ = self.create_service(
            UpdateMap, 'update_map', self.update_map_server,callback_group=self.callback_group)

        self.get_logger().info("Camera node has been started")

    # update map server
    def update_map_server(self, request, response):
        try:
            self.call_get_image()
            time.sleep(0.5)
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

        mask = cv2.inRange(hsv_img, self.lower_hsv, self.upper_hsv)
        maze = cv2.bitwise_not(mask)
        #maze = cv2.rotate(maze, cv2.ROTATE_180) #use to flip image

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

    # get image client
    def call_get_image(self):
        client = self.create_client(GetImage, "get_image",callback_group=self.callback_group)
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for get_image server...")

        request = GetImage.Request()

        self.future_done_event.clear()

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_get_image))

        self.future_done_event.wait()
        

    def callback_get_image(self, future):
        try:
            response = future.result()
            img = response.image
            try:
                self.cv_map_image_ = self.bridge.imgmsg_to_cv2(img, "bgr8")
                self.get_logger().info("Image from camera loaded!")
                self.convert_map(self.cv_map_image_)
                self.call_load_map(self.map_yaml_url_)
                self.future_done_event.set()
            except CvBridgeError as e:
                print(e)
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
