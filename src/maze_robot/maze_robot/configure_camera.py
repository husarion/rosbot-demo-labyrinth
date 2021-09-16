#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
import yaml
import os

from cv_bridge.core import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError
from ament_index_python.packages import get_package_share_directory

# TODO Parameters for map scalig

def empty_callback(x):
    pass

class ConfigureCameraNode(Node):
    def __init__(self):
        super().__init__("configure_camera")

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.params_url_ = os.path.join(get_package_share_directory(
            'maze_bringup'), 'config/camera_params.yaml')

        self.params_ = self.open_yaml(self.params_url_)
        self.cv_map_image_ = None
        self.bridge = CvBridge()

        # self.declare_parameter('camera_topic_config', 'usb_cam/image_raw')
        self.declare_parameter('camera_topic_config', self.params_[
                               'camera']['ros__parameters']['camera_topic'])
        self.declare_parameter('lower_hsv', self.params_[
                               'camera']['ros__parameters']['lower_hsv'])
        self.declare_parameter('upper_hsv', self.params_[
                               'camera']['ros__parameters']['upper_hsv'])

        self.camera_topic_ = self.get_parameter('camera_topic_config').value
        self.lower_hsv = np.array(self.get_parameter('lower_hsv').value)
        self.upper_hsv = np.array(self.get_parameter('upper_hsv').value)

        self.map_sub_ = self.create_subscription(
            Image, self.camera_topic_, self.map_sub_callback, qos_profile=qos_policy)
        self.timer_ = self.create_timer(0.01, self.convert_map)

        self.get_logger().info(
            "Camera configuration node has been started \nPress spacebar to save current hsv values\nPress Esc to exit")

        cv2.namedWindow('camera view')
        key = cv2.waitKey(10)

        cv2.createTrackbar(
            'H lower', 'camera view', self.lower_hsv[0], 179, empty_callback)
        cv2.createTrackbar(
            'S lower', 'camera view', self.lower_hsv[1], 255, empty_callback)
        cv2.createTrackbar(
            'V lower', 'camera view', self.lower_hsv[2], 255, empty_callback)
        cv2.createTrackbar(
            'H upper', 'camera view', self.upper_hsv[0], 179, empty_callback)
        cv2.createTrackbar(
            'S upper', 'camera view', self.upper_hsv[1], 255, empty_callback)
        cv2.createTrackbar(
            'V upper', 'camera view', self.upper_hsv[2], 255, empty_callback)

    # camera image subscriber callback
    def map_sub_callback(self, msg):
        try:
            self.cv_map_image_ = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

    # convert image from camera 
    def convert_map(self):

        if self.cv_map_image_ is None:
            return

        key = cv2.waitKey(10)

        hsv_img = cv2.cvtColor(self.cv_map_image_, cv2.COLOR_BGR2HSV)

        h_lower = cv2.getTrackbarPos('H lower', 'camera view')
        h_upper = cv2.getTrackbarPos('H upper', 'camera view')
        s_lower = cv2.getTrackbarPos('S lower', 'camera view')
        s_upper = cv2.getTrackbarPos('S upper', 'camera view')
        v_lower = cv2.getTrackbarPos('V lower', 'camera view')
        v_upper = cv2.getTrackbarPos('V upper', 'camera view')

        self.lower_hsv = np.array([h_lower, s_lower, v_lower])
        self.upper_hsv = np.array([h_upper, s_upper, v_upper])
        mask = cv2.inRange(hsv_img, self.lower_hsv, self.upper_hsv)

        maze = cv2.bitwise_not(mask)
        con = cv2.bitwise_and(self.cv_map_image_,
                                 self.cv_map_image_, mask=mask)

        if key == 32:
            self.save_hsv(self.lower_hsv, self.upper_hsv)

        if key == 27:
            exit()

        cv2.imshow('camera view', cv2.resize(con, (640, 320)))
        cv2.imshow('generated map', cv2.resize(maze, (640, 320)))

        cv2.waitKey(10)

        mask = cv2.inRange(hsv_img, self.lower_hsv, self.upper_hsv)
        maze = cv2.bitwise_not(mask)

    # save HSV values to yaml file
    def save_hsv(self, lower_hsv, upper_hsv):

        data = self.open_yaml(self.params_url_)

        data['camera']['ros__parameters']['lower_hsv'] = lower_hsv.tolist()
        data['camera']['ros__parameters']['upper_hsv'] = upper_hsv.tolist()

        with open(self.params_url_, 'w', encoding='utf8') as outfile:
            yaml.dump(data, outfile, default_flow_style=False,
                      allow_unicode=True)

        print('HSV values saved:')
        print('lower_hsv: ' + str(lower_hsv.tolist()))
        print('upper_hsv: ' + str(upper_hsv.tolist()))

    # open yaml file
    def open_yaml(self, url):
        with open(url, 'r') as stream:
            try:
                data = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        return data


def main(args=None):
    rclpy.init(args=args)
    node = ConfigureCameraNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
