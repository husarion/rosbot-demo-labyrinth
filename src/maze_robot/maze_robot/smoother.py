#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt

import numpy as np


class SmootherNode(Node):
    def __init__(self):
        super().__init__("smoother")

        self.path_sub_ = self.create_subscription(
            Path, 'plan', self.path_callback, 10)
        self.path_pub_ = self.create_publisher(Path, 'smoothed_plan', 10)
        self.poses_ = Path()

        self.get_logger().info("Smoother node has been started")

    def path_callback(self, msg):
        poses_ = Path()
        poses_ = msg.poses
        m_x = []
        m_y = []
        
        for pos in poses_:
            m_x.append(pos.pose.position.x)
            m_y.append(pos.pose.position.y)

        x = np.array(m_x)
        y = np.array(m_y)

        theta = np.polyfit(x, y ,55)
        model = np.poly1d(theta)

        print(str(model))
        px = x.tolist()
        py = model(x).tolist()

        smoothed_path_ = Path()
        smoothed_path_.header = msg.header
        for i, pos in enumerate(poses_):
            pos.pose.position.x = px[i]
            pos.pose.position.y = py[i]
            smoothed_path_.poses.append(pos)

        self.path_pub_.publish(smoothed_path_)

        

        # plt.plot(x, y, 'ro')
        # plt.plot(x, model(x))

        #print(m_x)


def main(args=None):
    rclpy.init(args=args)
    node = SmootherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
