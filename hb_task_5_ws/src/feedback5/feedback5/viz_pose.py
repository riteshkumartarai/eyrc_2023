#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
import cv2
from cv_bridge import CvBridge
import numpy as np

class VisualizationNode(Node):
    def __init__(self):
        super().__init__("visualization")
        self.cv_bridge = CvBridge()
        # Subscribe to the topic /transformed_image_raw
        self.image_subscription = self.create_subscription(
            Image, "/transformed_image_raw", self.image_callback, 10
        )
        self.pose_subscription_1 = self.create_subscription(Pose2D, "/pen1_pose", self.pose_callback_1, 10)
        self.pose_subscription_2 = self.create_subscription(Pose2D, "/pen2_pose", self.pose_callback_2, 10)
        self.pose_subscription_3 = self.create_subscription(Pose2D, "/pen3_pose", self.pose_callback_3, 10)
        self.pose_list_1 = []
        self.pose_list_2 = []
        self.pose_list_3 = []

    def image_callback(self, msg):
        # Convert ROS image to OpenCV image
        self.viz_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def pose_callback_1(self, msg):
        # Draw circle and append pose to the list for pen 1
        self.viz_img = cv2.circle(self.viz_img, (int(msg.x), int(msg.y)), 5, (0, 0, 255), 2)
        self.pose_list_1.append((int(msg.x), int(msg.y)))

    def pose_callback_2(self, msg):
        # Draw circle and append pose to the list for pen 2
        self.viz_img = cv2.circle(self.viz_img, (int(msg.x), int(msg.y)), 5, (0, 255, 0), 2)
        self.pose_list_2.append((int(msg.x), int(msg.y)))

    def pose_callback_3(self, msg):
        # Draw circle and append pose to the list for pen 3
        self.viz_img = cv2.circle(self.viz_img, (int(msg.x), int(msg.y)), 5, (255, 0, 0), 2)
        self.pose_list_3.append((int(msg.x), int(msg.y)))

def main(args=None):
    rclpy.init(args=args)

    visualization_node = VisualizationNode()
    
    while rclpy.ok():
        rclpy.spin_once(visualization_node)

        # Draw lines connecting consecutive poses for each pen
        for i in range(1, len(visualization_node.pose_list_1)):
            visualization_node.viz_img = cv2.line(visualization_node.viz_img,
                                                  visualization_node.pose_list_1[i - 1],
                                                  visualization_node.pose_list_1[i],
                                                  (0, 0, 255), 2)
        for i in range(1, len(visualization_node.pose_list_2)):
            visualization_node.viz_img = cv2.line(visualization_node.viz_img,
                                                  visualization_node.pose_list_2[i - 1],
                                                  visualization_node.pose_list_2[i],
                                                  (0, 255, 0), 2)
        for i in range(1, len(visualization_node.pose_list_3)):
            visualization_node.viz_img = cv2.line(visualization_node.viz_img,
                                                  visualization_node.pose_list_3[i - 1],
                                                  visualization_node.pose_list_3[i],
                                                  (255, 0, 0), 2)

        # Display the image with visualizations
        cv2.imshow("Visualization", visualization_node.viz_img)
        # cv2.imshow("transformed",visualization_node.img)
        cv2.waitKey(1)
        rclpy.spin_once(visualization_node)

    visualization_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
