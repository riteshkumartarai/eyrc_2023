#!/usr/bin/env python3

# Team ID:		1796
# Author List:	Soumitra Naik
# Filename:		feedback_node.py
# Topic:		publish-    "/pen1_pose","/pen2_pose","/pen3_pose"
#               subscribe-   "/transformed_image_raw" 
################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
import math
import cv2
import numpy as np
from cv_bridge import CvBridge
##########################################################

class FeedbackNode(Node):
    def __init__(self):
        super().__init__("feedback_node")
        self.cv_bridge = CvBridge()
        self.pose = Pose2D()
        self.pub_1 = self.create_publisher(Pose2D, "/pen1_pose", 1)
        self.pub_2 = self.create_publisher(Pose2D, "/pen2_pose", 1)
        self.pub_3 = self.create_publisher(Pose2D, "/pen3_pose", 1)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self.aruco_params = cv2.aruco.DetectorParameters()
        # Subscribe to the topic /transformed_image_raw
        self.sub = self.create_subscription(
            Image, "/transformed_image_raw", self.image_callback, 1
        )

    def image_callback(self, msg):
        # Convert ROS image to OpenCV image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.visualize_image = cv_image
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # Detect ArUco markers
        corners, ids, rejected_points = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )
        self.publish_bot_position(ids, corners)

    #calculate pose from its co-ordinate list
    def calculatePose(self, tpl):
        self.pose.x = (tpl[0][0] + tpl[2][0]) / 2.000
        self.pose.y = (tpl[0][1] + tpl[2][1]) / 2.000
        if float(-tpl[3][1] + tpl[0][1]) > 0:
            self.pose.theta = -math.acos(
                (-tpl[3][0] + tpl[0][0])
                / ((tpl[3][0] - tpl[0][0]) ** 2 + (tpl[3][1] - tpl[0][1]) ** 2) ** 0.5
            )
        else:
            self.pose.theta = math.acos(
                (-tpl[3][0] + tpl[0][0])
                / ((tpl[3][0] - tpl[0][0]) ** 2 + (tpl[3][1] - tpl[0][1]) ** 2) ** 0.5
            )
        return self.pose

    # Publish the bot coordinates to the topic /detected_aruco
    def publish_bot_position(self, bot_ids, aruco_coordinates):
        if bot_ids is not None:
                    for i in range(len(bot_ids)):
                        if bot_ids[i] == 1:
                            self.pub_1.publish(self.calculatePose(aruco_coordinates[i][0]))
                        if bot_ids[i] == 2:
                            self.pub_2.publish(self.calculatePose(aruco_coordinates[i][0]))
                        if bot_ids[i] == 3:
                            self.pub_3.publish(self.calculatePose(aruco_coordinates[i][0]))
                        
        

def main(args=None):
    rclpy.init(args=args)

    feedback_node = FeedbackNode()

    rclpy.spin(feedback_node)

    feedback_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
