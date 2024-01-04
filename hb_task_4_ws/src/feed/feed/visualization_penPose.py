#! /usr/bin/env python3


################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
import cv2
import numpy as np
from cv_bridge import CvBridge


# Import the required modules
##############################################################
class visualization(Node):
    def __init__(self):
        super().__init__("Visualization")
        self.cv_bridge = CvBridge()
        # Subscribe the topic /camera/image_raw
        self.imageSubscribe = self.create_subscription(
            Image, "/transformed_image_raw", self.image_callback, 10
        )
        self.pose_sub_1 = self.create_subscription(Pose2D, "/pen1_pose",self.poseCallback1, 10)
        self.pose_sub_2 = self.create_subscription(Pose2D, "/pen2_pose",self.poseCallback2, 10)
        self.pose_sub_3 = self.create_subscription(Pose2D, "/pen3_pose",self.poseCallback3, 10)
        self.poseList_1=[]
        self.poseList_2=[]
        self.poseList_3=[]
        
    def image_callback(self, msg):
        # convert ROS image to opencv image
        self.viz_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
    def poseCallback1(self,msg):
        self.viz_img=cv2.circle(self.viz_img,(int(msg.x),int(msg.y)),5,(0,0,255),2)
        self.poseList_1.append((int(msg.x),int(msg.y)))
        
    
    
    def poseCallback2(self,msg):
        self.viz_img=cv2.circle(self.viz_img,(int(msg.x),int(msg.y)),5,(0,255,0),2)
        self.poseList_2.append((int(msg.x),int(msg.y)))
        

    def poseCallback3(self,msg):
        self.viz_img=cv2.circle(self.viz_img,(int(msg.x),int(msg.y)),5,(255,0,0),2)
        self.poseList_3.append((int(msg.x),int(msg.y)))
        
    

        
        


def main(args=None):
    rclpy.init(args=args)

    visualize_pose = visualization()
    while rclpy.ok():
        rclpy.spin_once(visualize_pose)
        for i in range(1,len(visualize_pose.poseList_1)):
            visualize_pose.viz_img=cv2.line(visualize_pose.viz_img, visualize_pose.poseList_1[i - 1], visualize_pose.poseList_1[i], (0,0,255), 2)
        for i in range(1,len(visualize_pose.poseList_2)):
            visualize_pose.viz_img=cv2.line(visualize_pose.viz_img, visualize_pose.poseList_2[i - 1], visualize_pose.poseList_2[i], (0,255,0), 2)
        for i in range(1,len(visualize_pose.poseList_3)):
            visualize_pose.viz_img=cv2.line(visualize_pose.viz_img, visualize_pose.poseList_3[i - 1], visualize_pose.poseList_3[i], (255,0,0), 2)
        cv2.imshow("gg",visualize_pose.viz_img)
        cv2.waitKey(1)
        rclpy.spin_once(visualize_pose)

    visualize_pose .destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
