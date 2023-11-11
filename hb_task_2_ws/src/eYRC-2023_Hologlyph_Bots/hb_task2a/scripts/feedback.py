#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2A of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''
################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
import math
import cv2
from cv_bridge import CvBridge

# Import the required modules
##############################################################
class ArUcoDetector(Node):

    def __init__(self):
        super().__init__('ar_uco_detector')
        self.cvb=CvBridge()
        self.pose=Pose2D()
        self.pub=self.create_publisher(Pose2D,'/detected_aruco',10)
        self.bot_marker = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000) 
        self.params = cv2.aruco.DetectorParameters()
        # Subscribe the topic /camera/image_raw
        self.sub=self.create_subscription(Image,'/camera/image_raw',self.image_callback,10)

    def image_callback(self, msg):
        #convert ROS image to opencv image
        cv_image = self.cvb.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY )
        #Detect Aruco marker
        c,i,r=cv2.aruco.detectMarkers(gray,self.bot_marker,parameters=self.params)      
        # Publish the bot coordinates to the topic  /detected_aruco
        if i[1][0]!=1:
            pass
        else:
            self.publishPose(c[1][0])
        # cv2.imshow("s",cv_image)
        # cv2.waitKey(1)
    def publishPose(self,tpl):
        self.pose.x= ((tpl[0][0]+tpl[2][0])/2.000)
        self.pose.y= ((tpl[0][1]+tpl[2][1])/2.000)
        if(float(-tpl[0][1]+tpl[1][1])>0):
            self.pose.theta=-math.acos((-tpl[0][0]+tpl[1][0])/((tpl[0][0]-tpl[1][0])**2+(tpl[0][1]-tpl[1][1])**2)**.5)
        else:
            self.pose.theta=math.acos((-tpl[0][0]+tpl[1][0])/((tpl[0][0]-tpl[1][0])**2+(tpl[0][1]-tpl[1][1])**2)**.5)
        self.pub.publish(self.pose)
         


def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
