#! /usr/bin/env python3


################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
import math
import cv2
import numpy as np
from cv_bridge import CvBridge

# Import the required modules
##############################################################
class ArUcoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector_4')
        self.cvb=CvBridge()
        self.pose=Pose2D()
        self.pub_1=self.create_publisher(Pose2D,'\pen1_pose',10)
        self.pub_2=self.create_publisher(Pose2D,'\pen2_pose',10)
        self.pub_3=self.create_publisher(Pose2D,'\pen3_pose',10)
        self.bot_marker = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000) 
        self.params = cv2.aruco.DetectorParameters()
        # Subscribe the topic /camera/image_raw
        self.sub=self.create_subscription(Image,'/transformed_image_raw',self.image_callback,10)

    def image_callback(self, msg):
        #convert ROS image to opencv image
        cv_image = self.cvb.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY )
        #Detect Aruco marker
        c,i,r=cv2.aruco.detectMarkers(gray,self.bot_marker,parameters=self.params)      
        # Publish the bot coordinates to the topic  /detected_aruco
        self.bot_position(i,c)
        # cv2.imshow("s",cv_image)
        # cv2.waitKey(1)


    def publishPose(self,tpl):
        self.pose.x= ((tpl[0][0]+tpl[2][0])/2.000)
        self.pose.y= ((tpl[0][1]+tpl[2][1])/2.000)
        if(float(-tpl[0][1]+tpl[1][1])>0):
            self.pose.theta=-math.acos((-tpl[0][0]+tpl[1][0])/((tpl[0][0]-tpl[1][0])**2+(tpl[0][1]-tpl[1][1])**2)**.5)
        else:
            self.pose.theta=math.acos((-tpl[0][0]+tpl[1][0])/((tpl[0][0]-tpl[1][0])**2+(tpl[0][1]-tpl[1][1])**2)**.5)
        return self.pose
    
    def bot_position(self,bot_ids,aruco_coordinates):
        if np.argwhere(bot_ids==1).any():
            index=int(np.argwhere(bot_ids==1)[0][0])
            self.pub_1.publish(self.publishPose(aruco_coordinates[index][0])) 
        else:
            pass

        if np.argwhere(bot_ids==2).any():
            index=int(np.argwhere(bot_ids==2)[0][0])
            self.pub_2.publish(self.publishPose(aruco_coordinates[index][0])) 
        else:
            pass

        if np.argwhere(bot_ids==3).any():
            index=int(np.argwhere(bot_ids==3)[0][0])
            self.pub_3.publish(self.publishPose(aruco_coordinates[index][0])) 
        else:
            pass
         


def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
