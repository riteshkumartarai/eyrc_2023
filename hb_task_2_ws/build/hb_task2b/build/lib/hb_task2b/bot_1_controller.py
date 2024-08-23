#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2B of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''


# Team ID:		1796
# Author List:		Soumitra Naik,Ritesh Kumar Tarai,Shrijoni Ghose,Adarsh Priyaranjan
# Filename:		bot_1_controller.py
# Functions:
#			[ inverse_kinematics,turn,goalCallback,callback_pose]
# Nodes:		publish-'/hb_bot_1/left_wheel_force','/hb_bot_1/right_wheel_force','/hb_bot_1/rear_wheel_force'
#               subscribe-'/detected_aruco_1'

################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
import time
import math
from geometry_msgs.msg import Wrench,Pose2D
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.msg import Goal             

##############################################################
# Initialize Global variables
hb_x=250.0
hb_y=250.0
hb_theta=0.0

class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller_bot1')
        

        # Initialise the required variables
        self.bot_1_x = []
        self.bot_1_y = []
        self.bot_1_theta = 0.0

        # Initialze Publisher and Subscriber
        self.sub=self.create_subscription(Pose2D,'/detected_aruco_1',self.callback_pose,10)
        self.pub_left=self.create_publisher(Wrench,'/hb_bot_1/left_wheel_force',10)
        self.pub_right=self.create_publisher(Wrench,'/hb_bot_1/right_wheel_force',10)
        self.pub_rear=self.create_publisher(Wrench,'/hb_bot_1/rear_wheel_force',10)
       

        #Similar to this you can create subscribers for hb_bot_2 and hb_bot_3
        self.subscription = self.create_subscription(
            Goal,  
            'hb_bot_1/goal',  
            self.goalCallBack,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        )  

        self.subscription  # Prevent unused variable warning

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)
        #other required object
        self.msg_force_rear=Wrench()
        self.msg_force_right=Wrench()
        self.msg_force_left=Wrench()
        self.index=0
        self.kp_turn=50

    def inverse_kinematics(self,error_x,error_y,angular_vel):
        
        if(math.sqrt(error_x**2+error_y**2)<10):
            self.kp_straight=20
        elif(math.sqrt(error_x**2+error_y**2)>40):
            self.kp_straight=.75
        else: 
            self.kp_straight=7
        # force transfermation for left wheel
        self.msg_force_left.force.y=  (-error_x * 1/3 - error_y * 0.57735 + angular_vel * 1/3)*self.kp_straight
        # force transfermation for right wheel
        self.msg_force_right.force.y=  (-error_x * 1/3 + error_y * 0.57735 + angular_vel * 1/3)*self.kp_straight
        # force transfermation for rear wheel
        self.msg_force_rear.force.y=  (error_x * 2/3 + angular_vel * 1/3)*self.kp_straight
        # publishing to the corresponding wheel 
        self.pub_left.publish(self.msg_force_left)
        self.pub_right.publish(self.msg_force_right)
        self.pub_rear.publish(self.msg_force_rear)

    def goalCallBack(self, msg):
        self.bot_1_x = msg.x
        self.bot_1_y = msg.y
        self.bot_1_theta = msg.theta
        

    def callback_pose(self,data):
        global hb_theta,hb_x,hb_y
        hb_x=data.x
        hb_y=data.y
        hb_theta=data.theta
        

    def turn(self,y):
        self.msg_force_left.force.y=y*self.kp_turn
        self.pub_rear.publish(self.msg_force_left)  
        self.pub_right.publish(self.msg_force_left)
        self.pub_left.publish(self.msg_force_left)

def main(args=None):
    rclpy.init(args=args)
    
    hb_controller = HBController()
    while rclpy.ok():
        rclpy.spin_once(hb_controller)
        if any(hb_controller.bot_1_x):
            break 
    # Main loop
    while rclpy.ok():
        
        x_goal=hb_controller.bot_1_x[hb_controller.index]
        y_goal=hb_controller.bot_1_y[hb_controller.index]
        theta_goal=hb_controller.bot_1_theta
        if(theta_goal>math.pi):
            theta_goal=theta_goal-2*math.pi
    
        # Calculate Error from feedback
        error_x=x_goal-hb_x
        error_y=-y_goal+hb_y
        error_theta=theta_goal-hb_theta
        
        if (math.sqrt(error_x**2+error_y**2)>.5):
            if(abs(hb_theta)>.08):
                hb_controller.turn(-hb_theta)
            else:
                hb_controller.inverse_kinematics(error_x,error_y,0.1)
        else:
            hb_controller.index+=1
            if(len(hb_controller.bot_1_x)==hb_controller.index):
                hb_controller.index=0

        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
