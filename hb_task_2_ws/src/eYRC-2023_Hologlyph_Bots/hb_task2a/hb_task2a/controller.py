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


# Team ID:		1796
# Author List:		Soumitra Naik,Ritesh Kumar Tarai,Shrijoni Ghose,Adarsh Priyaranjan
# Filename:		feedback.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		publish-'/hb_bot_1/left_wheel_force','/hb_bot_1/right_wheel_force','/hb_bot_1/rear_wheel_force'
#               subscribe-'/detected_aruco'

################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench,Pose2D
import time
import math
from my_robot_interfaces.srv import NextGoal             

# You can add more if required
##############################################################
# Initialize Global variables
hb_x=250.0
hb_y=250.0
hb_theta=0
################# ADD UTILITY FUNCTIONS HERE #################

##############################################################


# Define the HBController class, which is a ROS node
class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')
        self.sub=self.create_subscription(Pose2D,'/detected_aruco',self.callback_pose,10)
        self.pub_left=self.create_publisher(Wrench,'/hb_bot_1/left_wheel_force',10)
        self.pub_right=self.create_publisher(Wrench,'/hb_bot_1/right_wheel_force',10)
        self.pub_rear=self.create_publisher(Wrench,'/hb_bot_1/rear_wheel_force',10)
       
        # For maintaining control loop rate.
        self.rate = self.create_rate(100)


        # client for the "next_goal" service
        self.cli = self.create_client(NextGoal, 'next_goal')      
        self.req = NextGoal.Request() 
        self.index = 0
        self.flag = 0

        #other required object
        self.msg_force_rear=Wrench()
        self.msg_force_right=Wrench()
        self.msg_force_left=Wrench()
        self.kp_straight=5
        self.kp_turn=100

    # callback function
    def callback_pose(self,data):
        global hb_theta,hb_x,hb_y
        hb_x=data.x
        hb_y=data.y
        hb_theta=data.theta
    
    # Method to create a request to the "next_goal" service
    def send_request(self, request_goal):
        self.req.request_goal = request_goal
        self.future = self.cli.call_async(self.req)
        

    def inverse_kinematics(self,error_x,error_y,error_theta):
        if(math.sqrt(error_x**2+error_y**2)<2):
            self.kp_straight=19.5
        elif(math.sqrt(error_x**2+error_y**2)>40):
            self.kp_straight=3
        else: 
            self.kp_straight=11
        # force transfermation for left wheel
        self.msg_force_left.force.y=  (-error_x * 1/3 - error_y * 0.57735 + error_theta * 1/3)*self.kp_straight
        # force transfermation for right wheel
        self.msg_force_right.force.y=  (-error_x * 1/3 + error_y * 0.57735 + error_theta * 1/3)*self.kp_straight
        # force transfermation for rear wheel
        self.msg_force_rear.force.y=  (error_x * 2/3 + error_theta * 1/3)*self.kp_straight
        # publishing to the corresponding wheel 
        self.pub_left.publish(self.msg_force_left)
        self.pub_right.publish(self.msg_force_right)
        self.pub_rear.publish(self.msg_force_rear)

    def turn(self,y):
        self.msg_force_left.force.y=y*self.kp_turn
        self.pub_rear.publish(self.msg_force_left)  
        self.pub_right.publish(self.msg_force_left)
        self.pub_left.publish(self.msg_force_left)
       


def main(args=None):
    rclpy.init(args=args)
    
    # Create an instance of the HBController class
    hb_controller = HBController()
   
    # Send an initial request with the index from ebot_controller.index
    hb_controller.send_request(hb_controller.index)
    
    # Main loop
    while rclpy.ok():
        
        # Check if the service call is done
        if hb_controller.future.done():
            try:
                # response from the service call
                response = hb_controller.future.result()
            except Exception as e:
                hb_controller.get_logger().infselfo(
                    'Service call failed %r' % (e,))
            else:
                #########           GOAL POSE             #########
                x_goal      = response.x_goal+250
                y_goal      = 250+response.y_goal
                theta_goal  = response.theta_goal
                hb_controller.flag = response.end_of_list
                ####################################################
                # Calculate Error from feedback
                error_x=x_goal-hb_x
                error_y=-y_goal+hb_y
                error_theta=theta_goal-hb_theta
                
                
                # print(f"{response.x_goal},{response.y_goal}")
                while rclpy.ok():
                    hb_controller.turn(0.0)
                                       
                    if (math.sqrt(error_x**2+error_y**2)>.5):
                        hb_controller.inverse_kinematics(error_x,error_y,error_theta)
                    else:
                        break    
                    rclpy.spin_once(hb_controller)
                    error_x=x_goal-hb_x
                    error_y=-y_goal+hb_y
                    error_theta=theta_goal-hb_theta
                    
                    
                ############     DO NOT MODIFY THIS       #########
                hb_controller.index += 1
                if hb_controller.flag == 1 :
                    hb_controller.index = 0
                hb_controller.send_request(hb_controller.index)
                ####################################################

        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)
        
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
