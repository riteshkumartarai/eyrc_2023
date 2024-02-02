#! /usr/bin/env python3


# Team ID:		1796
# Author List:		Soumitra Naik,Ritesh Kumar Tarai
# Filename:		bot_controller_4c.py
# Nodes:		publish-'/cmd_vel/bot1','/cmd_vel/bot2','/cmd_vel/bot3'
################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Pose2D
# from std_msgs.msg import Bool
import math


class BOT_Controller5(Node):
    def __init__(self):
        super().__init__("bot_control5")
        self.pub_bot_1 = self.create_publisher(Twist, "/cmd_vel/bot1", 1)
        self.pub_bot_2 = self.create_publisher(Twist, "/cmd_vel/bot2", 1)
        self.pub_bot_3 = self.create_publisher(Twist, "/cmd_vel/bot3", 1)
        # self.sub_bot_1 = self.create_subscription(Pose2D, "/pen2_pose",self.rectangle_callback, 10)
        self.sub_bot_2 = self.create_subscription(Pose2D, "/pen2_pose",self.hexagon_callback, 10)
        self.sub_bot_3 = self.create_subscription(Pose2D, "/pen1_pose",self.tringle_callback, 10)
        # self.sub_bot_3 = self.create_subscription(Pose2D, "/pen2_pose",self.rectangle_callback, 10)
        self.hexagon=[[200, 150], [175, 200], [125, 200], [100, 150], [125, 100], [175, 100],[200, 150]]
        self.hex_i=0
        self.hex_coordinate=self.hexagon[self.hex_i]
        self.tringle=[[300, 100], [400, 100], [300, 200], [300, 100]]
        self.tri_i=0
        self.tri_coordinate=self.tringle[self.tri_i]
        self.rectangle=[[200, 300], [400, 300], [400, 400], [200, 400], [200, 300]]
        self.rec_i=0
        self.rec_coordinate=self.rectangle[self.rec_i]
        self.kp_straight=2.5
        self.ki=0.0000 #.00001
        self.integral_left = 0.0
        self.integral_right = 0.0
        self.integral_rear = 0.0
    
    def pi_controller(self,error,integral):
    
        # Proportional term
        proportional = self.kp_straight * error

        # Integral term
        integral += error
        integral_term = self.ki * integral

        # Calculate the control signal
        control_signal = proportional + integral_term
        if( control_signal>70):
            control_signal=70
        if(control_signal<-70):
            control_signal=-70
        return control_signal, integral


    def inverse_kinematics(self,error_x,error_y,error_theta):
        # if(self.distance(error_x,error_y)>80):
        #     self.kp_straight=.5
        # elif(self.distance(error_x,error_y)>25):
        #     self.kp_straight=1.5
        # else:
        #     self.kp_straight=4
         # PI controller for left wheel
        force_left, self.integral_left = self.pi_controller(-error_x * 1/3 - error_y * 0.57735 + error_theta * 1/3,
                                                            self.integral_left)

        # PI controller for right wheel
        force_right, self.integral_right = self.pi_controller(-error_x * 1/3 + error_y * 0.57735 + error_theta * 1/3,
                                                              self.integral_right)

        # PI controller for rear wheel
        force_rear, self.integral_rear = self.pi_controller(error_x * 2/3 + error_theta * 1/3,
                                                            self.integral_rear)

        return float(-force_left)*1.5, float(-force_right), float(-force_rear)
        
        

    
    def hexagon_callback(self,msg):
        msg_bot = Twist()
        # Calculate Error from feedback
        error_x=self.hex_coordinate[0]-msg.x
        error_y=-self.hex_coordinate[1]+msg.y
        error_theta=-msg.theta*(180/math.pi)
        print(f"{self.hex_coordinate[0]}  -----  {self.hex_coordinate[1]}------  +{self.distance(error_x,error_y)}")
        print("-------")
        if(self.distance(error_x,error_y)>2):
            msg_bot.linear.x,msg_bot.linear.y,msg_bot.linear.z=self.inverse_kinematics(error_x,error_y,error_theta*7)
            self.pub_bot_3.publish(msg_bot)
        else:
            if self.hex_i<6:
                self.hex_i+=1
                self.hex_coordinate=self.hexagon[self.hex_i]
            else:
                msg_bot.linear.x,msg_bot.linear.y,msg_bot.linear.z=self.inverse_kinematics(0,0,0)
                self.pub_bot_3.publish(msg_bot)

        

    def rectangle_callback(self,msg):
        msg_bot = Twist()
        error_x=self.rec_coordinate[0]-msg.x
        error_y=-self.rec_coordinate[1]+msg.y
        error_theta=-msg.theta*(180/math.pi)
        print(f"{self.rec_coordinate[0]}  -----  {self.rec_coordinate[1]}------  +{self.distance(error_x,error_y)}")
        print("-------")
        if(self.distance(error_x,error_y)>5):
            msg_bot.linear.x,msg_bot.linear.y,msg_bot.linear.z=self.inverse_kinematics(error_x,error_y,error_theta*7)
            self.pub_bot_3.publish(msg_bot)
        else:
            if self.rec_i<4:
                self.rec_i+=1
                self.rec_coordinate=self.rectangle[self.rec_i]
                
            else:
                msg_bot.linear.x,msg_bot.linear.y,msg_bot.linear.z=self.inverse_kinematics(0,0,0)
                self.pub_bot_3.publish(msg_bot)
        
       
    def tringle_callback(self,msg):
        msg_bot = Twist()
        error_x=self.tri_coordinate[0]-msg.x
        error_y=-self.tri_coordinate[1]+msg.y
        error_theta=-msg.theta*(180/math.pi)
        print(f"{self.tri_coordinate[0]}  -----  {self.tri_coordinate[1]}------  +{self.distance(error_x,error_y)}")
        print(f"-------")
        if(self.distance(error_x,error_y)>2):
            msg_bot.linear.x,msg_bot.linear.y,msg_bot.linear.z=self.inverse_kinematics(error_x,error_y,error_theta*7)
            self.pub_bot_2.publish(msg_bot)
        else:
            if self.tri_i<3:
                self.tri_i+=1
                self.tri_coordinate=self.tringle[self.tri_i]
            else:
                msg_bot.linear.x,msg_bot.linear.y,msg_bot.linear.z=self.inverse_kinematics(0,0,0)
                self.pub_bot_2.publish(msg_bot)
        
    def distance(self,x_error,y_error):
        return math.sqrt(y_error**2+x_error**2)

        
def main(args=None):
    rclpy.init(args=args)
    bot_controller5= BOT_Controller5()
    rclpy.spin(bot_controller5)
    bot_controller5.destroy_node()
    rclpy.shutdown()


# Entry point of the script
if __name__ == "__main__":
    main()
