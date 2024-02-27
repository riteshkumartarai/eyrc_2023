#! /usr/bin/env python3


# Team ID:		1796
# Author List:		Soumitra Naik
# Filename:		bot_controller_5A.py
# Nodes:		publish-'/cmd_vel/bot1','/cmd_vel/bot2','/cmd_vel/bot3'
################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Pose2D
from std_msgs.msg import Bool
import math


class BOT_Controller5(Node):
    def __init__(self):
        super().__init__("bot_control5")
        self.pen1=self.create_publisher(Bool,'/pen1_down',1)
        self.pen2=self.create_publisher(Bool,'/pen2_down',1)
        self.pen3=self.create_publisher(Bool,'/pen3_down',1)
        self.pub_bot_1 = self.create_publisher(Twist, "/cmd_vel/bot1", 1)
        self.pub_bot_2 = self.create_publisher(Twist, "/cmd_vel/bot2", 1)
        self.pub_bot_3 = self.create_publisher(Twist, "/cmd_vel/bot3", 1)
        self.sub_bot_1 = self.create_subscription(Pose2D, "/pen1_pose",self.hexagon_callback, 1)
        self.sub_bot_2 = self.create_subscription(Pose2D, "/pen2_pose",self.rectangle_callback, 1)
        self.sub_bot_3 = self.create_subscription(Pose2D, "/pen3_pose",self.tringle_callback, 1)
        self.pen_down=Bool()
        self.msg_bot = Twist()
        self.hexagon=[[200,150], [175, 200], [125, 200], [100, 150], [125, 100], [175, 100],[200, 150]]
        self.hex_i=0
        self.hex_coordinate=self.hexagon[self.hex_i]
        self.tringle=[[300, 100],[400, 100], [350,150],[300, 200],[300, 150],[300, 100]]
        self.tri_i=0
        self.tri_coordinate=self.tringle[self.tri_i]
        self.rectangle=[[200, 300],[250, 300],[300, 300],[350, 300], [400, 300],[400, 350], [400, 400], [350, 400],[300, 400],[250, 400],[200, 400],[200, 350], [200, 300]]
        self.rec_i=0
        self.rec_coordinate=self.rectangle[self.rec_i]
        self.kp_straight=0
        self.kp_spin=0
        self.ki=0.00000 #.00001
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
        return control_signal, integral


    def inverse_kinematics(self,error_x,error_y,error_theta):
         # PI controller for left wheel
        force_left, self.integral_left = self.pi_controller(-error_x * 1/3 - error_y * 0.57735 + error_theta * 1/3,
                                                            self.integral_left)

        # PI controller for right wheel
        force_right, self.integral_right = self.pi_controller(-error_x * 1/3 + error_y * 0.57735 + error_theta * 1/3,
                                                              self.integral_right)

        # PI controller for rear wheel
        force_rear, self.integral_rear = self.pi_controller(error_x * 2/3 + error_theta * 1/3,
                                                            self.integral_rear)
        
        if abs(force_right)>60 or abs(force_left)>60 or abs(force_rear)>60:
            if abs(force_left) >= abs(force_right) and abs(force_left) >= abs(force_rear):
                force_right=(60*force_right)/abs(force_left)
                force_rear=(60*force_rear)/abs(force_left)
                force_left=(60*force_left)/abs(force_left)
            elif abs(force_right )>= abs(force_left) and abs(force_right) >= abs(force_rear):
                force_left=(60*force_left)/abs(force_right)
                force_rear=(60*force_rear)/abs(force_right)
                force_right=(60*force_right)/abs(force_right)
            else:
                force_left=(60*force_left)/abs(force_rear)
                force_right=(60*force_right)/abs(force_rear)
                force_rear=(60*force_rear)/abs(force_rear)

        return float(-force_left), float(-force_right), float(-force_rear)
        
        
    def thetaCorrection(self,error_theta):
        force=error_theta*self.kp_spin
        if(abs(force)>50):
            force=50*force/abs(force)
        return float(force), float(force), float(force)


    def hexagon_callback(self,msg):
        # Calculate Error from feedback
        error_x=self.hex_coordinate[0]-msg.x
        error_y=-self.hex_coordinate[1]+msg.y
        error_theta=-msg.theta
        if(self.hex_i>0 and self.hex_i<7):
            self.pen_down.data=True
            self.pen1.publish(self.pen_down)
        else:
            self.pen_down.data=False
            self.pen1.publish(self.pen_down)
        if(self.distance(error_x,error_y)>1):
            self.msg_bot.linear.x,self.msg_bot.linear.y,self.msg_bot.linear.z=self.inverse_kinematics(error_x,error_y,error_theta*100)
            self.pub_bot_1.publish(self.msg_bot)
        else:
            if self.hex_i<6:
                self.hex_i+=1
                self.hex_coordinate=self.hexagon[self.hex_i]
            else:
                self.hex_i+=1
                self.msg_bot.linear.x,self.msg_bot.linear.y,self.msg_bot.linear.z=self.inverse_kinematics(0,0,0)
                self.pub_bot_1.publish(self.msg_bot)
       


    def rectangle_callback(self,msg):
        error_x=self.rec_coordinate[0]-msg.x
        error_y=-self.rec_coordinate[1]+msg.y
        error_theta=-msg.theta
        if(self.rec_i>0 and self.rec_i<13):
            self.pen_down.data=True
            self.pen2.publish(self.pen_down)
        else:
            self.pen_down.data=False    
            self.pen2.publish(self.pen_down)
        # if(abs(error_theta)<0.1):
        if(self.distance(error_x,error_y)>1):
            self.msg_bot.linear.x,self.msg_bot.linear.y,self.msg_bot.linear.z=self.inverse_kinematics(error_x,error_y,error_theta*100)
            self.pub_bot_2.publish(self.msg_bot)
        else:
            if self.rec_i<12:
                self.rec_i+=1
                self.rec_coordinate=self.rectangle[self.rec_i]
            else:
                self.rec_i+=1
                self.msg_bot.linear.x,self.msg_bot.linear.y,self.msg_bot.linear.z=self.inverse_kinematics(0,0,0)
                self.pub_bot_2.publish(self.msg_bot)
        # else:
        #     self.msg_bot.linear.x,self.msg_bot.linear.y,self.msg_bot.linear.z=self.thetaCorrection(error_theta)
        #     self.pub_bot_2.publish(self.msg_bot)
        
       
    def tringle_callback(self,msg):
        error_x=self.tri_coordinate[0]-msg.x
        error_y=-self.tri_coordinate[1]+msg.y
        error_theta=-msg.theta
        if(self.tri_i>0 and self.tri_i<6):
            self.pen_down.data=True
            self.pen3.publish(self.pen_down)
        else:
            self.pen_down.data=False    
            self.pen3.publish(self.pen_down)
        
        if(self.distance(error_x,error_y)>1):
            self.msg_bot.linear.x,self.msg_bot.linear.y,self.msg_bot.linear.z=self.inverse_kinematics(error_x,error_y,error_theta*100)
            self.pub_bot_3.publish(self.msg_bot)
        else:
            if self.tri_i<5:
                self.tri_i+=1
                self.tri_coordinate=self.tringle[self.tri_i]
            else:
                self.tri_i+=1
                self.msg_bot.linear.x,self.msg_bot.linear.y,self.msg_bot.linear.z=self.inverse_kinematics(0,0,0)
                self.pub_bot_3.publish(self.msg_bot)
            # if abs(error_theta)>0.1:
            #     self.msg_bot.linear.x,self.msg_bot.linear.y,self.msg_bot.linear.z=self.thetaCorrection(error_theta)
            #     self.pub_bot_3.publish(self.msg_bot)


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
