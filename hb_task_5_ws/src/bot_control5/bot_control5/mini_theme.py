#! /usr/bin/env python3


# Team ID:		1796
# Author List:		Soumitra Naik,Ritesh Kumar Tarai
# Filename:		bot_controller_5B.py
# Nodes:		publish-'/cmd_vel/bot1','/cmd_vel/bot2','/cmd_vel/bot3'
################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Pose2D
from std_msgs.msg import Bool
import math
from std_srvs.srv import Empty


class BOT_Controller_miniTheme(Node):
    def __init__(self):
        super().__init__("mini_theme")
        self.pen1=self.create_publisher(Bool,'/pen1_down',5)
        self.pen2=self.create_publisher(Bool,'/pen2_down',5)
        self.pen3=self.create_publisher(Bool,'/pen3_down',5)
        self.pub_bot_1 = self.create_publisher(Twist, "/cmd_vel/bot1", 5)
        self.pub_bot_2 = self.create_publisher(Twist, "/cmd_vel/bot2", 5)
        self.pub_bot_3 = self.create_publisher(Twist, "/cmd_vel/bot3", 5)
        self.pen_down=Bool()
        self.t1=0.0
        self.t2=2*math.pi/3
        self.t3=4*math.pi/3
        self.sub_bot_1 = self.create_subscription(Pose2D, "/pen1_pose",self.callbackBot1, 5)
        self.sub_bot_2 = self.create_subscription(Pose2D, "/pen2_pose",self.callbackBot2, 5)
        self.sub_bot_3 = self.create_subscription(Pose2D, "/pen3_pose",self.callbackBot3, 5)
        self.kp_straight=0
        self.ki=0.0000 #.00001
        self.integral_left = 0.0
        self.integral_right = 0.0
        self.integral_rear = 0.0
        self.bot1_status=0
        self.bot2_status=0
        self.bot3_status=0
        # client for the "stopflag" service
        self.cli = self.create_client(Empty, 'Stop_Flag')  
        

    def callbackBot1(self,msg):
        msg_bot = Twist()
        x,y=self.lissaousPointGenerate(self.t1)
        # Calculate Error from feedback
        error_x=x-msg.x
        error_y=-y+msg.y
        error_theta=-msg.theta*(180/math.pi)
        if(self.t1>0 and self.t1<=2*math.pi/3):
            self.pen_down.data=True
            self.pen1.publish(self.pen_down)
        else:
            self.pen_down.data=False
            self.pen1.publish(self.pen_down)
        if(self.distance(error_x,error_y)>1 and self.t1<=2*math.pi/3):
            msg_bot.linear.x,msg_bot.linear.y,msg_bot.linear.z=self.inverse_kinematics(error_x,error_y,error_theta*5)
            self.pub_bot_1.publish(msg_bot)
        else:
            msg_bot.linear.x,msg_bot.linear.y,msg_bot.linear.z=self.inverse_kinematics(0,0,0)
            self.pub_bot_1.publish(msg_bot)
            if(self.t1<2*math.pi/3):
                self.t1=self.t1+.05
            else:
                self.pen_down.data=False
                self.pen1.publish(self.pen_down)

    def callbackBot2(self,msg):
        msg_bot = Twist()
        x,y=self.lissaousPointGenerate(self.t2)
        # Calculate Error from feedback
        error_x=x-msg.x
        error_y=-y+msg.y
        error_theta=-msg.theta*(180/math.pi)
        if(self.t2>2*math.pi/3 and self.t2<=4*math.pi/3):
            self.pen_down.data=True
            self.pen2.publish(self.pen_down)
        else:
            self.pen_down.data=False
            self.pen2.publish(self.pen_down)
        if(self.distance(error_x,error_y)>1):
            msg_bot.linear.x,msg_bot.linear.y,msg_bot.linear.z=self.inverse_kinematics(error_x,error_y,error_theta*5)
            self.pub_bot_2.publish(msg_bot)
        else:
            msg_bot.linear.x,msg_bot.linear.y,msg_bot.linear.z=self.inverse_kinematics(0,0,0)
            self.pub_bot_2.publish(msg_bot)
            if(self.t2<4*math.pi/3):
                self.t2=self.t2+.05
            else:
                self.pen_down.data=False
                self.pen2.publish(self.pen_down)

    def callbackBot3(self,msg):
        msg_bot = Twist()
        x,y=self.lissaousPointGenerate(self.t3)
        # Calculate Error from feedback
        error_x=x-msg.x
        error_y=-y+msg.y
        error_theta=-msg.theta*(180/math.pi)
        if(self.t3>4*math.pi/3 and self.t3<=2*math.pi):
            self.pen_down.data=True
            self.pen3.publish(self.pen_down)
        else:
            self.pen_down.data=False
            self.pen3.publish(self.pen_down)
        if(self.distance(error_x,error_y)>1 ):
            msg_bot.linear.x,msg_bot.linear.y,msg_bot.linear.z=self.inverse_kinematics(error_x,error_y,error_theta*5)
            self.pub_bot_3.publish(msg_bot)
        else:
            msg_bot.linear.x,msg_bot.linear.y,msg_bot.linear.z=self.inverse_kinematics(0,0,0)
            self.pub_bot_3.publish(msg_bot)
            if(self.t3<2*math.pi):
                self.t3=self.t3+.05
            else:
                self.pen_down.data=False
                self.pen3.publish(self.pen_down)

    
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
        
        if abs(force_right)>40 or abs(force_left)>40 or abs(force_rear)>40:
            if abs(force_left) >= abs(force_right) and abs(force_left) >= abs(force_rear):
                force_left=(40*force_left)/abs(force_left)
                force_right=(40*force_right)/abs(force_left)
                force_rear=(40*force_rear)/abs(force_left)
            elif abs(force_right )>= abs(force_left) and abs(force_right) >= abs(force_rear):
                force_left=(40*force_left)/abs(force_right)
                force_right=(40*force_right)/abs(force_right)
                force_rear=(40*force_rear)/abs(force_right)
            else:
                force_left=(40*force_left)/abs(force_rear)
                force_right=(40*force_right)/abs(force_rear)
                force_rear=(40*force_rear)/abs(force_rear)

        return float(-force_left)*1.5, float(-force_right), float(-force_rear)
        
    def lissaousPointGenerate(self,t):
        x = 200 * math.cos(t)
        y = 150 * math.sin(4 * t)
        return round(x)+250, 250-round(y) 

    def distance(self,x_error,y_error):
        return math.sqrt(y_error**2+x_error**2)    

    


        
def main(args=None):
    rclpy.init(args=args)
    bot_controller_miniTheme= BOT_Controller_miniTheme()
    rclpy.spin(bot_controller_miniTheme)
    bot_controller_miniTheme.destroy_node()
    rclpy.shutdown()


# Entry point of the script
if __name__ == "__main__":
    main()
