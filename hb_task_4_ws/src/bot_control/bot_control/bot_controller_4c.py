#! /usr/bin/env python3


# Team ID:		1796
# Author List:		Soumitra Naik,Ritesh Kumar Tarai
# Filename:		bot_controller_4c.py
# Nodes:		publish-'/cmd_vel/bot1','/cmd_vel/bot2','/cmd_vel/bot3'
################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class BOT_Controller1(Node):
    def __init__(self):
        super().__init__("bot_control")
        self.end_time_tringle_1 = time.time() + 3
        self.end_time_tringle_2 = self.end_time_tringle_1 + 3
        self.end_time_tringle_3 = self.end_time_tringle_2 + 3
        
        self.end_time_square_1 = time.time() + 3
        self.end_time_square_2 = self.end_time_square_1 + 3
        self.end_time_square_3 = self.end_time_square_2 + 3
        self.end_time_square_4 = self.end_time_square_3 + 3

        self.pub_bot_2 = self.create_publisher(Twist, "/cmd_vel/bot1", 3)
        self.pub_bot_3 = self.create_publisher(Twist, "/cmd_vel/bot3", 3)
        self.pub_bot_1 = self.create_publisher(Twist, "/cmd_vel/bot2", 3)
        self.timer = self.create_timer(1, self.timer_callback_tringle)
        self.timer1 = self.create_timer(1, self.timer_callback_square)
        self.timer2 = self.create_timer(1, self.timer_callback_circle)

    def timer_callback_circle(self):
        msg_bot = Twist()
        msg_bot.linear.x = 90.0
        msg_bot.linear.y = 10.0
        msg_bot.linear.z = 0.0
        self.pub_bot_1.publish(msg_bot)

    def timer_callback_square(self):
        msg_bot = Twist()
        if time.time()<self.end_time_square_1:
            msg_bot.linear.x = 90.0
            msg_bot.linear.y = -90.0
            msg_bot.linear.z = 0.0
        elif time.time()<self.end_time_square_2:
            msg_bot.linear.x = -20.0
            msg_bot.linear.y = -20.9422
            msg_bot.linear.z = 90.0
        elif time.time()<self.end_time_square_3:
            msg_bot.linear.x = -90.0
            msg_bot.linear.y = 90.0
            msg_bot.linear.z = 0.0
        elif time.time()<self.end_time_square_4:
            msg_bot.linear.y = 30.0
            msg_bot.linear.x = 25.9422
            msg_bot.linear.z = -90.0
        self.pub_bot_3.publish(msg_bot)
        if time.time() > self.end_time_square_4:
            self.end_time_square_1 = time.time() + 3
            self.end_time_square_2 = self.end_time_square_1 + 3
            self.end_time_square_3 = self.end_time_square_2 + 3
            self.end_time_square_4 = self.end_time_square_3 + 3

    def timer_callback_tringle(self):
        msg_bot = Twist()
        if time.time()<self.end_time_tringle_1:
            msg_bot.linear.x=0.0
            msg_bot.linear.y=90.0
            msg_bot.linear.z=-90.0
        elif time.time()<self.end_time_tringle_2:
            msg_bot.linear.x=-90.0
            msg_bot.linear.y=0.0
            msg_bot.linear.z=90.0
        elif time.time()<self.end_time_tringle_3:
            msg_bot.linear.x=90.0
            msg_bot.linear.y=-90.0
            msg_bot.linear.z=0.0
        self.pub_bot_2.publish(msg_bot)
        if time.time()>self.end_time_tringle_3:
            self.end_time_tringle_1 = time.time() + 3
            self.end_time_tringle_2 = self.end_time_tringle_1 + 3
            self.end_time_tringle_3 = self.end_time_tringle_2 + 3

def main(args=None):
    rclpy.init(args=args)
    bot_controller1 = BOT_Controller1()
    rclpy.spin(bot_controller1)
    bot_controller1.destroy_node()
    rclpy.shutdown()


# Entry point of the script
if __name__ == "__main__":
    main()
