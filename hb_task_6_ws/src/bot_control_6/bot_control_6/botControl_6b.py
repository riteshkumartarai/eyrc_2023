# #! /usr/bin/env python3

"""
* Team Id:          1796
* Author List:      Soumitra Naik
* Filename:         botControl_6b.py
* Theme:            Hologlyph Bot
* Functions:        init, callbackBot1, callbackBot2, callbackBot3, pi_controller, 
                    inverse_kinematics, getGoal, distance, main
* Global Variables: None
"""

################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Bool
import math
from std_srvs.srv import Empty
from imgList import *

############################################################


class Bot_Control(Node):
    """
    * Function Name: __init__
    * Input: None
    * Output: None
    * Logic: Initializes the Bot_Control class and creates publishers, subscribers, and clients.
    * Example Call: bot_Control = Bot_Control()
    """
    def __init__(self):
        super().__init__("botControl")

        # Create publishers to send Bool messages to control Pen mechanism.
        self.pen1 = self.create_publisher(Bool, "/pen1_down", 1)
        self.pen2 = self.create_publisher(Bool, "/pen2_down", 1)
        self.pen3 = self.create_publisher(Bool, "/pen3_down", 1)

        # Create publishers to send Twist messages to control the movement of each bot.
        self.pub_bot_1 = self.create_publisher(Twist, "/cmd_vel/bot1", 1)
        self.pub_bot_2 = self.create_publisher(Twist, "/cmd_vel/bot2", 1)
        self.pub_bot_3 = self.create_publisher(Twist, "/cmd_vel/bot3", 1)

        # Initialize initial index for each bot_goal_lists.
        self.contour_list_1=0
        self.contour_list_2=0
        self.contour_list_3=0

        self.bot1_goal_index=0
        self.bot2_goal_index=0
        self.bot3_goal_index=0


        self.pen_down = Bool() # an instance of the Bool message type
        self.msg_bot = Twist() # an instance of the Twist message type

        # Subscribe to Pose2D topics for each bot to receive position information.
        self.sub_bot_1 = self.create_subscription(Pose2D, "/pen1_pose", self.callbackBot1, 1)
        self.sub_bot_2 = self.create_subscription(Pose2D, "/pen2_pose", self.callbackBot2, 1)
        self.sub_bot_3 = self.create_subscription(Pose2D, "/pen3_pose", self.callbackBot3, 1)

        # Client for the "stopflag" service
        self.cli = self.create_client(Empty, "Stop_Flag")
        self.req = Empty.Request() # Request instance for the "stopflag" service

        ############ INITIALIZING REQUIRED VARIABLES ###################

        # Initialize P-I (Proportional-Integral) controller values for each bot.
        self.kp_straight = 10  # Proportional gain for straight movement
        self.kp_theta = 200    # Proportional gain for angular movement
        self.ki = 0.0000       # Integral gain
        self.integral_left = 0.0   # Integral error for left wheel
        self.integral_right = 0.0  # Integral error for right wheel
        self.integral_rear = 0.0   # Integral error for rear wheel

        # Initialize variables to keep track of the status of each bot.
        # 0 represents that the bot has not reached its first goal yet.
        # 1 represents that the bot has reached its end goal.
        self.bot1_status = 0
        self.bot2_status = 0
        self.bot3_status = 0

    """
    * Function Name: callbackBot1
    * Input: msg (geometry_msgs.msg.Pose2D)
    * Output: None
    * Logic: Callback function for /pen1_pose topic. Handles bot movement and pen control for Bot 1.
    * Example Call: Invoked automatically when a message is received on the /pen1_pose topic.
    """
    def callbackBot1(self, msg):
        
        ##### GOAL CALL #####
        x_goal, y_goal = self.getGoal(1)  
        
        ##### ERROR CALCULATION #####
        error_x = x_goal - msg.x
        error_y = -y_goal + msg.y
        error_theta = -msg.theta

        ##### PEN DOWN LOGIC ########
        if self.bot1_goal_index > 0 and self.bot1_goal_index<len(bot1_goal[self.contour_list_1]):
            self.pen_down.data = True
            self.pen1.publish(self.pen_down)
        else:
            self.pen_down.data = False
            self.pen1.publish(self.pen_down)

        ##### GO-TO GOAL LOGIC ######
        if self.distance(error_x, error_y) > 1:
            self.msg_bot.linear.x, self.msg_bot.linear.y, self.msg_bot.linear.z = (
                self.inverse_kinematics(error_x, error_y, error_theta)
            )
            self.pub_bot_1.publish(self.msg_bot)
        else:

            self.msg_bot.linear.x, self.msg_bot.linear.y, self.msg_bot.linear.z = (
                self.inverse_kinematics(0, 0, 0)
            )
            self.pub_bot_1.publish(self.msg_bot)
            
            self.bot1_goal_index += 10
        ######## NEXT GOAL LOGIC ##########
            if self.bot1_goal_index>=len(bot1_goal[self.contour_list_1]):
                self.contour_list_1+=1   
                if(self.contour_list_1>=len(bot1_goal)):
                    self.contour_list_1-=1   
                    self.bot1_goal_index -= 10
                    self.pen_down.data = False
                    self.pen1.publish(self.pen_down)
                    self.pub_bot_1.publish(self.msg_bot)
                     #status set to 1 indicating the end of goal points
                    self.bot1_status=1
                    #Calling stop_flag service after goal completion of all three bots
                    if(self.bot1_status*self.bot2_status*self.bot3_status==1):
                        self.cli.call_async(self.req)
                else:
                    self.bot1_goal_index=0
        
            

                

    """
    * Function Name: callbackBot2
    * Input: msg (geometry_msgs.msg.Pose2D)
    * Output: None
    * Logic: Callback function for /pen2_pose topic. Handles bot movement and pen control for Bot 2.
    * Example Call: Invoked automatically when a message is received on the /pen2_pose topic.
    """
    def callbackBot2(self, msg):
        
        ##### GOAL CALL #####
        x_goal, y_goal = self.getGoal(2)
        
        ##### ERROR CALCULATION #####
        error_x = x_goal - msg.x
        error_y = -y_goal + msg.y
        error_theta = -msg.theta

        ##### PEN DOWN LOGIC ########
        if self.bot2_goal_index > 0 and self.bot2_goal_index<len(bot2_goal[self.contour_list_2]):
            self.pen_down.data = True
            self.pen2.publish(self.pen_down)
        else:
            self.pen_down.data = False
            self.pen2.publish(self.pen_down)

        ##### GO-TO GOAL LOGIC ######
        if self.distance(error_x, error_y) > 1:
            self.msg_bot.linear.x, self.msg_bot.linear.y, self.msg_bot.linear.z = (
                self.inverse_kinematics(error_x, error_y, error_theta)
            )
            self.pub_bot_2.publish(self.msg_bot)
        else:

            self.msg_bot.linear.x, self.msg_bot.linear.y, self.msg_bot.linear.z = (
                self.inverse_kinematics(0, 0, 0)
            )
            self.pub_bot_2.publish(self.msg_bot)

        ######## NEXT GOAL LOGIC ##########
            self.bot2_goal_index += 10

            if self.bot2_goal_index>=len(bot2_goal[self.contour_list_2]):
                self.contour_list_2+=1

                if(self.contour_list_2>=len(bot2_goal)):
                    self.contour_list_2-=1
                    self.bot2_goal_index-=10 
                    self.pen_down.data = False
                    self.pen2.publish(self.pen_down)
                    self.pub_bot_2.publish(self.msg_bot)
                     #status set to 1 indicating the end of goal points
                    self.bot2_status=1
                    #Calling stop_flag service after goal completion of all three bots
                    if(self.bot1_status*self.bot2_status*self.bot3_status==1):
                        self.cli.call_async(self.req)
                else:
                    self.bot2_goal_index=0
                      

    """
    * Function Name: callbackBot3
    * Input: msg (geometry_msgs.msg.Pose2D)
    * Output: None
    * Logic: Callback function for /pen3_pose topic. Handles bot movement and pen control for Bot 3.
    * Example Call: Invoked automatically when a message is received on the /pen3_pose topic.
    """
    def callbackBot3(self, msg):

        ##### GOAL CALL ######
        x_goal, y_goal = self.getGoal(3)  

        ##### ERROR CALCULATION #####
        error_x = x_goal - msg.x
        error_y = -y_goal + msg.y
        error_theta = -msg.theta

        ##### PEN DOWN LOGIC ########
        if self.bot3_goal_index > 0 and self.bot3_goal_index<len(bot3_goal[self.contour_list_3]):
            self.pen_down.data = True
            self.pen3.publish(self.pen_down)
        else:
            self.pen_down.data = False
            self.pen3.publish(self.pen_down)

        ##### GO-TO GOAL LOGIC ######
        if self.distance(error_x, error_y) > 1:
            self.msg_bot.linear.x, self.msg_bot.linear.y, self.msg_bot.linear.z = (
                self.inverse_kinematics(error_x, error_y, error_theta)
            )
            self.pub_bot_3.publish(self.msg_bot)
        else:
        
            self.msg_bot.linear.x, self.msg_bot.linear.y, self.msg_bot.linear.z = (
                self.inverse_kinematics(0, 0, 0)
            )
            self.pub_bot_3.publish(self.msg_bot)

        ######## NEXT GOAL LOGIC ##########
            self.bot3_goal_index += 10

            if self.bot3_goal_index>=len(bot3_goal[self.contour_list_3]):
                self.contour_list_3+=1
                if(self.contour_list_3>=len(bot3_goal)):
                    self.contour_list_3-=1
                    self.bot3_goal_index -= 10
                    self.pen_down.data = False
                    self.pen3.publish(self.pen_down)
                    self.pub_bot_3.publish(self.msg_bot)
                    #status set to 1 indicating the end of goal points
                    self.bot3_status=1
                    #Calling stop_flag service after goal completion of all three bots
                    if(self.bot1_status*self.bot2_status*self.bot3_status==1):
                        self.cli.call_async(self.req)
                else:
                    self.bot3_goal_index=0

    """
    * Function Name: pi_controller
    * Input: error (float), integral (float)
    * Output: control_signal (float), integral (float)
    * Logic: Implements a PI controller algorithm.
    * Example Call: control_signal, integral = self.pi_controller(error, integral)
    """
    def pi_controller(self, error, integral):

        # Proportional term
        proportional = self.kp_straight * error

        # Integral term
        integral += error
        integral_term = self.ki * integral

        # Calculate the control signal
        control_signal = proportional + integral_term

        return control_signal, integral

    """
    * Function Name: inverse_kinematics
    * Input: error_x (float), error_y (float), error_theta (float)
    * Output: force_left (float), force_right (float), force_rear (float)
    * Logic: Calculates the control signals for the left, right, and rear wheels using inverse kinematics.
    * Example Call: force_left, force_right, force_rear = self.inverse_kinematics(error_x, error_y, error_theta)
    """
    def inverse_kinematics(self, error_x, error_y, error_theta):
        # PI controller for left wheel
        force_left, self.integral_left = self.pi_controller(
            -error_x * 1 / 3 - error_y * 0.57735 + error_theta * 1 / 3 * self.kp_theta,
            self.integral_left,
        )

        # PI controller for right wheel
        force_right, self.integral_right = self.pi_controller(
            -error_x * 1 / 3 + error_y * 0.57735 + error_theta * 1 / 3 * self.kp_theta,
            self.integral_right,
        )

        # PI controller for rear wheel
        force_rear, self.integral_rear = self.pi_controller(
            error_x * 2 / 3 + error_theta * 1 / 3 * self.kp_theta, self.integral_rear
        )

        # Limiting the maximum force to prevent exceeding a certain threshold.
        max_force = 60
        if abs(force_right) > max_force or abs(force_left) > max_force or abs(force_rear) > max_force:
            if abs(force_left) >= abs(force_right) and abs(force_left) >= abs(force_rear):
                force_right = (max_force * force_right) / abs(force_left)
                force_rear = (max_force * force_rear) / abs(force_left)
                force_left = (max_force * force_left) / abs(force_left)
            elif abs(force_right) >= abs(force_left) and abs(force_right) >= abs(force_rear):
                force_left = (max_force * force_left) / abs(force_right)
                force_rear = (max_force * force_rear) / abs(force_right)
                force_right = (max_force * force_right) / abs(force_right)
            else:
                force_left = (max_force * force_left) / abs(force_rear)
                force_right = (max_force * force_right) / abs(force_rear)
                force_rear = (max_force * force_rear) / abs(force_rear)

        return float(-force_left), float(-force_right), float(-force_rear)

    """
    * Function Name: getGoal
    * Input: index (int)
    * Output: x (int), y (int)
    * Logic: it calls the next goal for the bot from previosly saved countour list
    * Example Call: x, y = self.getGoal(theta)
    """
    def getGoal(self, index):
        if(index == 1):
            x=bot1_goal[self.contour_list_1][self.bot1_goal_index][0][0]
            y=bot1_goal[self.contour_list_1][self.bot1_goal_index][0][1]
        if(index == 2):
            x=bot2_goal[self.contour_list_2][self.bot2_goal_index][0][0]
            y=bot2_goal[self.contour_list_2][self.bot2_goal_index][0][1]
        if(index == 3):
            x=bot3_goal[self.contour_list_3][self.bot3_goal_index][0][0]
            y=bot3_goal[self.contour_list_3][self.bot3_goal_index][0][1]
        return x, y
    
    """
    * Function Name: distance
    * Input: x_error (float), y_error (float)
    * Output: distance (float)
    * Logic: Calculates the Euclidean distance between two points.
    * Example Call: dist = self.distance(x_error, y_error)
    """
    def distance(self, x_error, y_error):
        return math.sqrt(y_error**2 + x_error**2)
    

"""
* Function Name: main
* Input: args
* Output: None
* Logic: Initializes the Bot_Control node and spins it until shutdown.
* Example Call: main()
"""
def main(args=None):
    rclpy.init(args=args)
    bot_Control = Bot_Control()
    rclpy.spin(bot_Control)
    bot_Control.destroy_node()
    rclpy.shutdown()


# Entry point of the script
if __name__ == "__main__":
    main()
