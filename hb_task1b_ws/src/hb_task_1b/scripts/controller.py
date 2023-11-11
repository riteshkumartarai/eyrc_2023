#!/usr/bin/env python3
########################################################################################################################
########################################## eYRC 23-24 Hologlyph Bots Task 1A ###########################################
# Team ID:1796
# Team Leader Name:ADARSH PRIYARANJAN
# Team Members Name:RITESH KUMAR TARAI,SRIJONI GHOSH,SOUMITRA NAIK
# College:NATIONAL INSTITUTE OF TECHNOLOGY ROURKELA
########################################################################################################################


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal


class HBTask1BController(Node):

    def __init__(self):
        super().__init__('hb_task1b_controller')
        self.sub=self.create_subscription(Odometry,"/odom",self.odometryCb,10)
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        # Declare a Twist message
        self.vel = Twist()
        # For maintaining control loop rate.
        self.rate = self.create_rate(100)
        # self.msg=Odometry()
        # client for the "next_goal" service
        self.cli = self.create_client(NextGoal, 'next_goal')      
        self.req = NextGoal.Request() 
        self.index = 0
        #goal variable
        self.goalx = 0
        self.goaly = 0
        #goal lists
        self.xlist=[]
        self.ylist=[]
        #p_controllers
        self.kp=2
        self.kp_angular=25
        
    '''
    i/p:
    o/p:
    Explanation: 
    '''
    def odometryCb(self,msg):
        global hb_x,hb_y,hb_theta
        #position
        hb_x=msg.pose.pose.position.x
        hb_y=msg.pose.pose.position.y
        l=msg.pose.pose.orientation
        hb_theta=euler_from_quaternion([l.x,l.y,l.z,l.w])[2]#angular pose
        # print(f"{self.goalx}   {self.goaly}")
        self.rot()
        if(abs(self.dist_z())<0.1):
            self.goal(self.goalx,self.goaly)
        
            
        
    #movement    
    def goal(self,target_x,target_y):
        x=self.dist_x(target_x)*self.kp
        y=self.dist_y(target_y)*self.kp
        if(math.sqrt(x*x+y*y)>.2):
            self.vel.angular.z=0.0
            self.vel.linear.x=x
            self.vel.linear.y=y
            self.pub.publish(self.vel)
        elif(self.index<len(self.xlist)):
            # print("change")
            self.goalx=self.xlist[self.index]    
            self.goaly=self.ylist[self.index] 
            self.index+=1
        else:
            self.goalx=0    
            self.goaly=0
    #rotation
    def rot(self):
        self.vel.angular.z=-hb_theta*self.kp_angular
        self.vel.linear.x=0.0
        self.vel.linear.y=0.0 
        self.pub.publish(self.vel)      
    #errors        
    def dist_z(self):
        return -hb_theta            

    def dist_x(self,target_x):
        return -hb_x+target_x
    
    def dist_y(self,target_y):
        return -hb_y+target_y

    #service request
    def send_request(self, a):
        self.req.request_goal = a
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()



def main(args=None):
    rclpy.init(args=args)
    i=0
    # Create an instance of the EbotController class
    ebot_controller = HBTask1BController()
    while True:
            r=ebot_controller.send_request(i)
            #
            ebot_controller.xlist.append(r.x_goal)
            ebot_controller.ylist.append(r.y_goal)
            i+=1
            if(r.end_of_list==1):
                break

    rclpy.spin(ebot_controller)
    rclpy.shutdown()



if __name__ == '__main__':
    main()       