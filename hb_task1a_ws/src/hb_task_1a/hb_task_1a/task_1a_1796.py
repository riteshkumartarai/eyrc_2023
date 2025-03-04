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
from turtlesim.msg import Pose
from turtlesim.srv import Spawn,SetPen


class snowman(Node):
    def __init__(self):
        super().__init__("turtle")
        self.pub=self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.pub2=self.create_publisher(Twist,"/turtle2/cmd_vel",10)
        self.msg=Twist()
        self.i=1
        self.j=1
        self.start=Pose()
        self.sub=self.create_subscription(Pose,"/turtle1/pose",self.callback,10)
        self.sub2=self.create_subscription(Pose,"/turtle2/pose",self.callback2,10)
        
        
 

    def callback(self,pos:Pose):
        if(self.j==1):
            self.start=pos
            self.j+=1
        
        if (round(pos.x,2)==round(self.start.x,2) or round(pos.y,2)==round(self.start.y,2)) and self.i==1:
            self.msg.linear.x=.5
            self.msg.angular.z=.5
            self.i+=1
            self.pub.publish(self.msg)
        elif (round(pos.x,2)!=round(self.start.x,2) or round(pos.y,2)!=round(self.start.y,2)):
            self.msg.linear.x=.5
            self.msg.angular.z=.5
            self.pub.publish(self.msg)   
        else:
            self.msg.linear.x=0.0
            self.msg.angular.z=0.0
            self.pub.publish(self.msg)
            if(self.i==2):
                self.spawn(self.start.x,self.start.y,0.0,"turtle2")
                self.i+=1

    def callback2(self,pos:Pose):
        self.call_set_pen_service(179,184,255,4,0)
        if (round(pos.x,2)==round(self.start.x,2) or round(pos.y,2)==round(self.start.y,2)) and self.i==3:
            self.msg.linear.x=1.0
            self.msg.angular.z=-.6
            self.i+=1
            self.pub2.publish(self.msg)
        elif (round(pos.x,1)!=round(self.start.x,1) or round(pos.y,1)!=round(self.start.y,1)) or self.i<10:
            self.msg.linear.x=1.0
            self.msg.angular.z=-.6
            self.pub2.publish(self.msg)
            self.i+=1   
        else:
            self.msg.linear.x=0.0
            self.msg.angular.z=0.0
            self.pub2.publish(self.msg)
            exit()
            



    def spawn(self,x,y,theta,name):
        client=self.create_client(Spawn,"/spawn")
        req=Spawn.Request()
        req.x=x
        req.y=y
        req.theta=theta
        req.name=name
        future=client.call_async(req)

    def call_set_pen_service(self, r, g, b, width, off):
        client = self.create_client(SetPen,"/turtle2/set_pen")
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        future = client.call_async(req)    



def main(args=None):
    rclpy.init(args=args)
    node=snowman()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=='__main__':
    main()
