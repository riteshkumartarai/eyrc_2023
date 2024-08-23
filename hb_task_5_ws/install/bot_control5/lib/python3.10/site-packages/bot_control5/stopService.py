# stop_flag_server.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class StopFlagServer(Node):

    def __init__(self):
        super().__init__('stop_flag_server')
        self.pen1=self.create_publisher(Bool,'/pen1_down',1)
        self.pen2=self.create_publisher(Bool,'/pen2_down',1)
        self.pen3=self.create_publisher(Bool,'/pen3_down',1)
        self.pub_bot_1 = self.create_publisher(Twist, "/cmd_vel/bot1", 1)
        self.pub_bot_2 = self.create_publisher(Twist, "/cmd_vel/bot2", 1)
        self.pub_bot_3 = self.create_publisher(Twist, "/cmd_vel/bot3", 1)
        
        self.timer = self.create_timer(.01, self.stop_flag_callback)
    def stop_flag_callback(self):
        pen=Bool()
        msg=Twist()
        pen.data=False
        msg.linear.x=0.0
        msg.linear.y=0.0
        msg.linear.z=0.0
        self.pub_bot_1.publish(msg)
        self.pub_bot_2.publish(msg)
        self.pub_bot_3.publish(msg)
        self.pen1.publish(pen)
        self.pen2.publish(pen)
        self.pen3.publish(pen)

def main(args=None):
    rclpy.init(args=args)
    server = StopFlagServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
