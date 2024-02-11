# stop_flag_server.py
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class StopFlagServer(Node):

    def __init__(self):
        super().__init__('stop_flag_server')
        self.srv = self.create_service(Empty, 'Stop_Flag', self.stop_flag_callback)

    def stop_flag_callback(self, request, response):
        
        self.get_logger().info('Stop flag service received')
        response=Empty()
        return response

def main(args=None):
    rclpy.init(args=args)
    server = StopFlagServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
