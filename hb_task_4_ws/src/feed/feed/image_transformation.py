import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import numpy as np

class camera_node(Node):
    def __init__(self):
        super().__init__("Image_transformation")
        self.publish_undistorted_image=self.create_publisher(Image,'/undistorted_image_raw',10)
        self.publish_final_image=self.create_publisher(Image,'/transformed_image_raw',10)
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10)
        self.bot_marker = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000) 
        self.params = cv2.aruco.DetectorParameters()
        self.cv_bridge = cv_bridge.CvBridge()
        self.camera_matrix = np.array([[555.62655,   0.     , 331.80275],
                        [0.     , 556.58168, 196.31032],
                        [0.     ,   0.     ,   1.]])

        self.distortion_coeffs = np.array([-0.462336, 0.103705, 0.014759, -0.016075, 0.000000])
        self.top_left=[0,0]
        self.top_right=[0,640]
        self.down_left=[480,0]
        self.down_right=[480,640]
    
    def image_callback(self,msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Undistort the image
            undistorted_image = cv2.undistort(cv_image, self.camera_matrix, self.distortion_coeffs)

            gray = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY )
            
            #Detect Aruco marker
            corners,ids,r=cv2.aruco.detectMarkers(gray,self.bot_marker,parameters=self.params)
            
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(undistorted_image, corners)
                for i in range(len(ids)):
                    if ids[i]==8:
                        self.top_left=corners[i][0][0]
                    if ids[i]==10:
                        self.top_right=corners[i][0][1]
                    if ids[i]==12:
                        self.down_right=corners[i][0][2]
                    if ids[i]==4:
                        self.down_left=corners[i][0][3]
            
        
                        
            
            # Define the source points (coordinates of a rectangle in the original image)
            source_points = np.float32([self.top_left, self.top_right, self.down_left, self.down_right])

            # Define the destination points (coordinates of the corresponding rectangle in the transformed image)
            destination_points = np.float32([[0, 0], [500, 0], [0, 500], [500,500]])

            # Calculate the perspective transformation matrix
            perspective_matrix = cv2.getPerspectiveTransform(source_points, destination_points)

            # Apply the perspective transformation to the image
            transformed_image = cv2.warpPerspective(undistorted_image, perspective_matrix, (500, 500))


   
            
            
        except Exception as e:
            self.get_logger().error('Error converting ROS Image to OpenCV image: %s' % str(e))
            return
        
        #Publishing modified images
        self.publish_undistorted_image.publish(self.cv_bridge.cv2_to_imgmsg(np.array(undistorted_image), encoding="bgr8"))
        self.publish_final_image.publish(self.cv_bridge.cv2_to_imgmsg(np.array(transformed_image), encoding="bgr8"))
        

def main(args=None):
    rclpy.init(args=args)

    output = camera_node()

    rclpy.spin(output)

    output.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()