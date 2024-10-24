import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import image_geometry
from cv_bridge import CvBridge
import cv2
import numpy as np



class cv_node(Node):

    def __init__(self):
        super().__init__('cv_node')
        self.declare_parameter('lth',0.1)
        self.declare_parameter('hth',0.01)
        self.cv_bridge=CvBridge()
        self.cv_subscription=self.create_subscription(Image,"/image",self.callback,10)
        self.ci_subscription=self.create_subscription(CameraInfo,"/camera_info",self.callback_camera_info,10)
        #self.publisher_ = self.create_publisher(Image, '/processed_image', 10)
        self.model=image_geometry.PinholeCameraModel()
        
    def image_process_function(self,image):
        # code for pixel definition
        image=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)[:,:,0]
        image=(self.lth<image)*(self.hth>image)
        v=np.mean(image,axis=0)
        u=np.mean(image,axis=1)
        return u,v
    
    def callback(self,msg):
        self.lth=float(self.get_parameter('lth').get_parameter_value().double_value)
        self.hth=float(self.get_parameter('hth').get_parameter_value().double_value)
        cv_image=self.cv_bridge.imgmsg_to_cv2(msg)
        
        

        # image processing
        (u,v)=self.image_process_function(cv_image)

        line=self.model.projectPixelTo3dRay((u,v))
        print(line)
        #msg=self.bridge.cv2_to_imgmsg(self.cv_image)
    
    def callback_camera_info(self,msg):
        self.model.fromCameraInfo(msg)
        



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = cv_node()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()