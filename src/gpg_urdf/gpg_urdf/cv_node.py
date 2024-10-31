import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Float64MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import image_geometry
from cv_bridge import CvBridge
import cv2
import numpy as np



class cv_node(Node):

    def __init__(self):
        super().__init__('cv_node')
        self.declare_parameter('lth',50.)
        self.declare_parameter('hth',180.)
        self.declare_parameter('C_k',0.01)
        self.cv_bridge=CvBridge()
        self.cv_subscription=self.create_subscription(Image,"/image",self.callback,10)
        self.ci_subscription=self.create_subscription(CameraInfo,"/camera_info",self.callback_camera_info,10)
        self.camera_publisher = self.create_publisher(
            Float64MultiArray,
            '/servo_controller/commands',10)
        #self.publisher_ = self.create_publisher(Image, '/processed_image', 10)
        self.model=image_geometry.PinholeCameraModel()

        self.servo_pos=0
        
    def dumb_image_process_function(self,rgb):
        # code for pixel definition
        hsv=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV)
        lower_blue = np.array([self.lth,50,50])
        upper_blue = np.array([self.hth,255,255])
        print(self.hth)
        print(self.lth)
        #image=(self.lth<image)*(self.hth>image
        image = cv2.inRange(hsv, lower_blue, upper_blue)
        #cv2.imshow("camera", image)   

        v=np.argmax(np.sum(image,axis=0))
        u=np.argmax(np.sum(image,axis=1))
        print("u,v")
        print(u,v)
        cv2.imshow("filter",image)
        cv2.imshow("aim",cv2.circle(rgb,(u,v),radius=10,color=(0,0,255),thickness=2))
        cv2.waitKey(1)
        return u,v
    
    def image_process_function(self,rgb):
        # code for pixel definition
        hsv=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV)
        lower_blue = np.array([self.lth,50,50])
        upper_blue = np.array([self.hth,255,255])
        #image=(self.lth<image)*(self.hth>image
        image = cv2.inRange(hsv, lower_blue, upper_blue)
        #cv2.imshow("camera", image)   

        M=cv2.moments(image)
        if M['m00']==0:
            u,v=(int(320/2),int(240/2))
        else:
            u=int(M['m10']/M['m00'])
            v=int(M['m01']/M['m00'])
        cv2.imshow("filter",image)
        cv2.imshow("aim",cv2.circle(rgb,(u,v),radius=10,color=(0,0,255),thickness=2))
        cv2.waitKey(1)
        return u,v
    
    def callback(self,msg):
        self.lth=float(self.get_parameter('lth').get_parameter_value().double_value)
        self.hth=float(self.get_parameter('hth').get_parameter_value().double_value)
        self.C_k=float(self.get_parameter('C_k').get_parameter_value().double_value)
        cv_image=self.cv_bridge.imgmsg_to_cv2(msg)
        
        

        # image processing
        (u,v)=self.image_process_function(cv_image)

        try:
            line=self.model.projectPixelTo3dRay((u,v))
        except:
            line=None
            print("some error defining line")
            return

        # control servo
        #Minimize 2
        #if self.servo_pos<1.5 and self.servo_pos>-1.5:
        self.servo_pos=self.servo_pos-line[0]*self.C_k
        FP=Float64MultiArray()
        FP.data=[self.servo_pos]
        self.camera_publisher.publish(FP)

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