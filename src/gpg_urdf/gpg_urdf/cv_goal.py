import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Float64MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import image_geometry
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist,Pose2D,TwistStamped,PointStamped
import tf2_ros
import tf2_geometry_msgs



class cv_node(Node):

    def __init__(self):
        super().__init__('cv_node')
        self.declare_parameter('lth',50.)
        self.declare_parameter('hth',180.)
        self.declare_parameter('C_k',0.01)
        self.declare_parameter('camera_height',0.3)
        self.cv_bridge=CvBridge()
        self.cv_subscription=self.create_subscription(Image,"/image",self.callback,10)
        self.ci_subscription=self.create_subscription(CameraInfo,"/camera_info",self.callback_camera_info,10)
        self.camera_publisher = self.create_publisher(Float64MultiArray,'/servo_controller/commands',10)
        self.goal_publisher = self.create_publisher(PointStamped,'local/goal',10)
        #self.publisher_ = self.create_publisher(Image, '/processed_image', 10)
        self.model=image_geometry.PinholeCameraModel()

        self.servo_pos=0

        self.buffer=tf2_ros.Buffer()
        self.tf_listener=tf2_ros.TransformListener(self.buffer, self)
        
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
    
    def find_contact(self):
        self.camera_height=float(self.get_parameter('camera_height').get_parameter_value().double_value)
        alpha=-self.camera_height/self.line[1]
        if np.abs(alpha)>10000:
            z=None
            x=None
        else:
            z=alpha*self.line[2]
            x=alpha*self.line[0]
        return x,z
    
    def callback(self,msg):
        self.lth=float(self.get_parameter('lth').get_parameter_value().double_value)
        self.hth=float(self.get_parameter('hth').get_parameter_value().double_value)
        self.C_k=float(self.get_parameter('C_k').get_parameter_value().double_value)
        cv_image=self.cv_bridge.imgmsg_to_cv2(msg)
        
        

        # image processing
        (u,v)=self.image_process_function(cv_image)

        try:
            self.line=self.model.projectPixelTo3dRay((u,v))
        except:
            self.line=None
            print("some error defining line")
            return
        self.servo_pos=self.servo_pos-self.line[0]*self.C_k
        FP=Float64MultiArray()
        FP.data=[self.servo_pos]
        self.camera_publisher.publish(FP)

        x,z=self.find_contact()

        if x!=None:
            self.rp=PointStamped()

            self.rp.header.frame_id='camera_link'
            #self.rp.header.frame_id='base_link'
            self.rp.point.x=x
            self.rp.point.z=z
            self.rp.point.y=0.

            self.rp=self.buffer.transform(self.rp,"odom")


            self.goal_publisher.publish(self.rp)
        else:
            print("no object detected")
        # Transform point stamped to base
        #self.rp.theta=np.random.uniform(-np.pi,np.pi)
        
    
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