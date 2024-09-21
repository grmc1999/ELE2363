import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
import angles
import numpy as np
from tf_transformations import euler_from_quaternion


class Turtle_Goal_Sender(Node):

    def __init__(self):
        super().__init__('Turtle_Goal_Sender')
        self.x_max=2
        self.x_min=0
        self.y_min=0
        self.y_max=2
        self.linear_th=0.01
        self.declare_parameter('K_x_th',0.1)
        self.angular_th=0.001
        self.declare_parameter('K_z_th',0.1)
        self.generate_pose()
        self.goal_publisher = self.create_publisher(
            Pose,
            'local/goal',10)
        self.pose_subscriber = self.create_subscription(
            Odometry,
            'diff_drive_controller/odom',
            self.listener_callback,
            10)
        
        self.goal_publisher.publish(self.rp)

    def generate_pose(self):
        self.rp=Pose()
        self.rp.x=np.random.uniform(self.x_min,self.x_max)
        self.rp.y=np.random.uniform(self.y_min,self.y_max)
        self.rp.theta=np.random.uniform(-np.pi,np.pi)
    
    def listener_callback(self, msg):
        self.set_th()
        self.get_current_pose(msg)
        self.plain_distance=(
            (self.current_pose.x-self.rp.x)**2+\
            (self.current_pose.y-self.rp.y)**2)**0.5
        self.angular_distance=np.abs(
            angles.shortest_angular_distance(
                self.current_pose.theta,self.rp.theta))
        if (self.plain_distance < self.linear_th):
            self.generate_pose()
            self.goal_publisher.publish(self.rp)

    def get_current_pose(self,msg):
        #self.listener_callback(msg)
        #compute plain and angular distance
        self.current_pose_quaternion=msg
        self.current_pose=Pose()
	
        #self.plain_distance=(
        #    (self.current_pose_quaternion.pose.pose.position.x-self.goal_pose.x)**2+\
        #    (self.current_pose_quaternion.pose.pose.position.y-self.goal_pose.y)**2)**0.5
        self.euler_orientation=euler_from_quaternion([
		self.current_pose_quaternion.pose.pose.orientation.x,
		self.current_pose_quaternion.pose.pose.orientation.y,
		self.current_pose_quaternion.pose.pose.orientation.z,
		self.current_pose_quaternion.pose.pose.orientation.w,
        ])
        self.current_pose.x=self.current_pose_quaternion.pose.pose.position.x
        self.current_pose.y=self.current_pose_quaternion.pose.pose.position.y
        self.current_pose.theta=self.euler_orientation[2]


    def set_th(self):
        self.linear_th=float(self.get_parameter('K_x_th').get_parameter_value().double_value)
        self.angular_distance=float(self.get_parameter('K_z_th').get_parameter_value().double_value)
        #self.K_z=float(self.get_parameter('K_z').get_parameter_value().double_value)

def main(args=None):
    rclpy.init(args=args)

    GS = Turtle_Goal_Sender()

    rclpy.spin(GS)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    GS.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()