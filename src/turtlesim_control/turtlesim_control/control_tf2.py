import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist,Pose2D,TwistStamped
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import angles
import numpy as np
import math
import tf2_ros

import rospy


class Turtle_Controller(Node):

    def __init__(self):
        super().__init__('Turtle_Controller')
        #self.K_x=0.5
        self.declare_parameter('K_x',0.1)
        #self.K_y=0.5
        self.declare_parameter('K_y',0.1)
        #self.K_z=0.5
        self.declare_parameter('K_z',0.1)
#        self.current_pose=Pose()
        self.current_pose=Pose()

        self.goal_pose=None


        self.goal_subscription = self.create_subscription(
            Pose,'local/goal',
            self.set_goal,10)
        
        self.current_pose_subscription = self.create_subscription(
            Odometry,
	        'diff_drive_controller/odom',
            self.get_current_pose,10)
        
        self.velocity_publisher = self.create_publisher(
            TwistStamped,
		'diff_drive_controller/cmd_vel',
		10)

        self.buffer=tf2_ros.Buffer()

        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def set_goal(self,msg):
        # set mode for coupled orientation with distance
        #self.listener_callback(msg)
        #rospy.loginfo(rospy.get_caller_id() + 'I heard %f', msg)
        self.goal_pose=msg
    
    def get_current_pose(self,msg):
        self.current_pose_quaternion=msg
        self.current_pose=Pose()

        try:
            b_rw_T=self.buffer.lookup_transform('odom','base_link',rclpy.time.Time())
            print(b_rw_T)
        except:
            print("error")


    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def timer_callback(self):
        self.set_gains()
        v_u=TwistStamped()
        if self.goal_pose!=None:
            pd=self.pose_distance(self.current_pose,self.goal_pose)
            v_u.twist.linear.x,v_u.twist.angular.z=min(self.K_x*pd.x,0.1),self.K_z*pd.theta

        
            self.velocity_publisher.publish(v_u)

    def pose_distance(self,cp,gp):
        dp=Pose()
        dp.x,dp.y=gp.x-cp.x,gp.y-cp.y
        theta=math.atan2(dp.y,dp.x)
        dp.x=math.sqrt(dp.x**2+dp.y**2)
        dp.theta=angles.shortest_angular_distance(cp.theta,theta)
        return dp
    
    def set_gains(self):
        self.K_x=float(self.get_parameter('K_x').get_parameter_value().double_value)
        self.K_y=float(self.get_parameter('K_y').get_parameter_value().double_value)
        self.K_z=float(self.get_parameter('K_z').get_parameter_value().double_value)

def main(args=None):
    rclpy.init(args=args)

    TC = Turtle_Controller()

    rclpy.spin(TC)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    TC.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
