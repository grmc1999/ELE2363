#! /usr/bin/env python

from math import sin, cos, pi
import rospy
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion,TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster, TransformStamped,Buffer
#import untangle


class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)  # JointStates
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        

        self.robot_vel_subscriber = self.create_subscription(
            TwistStamped,
            'diff_drive_controller/cmd_vel',
            self.robot_vel_listener_callback,
            10)
        
        self.camera_vel_subscriber = self.create_subscription(
            Float32,
            'servo_controller/commands',
            self.camera_listener_callback,
            10)
        

        #for now manually reading
        #input_file = "/home/ros2_ws/src/gpg.urdf.xml" #Full path to your xml file
        #obj = untangle.parse(input_file)
        #print(obj)

        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        # robot state
        #tilt = 0.
        tinc = degree
        #swivel = 0.
        angle = 0.
        #height = 0.
        #hinc = 0.005
        base_link_to_left_wheel=0.
        base_link_to_right_wheel=0.
        base_link_to_camera=0.

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        joint_state = JointState()

        buffer=Buffer()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                #joint_state.name = ['swivel', 'tilt', 'periscope']
                #joint_state.position = [swivel, tilt, height]
                joint_state.name = [
                    "base_link_to_left_wheel",
                    "base_link_to_right_wheel",
                    "base_link_to_camera"
                    ]
                #joint_state.position = [
                #    base_link_to_left_wheel,
                #    base_link_to_right_wheel,
                #    base_link_to_camera
                #    ]

                # update transform
                # (moving in a circle with radius=2)
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = cos(angle)*2
                odom_trans.transform.translation.y = sin(angle)*2
                odom_trans.transform.translation.z = 0.1
                odom_trans.transform.rotation = \
                    euler_to_quaternion(0, 0, angle + pi/2) # roll,pitch,yaw

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(odom_trans)

                # Create new robot state
                #base_link_to_left_wheel += tinc
                #base_link_to_right_wheel += tinc
                #if base_link_to_left_wheel < -0.5 or base_link_to_left_wheel > 0.0:
                #    tinc *= -1

                #swivel += degree
                #angle += degree/4

                # This will adjust as needed per iteration
                b_lw_T=buffer.lookup_transform('base_link','left_wheel',rospy.time())
                print(b_lw_T)
                self.get_logger().info(b_lw_T)
                b_rw_T=buffer.lookup_transform('base_link','right_wheel',rospy.time())
                print(b_rw_T)
                self.get_logger().info(b_rw_T)
                
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass


    def robot_vel_listener_callback(self, msg):
        self.robot_vel=msg
    def camera_listener_callback(self,msg):
        self.camera_vel=msg

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = StatePublisher()


if __name__ == '__main__':
    main()
