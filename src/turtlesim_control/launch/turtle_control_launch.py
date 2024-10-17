from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
#        Node(
#            package='turtlesim',
#            #namespace='turtlesim1',
#            executable='turtlesim_node',
#            name='sim',
#            #parameters=["/home/ros2_ws/src/turtlesim_test/config/params.yaml"]
#        ),
    #    ExecuteProcess(
    #    cmd=[[
    #        'ros2 topic pub --rate 12 /turtle1/goal turtlesim/msg/Pose "{x: 7.0,y: 7.0,theta: 1.0}"'
    #    ]],
    #    shell=True
    #),

        Node(
            package='turtlesim_control',
            executable='turtlesim_sender_node',
            #namespace='turtle',
            name='sender_ns',
            parameters=["/home/Documents/ros2_ws/ELE2363/src/turtlesim_control/config/params.yaml"]
        ),
        Node(
            package='turtlesim_control',
            executable='turtlesim_control_node',
            #namespace='turtle',
            name='control_ns',
           parameters=["/home/Documents/ros2_ws/ELE2363/src/turtlesim_control/config/params.yaml"]
        ),
    ])
