from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess

from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params = PathJoinSubstitution(
        [
            FindPackageShare("turtlesim_control"),
            "params.yaml",
        ]
    )
    return LaunchDescription([

        Node(
            package='turtlesim_control',
            executable='turtlesim_sender_node',
            #namespace='turtle',
            name='sender_ns',
            parameters=[params]
        ),
        Node(
            package='turtlesim_control',
            executable='turtlesim_line_node',
            #namespace='turtle',
            name='control_ns',
           parameters=[params]
        ),
    ])
