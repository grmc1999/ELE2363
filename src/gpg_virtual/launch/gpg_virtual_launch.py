from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess

from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf= PathJoinSubstitution(
        [
            FindPackageShare("gpg_virtual"),
            "gpg.urdf.xml",
        ]
    )
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            #namespace='turtle',
            name='sender_ns',
            #parameters=["/home/lci/ELE2363/src/turtlesim_control/config/params.yaml"],
            arguments=[urdf]
        ),
    ])
