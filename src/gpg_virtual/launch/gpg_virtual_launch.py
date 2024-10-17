from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess

def generate_launch_description():
    urdf="/home/lci/Documents/ros2_ws/ELE2363/src/gpg.urdf.xml"
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
        #node for state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            #namespace='turtle',
            name='sender_ns',
            #parameters=["/home/lci/ELE2363/src/turtlesim_control/config/params.yaml"],
            arguments=[urdf]
        ),
        #node for control
        #Node(
        #    package='gpg_virtual',
        #    executable='state_publisher',
        #    #namespace='turtle',
        #    name='state_publisher',
        #   #parameters=["/home/lci/ELE2363/src/turtlesim_control/config/params.yaml"]
        #),
    ])
