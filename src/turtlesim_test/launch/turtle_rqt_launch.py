import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            namespace='/my_turtle',package="turtlesim",executable="turtlesim_node",output="screen",name='turtle_x',parameters=["/home/ros2_ws/src/turtlesim_test/config/params.yaml"]
            ),
        #launch_ros.actions.Node(
        #    namespace='/rqt',package="rqt_plot",executable="rqt_plot",output="screen",ros_arguments=["/my_turtle/turtle1/pose/x"]
        #    )
        ExecuteProcess(
        cmd=[[
            'ros2 run rqt_plot rqt_plot /my_turtle/turtle1/pose/x:y:theta'
        ]],
        shell=True
    ),
        ExecuteProcess(
        cmd=[[
            '/home/ros2_ws/src/turtlesim_test/scripts/setspeed.sh'
        ]],
        shell=True
    ),
        ExecuteProcess(
        cmd=[[
            '/home/ros2_ws/src/turtlesim_test/scripts/parse_velocity.sh'
        ]],
        shell=True
    )
    ])