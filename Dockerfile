FROM ros:jazzy-ros-base-noble

RUN export DEBIAN_FRONTEND="noninteractive"
RUN apt-get update -y
RUN apt-get install ros-${ROS_DISTRO}-joint-state-publisher-gui -y
RUN apt-get install ros-${ROS_DISTRO}-hardware-interface -y
RUN apt-get install ros-${ROS_DISTRO}-controller-interface -y
RUN apt-get install ros-${ROS_DISTRO}-controller-manager -y
RUN apt-get update -y
RUN apt-get install ros-${ROS_DISTRO}-ros-gz -y
RUN apt-get install ros-${ROS_DISTRO}-gz-ros2-control -y
RUN apt-get install ros-rolling-ros2-control -y
RUN apt-get install ros-rolling-ros2-controllers -y
RUN apt-get install ros-${ROS_DISTRO}-turtlesim -y
RUN apt-get install ros-${ROS_DISTRO}-tf-transformations -y



# gz_ros2_control
# ros-rolling-ros2-control
# ros-rolling-ros2-controllers
# controlller
#apt-get install ros-jazzy-turtlesim
#sudo apt-get install ros-<ros_distro>-tf-transformations

#RUN mkdir -p ~/gz_ros2_control_ws/src && cd ~/gz_ros2_control_ws/src && git clone https://github.com/ros-controls/gz_ros2_control -b {ROS_DISTRO} && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y && cd ~/gz_ros2_control_ws && colcon build
#RUN source ~/ros2_ws/gz_ros2_control/install/setup.bash

