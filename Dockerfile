FROM ros:jazzy-ros-base-noble

RUN export DEBIAN_FRONTEND="noninteractive"
RUN apt-get update -y
RUN apt-get install ros-${ROS_DISTRO}-joint-state-publisher-gui -y
RUN apt-get install ros-${ROS_DISTRO}-hardware-interface -y
RUN apt-get install ros-${ROS_DISTRO}-controller-interface -y
RUN apt-get install ros-${ROS_DISTRO}-controller-manager -y
RUN apt-get update -y
RUN apt-get install ros-${ROS_DISTRO}-ros-gz -y


RUN mkdir -p ~/gz_ros2_control_ws/src && cd ~/gz_ros2_control_ws/src && git clone https://github.com/ros-controls/gz_ros2_control -b {ROS_DISTRO} && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y && cd ~/gz_ros2_control_ws && colcon build
RUN source ~/ros2_ws/gz_ros2_control/install/setup.bash

