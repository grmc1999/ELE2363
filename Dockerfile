FROM ros:jazzy-ros-base-noble

RUN export DEBIAN_FRONTEND="noninteractive"
RUN apt-get update -y
RUN apt-get install ros-${ROS_DISTRO}-joint-state-publisher-gui -y

RUN apt-get install ros-${ROS_DISTRO}-ros-gz -y

