#! /bin/bash

#ros2 topic pub --rate 1 /my_turtle/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
ros2 topic pub --rate 5 /my_turtle/speed turtlesim_msgs/msg/Speed "{data: 2.0}"