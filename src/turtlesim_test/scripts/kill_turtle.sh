#! /bin/bash

#TOKILL=$(ros2 topic list | grep cmd_vel | sed "s/\/cmd_vel$//")
TOKILL=$(ros2 topic list | grep cmd_vel | cut -d '/' -f3 )

ros2 service call /my_turtle/kill turtlesim/srv/Kill "{name: "$TOKILL"}"

#ros2 launch turtlesim_test turtle_rqt_launch.py