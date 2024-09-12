#! /bin/bash
#ros2 topic pub --rate 0.5 /my_turtle/turtle1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.57}}' &&
while true; do
    SPEED=$(ros2 topic echo --once /my_turtle/speed --field data | tr  -d '\n---\-\-\-')
    echo $SPEED
    ros2 topic pub -r 5 --once /my_turtle/turtle1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: '$SPEED', y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0}}'
    ros2 topic pub -r 5 --once /my_turtle/turtle1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: '$SPEED', y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.57}}'
done