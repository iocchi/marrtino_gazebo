#!/bin/bash

cd ~/ros2_ws
colcon build && ros2 launch marrtino_gazebo marrtino.launch.py robot_name:=smarrtino world_file:=empty.world control_interface:=position
