#!/bin/bash

cd ~/ros2_ws
colcon build && ros2 launch marrtino_gazebo marrtino.launch.py robot_name:=smarrtino world_file:=empty.world control_interface:=position gz_args:="--gui-config src/marrtino_gazebo/config/gui2.config"
