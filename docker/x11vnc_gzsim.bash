#!/bin/bash

x11vnc -id `xwininfo -name "Gazebo Sim" | grep "Window id" |  awk '{print $4}'` -display :9 -autoport 5990 -nopw -forever &

websockify --web=/novnc 81 localhost:5990 &


