#!/bin/bash

# note: use GPU, window must be in foreground

x11vnc -id `xwininfo -d :0 -name "Gazebo Sim" | grep "Window id" |  awk '{print $4}'` -display :0 -autoport 5990 -nopw -forever &

websockify --web=/novnc 81 localhost:5990 &


