#!/bin/bash

docker pull ros:jazzy-ros-base-noble
docker pull iocchi/marrtino2:system
docker pull iocchi/marrtino2:gazebo
docker pull iocchi/marrtino2:user

docker tag iocchi/marrtino2:system marrtino2:system
docker tag iocchi/marrtino2:gazebo marrtino2:gazebo
docker tag iocchi/marrtino2:user marrtino2:user

