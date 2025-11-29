#!/bin/bash

UPAR="--build-arg UID=`id -u` --build-arg GID=`id -g`"

cd ..

docker build -t marrtino2:gazebo -f docker/Dockerfile.upgrade . && \
docker build $UPAR -t marrtino2:user -f docker/Dockerfile.user . 


