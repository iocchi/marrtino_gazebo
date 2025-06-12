#!/bin/bash

UPAR="--build-arg UID=`id -u` --build-arg GID=`id -g`"

cd ..

docker build $UPAR -t marrtino2:system -f docker/Dockerfile.system . && \
docker build -t marrtino2:gazebo -f docker/Dockerfile.gazebo .


