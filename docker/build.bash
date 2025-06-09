#!/bin/bash

UPAR="--build-arg UID=`id -u` --build-arg GID=`id -g`"

docker build $UPAR -t marrtino2:system -f Dockerfile.system . && \
docker build $UPAR -t marrtino2:gazebo -f Dockerfile.gazebo .


