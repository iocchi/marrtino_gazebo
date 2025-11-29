#!/bin/bash

DOCKER_RUNTIME="runc"

if nvidia-detector 2> /dev/null; then
  NVIDIA_DETECT=`nvidia-detector`
  if [ "$NVIDIA_DETECT" != "None"  ]; then
    DOCKER_RUNTIME="nvidia"
    echo "Nvidia detect: ${NVIDIA_DETECT} Using nvidia runtime !!!"
  fi
fi

DC="dc_x11.yml"

if [ "$1" == "vnc" ]; then
  DC="dc_vnc.yml"
  DOCKER_RUNTIME="runc"
fi

if [ "$1" == "x11" ]; then
    DOCKER_RUNTIME="runc"
fi


DOCKER_RUNTIME=${DOCKER_RUNTIME} USER_ID=`id -u` docker compose -f $DC up -d --remove-orphans && \
sleep 2 && \
docker exec -it marrtino2_gz tmux a

DOCKER_RUNTIME=${DOCKER_RUNTIME} docker compose -f $DC rm -f

