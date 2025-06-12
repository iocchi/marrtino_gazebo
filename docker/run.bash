#!/bin/bash

DOCKER_RUNTIME="runc"

if nvidia-detector 2> /dev/null; then
  NVIDIA_DETECT=`nvidia-detector`
  if [ "$NVIDIA_DETECT" != "None"  ]; then
    DOCKER_RUNTIME="nvidia"
    echo "Nvidia detect: ${NVIDIA_DETECT} Using nvidia runtime !!!"
  fi
fi


DOCKER_RUNTIME=${DOCKER_RUNTIME} docker compose -f ./docker-compose.yml up -d --force-recreate && \
sleep 1 && \
docker exec -it marrtino2_gazebo tmux a


