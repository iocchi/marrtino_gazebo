#!/bin/bash

if nvidia-detector 2> /dev/null; then
  DOCKER_RUNTIME="nvidia"
  echo "Using nvidia runtime !!!"
else
  DOCKER_RUNTIME="runc"
fi


DOCKER_RUNTIME=${DOCKER_RUNTIME} docker compose -f ./docker-compose.yml up -d --force-recreate && \
sleep 1 && \
docker exec -it marrtino2_gazebo tmux a


