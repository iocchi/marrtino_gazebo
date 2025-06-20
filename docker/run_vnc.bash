#!/bin/bash

DOCKER_RUNTIME="runc"

DOCKER_RUNTIME=${DOCKER_RUNTIME} docker compose -f ./dc_vnc.yml up -d --force-recreate && \
sleep 1 && \
docker exec -it marrtino2_gazebo tmux a


