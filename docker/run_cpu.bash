#!/bin/bash

DOCKER_RUNTIME="runc"

DOCKER_RUNTIME=${DOCKER_RUNTIME} USER_ID=`id -u` docker compose -f ./docker-compose.yml up -d --remove-orphans && \
sleep 2 && \
docker exec -it marrtino2_gz tmux a


