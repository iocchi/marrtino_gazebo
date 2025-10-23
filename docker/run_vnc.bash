#!/bin/bash

DOCKER_RUNTIME="runc"

DOCKER_RUNTIME=${DOCKER_RUNTIME} docker compose -f ./dc_vnc.yml up -d --remove-orphans && \
sleep 2 && \
docker exec -it marrtino2_gz tmux a


