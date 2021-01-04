#!/bin/bash

WORKSPACE_DIR=$(cd .. && pwd)
DOCKER_IMAGE=single-robot/ev3rt-v850:v1.0.0

sudo docker run -v ${WORKSPACE_DIR}:/root/workspace/ev3rt-athrill-v850e2m -it --rm --net host --name ev3rt-v850 ${DOCKER_IMAGE} 