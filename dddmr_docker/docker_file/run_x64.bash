#!/bin/bash

xhost +local:docker

docker run -it \
    --privileged \
    --network=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp:/tmp" \
    --volume="/dev:/dev" \
    --volume="${HOME}/dddmr_navigation:/root/dddmr_navigation" \
    --volume="${HOME}/dddmr_bags:/root/dddmr_bags" \
    --name="dddmr_humble" \
    dddmr:humble
