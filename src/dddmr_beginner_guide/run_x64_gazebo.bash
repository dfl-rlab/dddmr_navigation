#!/bin/bash

xhost +local:docker

docker run -it \
    --privileged \
    --network=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp:/tmp" \
    --volume="/dev:/dev" \
    --name="dddmr_humble_gazebo" \
    dddmr_gz:humble
