#!/bin/bash

xhost +local:docker

docker run -it \
    --privileged \
    --network=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp:/tmp" \
    --volume="/dev:/dev" \
    --volume="${HOME}/dddmr:/root/dddmr" \
    --volume="${HOME}/dddmr_bags:/root/dddmr_bags" \
    --name="dddmr_x64" \
    dddmr:x64
