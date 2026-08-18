#!/bin/bash

xhost +local:docker

is_cuda=$(docker image ls dddmr | grep jazzy-cuda)
is_l4t_r39=$(docker image ls dddmr | grep l4t_r39)

if [ "$is_cuda" != "" ] ;then
    echo "Detect image of dddmr:jazzy-cuda"
    echo "Enter ROS_DOMAIN_ID you want for the container."
    read domain_id
    docker run -it \
        --privileged \
        --network=host \
        --gpus=all \
        --env="NVIDIA_VISIBLE_DEVICES=all"\
        --env="NVIDIA_DRIVER_CAPABILITIES=all"\
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="ROS_DOMAIN_ID=${domain_id}" \
        --volume="/tmp:/tmp" \
        --volume="/dev:/dev" \
        --volume="${HOME}/dddmr_bags:/root/dddmr_bags" \
        --volume="${HOME}/dddmr_navigation:/root/dddmr_navigation" \
        --name="dddmr_jazzy_cuda_dev" \
        dddmr:jazzy-cuda
elif [ "$is_l4t_r39" != "" ] ;then 
    echo "Detect image of dddmr:l4t_r39"
    echo "Enter ROS_DOMAIN_ID you want for the container."
    read domain_id
    docker run -it \
        --privileged \
        --network=host \
        --runtime=nvidia\
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="NVIDIA_VISIBLE_DEVICES=all"\
        --env="NVIDIA_DRIVER_CAPABILITIES=all"\
        --env="ROS_DOMAIN_ID=${domain_id}" \
        --volume="/dev:/dev" \
        --volume="/tmp:/tmp" \
        --volume="${HOME}/dddmr_bags:/root/dddmr_bags" \
        --volume="${HOME}/dddmr_navigation:/root/dddmr_navigation" \
        --name="dddmr_jazzy_l4t_dev" \
        dddmr:l4t_r39
fi
