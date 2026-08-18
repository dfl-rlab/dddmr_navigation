#!/bin/bash

function build_x64(){
    docker build --network host -t dddmr:humble -f Dockerfile_x64 .
}

#-----select image
echo -n "Select image type (x64/l4t/x64_gz): "
read image_type

if [[ $image_type == "x64" ]]; then
    echo -n "Do you want to build image using cuda? (Y/N): "
    read is_cuda
    if [ "$is_cuda" != "${is_cuda#[Yy]}" ] ;then
        echo "----> Creating x64 image with cuda, the x64 image will be created first"
        build_x64
        echo "----> Starting second layer with CUDA"
        docker build --network host -t dddmr:humble-cuda -f Dockerfile_x64_cuda .
    else
        echo "----> Creating x64 image without cuda"
        build_x64
    fi

elif [[ $image_type == "l4t" ]]; then
    echo "----> Creating l4t image"
    docker build --network host -t dddmr:l4t_r36 -f Dockerfile_x64_l4t_r36 .

elif [[ $image_type == "x64_gz" ]]; then
    echo "----> Creating x64 image with cuda, the x64 image will be created first"
    build_x64
    echo "----> Starting second layer with gz"
    docker build --network host -t dddmr_gz:humble -f Dockerfile_humble_gazebo . --no-cache

else
    echo "Invalid image type. Please choose x64/l4t/gz"
fi

