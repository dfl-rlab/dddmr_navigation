#!/bin/bash

function build_x64(){
    docker build --network host -t dddmr:jazzy -f Dockerfile_x64 .
}

#-----select image
echo -n "Select image type (x64/l4t): "
read image_type

if [[ $image_type == "x64" ]]; then
    echo -n "Do you want to build image using cuda? (Y/N): "
    read is_cuda
    if [ "$is_cuda" != "${is_cuda#[Yy]}" ] ;then
        echo "----> Creating jazzy image with cuda, the jazzy image will be created first"
        build_x64
        echo "----> Starting second layer with CUDA"
        docker build --network host -t dddmr:jazzy-cuda -f Dockerfile_x64_cuda .
    else
        echo "----> Creating x64 image without cuda"
        build_x64
    fi

elif [[ $image_type == "l4t" ]]; then
    echo "----> Creating l4t image"
    
else
    echo "Invalid image type. Please choose x64/l4t/gz"
fi

