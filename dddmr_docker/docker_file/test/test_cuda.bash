#!/bin/bash

# Test CUDA inside the dddmr:humble-cuda docker image

IMAGE_NAME="dddmr:humble-cuda"

# Check if image exists
if ! docker image inspect $IMAGE_NAME > /dev/null 2>&1; then
    echo "Docker image $IMAGE_NAME not found."
    echo "Please build the image first using build.bash (choose x64 and Y for CUDA)."
    exit 1
fi

echo "Running CUDA test on image: $IMAGE_NAME"

# Get the directory of this script to properly mount it
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Run the python script inside a temporary container
# Note: No -it flag is used, making it suitable for GitHub Actions
docker run --rm \
    --network=host \
    --gpus=all \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --volume="${SCRIPT_DIR}:/workspace" \
    $IMAGE_NAME \
    python3 /workspace/test_cuda.py
