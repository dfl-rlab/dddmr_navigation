#!/bin/bash

IMAGE_NAME="dddmr:l4t_r39"

# Check if image exists
if ! docker image inspect $IMAGE_NAME > /dev/null 2>&1; then
    echo "Docker image $IMAGE_NAME not found."
    echo "Please build the image first using build.bash (choose l4t)."
    exit 1
fi

echo "Running CUDA test on image: $IMAGE_NAME"

# Get the directory of this script to properly mount it
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Run the python script inside a temporary container
# Note: No -it flag is used, making it suitable for GitHub Actions
docker run --rm \
    --network=host \
    --runtime=nvidia \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --volume="/dev:/dev" \
    --volume="/tmp:/tmp" \
    --volume="${SCRIPT_DIR}:/workspace" \
    $IMAGE_NAME \
    python3 /workspace/test_cuda.py