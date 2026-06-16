#!/bin/bash

xhost +local:docker

docker run -it \
    --privileged \
    --network=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="ROS_DOMAIN_ID=12" \
    --volume="/tmp:/tmp" \
    --volume="/dev:/dev" \
    --volume="${HOME}/dddmr_navigation:/root/dddmr_navigation" \
    --volume="${HOME}/dddmr_bags:/root/dddmr_bags" \
    --name="dddmr_zinger" \
    tsengapola/dddmr_simulator:zinger \
    bash -c "cd /root/ws_zinger && source install/setup.bash && ros2 launch zinger_bring_up zinger_bring_up.launch.py"
