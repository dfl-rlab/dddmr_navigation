# DDDMR P2P Move Base

This repo is the finite state machine for [dddmr_navigation](https://github.com/dfl-rlab/dddmr_navigation) that can control a mobile robot to move from one pose to another pose in 3D space.

## Mobile Robot Kinematics

We now support variant mobile kinematics models, we are working on unifying all kinds of mobile robots to run on [dddmr_navigation](https://github.com/dfl-rlab/dddmr_navigation).

In dddmr navigation, we extensively use the simulation to develop/debug our stack and later deploy to real mobile robot to test. Thus we think by providing different simulated mobile robot types for users would faciliate the developing process.

[Different Drive]

[Omni Direction](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_p2p_move_base/kinematics_md/OMNIDIRECTION.md)

[Ackermann Steering]

[Tricycle]

[Articulated Vehicle]


## Run Basic Demo with a bag
### 1. Create docker image
The package runs in the docker, so we need to build the image first. We support both x64 (tested in intel NUC) and arm64 (tested in nvidia jetson jpack6).
```
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```
### 2. Run demo
#### Create a docker container
> [!NOTE]
> The following command will create an interactive docker container using the image we built. The we can launch the demo in the container.
```
cd ~/dddmr_navigation/dddmr_docker && ./run_demo.bash
```
#### Download essential files
Pose graph (3.3MB) and a bag file (1.2GB) will be download to run the demo.
```
cd ~/dddmr_navigation/src/dddmr_mcl_3dl && ./download_files.bash
```
#### Launch p2p move base
```
cd ~/dddmr_navigation && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch p2p_move_base p2p_move_base_localization.launch
```
#### Play bag file in the container
We need another terminal to play the bag file. Open another terminal and run following command to get into the container:
```
docker exec -it dddmr_humble_dev bash
```
Once you are in the container, run:
```
cd ~/dddmr_navigation && source install/setup.bash
cd ~/dddmr_bags && ros2 bag play benanli_detention_basin_localization
```
#### Use Plugin on Rviz2 - Demonstration Video
Use 3D Pose Estimate to provide initial pose and use 3D Goal Pose to provide a goal.

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/p2p_move_base/p2p_move_base_annotated.png" width="720" height="420"/>
</p>

[![YouTube video thumbnail](https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/p2p_move_base/p2p_move_base_video.png)](https://www.youtube.com/watch?v=7zyrRIE7eaU)
