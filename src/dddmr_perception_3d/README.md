# dddmr_perception_3d
Perception 3D is graph-based framework allowing user to develope applications for mobile robots, such as path planning, marking/clearing obstacles, creating no-enter/speed limit layer.
You can reference:
- [dddmr_global_planner](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_global_planner)
- [dddmr_local_planner](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_local_planner)
<table>
  <tr width="100%">
    <td width="33%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/perception_3d/perception_3d_global_plan.gif"/>Global planning in 3D map</td>
    <td width="33%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/perception_3d/marking_tracking_clearing.gif"/>Marking/Tracking/Clearing</td>
    <td width="33%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/perception_3d/speed_limit_zone.png"/>Speed-limit/no-enter zone</td>
  </tr>
</table> 

Perception 3D:
- Sensor support:
  - [x] Multilayer spinning lidar (Velodyne/Ouster/Leishen)
  - [x] Depth camera (Realsense/oak)
  - [x] Scanning Lidar (Livox mid-360/Unitree 4D LiDAR L1)
- Zone feature support:
  - [x] Static layer
  - [x] Speed limit layer
  - [x] No enter layer

## Obstacles Marking and Clearing - Multilayer Lidar (Leishen Lidar C16)

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/perception_3d/multilayer_lidar_demo.gif" width="640" height="400"/>
</p>

<details><summary> <b>Click me to see tutorial</b> </summary>
  
### 1. Create docker image
The package runs in the docker, so we need to build the image first. We support both x64 (tested in intel NUC) and arm64 (tested in nvidia jetson jpack5.1.3/6).
```
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```
### 2. Download essential files
ROS2 bag that contains multilayer lidar from Leishen C16 will be download to run the demo.
```
cd ~/dddmr_navigation/src/dddmr_perception_3d && ./download_files.bash
```
### 3. Run demo
#### Create a docker container
> [!NOTE]
> The following command will create an interactive docker container using the image we built. We will launch the demo manually in the container.
```
cd ~/dddmr_navigation/dddmr_docker && ./run_demo.bash
```
##### Launch everything in the container
The bag file will be auto-played after 3 seconds when launching.
```
cd ~/dddmr_navigation && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch perception_3d multilayer_spinning_lidar_3d_ros_launch.py
```
</details>

## Obstacles Marking and Clearing - Multiple Depth Cameras(Realsense D455)

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/perception_3d/multi_depth_camera_demo.gif" width="640" height="400"/>
</p>

<details><summary> <b>Click me to see tutorial</b> </summary>
  
### 1. Create docker image
The package runs in the docker, so we need to build the image first. We support both x64 (tested in intel NUC) and arm64 (tested in nvidia jetson jpack5.1.3/6).
```
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```
### 2. Download essential files
ROS2 bag that contains depth images from two cameras will be download to run the demo.
```
cd ~/dddmr_navigation/src/dddmr_perception_3d && ./download_files.bash
```
### 3. Run demo
#### Create a docker container
> [!NOTE]
> The following command will create an interactive docker container using the image we built. We will launch the demo manually in the container.
```
cd ~/dddmr_navigation/dddmr_docker && ./run_demo.bash
```
##### Launch everything in the container
The bag file will be auto-played after 3 seconds when launching.
```
cd ~/dddmr_navigation && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch perception_3d multi_depth_camera_3d_ros_launch.py
```
</details>

## Obstacles Marking and Clearing - Scanning Lidar (Unitree G4)

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/perception_3d/scanning_lidar_demo.gif" width="640" height="400"/>
</p>

<details><summary> <b>Click me to see tutorial</b> </summary>
  
### 1. Create docker image
The package runs in the docker, so we need to build the image first. We support both x64 (tested in intel NUC) and arm64 (tested in nvidia jetson jpack5.1.3/6).
```
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```
### 2. Download essential files
ROS2 bag that contains depth images from two cameras will be download to run the demo.
```
cd ~/dddmr_navigation/src/dddmr_perception_3d && ./download_files.bash
```
### 3. Run demo
#### Create a docker container
> [!NOTE]
> The following command will create an interactive docker container using the image we built. We will launch the demo manually in the container.
```
cd ~/dddmr_navigation/dddmr_docker && ./run_demo.bash
```
##### Launch everything in the container
The bag file will be auto-played after 3 seconds when launching.
```
cd ~/dddmr_navigation && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch perception_3d scanning_lidar_3d_ros_launch.py
```
</details>

## Speed Zone and No-Enter Zone Creation
Using our zone editor utils to create speed zone or no-enter zone pointcloud.
The perception_3d plugins will load the point clouds and make them as the speed layer or the no-enter layer.

Check out the configure: [speed_layer_configuration](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_perception_3d/config/speed_limit_layer.yaml)

Also you can play around the utils:
```
ros2 launch perception_3d zone_editor_utils.launch 
```
[![YouTube video thumbnail](https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/perception_3d/point_cloud_editor.png)](https://youtu.be/DHgzRD4HrjU)

## Delete Point Cloud in .pcd file
Using Point Cloud Deleting Tool to delete the point cloud you don't want in .pcd file

Launch point cloud deleting using your .pcd file: [pcd file dir](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_perception_3d/launch/pc_delete_utils.launch)

Also you can play around the utils:
```
ros2 launch perception_3d pc_delete_utils.launch
```
[![YouTube video thumbnail](https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/perception_3d/point_cloud_delete.png)](https://www.youtube.com/watch?v=kmR9HWOe8NM)
