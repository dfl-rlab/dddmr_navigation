# DDDMR LeGO LOAM BOR
This repo is based on the [LeGO-LOAM-BOR](https://github.com/facontidavide/LeGO-LOAM-BOR), and has been modified for ground vehicles. We did not fork the repo due to significant modifications, variant features and frameworks are implemented.

The original authors deserve all the credits, we just stand on the shoulders of giants.

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_lego_loam_bor/mapping.gif" width="700" height="420"/>
</p>

The following features are provided and are different from original version:

- ROS2 version (Humble) of LeGO LOAM BOR, support online and offline(bag) mapping, IMU/Odometry is optional.
- Interactive mapping. Users can pause and resume mapping during bag playing, and change parameters accordingly.
- Pose graph visualization.
- Conditional loop closure mechanism for a more stable mapping result.
- The result is saved as a pose graph, users can leverage our pose graph editor to modify the result ex: manual loop closure.
- Our pose graph editor allow users to merge two pose graphs.
- The pose graph can be used in DDDMR MCL localization.
- [Optimized ICP](https://github.com/zm0612/optimized_ICP) is leveraged for the loop closure.

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_lego_loam_bor/legoloam_weiwuyin.png" width="700" height="320"/>
</p>

## LeGO LOAM with a tilted lidar

SLAM with a tilted lidar is supported, the static tf broadcaster should be setup:
<table align='center'>
  <tr width="100%">
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_lego_loam_bor/lidar_tilt_10deg.png" width="420" height="260"/><p align='center'>Lidar tilts 10 degree and rotates 180 deg at the yaw direction</p></td>
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_lego_loam_bor/lidar_no_tilt.png" width="420" height="260"/><p align='center'>Lidar with no tilting and rotates 180 deg at the yaw direction</p></td>
  </tr>
</table>

If your baselink is not on the ground, you can setup a base_footprint to connect the base_link, and control the robot at base_footprint frame.
<table align='center'>
  <tr width="100%">
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_lego_loam_bor/lidar_link_footprint.png" width="420" height="260"/><p align='center'>Frames of lidar, base_link and base_footprint</p></td>
  </tr>
</table>

## Ground FOV Setup
The ground fov setup allow the algorithm to find the ground more effectively. Following are illustrations of ground_fov setup.

Example 1, installation is horizontal: 

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_lego_loam_bor/ground_fov.png" width="700" height="420"/>
</p>

Example 2, installation is 90 degree tilted:

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_lego_loam_bor/ground_fov_tilted.png" width="700" height="420"/>
</p>


## Cite *LeGO-LOAM*

Many thanks to *LeGO-LOAM* paper: 
```
@inproceedings{legoloam2018,
  title={LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain},
  author={Tixiao Shan and Brendan Englot},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4758-4765},
  year={2018},
  organization={IEEE}
}
```

<details><summary><h2>💡 Click me to see DDDMR LeGO-LOAM Demo</h2></summary>

### 1. Create docker image
> [!NOTE]
> The package runs in the docker, so we need to build the image first. We support both x64 with or without GPU and arm64 (tested in nvidia jetson jpack6.2).
> 
> Follow the instruction to build either x64 or l4t docker images. It will take some time depending on your harware


```
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```


### 2. Download bag files
To play SLAM, you will need to download bag file (4.0GB). To play pose graph editor, you will need to download pose graph folder (2.6MB).
```
cd ~/dddmr_navigation/src/dddmr_lego_loam && ./download_files.bash
```
### 3. Run demo
#### Create a docker container
> [!NOTE]
> The following command will create an interactive docker container using the image we built. The we can launch the demo in the container.
```
cd ~/dddmr_navigation/dddmr_docker && ./run_demo.bash
```
#### Play mapping using bag files in docker container
```
cd ~/dddmr_navigation && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch lego_loam_bor lego_loam_bag.launch
```
In the Rviz2, click resume to start mapping, and change the parameter accordingly during mapping.

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_lego_loam_bor/interactive_mapping_panel.png" width="700" height="420"/>
</p>

</details>


<details><summary><h2>💡 Click me to see Pose Graph Editor Demo</h2></summary>
    
# Pose Graph Editor Tutorial
#### Play pose graph editor in docker container
```
cd ~/dddmr_navigation && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch lego_loam_bor pose_graph_editor.launch
```
## Edit the First Pose Graph
### Open a pose graph folder
<p align="center">
<img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/pose_graph_editor/open_file.gif" width="700" height="320"/>
</p>

### Select first key frame

Press "shift" and select a pose, the selected key frame will be red
<p align="center">
<img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/pose_graph_editor/select_key_frame.gif" width="600" height="280"/>
</p>

### Select second key frame

Select a pose without pressing any key, the selected key frame will be green
<p align="center">
<img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/pose_graph_editor/select_second_key_frame.gif" width="600" height="280"/>
</p>

### Use ICP to find an edge between the first key frame and the second key frame

Use buttons px+, px-, py+, py-, to move the second key frame to match the first frame, and then click ICP to optimize two keyframes transform. If the result is satisfied, click "Accept", an edge will be added
<p align="center">
<img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/pose_graph_editor/icp.gif" width="600" height="280"/>
</p>

****
## Merge two pose graphs

<table align='center'>
  <tr width="100%">
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/pose_graph_editor/map1.png" width="420" height="260"/><p align='center'>Map1</p></td>
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/pose_graph_editor/map2.png" width="420" height="260"/><p align='center'>Map2</p></td>
  </tr>
</table>
<table align='center'>
  <tr width="100%">
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/pose_graph_editor/map1+2.png" width="420" height="260"/><p align='center'>Map1+2</p></td>
  </tr>
</table>
The Merge ICP button will match the first key frame of each pose graph. i.e.: Merge red key frame of the first pose graph and the red key frame of the second pose graph.
After click accept merge, there will be a pop up window to ask the new directory to save the merged result.
Once the result is saved, you can go back to pose graph panel to keep working on the merge result.
Steps:

1. Switch to the first pose graph, and pressed "shift" to select the first key frame of pose graph1.

2. Switch to the second pose graph, and pressed "shift" to select the first key frame of pose graph2.

3. Tick Merge visualization on the Rviz, then you will see two key frames. You can ICP them and fine tune the result.

4. Click accpet to save the result.

#### Check the demo video:

[![YouTube video thumbnail](https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/pose_graph_editor/merge_yt.png)](https://www.youtube.com/watch?v=QKRoJy30-ds)

</details>
