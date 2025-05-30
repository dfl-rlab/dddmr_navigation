# dddmr_global_planner
DDDMR Global Planner is graph-based A* algorithm to find the shortest path in the point cloud, in our case the ground. The global planner takes in static graph (ground) and dynamic graph (sensors) to plan the path.

The dddmr_perception_3d is used in this global planner allowing ddd_navigation to find the path avoiding obstacles.

<table>
  <tr width="100%">
    <td width="50%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/global_planner/global_plan.png"/>Path planning on a zigzag wheelchair passage</td>
    <td width="50%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/global_planner/boundary_annotated.png"/>Boundary detection on the ground point cloud, making resulting path away from the edge.</td>
  </tr>
</table> 

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/global_planner/global_planner_diagram.png" width="640" height="420"/>
</p>

## Run The Demo
### 1. Create docker image
The package runs in the docker, so we need to build the image first. We support both x64 (tested in intel NUC) and arm64 (tested in nvidia jetson jpack6).
```
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation && git submodule update --init dddmr_docker src/dddmr_perception_3d src/dddmr_rviz_tools src/dddmr_mcl_3dl src/dddmr_sys_core src/dddmr_global_planner
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```
### 2. Run demo
#### Create a docker container
> [!NOTE]
> The following command will create an interactive docker container using the image we built. The we can launch the demo in the container.
```
cd ~/dddmr_navigation/dddmr_docker && ./run_demo.bash
```
#### Launch global planner
```
cd ~/dddmr_navigation && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch global_planner path_planning_on_static_layer.launch
```
#### Use Publish Point on Rviz2
Use Publish Point plugin to give a goal to global planner.
<p float='left'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/global_planner/global_planner_demo.png" width="640" height="400"/>
</p>


