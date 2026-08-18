# 🤖 dddmr_navigation
## 🧪 Automated CI/CD & Multi-LiDAR Test Suite

Configuring 3D LiDARs with non-zero tilt angles (pitch/roll/yaw) is often a major headache for beginners—frequently causing corrupted maps, bad ground-plane filtering, or broken coordinate transforms.

Our latest release introduces an automated **CI/CD Test Suite** that verifies sensor configurations and navigation compatibility before deployment:

* **Angle & Pitch Validation:** Automatically tests flat, forward-tilted, and custom-angled LiDAR mount geometries.
* **TF & Point Cloud Accuracy:** Ensures spatial transforms, ground projection, and obstacle clearing remain consistent across mounting angles.
* **Beginner-Friendly Benchmarking:** Pre-configured test scenarios serve as reliable reference setups for custom robot builds.

### 🔗 Quick Links
- **Run Tests Locally:**
Found the details in our [CICD docs](https://github.com/dfl-rlab/dddmr_navigation/tree/main/CICD_setup#dddmr-lego-loam-ci)
<table align='center'>
  <tr width="100%">
    <td width="33%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/CICD_setup/airy_t45_2x.gif" width="256" height="157"/><p align='center'>Airy tilted 45 degree</p></td>
    <td width="33%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/CICD_setup/c16_t0_2x.gif" width="256" height="157"/><p align='center'>C16 with no tilting</p></td>
    <td width="33%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/CICD_setup/mid360_t180_2x.gif" width="256" height="157"/><p align='center'>Mid360 rolling 180</p></td>
  </tr>
</table>

## 🚀 Go2 Simulator!
We’ve just integrated a Gazebo models using Unitree-go2 with the DDDMR Navigation Stack, unlocking true 3D navigation for simulation and testing. Using the latest quadruped robots go2 combined with our advanced stack, you can explore navigation capabilities that go far beyond traditional 2D navigation frameworks.

👉 Jump in, simulate, and experience features that Nav2 alone can’t achieve — multi-level mapping, ramp navigation, and obstacle handling in complex environments. 

[👾 Let's play go2 using dddmr navigation](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_beginner_guide)

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/3d_nav_gz.gif" width="700" height="420"/>
</p>

---

> [!NOTE]
> DDDMR Navigation Stack is designed to solve the issues that [Nav2](https://github.com/ros-navigation/navigation2) not able to handle: such as multi-layer floor mapping and localization, path planning in stereo structures and percption markings and clearings in a 3D point cloud map.

<table align='center'>
  <tr width="100%">
    <td width="40%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/multilevel_map.gif" width="400" height="260"/><p align='center'>Multilevel map</p></td>
    <td width="40%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/obstacle_avoidance.gif" width="400" height="260"/><p align='center'>Obstacle avoidance on ramps</p></td>
  </tr>
  <tr width="100%">
    <td width="40%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/mapping_navigating.gif" width="400" height="260"/><p align='center'>Navigating while mapping</p></td>
    <td width="40%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_semantic_segmentation/dddmr_semantic_segmentation_to_pointcloud.gif" width="400" height="260"/><p align='center'>Semantic segmentation and navigation (stay tuned🔥)</p></td>
  </tr>
</table>

DDDMR navigation (3D Mobile Robot Navigation) is a navigation stack allows users to map, localize and autonomously navigate in 3D environments. 

Below figure shows the comparison between 2D navigation stack and DDD(3D) navigation.
Our stack is a total solution for a mobile platform to navigate in 3D environments. There are plenty advantages for choosing DDD navigation:
  
✨ The standard procedures of DDD mobile robots and 2D mobile robots are the same, make it easier for 2D navigation stack users to transit to DDD navigation without difficulties:
  1. Mapping and refining the map using our packages and tools.
  2. Turn off mapping, use MCL to localize the robot by providing an initial pose.
  3. Send a goal to the robot, the robot will calculate the global plan and avoid obstacles using local planner.

✨ DDD navigation is no longer suffered from terrain situations. For example, ramps in factories or wheelchair accessible.

✨ DDD navigation has been well tested is many fields and is based on the cost-effective hardware, for example, 16 lines lidar, intel NUC/Jetson Orin Nano and consumer-grade imu. We are trying to make the solution as affordable as possible.

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/navigation_diagram.png" width="780" height="560"/>
</p>

## Citation

Please cite the following if you use this code or parts of it:

```
@software{dddmr_navigation_dfl-rlab,
  author = {CM, PS, Tarek Taha},
  title = {dddmr_navigation: 3D Mobile Robot Navigation},
  url = {https://github.com/dfl-rlab/dddmr_navigation},
  year = {2025}
}
```

😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫 I have a robot, but where to start?[Click me to see the beginner's guide](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/README.md)😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫
## 🏁 Detail documentations for each package
<details><summary> <b>💡 Click me to see Mapping</b> </summary>
https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_lego_loam
</details>
<details><summary> <b>💡 Click me to see Localization</b> </summary>
https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_mcl_3dl
</details>
<details><summary> <b>💡 Click me to see Perception</b> </summary>
https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_perception_3d
</details>
<details><summary> <b>💡 Click me to see Global planner</b> </summary>
https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_global_planner
</details>
<details><summary> <b>💡 Click me to see Local planner</b> </summary>
https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_local_planner
</details>
<details><summary> <b>💡 Click me to see Move base</b> </summary>
https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_p2p_move_base
</details>

## Demonstrations of DDD navigation functions
<table align='center'>
  <tr width="100%">
    <td width="50%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/mapping.gif" width="400" height="260"/><p align='center'>3D mapping</p></td>
    <td width="50%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/global_planner.gif" width="400" height="260"/><p align='center'>3D global planning</p></td>
  </tr>
  <tr width="100%">
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/local_planner.gif" width="400" height="260"/><p align='center'>3D local planning</p></td>
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/navigation.gif" width="400" height="260"/><p align='center'>3D navigation</p></td>
  </tr>
  <tr width="100%">
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/perception_3d/scanning_lidar_demo.gif" width="400" height="260"/><p align='center'>Support vairant sensors (Unitree G4)</p></td>
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/perception_3d/multi_depth_camera_demo.gif" width="400" height="260"/><p align='center'>Support vairant sensors (Depth Camera)</p></td>
  </tr>
</table>

