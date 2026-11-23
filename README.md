# 🤖 dddmr_navigation

**Next-Generation 3D Autonomous Navigation for Mobile Robots & Quadrupeds**

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/lite3_stair_3d_nav_tb.gif" width="800" height="560" alt="Quadruped Multi-Floor Stair Navigation in Gazebo and RViz"/>
</p>
<p align='center'><i>Quadruped robot navigating across multiple floors and stairs in Gazebo & RViz with real-time YOLO + TensorRT stair segmentation</i></p>

**`dddmr_navigation`** (3D Mobile Robot Navigation) is a complete, end-to-end 3D navigation stack engineered to conquer complex, multi-level, and unstructured 3D environments where traditional 2D navigation frameworks fall short. From multi-floor buildings and steep ramps to staircases, `dddmr_navigation` empowers wheeled and legged robots to map, localize, plan, and navigate autonomously in true 3D space.

> [!NOTE]
> `dddmr_navigation` is designed to solve the challenges that [Nav2](https://github.com/ros-navigation/navigation2) cannot handle—including multi-layer floor mapping and localization, 3D path planning in stereo structures, ramp and stair traversal, and dynamic perception markings and clearings directly in a 3D point cloud map.

---

## 🪜 Milestone Showcase: Multi-Floor Quadruped Stair Navigation (YOLO + TensorRT)

Bringing all layers of `dddmr_navigation` together, our latest milestone achieves **autonomous multi-floor navigation across staircases for quadruped robots**.

Together with `dddmr_navigation`'s 3D localization, multi-floor global path planning, real-time obstacle avoidance, and locomotion control, the robot dog autonomously navigates from start to goal across multiple floors and staircases.

👉 [Let's play Lite3 stair navigation using dddmr_navigation](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_beginner_guide)

---

## 🎥 Core Capabilities in Action

<table align='center'>
  <tr width="100%">
    <td width="33%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/multilevel_map.gif" width="280" height="230"/><p align='center'>Multilevel map</p></td>
    <td width="33%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/obstacle_avoidance.gif" width="280" height="230"/><p align='center'>Obstacle avoidance on ramps</p></td>
    <td width="33%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/mapping_navigating.gif" width="280" height="230"/><p align='center'>Navigating while mapping</p></td>
  </tr>
</table>

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
