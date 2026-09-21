# DDDMR BEGINNER GUIDE

[العربية](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.ar.md) | [繁體中文](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.zh-hant.md) | [简体中文](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.zh-hans.md) | [Русский](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.ru.md) | [Deutsch](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.de.md) | [Español](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.es.md) | [한국어](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.ko.md) | [Português](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.pt.md) | [Türkçe](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.tr.md) | [Tiếng Việt](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.vi.md) | [Français](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.fr.md) | [日本語](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.ja.md)

This README is a beginner's guide to the DDDMR Navigation Stack. With both a gazebo quadruped robot example and a real robot guide, it's designed to help you get up and running fast, explore, and have fun along the way.

## Table of Contents

| # | Section | Description |
|:-:|:--------|:------------|
| 1 | [DDDMR Navigation with Gazebo](#-dddmr-navigation-with-gazebo) | Simulation demo with quadruped robot |
| 2 | [DDDMR Navigation with a Real Robot](#-start-dddmr-navigation-with-a-real-robot) | Deploy on your real robot (wheeled robot, quadruped, humanoid, and more) |

## 🖥️ Software Requirements
- **Ubuntu 22.04** (tested in 22.04, should support 24.04)
- **Docker**  [install Docker](https://docs.docker.com/engine/install/)
## ✨ DDDMR Navigation with Gazebo
This demo demonstrates how to run the DDDMR Navigation Stack in Gazebo with a quadruped robot.
- Build the required images to prepare the environment.
- Run the system in two terminals — one for the Gazebo world and another for the navigation stack. 

### 1. Create docker image
Clone the repo and run ./build.bash, please select **`x64_gz`**, which already contains all the necessary components for both navigation and Gazebo.
```
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

### 2. Download navigation map
To play gazebo with dddmr_navigation, you will need to download demo navigation map (12.3MB).
```
cd ~ && mkdir dddmr_bags
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./download_files.bash
```

### 3. Prepare demo enviroment
#### Create a two docker container (gazebo and navigaiton)
> [!IMPORTANT] 
> The following command will start two interactive Docker containers using the image we built.  Please open **two separate terminals** to prepare the demo environment

#### 🖥️ Terminal 1: Start the Go2 Gazebo Simulation

In the first terminal, follow the [Gazebo Guide — Go2](GAZEBO_GUIDE.md#go2--quadruped) to build the simulation image and start Gazebo.

Once the Go2 simulation is running, keep this terminal open and return here to continue with **Terminal 2** below.

#### 🖥️ Terminal 2   (create the container for navigation system)

   - ##### Step 1 (on host): create the container for navigation system
```
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./run_x64_navigation.bash
```
   - ##### Step 2 (inside the navigation container): build and launch
```
cd dddmr_navigation/ && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash && ros2 launch p2p_move_base go2_localization.launch
```

### 4. Run demo 
- In the Gazebo demo, the map is already aligned, so you don’t need to set the initial pose unless you are mapping yourself
- Give the goal (3D Goal Pose) in RViz , then the robot will move to target point

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/give_goal_in_demo_.png" width="920" height="460"/>
</p>

### 5. Known Issues
> [!WARNING]
> The following are currently observed behaviors. They are under investigation and will be fixed in future updates.
 - Gazebo: Occasional slipping on slopes 
 - Mapping: Duplicate floor layers  



---

<p align="center">
  <b>━━━━━━━━━━ 🤖 Real Robot Tutorial Below 🤖 ━━━━━━━━━━</b>
</p>

---


## ✨ Start DDDMR Navigation with a Real Robot

This guide will walk you through setting up **3D navigation** on your real robot — from mapping to autonomous navigation.

DDDMR brings 3D navigation to your wheeled robot, quadruped, humanoid, and more. Let's get your robot moving!

### 1. Create docker image

Clone the repo and run ./build.bash, please select **`x64`** or **`l4t`** depending on your platform.

```
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

> **Note:** Our test platform uses Jetson Orin Nano, so we select **`l4t`**.

### 2. Test Platform

To quickly match this tutorial, here is the hardware setup we use:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/hardware_quadruped.png" width="500" height="250"/>
</p>

| Component | Model |
|:----------|:------|
| Robot | Quadruped (Lite3) |
| LiDAR | Dome LiDAR (RoboSense Airy), tilted 45° |
| Computer | Jetson Orin Nano |
| Power | External battery for LiDAR & Jetson |
| Connection | Ethernet (robot, Jetson, and LiDAR communicate over network) |

> **Note:** You don't need the exact same hardware. As long as your system meets the requirements below, DDDMR will work.

#### Requirements

| Item | Topic | Message Type | Description |
|:----:|:------|:-------------|:------------|
| 🎮 | `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity command to control your robot |
| 📡 | `/lidar_point_cloud` | `sensor_msgs/msg/PointCloud2` | 3D LiDAR point cloud (multi-layer, dome, solid, etc.) |
| 📍 | `/odom` | `nav_msgs/msg/Odometry` | Odometry with TF (error < 10% will be better) |
| 🌳 | `/tf` | `tf2_msgs/msg/TFMessage` | TF tree: `odom` → `base_link` → `lidar_link` |

> **Note:** 
> - DDDMR can be used on **wheeled robots, quadrupeds, and humanoids** as long as they meet the requirements above.
> - `/cmd_vel` is the **input** to your robot for motion control.
> - `/lidar_point_cloud`, `/odom`, `/tf` are **outputs** from your robot, used by DDDMR for mapping and localization.

#### TF Tree Structure

Make sure your `frame_id` in LiDAR topic matches your TF tree:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/tf_requirement.png" width="150" height="250"/>
</p>

#### TF Configuration Example

Here is the TF configuration using our quadruped robot as an example. The LiDAR is mounted at `x=0.2m`, `z=0.22m` from `base_link`, and pitched down by 45°:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/setup_lidar_testing_quadruped.png" width="400" height="250"/>
</p>

```xml
<!--- TF: x y z yaw pitch roll -->
<node pkg="tf2_ros" exec="static_transform_publisher" name="sensor2baselink" 
      args="0.2 0.0 0.22 0.0 0.785 0.0 base_link lidar" />
```

> **Note:** 45° ≈ 0.785 rad. Pitch down.

#### 👉 Advanced (Optional)

| Feature | Description |
|:--------|:------------|
| [3D Odometry](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_odom_3d) | Better localization & mapping on uneven terrain |

---

### 3. 🚀 RUN Mapping + Navigation for robot

DDDMR supports three navigation workflows:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/dddmr_nav_workflows.png" width="600" height="450"/>
</p>

> **Note:** This tutorial follows **② Online mapping + Nav** workflow.

---


### 3.1 🗺️ Mapping

Before starting, make sure your robot is publishing the required topics:

- Odometry topic
- LiDAR point cloud topic  
- TF (`odom` → `base_link` → `lidar_link`)

Then launch the mapping:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_mapping.launch
```

After launching, you should see the RViz interface like this:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/mapping_realrobot_rviz.png" width="800" height="500"/>
</p>

When you drive your robot around, you will notice:
- **Key frames** — should increase as the robot moves
- **Feature points** — extracted features from the environment
- **Ground points** — detected ground surface
- **2D projection** — helps you understand the scene and LiDAR FOV

#### Save the Map

After you finish mapping the area, open a new terminal and run:

```bash
ros2 service call /save_mapped_point_cloud std_srvs/srv/Empty
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/map_save_.png" width="500" height="250"/>
</p>

When you see `Create dir:` in the terminal, it means the map has been saved. If you are done mapping, you can close the mapping node.

The map will be saved in `/tmp`. You can move the folder to `/root/dddmr_bags`:

```bash
mv /tmp/2026_03_28_19_25_15/ /root/dddmr_bags/
```

> **Note:** The folder name `2026_03_28_19_25_15` is just an example. Your folder name will be different based on when you run the mapping.

---

### 3.2 📍 Localization + Navigation

First, open the configuration file and set your map path:

📄 `config/airy_tilt45_navigation.yaml`

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/robot_map_location.png" width="500" height="500"/>
</p>

Change `pose_graph_dir` to your map folder:

```yaml
pose_graph_dir: "/root/dddmr_bags/2026_03_28_19_25_15"
```

> **Note:** Replace `2026_03_28_19_25_15` with your actual map folder name.

Then launch the localization and navigation:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_navigation.launch
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/LOC_NAV_realrobot_rviz.png" width="800" height="500"/>
</p>

##### Step 1: Give Initial Pose

Click **「3D Pose Estimate」** in RViz toolbar and select a ground point that matches your robot's real-world position.

If the initial pose is correct, you will see the observed point cloud overlap well with the map features.

##### Step 2: Send Goal

Once initialization is done, click **「3D Goal Pose」** and select a ground point as the goal.

The robot should start moving toward the goal.
