# DDDMR BEGINNER GUIDE

[العربية](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.ar.md) | [繁體中文](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.zh-hant.md) | [简体中文](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.zh-hans.md) | [Русский](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.ru.md) | [Deutsch](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.de.md) | [Español](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.es.md) | [한국어](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.ko.md) | [Português](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.pt.md) | [Türkçe](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.tr.md) | [Tiếng Việt](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.vi.md) | [Français](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.fr.md) | [日本語](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.ja.md)

This README is a beginner's guide to the DDDMR Navigation Stack. With both a Lite3 Gazebo simulation and a real robot guide, it's designed to help you get up and running fast, explore, and have fun along the way.

## Table of Contents

| # | Section | Description |
|:-:|:--------|:------------|
| 1 | [DDDMR Navigation with Gazebo](#-dddmr-navigation-with-gazebo) | Lite3 RL quadruped simulation demo |
| 2 | [DDDMR Navigation with a Real Robot](#-start-dddmr-navigation-with-a-real-robot) | Deploy on your real robot (wheeled robot, quadruped, humanoid, and more) |

## 🖥️ Software Requirements

- **Ubuntu 22.04** (tested; Ubuntu 24.04 hosts can also run the Humble containers).
- **Docker** — see [Install Docker Engine](https://docs.docker.com/engine/install/).
- **NVIDIA GPU and NVIDIA Container Toolkit** for the GPU simulator and TensorRT navigation demo.

## ✨ DDDMR Navigation with Gazebo

Run the DDDMR Navigation Stack with the **Lite3 RL quadruped**, dome LiDAR and a multi-level Gazebo world. The demo uses two separate images and containers:

| Terminal | Purpose | Image | Container |
|:---------|:--------|:------|:----------|
| 1 | Lite3 Gazebo simulation | `dddmr_simulation:rl_lite3` | `rl_lite3` |
| 2 | Localization and 3D navigation with TensorRT | `dddmr:humble-cuda` | `dddmr_humble_cuda_dev` |

The commands below use `~/dddmr_navigation` for the navigation repository and `~/dfl_mobilerobot_simulator` for the simulator.

### 1. Create docker image

First, clone the navigation repository. This is needed for both image options below:

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
```

Then choose **one** of the following ways to obtain the navigation image.

#### Option A: Build locally

Run the image builder, select **`x64`**, then answer **`Y`** to the CUDA prompt. This builds `dddmr:humble-cuda`, including the CUDA and TensorRT dependencies.

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./build.bash
```

#### Option B: Pull the public image

A prebuilt image is already published. Pull it and add the local tag expected by `run_x64_gpu.bash`:

```bash
docker pull tsengapola/dddmr:humble-cuda
docker tag tsengapola/dddmr:humble-cuda dddmr:humble-cuda
```

After either option, continue with the map download below. The navigation workspace is compiled inside the container in Terminal 2.

The Lite3 simulation image is built separately in **Terminal 1** below.

### 2. Download navigation map

Download and extract the `stair_2_map.zip` navigation map for the Lite3 world. Answer **`Y`** when prompted:

```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide
./download_files.bash
```

The script creates `~/dddmr_bags` and extracts the map into `~/dddmr_bags/stair_2_map`. The navigation launcher mounts this directory at `/root/dddmr_bags` inside its container.

### 3. Prepare demo environment

Open **two separate terminals**: one for the simulator and one for navigation. Both containers must use **`ROS_DOMAIN_ID=22`**.

#### 🖥️ Terminal 1: Start the Lite3 Gazebo Simulation

Follow the [Gazebo Guide — Lite3](GAZEBO_GUIDE.md#lite3--rl-quadruped) to clone the `rl_lite3` branch, initialize its submodules, build the image and run `./rl_lite3_gpu.bash`.

After the robot drops onto the ground, keep the simulator terminal focused and press **`r` → `0` → `1` → `n`**. This resets the pose, stands the robot up, starts the RL controller and lets navigation send movement commands. **Wait at least one second between keys; after `0`, also wait until standing up is complete.**

| Key | Action |
|:----|:-------|
| `r` | Reset the robot to its starting pose in the world. |
| `0` | Stand up and prepare the robot for locomotion. |
| `1` | Activate the RL locomotion policy. |
| `n` | Enable `/cmd_vel` input so the navigation stack can control the robot. |

Keep the simulation terminal open, then continue with Terminal 2.

#### 🖥️ Terminal 2: Create the Navigation Container

##### Step 1 (on host): Start the CUDA container

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./run_x64_gpu.bash
```

Enter **`22`** when the script asks for `ROS_DOMAIN_ID`. It creates the `dddmr_humble_cuda_dev` container using `dddmr:humble-cuda` and mounts the navigation repository at `/root/dddmr_navigation`.

##### Step 2 (inside the navigation container): Build with TensorRT

```bash
cd /root/dddmr_navigation
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=22
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DTRT_ENABLED=ON -DTEST_ENABLED=ON
```

`TRT_ENABLED=ON` enables the navigation inference code. On a fresh checkout, also generate the TensorRT engine from the included model using the repository's conversion script:

```bash
cd /root/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/engine
bash onnx2trt.bash
cd /root/dddmr_navigation
```

##### Step 3 (inside the navigation container): Launch localization and navigation

```bash
source install/setup.bash
export ROS_DOMAIN_ID=22
ros2 launch p2p_move_base lite3_localization_launch.py
```

### 4. Run demo

1. Confirm Lite3 is in RL policy mode and `/cmd_vel` input is enabled in Terminal 1.
2. In RViz, check that the live point cloud overlaps the supplied map. The demo configuration provides an initial pose for this world; if the robot has been moved or reset elsewhere, use **3D Pose Estimate** to align it.
3. Select **3D Goal Pose** in RViz and choose a reachable ground point. Lite3 will follow the navigation commands toward the goal.

<p align="center">
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/3d_nav_rl_quadruped_deep_learning.png?raw=true" alt="Lite3 RL quadruped 3D navigation with deep learning in Gazebo and RViz" width="920" />
</p>



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
