# DDDMR 新手指南

本指南是 DDDMR 导航系统的入门教程，包含 Lite3 Gazebo 仿真与真实机器人部署，帮助你快速上手并探索 3D 导航。

## 目录

| # | 章节 | 描述 |
|:-:|:--------|:------------|
| 1 | [Gazebo 仿真导航](#-dddmr-navigation-with-gazebo) | Lite3 RL 四足机器人仿真演示 |
| 2 | [真实机器人导航](#-start-dddmr-navigation-with-a-real-robot) | 部署到你的真实机器人（轮式、四足、人形等） |

## 🖥️ 软件需求
- **Ubuntu 22.04**（已测试；Ubuntu 24.04 主机也可运行 Humble 容器）。
- **Docker** — [安装 Docker](https://docs.docker.com/engine/install/)
- **NVIDIA GPU 与 NVIDIA Container Toolkit**，用于 GPU 仿真与 TensorRT 导航演示。

## ✨ DDDMR Navigation with Gazebo

在多楼层 Gazebo 世界中，使用搭载穹顶式激光雷达的 **Lite3 RL 四足机器人**运行 DDDMR 导航。本演示使用两个独立的镜像与容器：

| 终端 | 用途 | 镜像 | 容器 |
|:---------|:--------|:------|:----------|
| 1 | Lite3 Gazebo 仿真 | `dddmr_simulation:rl_lite3` | `rl_lite3` |
| 2 | 使用 TensorRT 的定位与 3D 导航 | `dddmr:humble-cuda` | `dddmr_humble_cuda_dev` |

以下命令使用 `~/dddmr_navigation` 作为导航仓库路径，`~/dfl_mobilerobot_simulator` 作为仿真器路径。

### 1. 创建 Docker 镜像

先克隆导航仓库。以下两种获取镜像的方式都需要此步骤：

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
```

接着从以下方式中**选择一种**获取导航镜像。

#### 方式 A：在本地构建

运行构建脚本，选择 **`x64`**，并在 CUDA 提示时回答 **`Y`**。这会构建包含 CUDA 与 TensorRT 依赖项的 `dddmr:humble-cuda`。

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./build.bash
```

#### 方式 B：拉取公开镜像

预构建镜像已公开发布。拉取后，添加 `run_x64_gpu.bash` 使用的本地标签：

```bash
docker pull tsengapola/dddmr:humble-cuda
docker tag tsengapola/dddmr:humble-cuda dddmr:humble-cuda
```

完成任一方式后，继续下载下面的地图。导航工作空间会在终端 2 的容器内编译；Lite3 仿真镜像则在下面的**终端 1** 步骤中单独构建。

### 2. 下载导航地图

下载并解压 Lite3 世界使用的导航地图 `stair_2_map.zip`。出现提示时回答 **`Y`**：

```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide
./download_files.bash
```

脚本会创建 `~/dddmr_bags`，并将地图解压到 `~/dddmr_bags/stair_2_map`。导航容器的启动脚本会将此地图目录挂载到容器内的 `/root/dddmr_bags`。

### 3. 准备演示环境

打开**两个独立终端**，分别运行仿真器与导航。两个容器都必须使用 **`ROS_DOMAIN_ID=22`**。

#### 🖥️ 终端 1：启动 Lite3 Gazebo 仿真

按照 [Gazebo 指南 — Lite3](../GAZEBO_GUIDE.md#lite3--rl-quadruped)，克隆 `rl_lite3` 分支、初始化子模块、构建镜像，并运行 `./rl_lite3_gpu.bash`。

机器人落地后，保持仿真器终端处于输入焦点，依次按下 **`r` → `0` → `1` → `n`**，以重置姿态、站立、启动 RL 控制器并接受导航速度指令。**每次按键至少间隔一秒；按下 `0` 后，还要等站立动作完成。**

| 按键 | 功能 |
|:----|:-------|
| `r` | 将机器人重置到世界中的起始姿态。 |
| `0` | 站起来，准备行走。 |
| `1` | 启用 RL 行走策略。 |
| `n` | 启用 `/cmd_vel` 输入，让导航系统控制机器人。 |

保持仿真器终端开启，接着进行终端 2 的步骤。

#### 🖥️ 终端 2：创建导航容器

##### 步骤 1（主机上）：启动 CUDA 容器

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./run_x64_gpu.bash
```

脚本询问 `ROS_DOMAIN_ID` 时输入 **`22`**。它会使用 `dddmr:humble-cuda` 创建 `dddmr_humble_cuda_dev` 容器，并将导航仓库挂载到 `/root/dddmr_navigation`。

##### 步骤 2（导航容器内）：启用 TensorRT 编译

```bash
cd /root/dddmr_navigation
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=22
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DTRT_ENABLED=ON -DTEST_ENABLED=ON
```

`TRT_ENABLED=ON` 会启用导航推理代码。全新克隆的仓库还需要使用附带的模型与转换脚本生成 TensorRT 引擎：

```bash
cd /root/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/engine
bash onnx2trt.bash
cd /root/dddmr_navigation
```

##### 步骤 3（导航容器内）：启动定位与导航

```bash
source install/setup.bash
export ROS_DOMAIN_ID=22
ros2 launch p2p_move_base lite3_localization_launch.py
```

### 4. 运行演示

1. 确认终端 1 中的 Lite3 已进入 RL 策略模式，并启用 `/cmd_vel` 输入。
2. 在 RViz 中确认实时点云与提供的地图重叠。演示配置已包含此世界的初始姿态；若机器人被移动或重置到其他位置，请用 **3D Pose Estimate** 对齐。
3. 在 RViz 中选择 **3D Goal Pose**，并指定可到达的地面位置。Lite3 会按照导航指令前往目标。

<p align="center">
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/3d_nav_rl_quadruped_deep_learning.png?raw=true" alt="Lite3 RL 四足机器人在 Gazebo 与 RViz 中使用深度学习进行 3D 导航" width="920" />
</p>

---

<p align="center">
  <b>━━━━━━━━━━ 🤖 真实机器人教程如下 🤖 ━━━━━━━━━━</b>
</p>

---

## ✨ Start DDDMR Navigation with a Real Robot

本指南将引导你在真实机器人上设置 **3D 导航** — 从建图到自主导航。

DDDMR 为你的轮式机器人、四足机器人、人形机器人等带来 3D 导航能力。让你的机器人动起来吧！

### 1. 创建 Docker 镜像

克隆仓库并运行 ./build.bash，根据你的平台选择 **`x64`** 或 **`l4t`**。

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

> **注意：** 我们的测试平台使用 Jetson Orin Nano，所以选择 **`l4t`**。

### 2. 测试平台

为了快速匹配本教程，以下是我们使用的硬件配置：

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/hardware_quadruped.png" width="500" height="250"/>
</p>

| 组件 | 型号 |
|:----------|:------|
| 机器人 | 四足机器人（Lite3） |
| 激光雷达 | 穹顶式激光雷达（RoboSense Airy），倾斜 45° |
| 计算机 | Jetson Orin Nano |
| 电源 | 外接电池供给激光雷达和 Jetson |
| 连接方式 | 以太网（机器人、Jetson 和激光雷达通过网络通信） |

> **注意：** 你不需要完全相同的硬件。只要你的系统满足以下要求，DDDMR 就能正常工作。

#### 系统要求

| 项目 | 话题 | 消息类型 | 描述 |
|:----:|:------|:-------------|:------------|
| 🎮 | `/cmd_vel` | `geometry_msgs/msg/Twist` | 控制机器人的速度指令 |
| 📡 | `/lidar_point_cloud` | `sensor_msgs/msg/PointCloud2` | 3D 激光雷达点云（多线、穹顶、固态等） |
| 📍 | `/odom` | `nav_msgs/msg/Odometry` | 里程计与 TF（误差 < 10% 为佳） |
| 🌳 | `/tf` | `tf2_msgs/msg/TFMessage` | TF 树：`odom` → `base_link` → `lidar_link` |

> **注意：** 
> - DDDMR 可用于**轮式机器人、四足机器人和人形机器人**，只要满足上述要求。
> - `/cmd_vel` 是**输入**到你的机器人的运动控制指令。
> - `/lidar_point_cloud`、`/odom`、`/tf` 是你的机器人**输出**的数据，供 DDDMR 用于建图和定位。

#### TF 树结构

确保你的激光雷达话题的 `frame_id` 与你的 TF 树一致：

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/tf_requirement.png" width="150" height="250"/>
</p>

#### TF 配置示例

以下是以我们的四足机器人为例的 TF 配置。激光雷达安装在距离 `base_link` 的 `x=0.2m`、`z=0.22m` 处，向下倾斜 45°：

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/setup_lidar_testing_quadruped.png" width="400" height="250"/>
</p>

```xml
<!--- TF: x y z yaw pitch roll -->
<node pkg="tf2_ros" exec="static_transform_publisher" name="sensor2baselink" 
      args="0.2 0.0 0.22 0.0 0.785 0.0 base_link lidar" />
```

> **注意：** 45° ≈ 0.785 弧度。向下倾斜。

#### 👉 进阶（可选）

| 功能 | 描述 |
|:--------|:------------|
| [3D 里程计](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_odom_3d) | 在不平坦地形上获得更好的定位和建图效果 |

---

### 3. 🚀 运行建图 + 导航

DDDMR 支持三种导航工作流程：

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/dddmr_nav_workflows.png" width="600" height="450"/>
</p>

> **注意：** 本教程遵循 **② 在线建图 + 导航** 工作流程。

---

### 3.1 🗺️ 建图

开始之前，确保你的机器人正在发布所需的话题：

- 里程计话题
- 激光雷达点云话题
- TF（`odom` → `base_link` → `lidar_link`）

然后启动建图：

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_mapping.launch
```

启动后，你应该看到如下的 RViz 界面：

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/mapping_realrobot_rviz.png" width="800" height="500"/>
</p>

当你驾驶机器人四处移动时，你会注意到：
- **关键帧** — 随着机器人移动应该会增加
- **特征点** — 从环境中提取的特征
- **地面点** — 检测到的地面表面
- **2D 投影** — 帮助你理解场景和激光雷达视场

#### 保存地图

完成区域建图后，打开一个新终端并运行：

```bash
ros2 service call /save_mapped_point_cloud std_srvs/srv/Empty
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/map_save_.png" width="500" height="250"/>
</p>

当你在终端看到 `Create dir:` 时，表示地图已保存。如果建图完成，可以关闭建图节点。

地图将保存在 `/tmp`。你可以将文件夹移动到 `/root/dddmr_bags`：

```bash
mv /tmp/2026_03_28_19_25_15/ /root/dddmr_bags/
```

> **注意：** 文件夹名称 `2026_03_28_19_25_15` 只是示例。你的文件夹名称将根据建图时间而不同。

---

### 3.2 📍 定位 + 导航

首先，打开配置文件并设置你的地图路径：

📄 `config/airy_tilt45_navigation.yaml`

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/robot_map_location.png" width="500" height="500"/>
</p>

将 `pose_graph_dir` 修改为你的地图文件夹：

```yaml
pose_graph_dir: "/root/dddmr_bags/2026_03_28_19_25_15"
```

> **注意：** 将 `2026_03_28_19_25_15` 替换为你实际的地图文件夹名称。

然后启动定位和导航：

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_navigation.launch
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/LOC_NAV_realrobot_rviz.png" width="800" height="500"/>
</p>

##### 步骤 1：给定初始位姿

点击 RViz 工具栏中的 **「3D Pose Estimate」**，选择一个与机器人真实位置匹配的地面点。

如果初始位姿正确，你将看到观测到的点云与地图特征重叠良好。

##### 步骤 2：发送目标点

初始化完成后，点击 **「3D Goal Pose」**，选择一个地面点作为目标。

机器人应该开始向目标移动。
