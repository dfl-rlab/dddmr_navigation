# DDDMR 新手指南

本指南是 DDDMR 导航系统的入门教程。包含 Gazebo 四足机器人仿真示例和真实机器人部署指南，帮助你快速上手，探索并享受 3D 导航的乐趣。

## 目录

| # | 章节 | 描述 |
|:-:|:--------|:------------|
| 1 | [Gazebo 仿真导航](#-dddmr-navigation-with-gazebo) | 四足机器人仿真演示 |
| 2 | [真实机器人导航](#-start-dddmr-navigation-with-a-real-robot) | 部署到你的真实机器人（轮式、四足、人形等） |

## 🖥️ 软件需求
- **Ubuntu 22.04**（已在 22.04 测试，应支持 24.04）
- **Docker** — [安装 Docker](https://docs.docker.com/engine/install/)

## ✨ DDDMR Navigation with Gazebo

本演示展示如何在 Gazebo 中运行 DDDMR 导航系统与四足机器人。
- 构建所需镜像以准备环境。
- 在两个终端中运行系统 — 一个用于 Gazebo 世界，另一个用于导航系统。

### 1. 创建 Docker 镜像

克隆仓库并运行 ./build.bash，请选择 **`x64_gz`**，它已包含导航和 Gazebo 所需的所有组件。

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

### 2. 下载导航地图

要在 Gazebo 中运行 dddmr_navigation，你需要下载演示导航地图（12.3MB）。

```bash
cd ~ && mkdir dddmr_bags
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./download_files.bash
```

### 3. 准备演示环境

#### 创建两个 Docker 容器（Gazebo 和导航）

> [!IMPORTANT] 
> 以下命令将使用我们构建的镜像启动两个交互式 Docker 容器。请打开**两个独立的终端**来准备演示环境。

#### 🖥️ 终端 1：启动 Go2 Gazebo 仿真

在第一个终端中，按照 [Gazebo 指南 — Go2](../GAZEBO_GUIDE.md#go2--quadruped) 的两个步骤，构建仿真镜像并启动 Gazebo 容器。

Go2 仿真启动后，请保持此终端打开，并返回这里继续下方的**终端 2** 步骤。

#### 🖥️ 终端 2（创建导航系统容器）

- ##### 步骤 1（在主机上）：创建导航系统容器
```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./run_x64_navigation.bash
```

- ##### 步骤 2（在导航容器内）：构建并启动
```bash
cd dddmr_navigation/ && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash && ros2 launch p2p_move_base go2_localization.launch
```

### 4. 运行演示

- 在 Gazebo 演示中，地图已经对齐，除非你自己建图，否则不需要设置初始位姿
- 在 RViz 中给出目标点（3D Goal Pose），机器人将移动到目标点

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/give_goal_in_demo_.png" width="920" height="460"/>
</p>

### 5. 已知问题

> [!WARNING]
> 以下是目前观察到的行为，正在调查中，将在未来更新中修复。
> - Gazebo：斜坡上偶尔打滑
> - 建图：重复的地板层

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
