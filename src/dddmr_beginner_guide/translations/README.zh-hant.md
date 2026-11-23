# DDDMR 新手指南

本指南是 DDDMR 導航系統的入門教學，包含 Lite3 Gazebo 模擬與真實機器人部署，幫助你快速上手並探索 3D 導航。

## 目錄

| # | 章節 | 描述 |
|:-:|:--------|:------------|
| 1 | [Gazebo 模擬導航](#-dddmr-navigation-with-gazebo) | Lite3 RL 四足機器人模擬展示 |
| 2 | [真實機器人導航](#-start-dddmr-navigation-with-a-real-robot) | 部署到你的真實機器人（輪式、四足、人形等） |

## 🖥️ 軟體需求
- **Ubuntu 22.04**（已測試；Ubuntu 24.04 主機也可執行 Humble 容器）。
- **Docker** — [安裝 Docker](https://docs.docker.com/engine/install/)
- **NVIDIA GPU 與 NVIDIA Container Toolkit**，供 GPU 模擬與 TensorRT 導航展示使用。

## ✨ DDDMR Navigation with Gazebo

在多樓層 Gazebo 世界中，使用搭載穹頂式光達的 **Lite3 RL 四足機器人**執行 DDDMR 導航。此展示使用兩個獨立的映像檔與容器：

| 終端機 | 用途 | 映像檔 | 容器 |
|:---------|:--------|:------|:----------|
| 1 | Lite3 Gazebo 模擬 | `dddmr_simulation:rl_lite3` | `rl_lite3` |
| 2 | 使用 TensorRT 的定位與 3D 導航 | `dddmr:humble-cuda` | `dddmr_humble_cuda_dev` |

以下指令使用 `~/dddmr_navigation` 作為導航儲存庫路徑，`~/dfl_mobilerobot_simulator` 作為模擬器路徑。

### 1. 建立 Docker 映像檔

先複製導航儲存庫。以下兩種取得映像檔的方式都需要此步驟：

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
```

接著從以下方式中**選擇一種**取得導航映像檔。

#### 方式 A：在本機建構

執行建構腳本，選擇 **`x64`**，並在 CUDA 提示時回答 **`Y`**。這會建構包含 CUDA 與 TensorRT 相依套件的 `dddmr:humble-cuda`。

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./build.bash
```

#### 方式 B：拉取公開映像檔

預先建構的映像檔已公開發布。拉取後，加上 `run_x64_gpu.bash` 使用的本機標籤：

```bash
docker pull tsengapola/dddmr:humble-cuda
docker tag tsengapola/dddmr:humble-cuda dddmr:humble-cuda
```

完成任一方式後，繼續下載下方的地圖。導航工作空間會在終端機 2 的容器內編譯；Lite3 模擬映像檔則在下方的**終端機 1** 步驟另外建構。

### 2. 下載導航地圖

下載並解壓縮 Lite3 世界使用的導航地圖 `stair_2_map.zip`。出現提示時回答 **`Y`**：

```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide
./download_files.bash
```

腳本會建立 `~/dddmr_bags`，並將地圖解壓縮至 `~/dddmr_bags/stair_2_map`。導航容器的啟動腳本會將此地圖目錄掛載到容器內的 `/root/dddmr_bags`。

### 3. 準備展示環境

開啟**兩個獨立終端機**，分別執行模擬器與導航。兩個容器都必須使用 **`ROS_DOMAIN_ID=22`**。

#### 🖥️ 終端機 1：啟動 Lite3 Gazebo 模擬

依照 [Gazebo 指南 — Lite3](../GAZEBO_GUIDE.md#lite3--rl-quadruped)，複製 `rl_lite3` 分支、初始化子模組、建構映像檔，並執行 `./rl_lite3_gpu.bash`。

機器人落地後，讓模擬器終端機保持輸入焦點，依序按下 **`r` → `0` → `1` → `n`**，以重設姿態、站立、啟動 RL 控制器並接受導航速度指令。**每次按鍵至少間隔一秒；按下 `0` 後，也要等站立動作完成。**

| 按鍵 | 功能 |
|:----|:-------|
| `r` | 將機器人重設至世界中的起始姿態。 |
| `0` | 站起來，準備行走。 |
| `1` | 啟用 RL 行走策略。 |
| `n` | 啟用 `/cmd_vel` 輸入，讓導航系統控制機器人。 |

保持模擬器終端機開啟，接著進行終端機 2 的步驟。

#### 🖥️ 終端機 2：建立導航容器

##### 步驟 1（主機上）：啟動 CUDA 容器

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./run_x64_gpu.bash
```

腳本詢問 `ROS_DOMAIN_ID` 時輸入 **`22`**。它會使用 `dddmr:humble-cuda` 建立 `dddmr_humble_cuda_dev` 容器，並將導航儲存庫掛載至 `/root/dddmr_navigation`。

##### 步驟 2（導航容器內）：啟用 TensorRT 編譯

```bash
cd /root/dddmr_navigation
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=22
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DTRT_ENABLED=ON -DTEST_ENABLED=ON
```

`TRT_ENABLED=ON` 會啟用導航推論程式碼。全新複製的儲存庫還需要使用內附的模型與轉換腳本產生 TensorRT 引擎：

```bash
cd /root/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/engine
bash onnx2trt.bash
cd /root/dddmr_navigation
```

##### 步驟 3（導航容器內）：啟動定位與導航

```bash
source install/setup.bash
export ROS_DOMAIN_ID=22
ros2 launch p2p_move_base lite3_localization_launch.py
```

### 4. 執行展示

1. 確認終端機 1 中的 Lite3 已進入 RL 策略模式，並啟用 `/cmd_vel` 輸入。
2. 在 RViz 中確認即時點雲與提供的地圖重疊。展示設定已包含此世界的初始姿態；若機器人被移動或重設到其他位置，請用 **3D Pose Estimate** 對齊。
3. 在 RViz 中選擇 **3D Goal Pose**，並指定可到達的地面位置。Lite3 會依照導航指令前往目標。

<p align="center">
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/3d_nav_rl_quadruped_deep_learning.png?raw=true" alt="Lite3 RL 四足機器人在 Gazebo 與 RViz 中使用深度學習進行 3D 導航" width="920" />
</p>

---

<p align="center">
  <b>━━━━━━━━━━ 🤖 真實機器人教學如下 🤖 ━━━━━━━━━━</b>
</p>

---

## ✨ Start DDDMR Navigation with a Real Robot

本指南將引導你在真實機器人上設定 **3D 導航** — 從建圖到自主導航。

DDDMR 為你的輪式機器人、四足機器人、人形機器人等帶來 3D 導航能力。讓你的機器人動起來吧！

### 1. 建立 Docker 映像檔

複製儲存庫並執行 ./build.bash，根據你的平台選擇 **`x64`** 或 **`l4t`**。

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

> **注意：** 我們的測試平台使用 Jetson Orin Nano，所以選擇 **`l4t`**。

### 2. 測試平台

為了快速配合本教學，以下是我們使用的硬體配置：

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/hardware_quadruped.png" width="500" height="250"/>
</p>

| 元件 | 型號 |
|:----------|:------|
| 機器人 | 四足機器人（Lite3） |
| 光達 | 穹頂式光達（RoboSense Airy），傾斜 45° |
| 電腦 | Jetson Orin Nano |
| 電源 | 外接電池供給光達和 Jetson |
| 連接方式 | 乙太網路（機器人、Jetson 和光達透過網路通訊） |

> **注意：** 你不需要完全相同的硬體。只要你的系統滿足以下要求，DDDMR 就能正常運作。

#### 系統要求

| 項目 | 話題 | 訊息類型 | 描述 |
|:----:|:------|:-------------|:------------|
| 🎮 | `/cmd_vel` | `geometry_msgs/msg/Twist` | 控制機器人的速度指令 |
| 📡 | `/lidar_point_cloud` | `sensor_msgs/msg/PointCloud2` | 3D 光達點雲（多線、穹頂、固態等） |
| 📍 | `/odom` | `nav_msgs/msg/Odometry` | 里程計與 TF（誤差 < 10% 為佳） |
| 🌳 | `/tf` | `tf2_msgs/msg/TFMessage` | TF 樹：`odom` → `base_link` → `lidar_link` |

> **注意：** 
> - DDDMR 可用於**輪式機器人、四足機器人和人形機器人**，只要滿足上述要求。
> - `/cmd_vel` 是**輸入**到你的機器人的運動控制指令。
> - `/lidar_point_cloud`、`/odom`、`/tf` 是你的機器人**輸出**的資料，供 DDDMR 用於建圖和定位。

#### TF 樹結構

確保你的光達話題的 `frame_id` 與你的 TF 樹一致：

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/tf_requirement.png" width="150" height="250"/>
</p>

#### TF 設定範例

以下是以我們的四足機器人為例的 TF 設定。光達安裝在距離 `base_link` 的 `x=0.2m`、`z=0.22m` 處，向下傾斜 45°：

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/setup_lidar_testing_quadruped.png" width="400" height="250"/>
</p>

```xml
<!--- TF: x y z yaw pitch roll -->
<node pkg="tf2_ros" exec="static_transform_publisher" name="sensor2baselink" 
      args="0.2 0.0 0.22 0.0 0.785 0.0 base_link lidar" />
```

> **注意：** 45° ≈ 0.785 弧度。向下傾斜。

#### 👉 進階（選用）

| 功能 | 描述 |
|:--------|:------------|
| [3D 里程計](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_odom_3d) | 在不平坦地形上獲得更好的定位和建圖效果 |

---

### 3. 🚀 執行建圖 + 導航

DDDMR 支援三種導航工作流程：

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/dddmr_nav_workflows.png" width="600" height="450"/>
</p>

> **注意：** 本教學遵循 **② 線上建圖 + 導航** 工作流程。

---

### 3.1 🗺️ 建圖

開始之前，確保你的機器人正在發布所需的話題：

- 里程計話題
- 光達點雲話題
- TF（`odom` → `base_link` → `lidar_link`）

然後啟動建圖：

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_mapping.launch
```

啟動後，你應該看到如下的 RViz 介面：

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/mapping_realrobot_rviz.png" width="800" height="500"/>
</p>

當你駕駛機器人四處移動時，你會注意到：
- **關鍵幀** — 隨著機器人移動應該會增加
- **特徵點** — 從環境中提取的特徵
- **地面點** — 偵測到的地面表面
- **2D 投影** — 幫助你理解場景和光達視野

#### 儲存地圖

完成區域建圖後，開啟一個新終端機並執行：

```bash
ros2 service call /save_mapped_point_cloud std_srvs/srv/Empty
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/map_save_.png" width="500" height="250"/>
</p>

當你在終端機看到 `Create dir:` 時，表示地圖已儲存。如果建圖完成，可以關閉建圖節點。

地圖將儲存在 `/tmp`。你可以將資料夾移動到 `/root/dddmr_bags`：

```bash
mv /tmp/2026_03_28_19_25_15/ /root/dddmr_bags/
```

> **注意：** 資料夾名稱 `2026_03_28_19_25_15` 只是範例。你的資料夾名稱將根據建圖時間而不同。

---

### 3.2 📍 定位 + 導航

首先，開啟設定檔並設定你的地圖路徑：

📄 `config/airy_tilt45_navigation.yaml`

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/robot_map_location.png" width="500" height="500"/>
</p>

將 `pose_graph_dir` 修改為你的地圖資料夾：

```yaml
pose_graph_dir: "/root/dddmr_bags/2026_03_28_19_25_15"
```

> **注意：** 將 `2026_03_28_19_25_15` 替換為你實際的地圖資料夾名稱。

然後啟動定位和導航：

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_navigation.launch
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/LOC_NAV_realrobot_rviz.png" width="800" height="500"/>
</p>

##### 步驟 1：給定初始位姿

點擊 RViz 工具列中的 **「3D Pose Estimate」**，選擇一個與機器人真實位置相符的地面點。

如果初始位姿正確，你將看到觀測到的點雲與地圖特徵重疊良好。

##### 步驟 2：發送目標點

初始化完成後，點擊 **「3D Goal Pose」**，選擇一個地面點作為目標。

機器人應該開始向目標移動。
