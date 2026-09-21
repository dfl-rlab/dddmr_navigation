# DDDMR 新手指南

本指南是 DDDMR 導航系統的入門教學。包含 Gazebo 四足機器人模擬範例和真實機器人部署指南，幫助你快速上手，探索並享受 3D 導航的樂趣。

## 目錄

| # | 章節 | 描述 |
|:-:|:--------|:------------|
| 1 | [Gazebo 模擬導航](#-dddmr-navigation-with-gazebo) | 四足機器人模擬展示 |
| 2 | [真實機器人導航](#-start-dddmr-navigation-with-a-real-robot) | 部署到你的真實機器人（輪式、四足、人形等） |

## 🖥️ 軟體需求
- **Ubuntu 22.04**（已在 22.04 測試，應支援 24.04）
- **Docker** — [安裝 Docker](https://docs.docker.com/engine/install/)

## ✨ DDDMR Navigation with Gazebo

本展示說明如何在 Gazebo 中執行 DDDMR 導航系統與四足機器人。
- 建構所需映像檔以準備環境。
- 在兩個終端機中執行系統 — 一個用於 Gazebo 世界，另一個用於導航系統。

### 1. 建立 Docker 映像檔

複製儲存庫並執行 ./build.bash，請選擇 **`x64_gz`**，它已包含導航和 Gazebo 所需的所有元件。

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

### 2. 下載導航地圖

要在 Gazebo 中執行 dddmr_navigation，你需要下載展示用導航地圖（12.3MB）。

```bash
cd ~ && mkdir dddmr_bags
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./download_files.bash
```

### 3. 準備展示環境

#### 建立兩個 Docker 容器（Gazebo 和導航）

> [!IMPORTANT] 
> 以下指令將使用我們建構的映像檔啟動兩個互動式 Docker 容器。請開啟**兩個獨立的終端機**來準備展示環境。

#### 🖥️ 終端機 1：啟動 Go2 Gazebo 模擬

在第一個終端機中，依照 [Gazebo 指南 — Go2](../GAZEBO_GUIDE.md#go2--quadruped) 的兩個步驟，建立模擬映像檔並啟動 Gazebo 容器。

Go2 模擬啟動後，請保持此終端機開啟，並回到這裡繼續下方的**終端機 2** 步驟。

#### 🖥️ 終端機 2（建立導航系統容器）

- ##### 步驟 1（在主機上）：建立導航系統容器
```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./run_x64_navigation.bash
```

- ##### 步驟 2（在導航容器內）：建構並啟動
```bash
cd dddmr_navigation/ && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash && ros2 launch p2p_move_base go2_localization.launch
```

### 4. 執行展示

- 在 Gazebo 展示中，地圖已經對齊，除非你自己建圖，否則不需要設定初始位姿
- 在 RViz 中給定目標點（3D Goal Pose），機器人將移動到目標點

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/give_goal_in_demo_.png" width="920" height="460"/>
</p>

### 5. 已知問題

> [!WARNING]
> 以下是目前觀察到的行為，正在調查中，將在未來更新中修復。
> - Gazebo：斜坡上偶爾打滑
> - 建圖：重複的地板層

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
