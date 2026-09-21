# Hướng dẫn cho người mới bắt đầu DDDMR

Hướng dẫn này là bài giới thiệu về hệ thống điều hướng DDDMR. Bao gồm ví dụ mô phỏng robot bốn chân trong Gazebo và hướng dẫn triển khai trên robot thực, giúp bạn nhanh chóng bắt đầu và tận hưởng điều hướng 3D.

## Mục lục

| # | Phần | Mô tả |
|:-:|:--------|:------------|
| 1 | [Điều hướng mô phỏng Gazebo](#-dddmr-navigation-with-gazebo) | Demo mô phỏng robot bốn chân |
| 2 | [Điều hướng robot thực](#-start-dddmr-navigation-with-a-real-robot) | Triển khai trên robot thực của bạn (robot bánh xe, bốn chân, hình người, v.v.) |

## 🖥️ Yêu cầu phần mềm
- **Ubuntu 22.04** (đã test trên 22.04, hỗ trợ 24.04)
- **Docker** — [Cài đặt Docker](https://docs.docker.com/engine/install/)

## ✨ DDDMR Navigation with Gazebo

Demo này trình bày cách chạy DDDMR Navigation Stack trong Gazebo với robot bốn chân.
- Xây dựng các image cần thiết để chuẩn bị môi trường.
- Chạy hệ thống trong hai terminal — một cho Gazebo world, một cho navigation stack.

### 1. Tạo Docker image

Clone repo và chạy ./build.bash, chọn **`x64_gz`**, đã bao gồm tất cả các thành phần cần thiết cho navigation và Gazebo.

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

### 2. Tải bản đồ điều hướng

Để chạy dddmr_navigation trong Gazebo, bạn cần tải bản đồ điều hướng demo (12.3MB).

```bash
cd ~ && mkdir dddmr_bags
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./download_files.bash
```

### 3. Chuẩn bị môi trường demo

#### Tạo hai Docker container (Gazebo và navigation)

> [!IMPORTANT] 
> Lệnh sau sẽ khởi động hai Docker container tương tác sử dụng image đã xây dựng. Vui lòng mở **hai terminal riêng biệt** để chuẩn bị môi trường demo.

#### 🖥️ Terminal 1: Khởi chạy mô phỏng Go2 trong Gazebo

Trong terminal đầu tiên, làm theo cả hai bước trong [Hướng dẫn Gazebo — Go2](../GAZEBO_GUIDE.md#go2--quadruped) để xây dựng image mô phỏng và khởi động container Gazebo.

Khi mô phỏng Go2 đã chạy, hãy giữ terminal này mở và quay lại đây để tiếp tục với **Terminal 2** bên dưới.

#### 🖥️ Terminal 2 (tạo container cho hệ thống navigation)

- ##### Bước 1 (trên host): tạo container cho hệ thống navigation
```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./run_x64_navigation.bash
```

- ##### Bước 2 (trong container navigation): build và khởi chạy
```bash
cd dddmr_navigation/ && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash && ros2 launch p2p_move_base go2_localization.launch
```

### 4. Chạy demo

- Trong demo Gazebo, bản đồ đã được căn chỉnh sẵn, nên bạn không cần đặt pose ban đầu trừ khi tự tạo bản đồ
- Đặt mục tiêu (3D Goal Pose) trong RViz, robot sẽ di chuyển đến điểm mục tiêu

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/give_goal_in_demo_.png" width="920" height="460"/>
</p>

### 5. Các vấn đề đã biết

> [!WARNING]
> Sau đây là các hành vi hiện được quan sát, đang được điều tra và sẽ được sửa trong các bản cập nhật tương lai.
> - Gazebo: Thỉnh thoảng trượt trên dốc
> - Mapping: Các lớp sàn bị trùng lặp

---

<p align="center">
  <b>━━━━━━━━━━ 🤖 Hướng dẫn robot thực bên dưới 🤖 ━━━━━━━━━━</b>
</p>

---

## ✨ Start DDDMR Navigation with a Real Robot

Hướng dẫn này sẽ đưa bạn qua quá trình thiết lập **điều hướng 3D** trên robot thực của bạn — từ mapping đến điều hướng tự động.

DDDMR mang điều hướng 3D đến robot bánh xe, robot bốn chân, robot hình người của bạn và hơn thế nữa. Hãy làm cho robot của bạn di chuyển!

### 1. Tạo Docker image

Clone repo và chạy ./build.bash, chọn **`x64`** hoặc **`l4t`** tùy theo nền tảng của bạn.

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

> **Lưu ý:** Nền tảng test của chúng tôi sử dụng Jetson Orin Nano, nên chọn **`l4t`**.

### 2. Nền tảng test

Để nhanh chóng làm theo hướng dẫn này, đây là cấu hình phần cứng chúng tôi sử dụng:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/hardware_quadruped.png" width="500" height="250"/>
</p>

| Thành phần | Model |
|:----------|:------|
| Robot | Robot bốn chân (Lite3) |
| LiDAR | Dome LiDAR (RoboSense Airy), nghiêng 45° |
| Máy tính | Jetson Orin Nano |
| Nguồn | Pin ngoài cho LiDAR và Jetson |
| Kết nối | Ethernet (robot, Jetson và LiDAR giao tiếp qua mạng) |

> **Lưu ý:** Bạn không cần phần cứng giống hệt. Miễn là hệ thống của bạn đáp ứng các yêu cầu dưới đây, DDDMR sẽ hoạt động.

#### Yêu cầu hệ thống

| Mục | Topic | Loại message | Mô tả |
|:----:|:------|:-------------|:------------|
| 🎮 | `/cmd_vel` | `geometry_msgs/msg/Twist` | Lệnh vận tốc để điều khiển robot |
| 📡 | `/lidar_point_cloud` | `sensor_msgs/msg/PointCloud2` | Point cloud LiDAR 3D (đa tầng, dome, solid, v.v.) |
| 📍 | `/odom` | `nav_msgs/msg/Odometry` | Odometry với TF (sai số < 10% là tốt) |
| 🌳 | `/tf` | `tf2_msgs/msg/TFMessage` | TF tree: `odom` → `base_link` → `lidar_link` |

> **Lưu ý:** 
> - DDDMR có thể được sử dụng trên **robot bánh xe, robot bốn chân và robot hình người** miễn là đáp ứng các yêu cầu trên.
> - `/cmd_vel` là **đầu vào** cho điều khiển chuyển động robot.
> - `/lidar_point_cloud`, `/odom`, `/tf` là **đầu ra** từ robot, được DDDMR sử dụng cho mapping và localization.

#### Cấu trúc TF Tree

Đảm bảo `frame_id` trong topic LiDAR khớp với TF tree của bạn:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/tf_requirement.png" width="150" height="250"/>
</p>

#### Ví dụ cấu hình TF

Đây là cấu hình TF sử dụng robot bốn chân của chúng tôi làm ví dụ. LiDAR được gắn tại `x=0.2m`, `z=0.22m` từ `base_link`, và nghiêng xuống 45°:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/setup_lidar_testing_quadruped.png" width="400" height="250"/>
</p>

```xml
<!--- TF: x y z yaw pitch roll -->
<node pkg="tf2_ros" exec="static_transform_publisher" name="sensor2baselink" 
      args="0.2 0.0 0.22 0.0 0.785 0.0 base_link lidar" />
```

> **Lưu ý:** 45° ≈ 0.785 radian. Nghiêng xuống.

#### 👉 Nâng cao (Tùy chọn)

| Tính năng | Mô tả |
|:--------|:------------|
| [Odometry 3D](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_odom_3d) | Localization và mapping tốt hơn trên địa hình gồ ghề |

---

### 3. 🚀 Chạy Mapping + Navigation

DDDMR hỗ trợ ba workflow điều hướng:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/dddmr_nav_workflows.png" width="600" height="450"/>
</p>

> **Lưu ý:** Hướng dẫn này theo workflow **② Online mapping + Nav**.

---

### 3.1 🗺️ Mapping

Trước khi bắt đầu, đảm bảo robot của bạn đang publish các topic cần thiết:

- Topic odometry
- Topic point cloud LiDAR
- TF (`odom` → `base_link` → `lidar_link`)

Sau đó khởi chạy mapping:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_mapping.launch
```

Sau khi khởi chạy, bạn sẽ thấy giao diện RViz như sau:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/mapping_realrobot_rviz.png" width="800" height="500"/>
</p>

Khi bạn lái robot đi xung quanh, bạn sẽ nhận thấy:
- **Key frames** — tăng lên khi robot di chuyển
- **Feature points** — các đặc trưng được trích xuất từ môi trường
- **Ground points** — bề mặt mặt đất được phát hiện
- **2D projection** — giúp hiểu scene và FOV của LiDAR

#### Lưu bản đồ

Sau khi hoàn thành mapping khu vực, mở terminal mới và chạy:

```bash
ros2 service call /save_mapped_point_cloud std_srvs/srv/Empty
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/map_save_.png" width="500" height="250"/>
</p>

Khi bạn thấy `Create dir:` trong terminal, nghĩa là bản đồ đã được lưu. Nếu mapping hoàn tất, bạn có thể đóng node mapping.

Bản đồ sẽ được lưu trong `/tmp`. Bạn có thể di chuyển thư mục đến `/root/dddmr_bags`:

```bash
mv /tmp/2026_03_28_19_25_15/ /root/dddmr_bags/
```

> **Lưu ý:** Tên thư mục `2026_03_28_19_25_15` chỉ là ví dụ. Tên thư mục của bạn sẽ khác tùy theo thời điểm bạn chạy mapping.

---

### 3.2 📍 Localization + Navigation

Đầu tiên, mở file cấu hình và đặt đường dẫn bản đồ của bạn:

📄 `config/airy_tilt45_navigation.yaml`

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/robot_map_location.png" width="500" height="500"/>
</p>

Thay đổi `pose_graph_dir` thành thư mục bản đồ của bạn:

```yaml
pose_graph_dir: "/root/dddmr_bags/2026_03_28_19_25_15"
```

> **Lưu ý:** Thay `2026_03_28_19_25_15` bằng tên thư mục bản đồ thực của bạn.

Sau đó khởi chạy localization và navigation:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_navigation.launch
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/LOC_NAV_realrobot_rviz.png" width="800" height="500"/>
</p>

##### Bước 1: Đặt Pose ban đầu

Click **「3D Pose Estimate」** trong thanh công cụ RViz và chọn một điểm trên mặt đất khớp với vị trí thực của robot.

Nếu pose ban đầu đúng, bạn sẽ thấy point cloud quan sát được trùng khớp tốt với các đặc trưng của bản đồ.

##### Bước 2: Gửi mục tiêu

Sau khi khởi tạo xong, click **「3D Goal Pose」** và chọn một điểm trên mặt đất làm mục tiêu.

Robot sẽ bắt đầu di chuyển về phía mục tiêu.
