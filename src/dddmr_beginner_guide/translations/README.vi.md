# Hướng dẫn cho người mới bắt đầu DDDMR

Hướng dẫn này giúp bạn bắt đầu với DDDMR Navigation. Mô phỏng Lite3 trong Gazebo và hướng dẫn triển khai trên robot thực giúp bạn nhanh chóng khám phá điều hướng 3D.

## Mục lục

| # | Phần | Mô tả |
|:-:|:--------|:------------|
| 1 | [Điều hướng mô phỏng Gazebo](#-dddmr-navigation-with-gazebo) | Demo mô phỏng robot bốn chân Lite3 dùng RL |
| 2 | [Điều hướng robot thực](#-start-dddmr-navigation-with-a-real-robot) | Triển khai trên robot thực của bạn (robot bánh xe, bốn chân, hình người, v.v.) |

## 🖥️ Yêu cầu phần mềm
- **Ubuntu 22.04** (đã kiểm thử; máy chủ Ubuntu 24.04 cũng có thể chạy container Humble).
- **Docker** — [Cài đặt Docker](https://docs.docker.com/engine/install/)
- **GPU NVIDIA và NVIDIA Container Toolkit** để chạy mô phỏng bằng GPU và demo điều hướng với TensorRT.

## ✨ DDDMR Navigation with Gazebo

Chạy DDDMR Navigation với **robot bốn chân Lite3 điều khiển bằng RL**, LiDAR dạng vòm và thế giới Gazebo nhiều tầng. Demo sử dụng hai image và container riêng biệt:

| Terminal | Mục đích | Image | Container |
|:---------|:--------|:------|:----------|
| 1 | Mô phỏng Lite3 trong Gazebo | `dddmr_simulation:rl_lite3` | `rl_lite3` |
| 2 | Định vị và điều hướng 3D với TensorRT | `dddmr:humble-cuda` | `dddmr_humble_cuda_dev` |

Các lệnh bên dưới dùng `~/dddmr_navigation` cho kho mã điều hướng và `~/dfl_mobilerobot_simulator` cho trình mô phỏng.

### 1. Tạo Docker image

Trước tiên, sao chép kho mã điều hướng. Cả hai cách lấy image bên dưới đều cần bước này:

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
```

Sau đó chọn **một** trong các cách sau để lấy image điều hướng.

#### Cách A: Tự dựng trên máy

Chạy script dựng image, chọn **`x64`**, rồi trả lời **`Y`** khi được hỏi về CUDA. Script tạo `dddmr:humble-cuda` với các thành phần phụ thuộc CUDA và TensorRT.

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./build.bash
```

#### Cách B: Tải image công khai

Image dựng sẵn đã được công bố. Tải image và thêm tag cục bộ mà `run_x64_gpu.bash` sử dụng:

```bash
docker pull tsengapola/dddmr:humble-cuda
docker tag tsengapola/dddmr:humble-cuda dddmr:humble-cuda
```

Sau khi hoàn thành một trong hai cách, tiếp tục tải bản đồ bên dưới. Workspace điều hướng được biên dịch trong container ở Terminal 2. Image mô phỏng Lite3 được dựng riêng tại bước **Terminal 1** bên dưới.

### 2. Tải bản đồ điều hướng

Tải và giải nén bản đồ `stair_2_map.zip` cho thế giới Lite3. Trả lời **`Y`** khi được hỏi:

```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide
./download_files.bash
```

Script tạo `~/dddmr_bags` và giải nén bản đồ vào `~/dddmr_bags/stair_2_map`. Script khởi động gắn thư mục bản đồ vào `/root/dddmr_bags` trong container điều hướng.

### 3. Chuẩn bị môi trường demo

Mở **hai terminal riêng biệt**: một cho trình mô phỏng và một cho điều hướng. Cả hai container phải dùng **`ROS_DOMAIN_ID=22`**.

#### 🖥️ Terminal 1: Khởi động mô phỏng Lite3 trong Gazebo

Làm theo [hướng dẫn Gazebo — Lite3](../GAZEBO_GUIDE.md#lite3--rl-quadruped) để sao chép nhánh `rl_lite3`, khởi tạo các submodule, dựng image và chạy `./rl_lite3_gpu.bash`.

Sau khi robot rơi xuống mặt đất, đặt tiêu điểm vào terminal mô phỏng và nhấn lần lượt **`r` → `0` → `1` → `n`**. Các bước này đặt lại tư thế, cho robot đứng dậy, khởi động bộ điều khiển RL và cho phép nhận lệnh điều hướng. **Chờ ít nhất một giây giữa các phím; sau `0`, cũng phải chờ robot đứng dậy xong.**

| Phím | Chức năng |
|:----|:-------|
| `r` | Đặt robot về tư thế ban đầu trong thế giới mô phỏng. |
| `0` | Cho robot đứng dậy và chuẩn bị di chuyển. |
| `1` | Kích hoạt chính sách di chuyển RL. |
| `n` | Bật đầu vào `/cmd_vel` để hệ thống điều hướng có thể điều khiển robot. |

Giữ terminal mô phỏng mở rồi chuyển sang Terminal 2.

#### 🖥️ Terminal 2: Tạo container điều hướng

##### Bước 1 (trên máy chủ): Khởi động container CUDA

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./run_x64_gpu.bash
```

Nhập **`22`** khi script hỏi `ROS_DOMAIN_ID`. Script tạo container `dddmr_humble_cuda_dev` từ `dddmr:humble-cuda` và gắn kho mã vào `/root/dddmr_navigation`.

##### Bước 2 (trong container điều hướng): Biên dịch với TensorRT

```bash
cd /root/dddmr_navigation
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=22
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DTRT_ENABLED=ON -DTEST_ENABLED=ON
```

`TRT_ENABLED=ON` bật mã suy luận của hệ thống điều hướng. Với kho mã mới sao chép, hãy tạo thêm engine TensorRT từ mô hình có sẵn bằng script chuyển đổi:

```bash
cd /root/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/engine
bash onnx2trt.bash
cd /root/dddmr_navigation
```

##### Bước 3 (trong container điều hướng): Khởi động định vị và điều hướng

```bash
source install/setup.bash
export ROS_DOMAIN_ID=22
ros2 launch p2p_move_base lite3_localization_launch.py
```

### 4. Chạy demo

1. Xác nhận Lite3 ở Terminal 1 đang dùng chính sách RL và đầu vào `/cmd_vel` đã được bật.
2. Trong RViz, kiểm tra đám mây điểm trực tiếp có trùng với bản đồ được cung cấp hay không. Cấu hình demo có sẵn tư thế ban đầu cho thế giới này. Nếu robot đã được di chuyển hoặc đặt lại ở vị trí khác, dùng **3D Pose Estimate** để căn chỉnh.
3. Chọn **3D Goal Pose** trong RViz và chỉ định một điểm có thể tiếp cận trên mặt đất. Lite3 sẽ làm theo lệnh điều hướng để đến đích.

<p align="center">
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/3d_nav_rl_quadruped_deep_learning.png?raw=true" alt="Điều hướng 3D với học sâu cho robot bốn chân Lite3 RL trong Gazebo và RViz" width="920" />
</p>

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
