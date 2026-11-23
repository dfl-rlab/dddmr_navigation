# DDDMR 초보자 가이드

이 가이드는 DDDMR 내비게이션 입문 튜토리얼입니다. Lite3 Gazebo 시뮬레이션과 실제 로봇 배포 과정을 따라 3D 내비게이션을 시작할 수 있습니다.

## 목차

| # | 섹션 | 설명 |
|:-:|:--------|:------------|
| 1 | [Gazebo 시뮬레이션 내비게이션](#-dddmr-navigation-with-gazebo) | Lite3 RL 4족 로봇 시뮬레이션 데모 |
| 2 | [실제 로봇 내비게이션](#-start-dddmr-navigation-with-a-real-robot) | 실제 로봇에 배포 (바퀴형, 4족, 휴머노이드 등) |

## 🖥️ 소프트웨어 요구 사항
- **Ubuntu 22.04** (테스트 완료. Ubuntu 24.04 호스트에서도 Humble 컨테이너를 실행할 수 있습니다).
- **Docker** — [Docker 설치](https://docs.docker.com/engine/install/)
- GPU 시뮬레이션과 TensorRT 내비게이션 데모를 위한 **NVIDIA GPU 및 NVIDIA Container Toolkit**.

## ✨ DDDMR Navigation with Gazebo

돔형 LiDAR를 장착한 **Lite3 RL 4족 로봇**으로 여러 층으로 구성된 Gazebo 월드에서 DDDMR 내비게이션을 실행합니다. 데모는 두 개의 별도 이미지와 컨테이너를 사용합니다.

| 터미널 | 용도 | 이미지 | 컨테이너 |
|:---------|:--------|:------|:----------|
| 1 | Lite3 Gazebo 시뮬레이션 | `dddmr_simulation:rl_lite3` | `rl_lite3` |
| 2 | TensorRT를 사용하는 위치 추정 및 3D 내비게이션 | `dddmr:humble-cuda` | `dddmr_humble_cuda_dev` |

아래 명령은 내비게이션 저장소 경로로 `~/dddmr_navigation`, 시뮬레이터 경로로 `~/dfl_mobilerobot_simulator`를 사용합니다.

### 1. Docker 이미지 생성

먼저 내비게이션 저장소를 복제합니다. 아래의 두 이미지 준비 방법 모두 이 단계가 필요합니다.

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
```

다음 방법 중 **하나를 선택**하여 내비게이션 이미지를 준비합니다.

#### 방법 A: 로컬 빌드

빌드 스크립트를 실행하여 **`x64`**를 선택하고 CUDA 사용 질문에 **`Y`**로 답합니다. CUDA 및 TensorRT 의존성이 포함된 `dddmr:humble-cuda`가 생성됩니다.

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./build.bash
```

#### 방법 B: 공개 이미지 가져오기

미리 빌드된 이미지가 공개되어 있습니다. 이미지를 가져온 다음 `run_x64_gpu.bash`에서 사용하는 로컬 태그를 추가합니다.

```bash
docker pull tsengapola/dddmr:humble-cuda
docker tag tsengapola/dddmr:humble-cuda dddmr:humble-cuda
```

어느 방법이든 완료한 후 아래의 지도를 다운로드합니다. 내비게이션 작업 공간은 터미널 2의 컨테이너 안에서 컴파일합니다. Lite3 시뮬레이션 이미지는 아래 **터미널 1** 단계에서 별도로 빌드합니다.

### 2. 내비게이션 지도 다운로드

Lite3 월드용 지도 `stair_2_map.zip`을 다운로드하고 압축을 풉니다. 질문이 나오면 **`Y`**로 답합니다.

```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide
./download_files.bash
```

스크립트는 `~/dddmr_bags`를 생성하고 지도를 `~/dddmr_bags/stair_2_map`에 풉니다. 내비게이션 시작 스크립트는 지도 디렉터리를 컨테이너 안의 `/root/dddmr_bags`에 마운트합니다.

### 3. 데모 환경 준비

시뮬레이터용과 내비게이션용으로 **별도의 터미널 두 개**를 엽니다. 두 컨테이너 모두 **`ROS_DOMAIN_ID=22`**를 사용해야 합니다.

#### 🖥️ 터미널 1: Lite3 Gazebo 시뮬레이션 시작

[Gazebo 가이드 — Lite3](../GAZEBO_GUIDE.md#lite3--rl-quadruped)를 따라 `rl_lite3` 브랜치를 복제하고 서브모듈을 초기화한 뒤 이미지를 빌드하고 `./rl_lite3_gpu.bash`를 실행합니다.

로봇이 바닥에 내려오면 시뮬레이터 터미널에 포커스를 두고 **`r` → `0` → `1` → `n`** 순서로 누릅니다. 자세 초기화, 일어서기, RL 제어기 시작, 내비게이션 속도 명령 허용 순서입니다. **각 키 입력 사이에 최소 1초를 기다리고, `0` 이후에는 일어서기 동작이 완료될 때까지 기다립니다.**

| 키 | 동작 |
|:----|:-------|
| `r` | 로봇을 월드의 시작 자세로 초기화합니다. |
| `0` | 일어서서 이동을 준비합니다. |
| `1` | RL 보행 정책을 활성화합니다. |
| `n` | 내비게이션이 로봇을 제어할 수 있도록 `/cmd_vel` 입력을 활성화합니다. |

시뮬레이션 터미널을 열어 둔 채 터미널 2 단계로 진행합니다.

#### 🖥️ 터미널 2: 내비게이션 컨테이너 생성

##### 단계 1 (호스트): CUDA 컨테이너 시작

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./run_x64_gpu.bash
```

`ROS_DOMAIN_ID`를 물으면 **`22`**를 입력합니다. `dddmr:humble-cuda` 이미지로 `dddmr_humble_cuda_dev` 컨테이너를 생성하고 저장소를 `/root/dddmr_navigation`에 마운트합니다.

##### 단계 2 (내비게이션 컨테이너 내부): TensorRT를 활성화하여 빌드

```bash
cd /root/dddmr_navigation
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=22
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DTRT_ENABLED=ON -DTEST_ENABLED=ON
```

`TRT_ENABLED=ON`은 내비게이션 추론 코드를 활성화합니다. 새로 복제한 저장소에서는 포함된 모델과 변환 스크립트로 TensorRT 엔진도 생성해야 합니다.

```bash
cd /root/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/engine
bash onnx2trt.bash
cd /root/dddmr_navigation
```

##### 단계 3 (내비게이션 컨테이너 내부): 위치 추정 및 내비게이션 시작

```bash
source install/setup.bash
export ROS_DOMAIN_ID=22
ros2 launch p2p_move_base lite3_localization_launch.py
```

### 4. 데모 실행

1. 터미널 1의 Lite3가 RL 정책 모드이고 `/cmd_vel` 입력이 활성화되어 있는지 확인합니다.
2. RViz에서 실시간 포인트 클라우드와 제공된 지도가 겹치는지 확인합니다. 데모 설정에는 이 월드의 초기 자세가 포함됩니다. 로봇을 다른 위치로 이동하거나 초기화했다면 **3D Pose Estimate**로 정렬합니다.
3. RViz에서 **3D Goal Pose**를 선택하고 도달 가능한 지면 위치를 지정합니다. Lite3가 내비게이션 명령에 따라 목표로 이동합니다.

<p align="center">
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/3d_nav_rl_quadruped_deep_learning.png?raw=true" alt="Gazebo 및 RViz에서 딥 러닝을 사용하는 Lite3 RL 4족 로봇의 3D 내비게이션" width="920" />
</p>

---

<p align="center">
  <b>━━━━━━━━━━ 🤖 실제 로봇 튜토리얼 아래 🤖 ━━━━━━━━━━</b>
</p>

---

## ✨ Start DDDMR Navigation with a Real Robot

이 가이드는 실제 로봇에서 **3D 내비게이션**을 설정하는 과정을 안내합니다 — 매핑부터 자율 내비게이션까지.

DDDMR은 바퀴형 로봇, 4족 로봇, 휴머노이드 등에 3D 내비게이션 기능을 제공합니다. 로봇을 움직여 봅시다!

### 1. Docker 이미지 생성

저장소를 복제하고 ./build.bash를 실행하세요. 플랫폼에 따라 **`x64`** 또는 **`l4t`**를 선택하세요.

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

> **참고:** 테스트 플랫폼은 Jetson Orin Nano를 사용하므로 **`l4t`**를 선택합니다.

### 2. 테스트 플랫폼

이 튜토리얼에 빠르게 맞추기 위해, 사용하는 하드웨어 구성은 다음과 같습니다:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/hardware_quadruped.png" width="500" height="250"/>
</p>

| 구성 요소 | 모델 |
|:----------|:------|
| 로봇 | 4족 로봇 (Lite3) |
| LiDAR | 돔형 LiDAR (RoboSense Airy), 45° 기울임 |
| 컴퓨터 | Jetson Orin Nano |
| 전원 | LiDAR 및 Jetson용 외부 배터리 |
| 연결 | 이더넷 (로봇, Jetson, LiDAR가 네트워크로 통신) |

> **참고:** 정확히 같은 하드웨어가 필요하지 않습니다. 시스템이 아래 요구 사항을 충족하면 DDDMR이 작동합니다.

#### 시스템 요구 사항

| 항목 | 토픽 | 메시지 타입 | 설명 |
|:----:|:------|:-------------|:------------|
| 🎮 | `/cmd_vel` | `geometry_msgs/msg/Twist` | 로봇을 제어하는 속도 명령 |
| 📡 | `/lidar_point_cloud` | `sensor_msgs/msg/PointCloud2` | 3D LiDAR 포인트 클라우드 (멀티 레이어, 돔형, 솔리드 등) |
| 📍 | `/odom` | `nav_msgs/msg/Odometry` | TF가 포함된 오도메트리 (오차 < 10% 권장) |
| 🌳 | `/tf` | `tf2_msgs/msg/TFMessage` | TF 트리: `odom` → `base_link` → `lidar_link` |

> **참고:** 
> - DDDMR은 위 요구 사항을 충족하는 **바퀴형 로봇, 4족 로봇, 휴머노이드**에서 사용할 수 있습니다.
> - `/cmd_vel`은 로봇 모션 제어를 위한 **입력**입니다.
> - `/lidar_point_cloud`, `/odom`, `/tf`는 로봇의 **출력**으로, DDDMR이 매핑 및 위치 추정에 사용합니다.

#### TF 트리 구조

LiDAR 토픽의 `frame_id`가 TF 트리와 일치하는지 확인하세요:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/tf_requirement.png" width="150" height="250"/>
</p>

#### TF 설정 예제

다음은 4족 로봇을 예로 든 TF 설정입니다. LiDAR는 `base_link`에서 `x=0.2m`, `z=0.22m` 위치에 장착되어 있으며, 45° 아래로 기울어져 있습니다:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/setup_lidar_testing_quadruped.png" width="400" height="250"/>
</p>

```xml
<!--- TF: x y z yaw pitch roll -->
<node pkg="tf2_ros" exec="static_transform_publisher" name="sensor2baselink" 
      args="0.2 0.0 0.22 0.0 0.785 0.0 base_link lidar" />
```

> **참고:** 45° ≈ 0.785 라디안. 아래로 기울임.

#### 👉 고급 (선택 사항)

| 기능 | 설명 |
|:--------|:------------|
| [3D 오도메트리](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_odom_3d) | 불규칙한 지형에서 더 나은 위치 추정 및 매핑 |

---

### 3. 🚀 매핑 + 내비게이션 실행

DDDMR은 세 가지 내비게이션 워크플로우를 지원합니다:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/dddmr_nav_workflows.png" width="600" height="450"/>
</p>

> **참고:** 이 튜토리얼은 **② 온라인 매핑 + 내비게이션** 워크플로우를 따릅니다.

---

### 3.1 🗺️ 매핑

시작하기 전에 로봇이 필요한 토픽을 발행하고 있는지 확인하세요:

- 오도메트리 토픽
- LiDAR 포인트 클라우드 토픽
- TF (`odom` → `base_link` → `lidar_link`)

그런 다음 매핑을 시작하세요:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_mapping.launch
```

실행 후 다음과 같은 RViz 인터페이스가 표시됩니다:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/mapping_realrobot_rviz.png" width="800" height="500"/>
</p>

로봇을 주변으로 움직이면 다음을 확인할 수 있습니다:
- **키프레임** — 로봇이 이동함에 따라 증가해야 합니다
- **특징점** — 환경에서 추출된 특징
- **지면점** — 감지된 지면 표면
- **2D 투영** — 장면과 LiDAR FOV를 이해하는 데 도움

#### 맵 저장

영역 매핑을 완료한 후 새 터미널을 열고 실행하세요:

```bash
ros2 service call /save_mapped_point_cloud std_srvs/srv/Empty
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/map_save_.png" width="500" height="250"/>
</p>

터미널에서 `Create dir:`가 표시되면 맵이 저장된 것입니다. 매핑이 완료되면 매핑 노드를 종료할 수 있습니다.

맵은 `/tmp`에 저장됩니다. 폴더를 `/root/dddmr_bags`로 이동할 수 있습니다:

```bash
mv /tmp/2026_03_28_19_25_15/ /root/dddmr_bags/
```

> **참고:** 폴더 이름 `2026_03_28_19_25_15`는 예시입니다. 실제 폴더 이름은 매핑 실행 시간에 따라 다릅니다.

---

### 3.2 📍 위치 추정 + 내비게이션

먼저 설정 파일을 열고 맵 경로를 설정하세요:

📄 `config/airy_tilt45_navigation.yaml`

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/robot_map_location.png" width="500" height="500"/>
</p>

`pose_graph_dir`을 맵 폴더로 변경하세요:

```yaml
pose_graph_dir: "/root/dddmr_bags/2026_03_28_19_25_15"
```

> **참고:** `2026_03_28_19_25_15`를 실제 맵 폴더 이름으로 교체하세요.

그런 다음 위치 추정 및 내비게이션을 시작하세요:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_navigation.launch
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/LOC_NAV_realrobot_rviz.png" width="800" height="500"/>
</p>

##### 단계 1: 초기 포즈 설정

RViz 툴바에서 **「3D Pose Estimate」**를 클릭하고 로봇의 실제 위치와 일치하는 지면점을 선택하세요.

초기 포즈가 정확하면 관측된 포인트 클라우드가 맵 특징과 잘 겹치는 것을 볼 수 있습니다.

##### 단계 2: 목표 전송

초기화가 완료되면 **「3D Goal Pose」**를 클릭하고 지면점을 목표로 선택하세요.

로봇이 목표를 향해 이동을 시작합니다.
