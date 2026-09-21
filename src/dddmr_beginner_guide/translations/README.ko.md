# DDDMR 초보자 가이드

이 가이드는 DDDMR 내비게이션 시스템의 입문 튜토리얼입니다. Gazebo 4족 로봇 시뮬레이션 예제와 실제 로봇 배포 가이드를 포함하여, 빠르게 시작하고 3D 내비게이션을 탐색하고 즐길 수 있도록 도와드립니다.

## 목차

| # | 섹션 | 설명 |
|:-:|:--------|:------------|
| 1 | [Gazebo 시뮬레이션 내비게이션](#-dddmr-navigation-with-gazebo) | 4족 로봇 시뮬레이션 데모 |
| 2 | [실제 로봇 내비게이션](#-start-dddmr-navigation-with-a-real-robot) | 실제 로봇에 배포 (바퀴형, 4족, 휴머노이드 등) |

## 🖥️ 소프트웨어 요구 사항
- **Ubuntu 22.04** (22.04에서 테스트됨, 24.04 지원 예정)
- **Docker** — [Docker 설치](https://docs.docker.com/engine/install/)

## ✨ DDDMR Navigation with Gazebo

이 데모는 Gazebo에서 4족 로봇과 함께 DDDMR 내비게이션 스택을 실행하는 방법을 보여줍니다.
- 환경 준비를 위한 필요한 이미지를 빌드합니다.
- 두 개의 터미널에서 시스템을 실행합니다 — 하나는 Gazebo 월드용, 다른 하나는 내비게이션 스택용입니다.

### 1. Docker 이미지 생성

저장소를 복제하고 ./build.bash를 실행하세요. **`x64_gz`**를 선택하세요. 내비게이션과 Gazebo에 필요한 모든 구성 요소가 포함되어 있습니다.

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

### 2. 내비게이션 맵 다운로드

Gazebo에서 dddmr_navigation을 실행하려면 데모용 내비게이션 맵(12.3MB)을 다운로드해야 합니다.

```bash
cd ~ && mkdir dddmr_bags
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./download_files.bash
```

### 3. 데모 환경 준비

#### 두 개의 Docker 컨테이너 생성 (Gazebo 및 내비게이션)

> [!IMPORTANT] 
> 다음 명령은 빌드한 이미지를 사용하여 두 개의 대화형 Docker 컨테이너를 시작합니다. 데모 환경을 준비하기 위해 **두 개의 별도 터미널**을 여세요.

#### 🖥️ 터미널 1: Go2 Gazebo 시뮬레이션 시작

첫 번째 터미널에서 [Gazebo 가이드 — Go2](../GAZEBO_GUIDE.md#go2--quadruped)의 두 단계를 따라 시뮬레이션 이미지를 빌드하고 Gazebo 컨테이너를 시작하세요.

Go2 시뮬레이션이 실행되면 이 터미널을 열린 상태로 유지하고, 여기로 돌아와 아래의 **터미널 2** 단계를 진행하세요.

#### 🖥️ 터미널 2 (내비게이션 시스템 컨테이너 생성)

- ##### 단계 1 (호스트에서): 내비게이션 시스템 컨테이너 생성
```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./run_x64_navigation.bash
```

- ##### 단계 2 (내비게이션 컨테이너 내부): 빌드 및 실행
```bash
cd dddmr_navigation/ && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash && ros2 launch p2p_move_base go2_localization.launch
```

### 4. 데모 실행

- Gazebo 데모에서는 맵이 이미 정렬되어 있으므로, 직접 매핑하지 않는 한 초기 포즈를 설정할 필요가 없습니다
- RViz에서 목표점(3D Goal Pose)을 지정하면 로봇이 목표 지점으로 이동합니다

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/give_goal_in_demo_.png" width="920" height="460"/>
</p>

### 5. 알려진 문제

> [!WARNING]
> 다음은 현재 관찰된 동작이며, 조사 중이고 향후 업데이트에서 수정될 예정입니다.
> - Gazebo: 경사면에서 가끔 미끄러짐
> - 매핑: 중복된 바닥 레이어

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
