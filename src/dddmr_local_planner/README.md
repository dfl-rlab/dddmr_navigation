# 🤖 dddmr_local_planner

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C.svg)](https://isocpp.org/)
[![License](https://img.shields.io/badge/License-BSD--3--Clause-green.svg)](LICENSE)

`dddmr_local_planner` is a modular, plugin-based local planner tailored for **3D autonomous mobile robot navigation**. Inspired by Nav2's DWB planner, it extends navigation capabilities into 3D environments with advanced motor constraints and multi-generator flexibility.

---

## ✨ Key Features

- 🔌 **Plugin-Based Architecture**: Modular support for custom:
  - **Trajectory Generators**
  - **Critics**
  - **Recovery Behaviors**
- 🧊 **True 3D Navigation & Collision Checking**: Computes trajectory rating and collision checks natively in 3D (point cloud verification inside 3D bounding cuboids vs. traditional 2D polygon checks).
- ⚖️ **Flexible Per-Generator Weighting**: Each trajectory generator supports independent critics and weight profiles, enabling customizable robot behavior modes.
- ⚙️ **Motor Constraint-Aware Generation**: `dd_simple_trajectory_generator_theory` models actual physical motor RPM limits (e.g. differential drive maximum motor output constraints) during trajectory generation to ensure velocity commands remain dynamically executable.
  > *Example*: When the maximum linear speed matches the motor's maximum capability (e.g., 100 RPM ≈ 1 m/s), attempting to add an angular velocity (e.g., 0.1 rad/s) would exceed the maximum RPM on the outer wheel. Motor constraint awareness dynamically scales trajectories to maximize motor output within physical bounds.

<p align="center">
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/local_planner/local_planner_play_ground_annotated.png" width="720" height="420" alt="Local Planner Playground Annotated"/>
</p>

---

## 🚀 Quick Start & Demo

### 1. Build Docker Image
The package runs inside Docker. Images are supported for **x86_64** (Intel NUC) and **ARM64** (NVIDIA Jetson JetPack 6).

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

### 2. Run Interactive Container & Launch Demo

> [!NOTE]
> The command below starts an interactive Docker container environment pre-configured for the local planner demo.

```bash
cd ~/dddmr_navigation/dddmr_docker && ./run_demo.bash
```

Inside the container, compile and launch the playground:

```bash
cd ~/dddmr_navigation
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch local_planner local_planner_play_ground.launch
```

### 3. Interactive Goal Setting in RViz2
Use the **Publish Point** tool in RViz2 to set a navigation goal. The local planner will immediately generate and prune plan trajectories toward the specified target.

<p align="center">
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/local_planner/local_planner_play_ground.gif" width="700" height="440" alt="Local Planner RViz Demo"/>
</p>

---

## 📋 Release Notes

### 📦 v2.0.0

#### ⚡ Actuator Type Explicit Definition
- Added mandatory `actuator_type_` definition in `TrajectoryGeneratorTheory`.
- Trajectory generators must specify either `dddmr_sys_core::ActuatorType::MOTOR` or `dddmr_sys_core::ActuatorType::STEERING` so `move_base` accurately determines the output command type (`Twist` vs. `Ackermann`).

#### 🛠️ TrajectoryGeneratorTheory Restructuring & API Cleanup
- **Naming Corrections:**
  - Renamed `addPoint()` → `addPoseCuboid()`
  - Renamed `resetPoints()` → `resetPoses()`
- **Deprecations & Removal:**
  - Removed `virtual bool hasMoreTrajectories()`
  - Removed `virtual bool nextTrajectory(base_trajectory::Trajectory& _traj)`
- **OpenMP Parallelization Support:**
  - Introduced `virtual size_t getSamplingSize()`
  - Introduced `virtual void getSamplingTrajectoryByIndex(size_t index, base_trajectory::Trajectory& _traj)` to enable index-based sampling for parallel trajectory processing using OpenMP.
- **MPPI Integration:**
  - Added `expertScoring(...)` interface to support Model Predictive Path Integral (MPPI) control strategies:

```cpp
void expertScoring(
    std::vector<base_trajectory::Trajectory>& accepted_trajectories,
    std::map<std::string, std::vector<base_trajectory::Trajectory>>& rejected_trajectories,
    base_trajectory::Trajectory& best_traj
) override;
```