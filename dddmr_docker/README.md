# DDDMR Docker Environments

This directory provides containerized development environments for the DDDMR Navigation platform across x86_64 host systems and NVIDIA Jetson (L4T) platforms.

The environments are split into two generations:
- **`docker_file_2404/`**: Ubuntu 24.04 LTS with **ROS 2 Jazzy Jalisco**, CUDA 13.x toolchains, and modern JetPack **L4T R39** support.
- **`docker_file/`**: Ubuntu 22.04 LTS with **ROS 2 Humble Hawksbill**, CUDA 12.x toolchains, and JetPack **L4T R36** support.

---

## Key Components & Version Matrix

| Component | `jazzy` | `jazzy-cuda` | `l4t_r39` | `humble` | `humble-cuda` | `l4t_r36` |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Base OS / Image** | `ubuntu:24.04` | `dddmr:jazzy` | `whitesscott/l4t-jetpack:r39.2.1` | `ubuntu:22.04` | `dddmr:humble` | `nvcr.io/nvidia/l4t-jetpack:r36.4.0` |
| **ROS 2 Distro** | Jazzy | Jazzy | Jazzy | Humble | Humble | Humble |
| **Compiler** | GCC / G++ 13 | GCC / G++ 13 | GCC / G++ 13 | GCC / G++ 13 | GCC / G++ 13 | GCC / G++ 13 |
| **CMake** | System | System | 3.25.3 | System | System | 3.25.3 |
| **CUDA Toolkit** | — | 13.3.1 (`cuda-toolkit-13-3`) | 13.2 compatible | — | 12.6.3 (`cuda-toolkit-12-6`) | JetPack 6.1 (CUDA 12.6) |
| **cuDNN** | — | 9.25.0 | JetPack / System | — | 9.6.0 | 9.13.1 (`cudnn9-cuda-12`) |
| **PyTorch** | — | 2.11.0 | 2.14.1+cu132 | — | 2.8.0 | 2.8.0 (Jetson AI Lab cu126) |
| **TorchVision** | — | 0.28.0 | 0.29.1+cu132 | — | 0.23.0 | Compatible Jetson AI Lab wheel |
| **TorchAudio** | — | 2.11.0 | 2.11.0.dev20261013+cu132 | — | 2.8.0 | Compatible Jetson AI Lab wheel |
| **PyCUDA** | — | 2026.1 | 2026.1 | — | 2024.1.2 | 2025.1.2 |
| **TensorRT (C++)** | — | 10.16.0 | JetPack included | — | 10.7.0 | JetPack included |
| **TensorRT (Python)**| — | 11.1.0.106 | — | — | 10.7.0 | — |
| **ONNX** | — | 1.21.0 | 1.18.0 | — | 1.17.0 | 1.17.0 |
| **OpenCV** | System 4.x | Source 4.x (`CUDA`, `CUBLAS`) | Source 4.x (`CUDA`, `CUBLAS`) | System 4.x | Source 4.x (`CUDA`, `CUBLAS`) | 4.11.0 (`CUDA`, `CUBLAS`) |
| **OpenCV CUDA Arch** | — | `7.5 8.0 8.6 8.7 8.9 9.0 12.0` (PTX `12.0`) | `8.7` (Jetson Orin) | — | `7.0 7.5 8.0 8.6 8.7 8.9 9.0` (PTX `8.9`) | `8.7` (Jetson Orin) |
| **GTSAM** | 4.2a9 | 4.2a9 | 4.2a9 | 4.2a9 | 4.2a9 | 4.2a9 |
| **PCL** | 1.15.0 | 1.15.0 | 1.15.0 | 1.15.0 | 1.15.0 | 1.15.0 |
| **Small GICP** | v1.0.1 | v1.0.1 | v1.0.1 | v1.0.1 | v1.0.1 | v1.0.1 |

---

## Directory Overview

```
dddmr_docker/
├── docker_file_2404/                # Ubuntu 24.04 & ROS 2 Jazzy
│   ├── Dockerfile_x64              # Base x86_64 image (dddmr:jazzy)
│   ├── Dockerfile_x64_cuda         # x86_64 image with CUDA 13.3 & TensorRT (dddmr:jazzy-cuda)
│   ├── Dockerfile_x64_l4t_r39      # Jetson Orin L4T R39 JetPack image (dddmr:l4t_r39)
│   ├── bash/                       # Build scripts for gtsam, pcl, libuvc
│   ├── pkgs/                       # Offline package bundles (e.g. serial-foxy)
│   ├── build.bash                  # Interactive build menu for 24.04 images
│   ├── run_x64.bash                # Launch script for dddmr:jazzy
│   ├── run_x64_gpu.bash            # Launch script for dddmr:jazzy-cuda or dddmr:l4t_r39
│   └── test/                       # CUDA verification test suite
│       ├── test_cuda.py            # Validates PyTorch and OpenCV CUDA availability
│       ├── test_cuda.bash          # Runs test_cuda.py inside dddmr:jazzy-cuda
│       └── test_cuda_l4t.bash      # Runs test_cuda.py inside dddmr:l4t_r39
│
├── docker_file/                     # Ubuntu 22.04 & ROS 2 Humble
│   ├── Dockerfile_x64              # Base x86_64 image (dddmr:humble)
│   ├── Dockerfile_x64_cuda         # x86_64 image with CUDA 12.6 & TensorRT (dddmr:humble-cuda)
│   ├── Dockerfile_x64_gazebo       # Gazebo simulation extension (dddmr_gz:humble)
│   ├── Dockerfile_x64_l4t_r36      # Jetson JetPack r36.4.0 image (dddmr:l4t_r36)
│   ├── bash/                       # Build scripts for dependencies
│   ├── build.bash                  # Interactive build menu for 22.04 images
│   ├── run_x64.bash / run_x64_gpu.bash
│   └── test/                       # CUDA verification suite for Humble & L4T R36
│       ├── test_cuda.py
│       ├── test_cuda.bash
│       └── test_cuda_l4t.bash
│
├── rtps_profile/                    # FastDDS / CycloneDDS network QoS configs
├── system_setup/                    # Host-level USB and system configuration scripts
├── run_demo.bash                    # Root runner script for Humble containers
└── README.md
```

---

## Newly Added Features

### 1. ROS 2 Jazzy & Ubuntu 24.04 Environment (`docker_file_2404/`)
- **Modern Linux Toolchain**: Runs Ubuntu 24.04 LTS with ROS 2 Jazzy Jalisco desktop. Upgrades default compiler to **GCC / G++ 13**.
- **CUDA 13.3 & Deep Learning Stack** (`Dockerfile_x64_cuda`):
  - NVIDIA CUDA Toolkit **13.3.1** and cuDNN **9.25.0**.
  - PyTorch **2.11.0**, TorchVision **0.28.0**, and PyCUDA **2026.1**.
  - NVIDIA TensorRT **10.16.0** C++ libraries and Python bindings **11.1.0**.
  - OpenCV compiled from source with CUDA backend, CUBLAS, and multi-architecture SASS/PTX support (Turing, Ampere, Ada, Hopper, and future architectures up to compute capability 12.0).
- **Jetson L4T R39 Support** (`Dockerfile_x64_l4t_r39`):
  - Uses `whitesscott/l4t-jetpack:r39.2.1` base image to enable ROS 2 Jazzy on NVIDIA Jetson Orin platforms under Ubuntu 24.04.
  - Custom-compiled OpenCV with CUDA compute capability `8.7`.
  - Bundles ROS serial communications (`serial-foxy`) and apriltag support.

### 2. CUDA Verification Test Suite (`test/`)
Automated scripts are included to guarantee GPU acceleration is functional across containers:
- **`test_cuda.py`**:
  - **PyTorch Check**: Asserts `torch.cuda.is_available() == True` and prints detected CUDA version.
  - **OpenCV Check**: Asserts OpenCV build information contains `CUDA`, checks for `cv2.cuda`, and verifies `cv2.cuda.getCudaEnabledDeviceCount() > 0`.
- **`test_cuda.bash`**:
  - Launches headless testing in `dddmr:jazzy-cuda` (or `dddmr:humble-cuda`) passing `--gpus=all` and NVIDIA driver capabilities. CI/CD and GitHub Actions ready (does not require an interactive TTY).
- **`test_cuda_l4t.bash`**:
  - Launches headless testing in `dddmr:l4t_r39` (or `dddmr:l4t_r36`) using the Jetson NVIDIA container runtime (`--runtime=nvidia`).

---

## How to Build and Run

### 1. Building the Docker Images
Run `build.bash` in the desired version directory:

**For ROS 2 Jazzy / Ubuntu 24.04:**
```bash
cd dddmr_docker/docker_file_2404
./build.bash
# Follow the interactive prompts:
# - Type 'x64' and 'N' for CPU-only (dddmr:jazzy)
# - Type 'x64' and 'Y' for CUDA acceleration (dddmr:jazzy-cuda)
# - Type 'l4t' for Jetson L4T R39 image (dddmr:l4t_r39)
```

**For ROS 2 Humble / Ubuntu 22.04:**
```bash
cd dddmr_docker/docker_file
./build.bash
# Follow prompts to build dddmr:humble, dddmr:humble-cuda, dddmr:l4t_r36, or dddmr_gz:humble
```

### 2. Verifying CUDA Enablement
To test that CUDA is correctly recognized by both PyTorch and OpenCV:

**On x86_64 GPU Workstations:**
```bash
cd dddmr_docker/docker_file_2404/test
./test_cuda.bash
```

**On NVIDIA Jetson Devices:**
```bash
cd dddmr_docker/docker_file_2404/test
./test_cuda_l4t.bash
```

### 3. Running Containers
Launch container with X11 display forwarding, device nodes, and volume mounts (`~/dddmr_navigation` and `~/dddmr_bags`):

```bash
# In docker_file_2404:
./run_x64_gpu.bash   # Prompts for ROS_DOMAIN_ID and launches dddmr:jazzy-cuda or dddmr:l4t_r39
./run_x64.bash       # Launches CPU-only dddmr:jazzy
```

### 4. Build Shortcuts Inside Containers
Containers come with convenient pre-configured shell alias functions in `~/.bashrc`:
- `ws_build`: Builds workspace in Release mode (`colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`).
- `ws_build_wi_test`: Builds workspace with tests enabled and TensorRT disabled (`-DTRT_ENABLED=OFF -DTEST_ENABLED=ON`).
- `ws_build_wi_test_trt`: Builds workspace with both tests and TensorRT enabled (`-DTRT_ENABLED=ON -DTEST_ENABLED=ON`).
- `ws_build_wi_test_trt_single_thread`: (Jetson only) Serialized single-thread build to prevent out-of-memory errors on embedded boards.
