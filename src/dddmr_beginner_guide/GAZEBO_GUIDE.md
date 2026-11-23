# DDDMR Navigation Gazebo Guide

Explore DDDMR navigation with different robot models in Gazebo. Use separate terminals for the simulation and navigation stack.

> [!IMPORTANT]
> The Gazebo container and the navigation container must use the same `ROS_DOMAIN_ID` so their ROS nodes can communicate. Check both containers before launching simulation and navigation.

| Robot | Model | Guide |
|:------|:------|:------|
| Lite3 | Quadruped with an RL locomotion policy | [Lite3 simulation](#lite3--rl-quadruped) |
| Zinger | Four-wheel steering (4WS), using the omni drive model | [Zinger simulation](#zinger--four-wheel-steering) |
| Saye | Ackermann steering | [Saye simulation](#saye--ackermann-steering) |

## Software Requirements

- **Ubuntu 22.04** (tested host platform).
- **Docker** — see [Install Docker Engine](https://docs.docker.com/engine/install/).

## Lite3 — RL Quadruped

Use the `rl_lite3` branch of [dfl_mobilerobot_simulator](https://github.com/dfl-rlab/dfl_mobilerobot_simulator/tree/rl_lite3). It provides ROS 2 Humble / Gazebo Classic, the Lite3 HIMLoco RL policy, dome LiDAR, odometry and a multi-level terrain. See the [simulator README](https://github.com/dfl-rlab/dfl_mobilerobot_simulator/blob/rl_lite3/README.md) for its full instructions.

### Step 1 (on host): Build the Simulation Image

```bash
cd ~
git clone --branch rl_lite3 \
  https://github.com/dfl-rlab/dfl_mobilerobot_simulator.git
cd dfl_mobilerobot_simulator
git submodule update --init --recursive
cd docker
docker build -t dddmr_simulation:rl_lite3 -f Dockerfile ..
```

### Step 2 (on host): Start the GPU Simulation

Run from a graphical desktop. NVIDIA GPUs require NVIDIA Container Toolkit.

```bash
cd ~/dfl_mobilerobot_simulator/docker
./rl_lite3_gpu.bash
```

This creates the `rl_lite3` container using `dddmr_simulation:rl_lite3` and starts Gazebo, Lite3, its policy controller, dome LiDAR and odometry. The launcher sets **`ROS_DOMAIN_ID=22`** and loads the included Fast DDS transport profile for the point cloud. Use domain **22** in the navigation container too.

### Step 3: Enable the RL Policy and Navigation Commands

After the robot drops onto the ground, keep the simulation terminal focused and press **`r` → `0` → `1` → `n`**. These steps reset the pose, stand the robot up, start the RL controller and enable navigation commands. **Wait at least one second between keys; after `0`, also wait until standing up is complete.**

| Key | Action |
|:----|:-------|
| `r` | Reset the robot to its starting pose in the world. |
| `0` | Stand up and prepare the robot for locomotion. |
| `1` | Activate the RL locomotion policy. |
| `n` | Enable `/cmd_vel` input from navigation. |

Keep this terminal open and continue with **Terminal 2** in the [beginner guide](README.md#-dddmr-navigation-with-gazebo). Press Ctrl-C in the simulation terminal when you want to stop the simulator.

## Zinger — Four-Wheel Steering

This simulation uses a four-wheel steering robot with the omni drive model. The startup steps come from the [Omni Drive tutorial](../dddmr_p2p_move_base/kinematics_md/OMNIDIRECTION.md).

### Bring Up the Zinger Robot

```bash
cd ~/dddmr_navigation/src/dddmr_p2p_move_base/simulation/
./zinger_bring_up.bash
```

This starts the simulated 4WS robot in a Docker container. Continue with **Run Navigation Stack** in the [Omni Drive tutorial](../dddmr_p2p_move_base/kinematics_md/OMNIDIRECTION.md#run-navigation-stack).

The existing Zinger setup uses `ROS_DOMAIN_ID=12`. It can be changed in [zinger_bring_up.bash](../dddmr_p2p_move_base/simulation/zinger_bring_up.bash); use the same domain ID in the navigation environment.

## Saye — Ackermann Steering

### 1. Build the Simulation Image

Prepare the Saye Gazebo simulation image using the `ackermann` branch of [dfl_mobilerobot_simulator](https://github.com/dfl-rlab/dfl_mobilerobot_simulator/tree/ackermann). Its Dockerfile clones the [original simulation](https://github.com/alitekes1/ackermann-vehicle-gzsim-ros2) unchanged and builds it alongside the plugin in the same workspace. The plugin adds the joint-state bridge and Ackermann feedback node through a separate launch file.

Run these commands on the host:

```bash
cd ~
git clone --branch ackermann https://github.com/dfl-rlab/dfl_mobilerobot_simulator.git
cd dfl_mobilerobot_simulator/docker
docker build -t dddmr_simulation:ackermann -f Dockerfile_po_builtin .
```

### 2. Start the Saye Gazebo Simulation

On the host, run `ackermann_bring_up.bash` from the `dfl_mobilerobot_simulator` repository's `docker` directory:

```bash
cd ~/dfl_mobilerobot_simulator/docker
./ackermann_bring_up.bash
```

The script uses the `dddmr_simulation:ackermann` image to create the `dddmr_ackermann` container and automatically launch the Saye robot and Gazebo world. Keep this terminal open while navigating.

Like the Zinger launcher, the script sets a fixed ROS domain ID; the Ackermann launcher uses `ROS_DOMAIN_ID=14`. Set the same value **inside the navigation container** before running the navigation test:

```bash
export ROS_DOMAIN_ID=14
```

Once Gazebo is running, continue with the [Ackermann P2P navigation tutorial](../dddmr_p2p_move_base/kinematics_md/ACKERMANN.md#3-run-the-navigation-test).
