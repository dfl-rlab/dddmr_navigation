# DDDMR Navigation Gazebo Guide

Explore DDDMR navigation with different robot models in Gazebo. Use separate terminals for the simulation and navigation stack.

> [!IMPORTANT]
> The Gazebo container and the navigation container must use the same `ROS_DOMAIN_ID` so their ROS nodes can communicate. Check both containers before launching simulation and navigation.

| Robot | Model | Guide |
|:------|:------|:------|
| Go2 | Quadruped with a conventional walking gait | [Go2 simulation](#go2--quadruped) |
| Zinger | Four-wheel steering (4WS), using the omni drive model | [Zinger simulation](#zinger--four-wheel-steering) |
| Saye | Ackermann steering | [Saye simulation](#saye--ackermann-steering) |

## Software Requirements

- **Ubuntu 22.04** (the existing Go2 guide was tested on 22.04).
- **Docker** — see [Install Docker Engine](https://docs.docker.com/engine/install/).

## Go2 — Quadruped

Build the simulation using the `go2` branch of [dfl_mobilerobot_simulator](https://github.com/dfl-rlab/dfl_mobilerobot_simulator/tree/go2), based on [unitree-go2-ros2](https://github.com/anujjain-dev/unitree-go2-ros2) for ROS 2 Humble and Gazebo Classic.

### Step 1 (on host): Build the Simulation Image

```bash
cd ~
git clone --branch go2 https://github.com/dfl-rlab/dfl_mobilerobot_simulator.git
cd dfl_mobilerobot_simulator/docker
docker build -t dddmr_gz:humble -f Dockerfile_x64_gazebo .
```

### Step 2 (on host): Start the Simulation

```bash
cd ~/dfl_mobilerobot_simulator/docker
./run_x64_gazebo.bash
```

The script starts the Go2 simulation in the `dddmr_humble_gazebo` container using the workspace compiled into the image. Keep this terminal open, then continue with **Terminal 2** in the [beginner guide](README.md#-dddmr-navigation-with-gazebo).

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
