# Ackermann Steering Simulation

<div align="center">
<table align="center">
  <tr>
    <td align="center"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/local_planner/trajectory/dddmr_ackermann_gazebo.gif?raw=true" alt="DDDMR Ackermann navigation in Gazebo" width="480"/><br/>Gazebo</td>
    <td align="center"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/local_planner/trajectory/dddmr_ackermann_in_a2rl.gif?raw=true" alt="DDDMR Ackermann navigation in A2RL simulation" width="480"/><br/>A2RL Simulation</td>
  </tr>
</table>
</div>

DDDMR has been tested with Ackermann vehicles in Gazebo, using the publicly available Saye simulator, and in an A2RL simulation environment used for internal testing. These examples provide a practical reference for exploring navigation with different vehicle models and simulation environments.

## Ackermann Navigation with Gazebo

This tutorial demonstrates how to run the DDDMR Navigation Stack's Ackermann point-to-point (P2P) test in Gazebo:

- Download the navigation map that matches the Ackermann Gazebo environment.
- Prepare two Docker containers: one for Gazebo and one for navigation.

### 1. Download the Navigation Map

Run the download script on the host to prepare the map for the Ackermann test:

```bash
mkdir -p ~/dddmr_bags
cd ~/dddmr_navigation/CICD_setup && ./download_CI_bags.bash
```

The script offers several datasets. To download only the Ackermann test data, answer `N` to the other prompts and `Y` when this prompt appears:

> `Do you want to download nav_ackermann_p2p (Y/N):` → **Press `Y`**

### 2. Start the Ackermann Gazebo Simulation

Follow the **Saye — Ackermann Steering** section of the [DDDMR Navigation Gazebo Guide](../../dddmr_beginner_guide/GAZEBO_GUIDE.md#saye--ackermann-steering) to prepare the simulation environment. Keep Gazebo running in its own terminal before starting the navigation test.

### 3. Run the Navigation Test

Open another terminal to run the navigation stack. If you already have a DDDMR development container, enter that container and skip the container creation step below. Otherwise, first prepare the `dddmr:humble` image using the [DDDMR Docker setup](../../../dddmr_docker/README.md), then run these commands on the host:

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file/
./run_x64.bash
```

You should now be inside the `dddmr_humble` container. Make sure the navigation test map is available at :

```text
ls /root/dddmr_bags/cicdtest/nav_ackermann_p2p/pg
```

 Use the same `ROS_DOMAIN_ID` for the simulation and navigation environments so their ROS nodes can communicate (ackermann gazebo use 14).
```text
export ROS_DOMAIN_ID=14
```

Compile the navigation stack and run the test:

```bash
cd ~/dddmr_navigation
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
launch_test src/dddmr_p2p_move_base/test/test/nav_ackermann_p2p_test.py enable_rviz:=true
```

The test waits for the configured startup delay and navigation readiness, then sends the three goals in order. Each successful move-base action advances to the next goal. Goal coordinates, startup delay, total timeout, and the result file are configured in [nav_ackermann_p2p_test.yaml](../test/config/nav_ackermann_p2p_test.yaml).

At completion, the test prints a summary and saves the target poses, measured poses, and test results to `/tmp/nav_ackermann_p2p_results.csv` inside the container.
