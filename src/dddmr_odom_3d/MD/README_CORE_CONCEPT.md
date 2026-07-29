# DDDMR ODOM CORE CONCEPT
This README details the core odometry concepts used in DDDMR navigation. A broader term for this is **State Estimation**, which describes the location and velocity of a robot within its body frame.

### Coordinate Definition
The standard odometry coordinate used in DDDMR for a robot moving in a 3D world is defined as:
* x: The forward/backward translation of the robot in a given reference frame. (Red color in Rviz)
* y: The lateral (sideways / strafing) translation of the robot. (Green color in Rviz)
* z: The direction is pointing to sky. (Blue color in Rviz)
* $\theta$ (Yaw): The heading, or angular orientation, of the robot, rotate around z axes.

Following the above definition, suppose a robot moves forward 1 meter, turns 90 degrees to the left, and then moves forward another 1 meter. The final result in the **odom** frame should be xyz(1, 1, 0) and RPY(0, 0, 1.5708). 

In addition, be aware of **base_link** frame, the z axis should be point to sky as well.
<table align='center'>
  <tr width="40%">
    <td width="40%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/odom_3d/odom_coordinate_definition_correct.png" width="400" height="260"/><p align='center'>Correct odom setup.</p></td>
    <td width="40%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/odom_3d/odom_coordinate_definition_incorrect.png" width="400" height="260"/><p align='center'>Wrong odom setup.</p></td>
  </tr>
</table>

> [!NOTE]
> There are plenty of odometry/state estimation packages that can provide odometry data to the robot.
> To incorporate those packages into DDDMR, simply convert their outputs to follow the DDDMR coordinate definition.

### Accuracy Requirement
DDDMR navigation stack relies on odometry heavily across mutiple packages. DDDMR expects 10% less estimation error without any huge jump. A slowly drift is accetable but sudden jump is devastating. 
See following time series odometry data as the example:
<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/odom_3d/drifting_explanation.png" width="848" height="311"/>
</p>

> [!NOTE]
>For example, if the robot move 1 meter but your odometry only shows 0.9 meter, it is ok!

For a slowly drifting odometry, both LEGO LOAM BOR and MCL 3DL can handle the drift and correct the results.

1. LEGO LOAM BOR:
  * Pre integrate the lidar scan with the odometry (using t and t-1 observation), therefore, a huge jump will make LO optimization fail.
2. MCL 3DL:
  * Generate particles based on the odometry (using t and t-1 observation), therefore, a huge jump will generate a huge covariance of pose guessing.

### Quadruped/Humanoid Robot Odometry
dddmr_odom_3d package can be used to compute a rough odometry in 3D space:
* dddmr_odm_3d node subscribes 2D odometry and IMU to fuse a 3D odometry. The velocities of 2D odometry topic are used, and the orientation estimation from IMU topic is used. Therefore, a key point is that the velocity estimation should be accurate. The issue usually occurs in the velocities estimation, IMU is pretty much stable across different platforms. 
* The above concept is used in Lite3 platform and it works well. We notice the position estimation in the odometry is not accurate, but the speed estimation is acceptable, therefore, the integrated result is good enough.
<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/odom_3d/dddmr_odom_3d.png" width="803" height="351"/>
</p>

> [!NOTE]
> The orientation from the IMU requires conversion if it does not follow DDDMR coordinate definition
