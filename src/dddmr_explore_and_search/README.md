# dddmr_explore_and_search

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_explore_and_search/explore.gif" width="640" height="320"/><p align='center'>Explore and Navigation</p>
</p>

This package demonstrate a random exploring strategy using the ground explored edge.
The structure is comprised of:
1. SLAM
2. Navigation and obstacle avoidance
3. Randomly exploring strategy

### Step 1:
Launch mapping and navigation, you should have lidar and odometry topic being published and a TF from lidar to your odom child frame.
Execute into the docker container, and run
```
cd /root/dddmr_navigation
source install/setup.bash
ros2 launch dddmr_explore_and_search dddmr_explore_and_search.launch
```

### Step2:
Launch explore node which randomly chose a edge point as goal and send it to p2p move base.
Execute into the docker container, and run
```
cd /root/dddmr_navigation
source install/setup.bash
ros2 launch dddmr_explore_and_search dddmr_explore_and_search.launch
```

> [!NOTE]
> A smart exploring strategy is under development. However, [dddmr_explore_and_search](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_explore_and_search/src/dddmr_explore_and_search.cpp) demonstrates a key structure to interface our dddmr_lego_loam in realtime. 
