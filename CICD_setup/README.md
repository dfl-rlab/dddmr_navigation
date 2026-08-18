# dddmr CICD Setup

## CI Setup
### Data-driven regression testing:
| Stage | Description |
| :--- | :--- |
| **Trigger** | GitHub Actions fires on `push` / `pull_request`. |
| **Environment** | Spins up a container or runner with ROS 2 and your package dependencies. |
| **Execution** | Plays back a reproducible dataset (the bag file) through implementation. |
| **Verification** | Compares the generated result against a verified results |

### Example:

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/CICD_setup/slam_ci.png" width="920" height="460"/>
</p>

### Download Bags
These bag files come directly from user-reported issues—the DDDMR Navigation Stack couldn't grow without your support and contributions!
```
cd ~/dddmr_navigation/CICD_setup/
./download_CI_bags.bash
```

### DDDMR LEGO LOAM CI
Following example launch the test of Lego Loam using Airy tilted 45 degree.
```
docker exec -it dddmr_humble_cuda_dev bash
cd dddmr_navigation && source install/setup.bash
launch_test src/dddmr_lego_loam/lego_loam_bor/test/test/mapping_airy_t45_test.py enable_rviz:=true
```
This will launch a test with Rviz2 showing up.

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/CICD_setup/lego_loam_ci.gif" width="640" height="384"/>
</p>

Once the bag finishes playing, you will see the following result showing the test is ok, since we are doing ICP of the result to a verified result, the ICP score should be very small.
```
...
[INFO] [lego_loam_bag-2]: process has finished cleanly [pid 576]
[mapping_test_node-4] [INFO] [1785921408.903531535] [mapping_airy_t45]: Changing state to ANALYZE_MAPPING_RESULT
[mapping_test_node-4] [INFO] [1785921409.487136630] [mapping_airy_t45]: ICP Score: 0.01
[mapping_test_node-4] [INFO] [1785921409.593369385] [mapping_airy_t45]: Changing state to SUCCEED
[mapping_test_node-4] [INFO] [1785921409.702251627] [mapping_airy_t45]: Mapping test SUCCEEDED! ICP score: 0.01
[mapping_test_node-4] DoneSuccess
ok

----------------------------------------------------------------------
Ran 1 test in 67.376s

OK
[INFO] [mapping_test_node-4]: sending signal 'SIGINT' to process[mapping_test_node-4]
[INFO] [static_transform_publisher-1]: sending signal 'SIGINT' to process[static_transform_publisher-1]
[static_transform_publisher-1] [INFO] [1785921409.797840983] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ERROR] [mapping_test_node-4]: process has died [pid 657, exit code -2, cmd '/root/dddmr_navigation/install/lego_loam_bor/lib/lego_loam_bor/mapping_test_node --ros-args -r __node:=mapping_airy_t45'].
[INFO] [static_transform_publisher-1]: process has finished cleanly [pid 574]
test_assertion_message (mapping_airy_t45_test.TestStdOutput) ... ok
test_lego_loam_bag_node_exit_code (mapping_airy_t45_test.TestStdOutput) ... ok

----------------------------------------------------------------------
Ran 2 tests in 0.000s

OK

```

> [!NOTE]
> You can find a list of test [here](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_lego_loam/lego_loam_bor/test/test)

### DDDMR MCL 3DL CI
Following example launch the test of mcl 3dl using Leishen C16
```
docker exec -it dddmr_humble_cuda_dev bash
cd dddmr_navigation && source install/setup.bash
launch_test src/dddmr_mcl_3dl/test/test/mcl_3dl_c16_test.py enable_rviz:=true
```

## CD Setup
