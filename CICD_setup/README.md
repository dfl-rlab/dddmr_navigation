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
SLAM->
MCL->
## CD Setup
