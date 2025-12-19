# Module 3 Quickstart Guide: The AI-Robot Brain (NVIDIA Isaac™)

## Overview

This quickstart guide provides a step-by-step process to get your AI-Robot Brain up and running with NVIDIA Isaac technology stack. Follow these steps to establish a complete pipeline from simulation to autonomous navigation.

## Prerequisites

### Hardware Requirements
- **GPU**: NVIDIA RTX GPU (Compute Capability ≥ 6.0)
- **Memory**: 16GB+ RAM recommended
- **Storage**: 20GB+ free space for Isaac Sim and assets
- **OS**: Ubuntu 22.04 LTS

### Software Requirements
- **CUDA**: 12.x toolkit
- **ROS 2**: Humble Hawksbill
- **Isaac Sim**: 2023.1 or newer
- **Isaac ROS**: Compatible packages installed

## Step 1: Environment Setup

### Install NVIDIA Drivers and CUDA
```bash
# Install NVIDIA drivers
sudo apt update
sudo apt install nvidia-driver-535

# Install CUDA 12.x
wget https://developer.download.nvidia.com/compute/cuda/12.3.0/local_installers/cuda_12.3.0_545.23.06_linux.run
sudo sh cuda_12.3.0_545.23.06_linux.run

# Set environment variables
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

### Install Isaac Sim
```bash
# Download and install Isaac Sim 2023.1+
# Follow installation guide in docs/module-03-isaac-brain/01-omniverse-isaac.md
```

### Install ROS 2 Humble
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2

# Source ROS 2
source /opt/ros/humble/setup.bash
```

## Step 2: Install Isaac ROS Packages

```bash
# Install Isaac ROS packages
sudo apt install ros-humble-isaac-ros-common
sudo apt install ros-humble-isaac-ros-visual-slam
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## Step 3: Set Up the AI-Robot Brain

### Clone and Build Workspace
```bash
# Create workspace
mkdir -p ~/isaac_ws/src
cd ~/isaac_ws

# Build workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Launch Isaac Sim Environment
```bash
# Launch Isaac Sim with your robot configuration
# Detailed instructions in docs/module-03-isaac-brain/01-omniverse-isaac.md
```

## Step 4: Synthetic Data Generation

### Configure Replicator
```bash
# Navigate to the synthetic data directory
cd simulation-assets/isaac-sim/

# Run the RGB generation script
python3 rgb_generation_script.py

# Run the segmentation generation script
python3 segmentation_generation_script.py

# Validate the generated dataset
python3 synthetic_data_qa_script.py --dataset_path output/synthetic_data
```

### Generate Example Dataset
```bash
# Create an example dataset for testing
python3 example_synthetic_dataset.py
```

## Step 5: Visual SLAM Setup

### Launch VSLAM Pipeline
```bash
# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source ~/isaac_ws/install/setup.bash

# Launch VSLAM with Isaac ROS GEMs
# Use configuration from docs/module-03-isaac-brain/03-isaac-ros-vslam.md
```

### Verify Loop Closure
```bash
# Run loop closure verification
python3 loop_closure_verification.py

# Monitor VSLAM performance
python3 vslam_benchmarking_tools.py
```

## Step 6: Navigation Configuration

### Launch Nav2 Stack
```bash
# Source environment
source /opt/ros/humble/setup.bash
source ~/isaac_ws/install/setup.bash

# Launch Nav2 with Isaac Sim integration
python3 isaac_sim_nav2_integration.py

# Load humanoid-specific costmap parameters
ros2 param load nav2_controller local_costmap.local_costmap ros__parameters --use-file simulation-assets/isaac-sim/costmap_params_humanoid.yaml
```

### Test Navigation
```bash
# Send a simple navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}"
```

## Step 7: Complete Integration Test

### Run End-to-End Pipeline
```bash
# 1. Start Isaac Sim with your robot
# 2. Launch VSLAM system
# 3. Launch Nav2 stack
# 4. Send navigation goals

# Example complete pipeline script
cd simulation-assets/isaac-sim/
python3 -c "
from goal_pose_and_path_planning import IsaacSimPathPlanner
import rclpy

rclpy.init()
planner = IsaacSimPathPlanner()

# Set up a simple navigation task
# This would typically be done through ROS 2 commands
print('AI-Robot Brain pipeline ready for navigation tasks')
"
```

## Step 8: Performance Validation

### Run Benchmarking Tools
```bash
# Launch navigation benchmarking
python3 navigation_benchmarking_tools.py

# Monitor VSLAM performance
python3 vslam_benchmarking_tools.py

# Validate synthetic data quality
python3 synthetic_data_qa_script.py
```

## Common Commands

### System Status
```bash
# Check ROS 2 nodes
ros2 node list

# Check topics
ros2 topic list

# Check Isaac Sim status
nvidia-smi  # Verify GPU utilization
```

### Troubleshooting
```bash
# Check Isaac Sim logs
# Usually located at ~/isaac_sim_logs/

# Check ROS 2 logs
~/.ros/log/

# Verify GPU availability
nvidia-smi
```

## Configuration Files Reference

### Key Configuration Files
- `simulation-assets/isaac-sim/costmap_params_humanoid.yaml` - Costmap parameters
- `simulation-assets/isaac-sim/nav2_params_isaac_sim.yaml` - Nav2 parameters
- `simulation-assets/isaac-sim/navigation_behavior_trees.xml` - Behavior trees
- `docs/module-03-isaac-brain/03-isaac-ros-vslam.md` - VSLAM guide

## Next Steps

### Immediate Actions
1. **Customize Robot**: Adapt the system to your specific robot model
2. **Environment Setup**: Configure custom environments for testing
3. **Parameter Tuning**: Optimize parameters for your specific use case
4. **Integration Testing**: Run comprehensive end-to-end tests

### Advanced Topics
- Explore advanced synthetic data generation techniques
- Implement custom behavior trees for specialized tasks
- Optimize for edge deployment on Jetson platforms
- Develop sim-to-real transfer protocols

## Support Resources

- **Documentation**: All guides in `docs/module-03-isaac-brain/`
- **Configuration**: Examples in `simulation-assets/isaac-sim/`
- **Scripts**: Implementation tools in `simulation-assets/isaac-sim/`
- **Troubleshooting**: See `docs/module-03-isaac-brain/module-summary-next-steps.md`

---

**Success Criteria Check**:
- [ ] Isaac Sim running with photorealistic rendering
- [ ] Synthetic data pipeline generating >100 images with labels
- [ ] VSLAM system creating accurate maps with &lt;1m localization
- [ ] Nav2 stack navigating with >95% success rate
- [ ] Complete pipeline integrated and tested

Your AI-Robot Brain is now operational! Refer to the detailed guides in each chapter for advanced configuration and optimization.