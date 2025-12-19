# ROS 2 Installation Guide for Ubuntu 22.04

This guide will walk you through installing ROS 2 Humble Hawksbill on Ubuntu 22.04 LTS. ROS 2 Humble is the Long Term Support (LTS) distribution that will be supported until 2027, making it ideal for educational and production use.

## Prerequisites

Before installing ROS 2, ensure your system meets the following requirements:

- Ubuntu 22.04 LTS (Jammy Jellyfish)
- At least 2GB RAM
- At least 10GB free disk space
- Internet connection for package installation

## Setup Locale

Make sure your locale is set to UTF-8:

```bash
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US.UTF-8
```

## Add ROS 2 apt Repository

First, add the ROS 2 apt repository to your system:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## Install ROS 2 Packages

Update your apt index and install ROS 2 Humble Desktop:

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

This will install the full ROS 2 desktop environment, which includes development tools, visualization packages, and common robot libraries.

## Install Python 3 Dependencies

Install additional Python 3 dependencies needed for ROS 2 development:

```bash
sudo apt install python3-dev python3-pip
pip3 install -U rclpy
```

## Environment Setup

Set up your ROS 2 environment by sourcing the setup script:

```bash
source /opt/ros/humble/setup.bash
```

To automatically source the ROS 2 environment in new terminals, add to your `~/.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Verify Installation

Test that your installation works by running a simple example:

```bash
# Terminal 1
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 2
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

You should see messages being published by the talker and received by the listener.

## Create a Workspace (Optional but Recommended)

For development, create a workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Troubleshooting

### Common Issues:

1. **Package not found**: Make sure you've added the ROS 2 repository correctly and run `sudo apt update`
2. **Permission denied**: Use `sudo` when installing system packages
3. **Python import errors**: Ensure you're using Python 3.10+ and have installed the necessary packages

### Checking Installation:

```bash
# Check ROS 2 version
ros2 --version

# List available packages
ros2 pkg list

# Check environment variables
printenv | grep -i ros
```

## Safety Considerations

⚠️ **Important Safety Note**: The code examples in this curriculum are designed for simulation environments. When deploying to physical robots, additional safety measures must be implemented, including:
- Emergency stop mechanisms
- Collision detection and avoidance
- Range and speed limitations
- Proper mechanical safety systems

Always follow your organization's safety protocols when working with physical robots.

## Next Steps

Now that you have ROS 2 installed, you can:
- Continue with the next chapter to learn about Python-based robot communication
- Explore the ROS 2 tutorials at https://docs.ros.org/en/humble/Tutorials.html
- Create your first ROS 2 package using `ros2 pkg create`