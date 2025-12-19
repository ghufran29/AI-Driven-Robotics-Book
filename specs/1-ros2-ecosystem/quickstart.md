# Quickstart Guide: Module 1: The Robotic Nervous System (ROS 2)

## Prerequisites

Before starting this module, ensure you have:
- Ubuntu 22.04 LTS installed
- Basic Python programming knowledge
- Familiarity with command-line interfaces
- Internet access for package installation

## Environment Setup

### 1. Install ROS 2 Humble Hawksbill

```bash
# Set locale
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
```

### 2. Source ROS 2 Environment

```bash
source /opt/ros/humble/setup.bash
```

To automatically source the ROS 2 environment in new terminals, add to your `~/.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 3. Install Python 3.10+ and Dependencies

```bash
sudo apt update
sudo apt install python3-dev python3-pip
pip3 install -U rclpy
```

## Your First ROS 2 Node with rclpy

### 1. Create a Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 2. Create a Simple Publisher Node

Create `~/ros2_ws/src/my_robot_tutorial/my_robot_tutorial/publisher_member_function.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello Robot: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Create a Simple Subscriber Node

Create `~/ros2_ws/src/my_robot_tutorial/my_robot_tutorial/subscriber_member_function.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Run the Publisher-Subscriber Example

Terminal 1 (Publisher):
```bash
cd ~/ros2_ws
source install/setup.bash
python3 src/my_robot_tutorial/my_robot_tutorial/publisher_member_function.py
```

Terminal 2 (Subscriber):
```bash
cd ~/ros2_ws
source install/setup.bash
python3 src/my_robot_tutorial/my_robot_tutorial/subscriber_member_function.py
```

## Hello Robot Simulation

### 1. Create a Simple URDF Robot Model

Create `~/ros2_ws/src/my_robot_tutorial/my_robot_tutorial/robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="hello_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <link name="sensor_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="sensor_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_link"/>
    <origin xyz="0.2 0 0.1"/>
  </joint>
</robot>
```

### 2. Visualize in RViz2

```bash
# Terminal 1: Launch RViz2
rviz2

# Terminal 2: Publish robot description
cd ~/ros2_ws/src/my_robot_tutorial
ros2 run joint_state_publisher joint_state_publisher &
ros2 run robot_state_publisher robot_state_publisher
```

In RViz2, add a RobotModel display and set the Robot Description parameter to "robot_description".

## Key Concepts Demonstrated

1. **Nodes**: Independent processes that communicate via ROS 2
2. **Topics**: Publisher-subscriber communication pattern
3. **Messages**: Data structures exchanged between nodes
4. **URDF**: Robot description format for modeling
5. **Visualization**: Using RViz2 to visualize robot models

## Next Steps

- Explore services and actions in Chapter 3
- Learn more complex URDF modeling in Chapter 4
- Understand TF transforms for spatial reasoning
- Implement the complete "Hello Robot" simulation with all communication patterns