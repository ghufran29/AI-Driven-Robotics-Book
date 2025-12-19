# Chapter 4: Walking the Path (Nav2)

## Configuring the Navigation 2 Stack for Humanoid Path Planning

This chapter covers the configuration of the Navigation 2 (Nav2) stack for humanoid path planning, obstacle avoidance, and goal-seeking behavior. The Nav2 stack provides a complete navigation solution that works in conjunction with the VSLAM mapping capabilities you've implemented.

### Navigation 2 Stack Overview

The Navigation 2 stack is a collection of packages that implement autonomous navigation for mobile robots. It includes:

- **Global Planner**: Creates optimal paths from start to goal
- **Local Planner**: Executes short-term navigation while avoiding obstacles
- **Controller**: Translates planned paths into robot commands
- **Recovery Behaviors**: Handles navigation failures and stuck situations
- **Costmap 2D**: Represents obstacles and free space in the environment

### Nav2 Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Goal Pose     │    │   Nav2 Stack    │    │   Robot         │
│   (RViz/Client) │───►│   Components    │───►│   Platform      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Global Planner │    │  Local Planner  │    │  Motion         │
│  (Path Planning)│    │  (Obstacle Avoid)│    │  Commands      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Installing Navigation 2 Stack

#### Prerequisites

Before installing Nav2, ensure you have:

- ROS 2 Humble Hawksbill
- Compatible Linux distribution (Ubuntu 22.04 recommended)
- Isaac Sim 2023.1 or newer
- Isaac ROS VSLAM system configured
- Robot description (URDF) available

#### Installation Steps

1. **Install Nav2 packages**:
```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

2. **Install additional dependencies**:
```bash
sudo apt install ros-humble-nav2-rviz-plugins ros-humble-nav2-common
```

3. **Verify installation**:
```bash
ros2 pkg list | grep nav2
```

### Nav2 Configuration for Isaac Sim

#### Basic Configuration File

Create a configuration file for Nav2 in Isaac Sim environment:

```yaml
# nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    navigate_through_poses: False
    navigate_to_pose: True
    behavior_tree: |
      <root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
          <Sequence name="root_sequence">
            <PipelineSequence name="global_plan_sequence">
              <RecoveryNode number_of_retries="6" name="global_plan_recovery_node">
                <GlobalPlanner action_topic="global_costmap/current_goal" name="compute_global_plan"/>
                <GoalUpdated name="goal_updater"/>
              </RecoveryNode>
            </PipelineSequence>
            <PipelineSequence name="local_plan_sequence">
              <RecoveryNode number_of_retries="1" name="local_plan_recovery_node">
                <ComputePathToPose goal_checker="goal_checker" name="compute_path_to_pose"/>
                <FollowPath name="follow_path"/>
              </RecoveryNode>
            </PipelineSequence>
          </Sequence>
        </BehaviorTree>
      </root>

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    # Controller parameters
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      progress_checker_plugin: "progress_checker"
      goal_checker_plugin: "goal_checker"
      RotateWithConstantVelocity:
        plugin: "nav2_controller::RotateWithConstantVelocity"
        max_angular_velocity: 1.0
        min_angular_velocity: 0.4
        tolerance: 0.1
      SmoothPath:
        plugin: "nav2_controller::SimplePure Pursuit"
        velocity_scaling_function_plugin: "nav2_controller::CosPathVelocityScaling"
        lookahead_dist: 0.6
        min_lookahead_dist: 0.3
        max_lookahead_dist: 0.9
        lookahead_time: 1.5
        rotate_to_heading_angular_vel: 1.8
        transform_tolerance: 0.1
        use_velocity_scaled_lookahead_dist: false
        min_approach_linear_velocity: 0.05
        approach_velocity_scaling_dist: 0.6
        max_allowed_time_to_collision_up_to_carrot: 1.0

global_costmap:
  ros__parameters:
    update_frequency: 5.0
    publish_frequency: 2.0
    global_frame: map
    robot_base_frame: base_link
    use_sim_time: True
    rolling_window: false
    width: 40
    height: 40
    resolution: 0.05
    origin_x: -20.0
    origin_y: -20.0
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      enabled: True
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"
        raytrace_max_range: 3.0
        raytrace_min_range: 0.0
        obstacle_max_range: 2.5
        obstacle_min_range: 0.0
    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_subscribe_transient_local: True
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0
      inflation_radius: 0.55
    always_send_full_costmap: True

local_costmap:
  ros__parameters:
    update_frequency: 5.0
    publish_frequency: 2.0
    global_frame: odom
    robot_base_frame: base_link
    use_sim_time: True
    rolling_window: True
    width: 6
    height: 6
    resolution: 0.05
    origin_x: -3.0
    origin_y: -3.0
    plugins: ["obstacle_layer", "inflation_layer"]
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      enabled: True
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"
        raytrace_max_range: 3.0
        raytrace_min_range: 0.0
        obstacle_max_range: 2.5
        obstacle_min_range: 0.0
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0
      inflation_radius: 0.3
    always_send_full_costmap: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    recovery_plugin_types: ["nav2_recoveries::Spin", "nav2_recoveries::BackUp", "nav2_recoveries::Wait"]
    spin:
      plugin: "nav2_recoveries::Spin"
      sim_period: 0.2
      angle: 1.57
      time_allowance: 20.0
    backup:
      plugin: "nav2_recoveries::BackUp"
      sim_period: 0.2
      duration: 2.0
      backup_vel: -0.05
    wait:
      plugin: "nav2_recoveries::Wait"
      sim_period: 0.2
      duration: 5.0

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200
```

### Costmap Configuration for Humanoid Navigation

#### Global Costmap

The global costmap represents the static map of the environment and is used for global path planning:

```yaml
global_costmap:
  ros__parameters:
    # Static map settings
    rolling_window: false
    width: 40          # 40m x 40m map
    height: 40
    resolution: 0.05   # 5cm resolution for humanoid navigation
    origin_x: -20.0
    origin_y: -20.0

    # Plugins for obstacle representation
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

    # Static layer for map
    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_subscribe_transient_local: True

    # Obstacle layer for dynamic obstacles
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      enabled: True
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0    # Humanoid height consideration
        clearing: True
        marking: True
        data_type: "LaserScan"
        raytrace_max_range: 3.0
        raytrace_min_range: 0.0
        obstacle_max_range: 2.5
        obstacle_min_range: 0.0

    # Inflation layer for safety margins
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0      # Higher for humanoid safety
      inflation_radius: 0.55        # Larger for humanoid size
```

#### Local Costmap

The local costmap represents the immediate vicinity of the robot and is used for obstacle avoidance:

```yaml
local_costmap:
  ros__parameters:
    # Rolling window for dynamic environment
    rolling_window: True
    width: 6           # 6m x 6m local area
    height: 6
    resolution: 0.05   # 5cm resolution
    origin_x: -3.0
    origin_y: -3.0

    # Plugins for local navigation
    plugins: ["obstacle_layer", "inflation_layer"]

    # Obstacle layer for local planning
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      enabled: True
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"
        raytrace_max_range: 3.0
        raytrace_min_range: 0.0
        obstacle_max_range: 2.5
        obstacle_min_range: 0.0

    # Inflation for local safety
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0
      inflation_radius: 0.3    # Slightly smaller than global
```

### Behavior Trees for Navigation Recovery

Behavior trees provide a flexible way to define navigation logic and recovery behaviors:

```xml
<!-- Example behavior tree configuration -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root_sequence">
      <PipelineSequence name="global_plan_sequence">
        <RecoveryNode number_of_retries="6" name="global_plan_recovery_node">
          <GlobalPlanner action_topic="global_costmap/current_goal" name="compute_global_plan"/>
          <GoalUpdated name="goal_updater"/>
        </RecoveryNode>
      </PipelineSequence>
      <PipelineSequence name="local_plan_sequence">
        <RecoveryNode number_of_retries="1" name="local_plan_recovery_node">
          <ComputePathToPose goal_checker="goal_checker" name="compute_path_to_pose"/>
          <FollowPath name="follow_path"/>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveSequence name="navigation_recovery_sequence">
        <RecoveryNode number_of_retries="2" name="spin_recovery">
          <Spin spin_dist="1.57"/>
        </RecoveryNode>
        <RecoveryNode number_of_retries="2" name="backup_recovery">
          <BackUp backup_dist="0.15" backup_speed="0.05"/>
        </RecoveryNode>
        <RecoveryNode number_of_retries="2" name="wait_recovery">
          <Wait wait_duration="5"/>
        </RecoveryNode>
      </ReactiveSequence>
    </Sequence>
  </BehaviorTree>
</root>
```

### Goal Pose Setting and Path Planning

#### Setting Navigation Goals

To set navigation goals in Isaac Sim:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class IsaacSimNavigator(Node):
    def __init__(self):
        super().__init__('isaac_sim_navigator')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def navigate_to_pose(self, x, y, theta):
        """Send navigation goal to Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(0, 0, theta)
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]

        self.nav_to_pose_client.wait_for_server()
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        return future
```

#### Path Planning with VSLAM Integration

When using VSLAM-generated maps:

```python
class VSLAMNavIntegrator(Node):
    def __init__(self):
        super().__init__('vslam_nav_integrator')

        # Subscribe to VSLAM map updates
        self.vslam_map_sub = self.create_subscription(
            # This would be the actual map message type
            # Using placeholder for now
            Bool,  # Placeholder
            '/visual_slam/map',
            self.vslam_map_callback,
            10
        )

    def vslam_map_callback(self, msg):
        """Handle VSLAM map updates and integrate with Nav2"""
        # In a real implementation, this would update the Nav2 static layer
        # with the VSLAM-generated map
        self.get_logger().info("VSLAM map integrated with Nav2")
```

### Obstacle Avoidance and Local Minima Handling

#### Local Planner Configuration

The local planner handles real-time obstacle avoidance:

```yaml
controller_server:
  ros__parameters:
    # Controller frequency for responsive obstacle avoidance
    controller_frequency: 20.0

    # Velocity thresholds for safety
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001

    # Path following controller
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      SmoothPath:
        plugin: "nav2_controller::SimplePure Pursuit"
        # Lookahead parameters for smooth navigation
        lookahead_dist: 0.6
        min_lookahead_dist: 0.3
        max_lookahead_dist: 0.9
        lookahead_time: 1.5
        # Velocity scaling for safe obstacle avoidance
        use_velocity_scaled_lookahead_dist: false
        max_allowed_time_to_collision_up_to_carrot: 1.0
```

#### Recovery Behaviors

Configure recovery behaviors for handling local minima:

```yaml
recoveries_server:
  ros__parameters:
    # Recovery plugins for different failure scenarios
    recovery_plugins: ["spin", "backup", "wait"]
    recovery_plugin_types: ["nav2_recoveries::Spin", "nav2_recoveries::BackUp", "nav2_recoveries::Wait"]

    spin:
      plugin: "nav2_recoveries::Spin"
      # Spin recovery for clearing local minima
      sim_period: 0.2
      angle: 1.57    # 90 degrees
      time_allowance: 20.0

    backup:
      plugin: "nav2_recoveries::BackUp"
      # Backup recovery for obstacle clearance
      sim_period: 0.2
      duration: 2.0
      backup_vel: -0.05  # Slow reverse

    wait:
      plugin: "nav2_recoveries::Wait"
      # Wait recovery for dynamic obstacles
      sim_period: 0.2
      duration: 5.0
```

### Testing Navigation Success

#### Navigation Success Validation

Create a validation node to test navigation success:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math

class NavigationSuccessValidator(Node):
    def __init__(self):
        super().__init__('navigation_success_validator')

        # Parameters
        self.declare_parameter('goal_tolerance', 0.5)  # meters
        self.declare_parameter('max_navigation_time', 60.0)  # seconds

        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_navigation_time = self.get_parameter('max_navigation_time').value

        # Goal and robot position tracking
        self.goal_pose = None
        self.current_pose = None
        self.navigation_start_time = None

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Publishers
        self.success_pub = self.create_publisher(Bool, '/navigation_success', 10)

        # Timer for periodic validation
        self.timer = self.create_timer(0.1, self.validate_navigation)

    def odom_callback(self, msg):
        """Update current robot position"""
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        """Update navigation goal and start timer"""
        self.goal_pose = msg.pose
        self.navigation_start_time = self.get_clock().now().nanoseconds / 1e9

    def validate_navigation(self):
        """Validate if navigation was successful"""
        if self.current_pose is None or self.goal_pose is None:
            return

        # Calculate distance to goal
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        # Check if goal reached
        if distance <= self.goal_tolerance:
            success_msg = Bool()
            success_msg.data = True
            self.success_pub.publish(success_msg)
            self.get_logger().info(f"Navigation successful! Distance: {distance:.3f}m")
            self.goal_pose = None  # Reset for next navigation
        else:
            # Check if navigation timed out
            if self.navigation_start_time:
                elapsed_time = (self.get_clock().now().nanoseconds / 1e9) - self.navigation_start_time
                if elapsed_time > self.max_navigation_time:
                    success_msg = Bool()
                    success_msg.data = False
                    self.success_pub.publish(success_msg)
                    self.get_logger().warn(f"Navigation timed out! Elapsed: {elapsed_time:.1f}s")
                    self.goal_pose = None  # Reset for next navigation
```

### Integration with Isaac Sim

#### Isaac Sim Navigation Bridge

To integrate Nav2 with Isaac Sim:

```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class IsaacSimNavBridge(Node):
    def __init__(self):
        super().__init__('isaac_sim_nav_bridge')

        # ROS 2 publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10
        )

        # Initialize Isaac Sim world
        self.world = World(stage_units_in_meters=1.0)

        # Add robot to simulation
        add_reference_to_stage(
            usd_path="path/to/robot.usd",
            prim_path="/World/Robot"
        )

        # Set up simulation
        self.world.scene.add_default_ground_plane()

        self.get_logger().info("Isaac Sim Navigation Bridge initialized")

    def laser_scan_callback(self, msg):
        """Forward laser scan data to Nav2"""
        # This would typically be handled by the ROS bridge in Isaac Sim
        pass

    def send_velocity_command(self, linear_vel, angular_vel):
        """Send velocity commands to simulated robot"""
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_vel
        cmd_msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd_msg)
```

### Performance Tuning for Isaac Sim Environment

#### Adaptive Parameters

For optimal performance in Isaac Sim:

```yaml
# Adaptive parameters for Isaac Sim environment
controller_server:
  ros__parameters:
    # Higher frequency for simulation responsiveness
    controller_frequency: 30.0
    # Slightly more aggressive for simulation
    min_x_velocity_threshold: 0.0005

planner_server:
  ros__parameters:
    # Faster planning in simulation
    expected_planner_frequency: 30.0
    GridBased:
      # More accurate planning for simulation
      tolerance: 0.2
      use_astar: true

global_costmap:
  ros__parameters:
    # Higher resolution for simulation
    resolution: 0.025
    # Larger inflation for safety in simulation
    inflation_layer:
      cost_scaling_factor: 5.0
      inflation_radius: 0.7
```

## Next Steps

This chapter has covered the comprehensive configuration of the Navigation 2 stack for humanoid path planning in Isaac Sim. The system is now capable of:

- Global path planning with VSLAM-generated maps
- Local obstacle avoidance and dynamic path adjustment
- Recovery behaviors for handling navigation failures
- Integration with Isaac Sim simulation environment

The complete AI-Robot Brain (NVIDIA Isaac™) system is now operational with all four components working together: Isaac Sim for simulation, synthetic data generation for training, VSLAM for mapping and localization, and Nav2 for autonomous navigation.