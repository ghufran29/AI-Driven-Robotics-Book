# Data Model: Module 2 - The Digital Twin (Gazebo & Unity)

**Created**: 2025-12-17
**Feature**: Module 2: The Digital Twin (Gazebo & Unity)

## Simulation Environment Model

### World Parameters
- **gravity**: Vector3 (x, y, z) - Gravitational acceleration in m/s²
- **friction_coefficient**: Float - Coefficient of friction for surfaces
- **collision_tolerance**: Float - Collision detection tolerance in meters
- **time_step**: Float - Physics simulation time step in seconds
- **real_time_factor**: Float - Real-time factor for simulation speed

### Object Properties
- **name**: String - Unique identifier for the object
- **position**: Vector3 (x, y, z) - Position in world coordinates
- **orientation**: Quaternion (x, y, z, w) - Orientation in world coordinates
- **mass**: Float - Mass in kilograms
- **inertia**: Vector3 (ixx, iyy, izz) - Principal moments of inertia
- **collision_mesh**: String - Path to collision mesh file
- **visual_mesh**: String - Path to visual mesh file

## Sensor Data Model

### LiDAR Data
- **ranges**: Array of Float - Distance measurements in meters
- **intensities**: Array of Float - Intensity values for each range
- **angle_min**: Float - Minimum angle of the scan in radians
- **angle_max**: Float - Maximum angle of the scan in radians
- **angle_increment**: Float - Angular distance between measurements in radians
- **time_increment**: Float - Time between measurements in seconds
- **scan_time**: Float - Time between scans in seconds
- **range_min**: Float - Minimum range value in meters
- **range_max**: Float - Maximum range value in meters
- **timestamp**: Time - Timestamp of the scan

### Camera Data
- **width**: Int - Image width in pixels
- **height**: Int - Image height in pixels
- **encoding**: String - Pixel encoding (e.g., "rgb8", "bgr8", "mono8")
- **is_bigendian**: Bool - Endianness of the image data
- **step**: Int - Full row length in bytes
- **data**: Array of Bytes - Image data
- **header**: Header - ROS message header with timestamp and frame_id
- **distortion_model**: String - Camera distortion model
- **D**: Array of Float - Distortion coefficients
- **K**: Array of Float - 3x3 camera matrix
- **R**: Array of Float - 3x3 rectification matrix
- **P**: Array of Float - 3x4 projection matrix

### IMU Data
- **orientation**: Quaternion (x, y, z, w) - Orientation estimate
- **orientation_covariance**: Array of Float - Covariance matrix for orientation
- **angular_velocity**: Vector3 (x, y, z) - Angular velocity in rad/s
- **angular_velocity_covariance**: Array of Float - Covariance matrix for angular velocity
- **linear_acceleration**: Vector3 (x, y, z) - Linear acceleration in m/s²
- **linear_acceleration_covariance**: Array of Float - Covariance matrix for linear acceleration
- **header**: Header - ROS message header with timestamp and frame_id

## Robot State Model

### Joint State
- **name**: Array of String - Joint names
- **position**: Array of Float - Joint positions in radians or meters
- **velocity**: Array of Float - Joint velocities in rad/s or m/s
- **effort**: Array of Float - Joint efforts in N or Nm
- **header**: Header - ROS message header with timestamp and frame_id

### Transform (TF) Tree
- **child_frame_id**: String - Child frame identifier
- **header**: Header - ROS message header with timestamp and frame_id
- **transform**: Transform - Translation and rotation from parent to child
  - **translation**: Vector3 (x, y, z) - Translation vector
  - **rotation**: Quaternion (x, y, z, w) - Rotation quaternion

### Robot Physical Properties
- **base_frame**: String - Base coordinate frame of the robot
- **end_effector_frame**: String - End effector coordinate frame
- **joint_limits**: Array of JointLimit objects - Joint limit definitions
  - **min_position**: Float - Minimum joint position
  - **max_position**: Float - Maximum joint position
  - **max_velocity**: Float - Maximum joint velocity
  - **max_effort**: Float - Maximum joint effort
- **collision_geometry**: Array of Collision objects - Collision mesh definitions
- **visual_geometry**: Array of Visual objects - Visual mesh definitions

## Bridge Communication Model

### ROS Message Mapping
- **topic_name**: String - ROS topic name
- **message_type**: String - ROS message type (e.g., "sensor_msgs/LaserScan")
- **unity_topic**: String - Unity-side topic name
- **conversion_function**: String - Function to convert between formats
- **qos_profile**: QoSProfile - Quality of service settings

### Performance Metrics
- **latency**: Float - Communication delay in milliseconds
- **bandwidth**: Float - Data transfer rate in bytes/second
- **packet_loss**: Float - Percentage of lost messages
- **synchronization_error**: Float - Time difference between systems in seconds

## Validation Rules

### Physics Validation
- Gravity must be within realistic range (-15 to -5 m/s² for z-axis)
- Friction coefficients must be positive values (typically 0.0 to 2.0)
- Time steps must be small enough to maintain numerical stability (< 0.01s)
- Mass values must be positive
- Collision meshes must be closed and manifold

### Sensor Data Validation
- LiDAR ranges must be within sensor specifications (min to max range)
- Camera image dimensions must match sensor specifications
- IMU values must be within realistic bounds for the sensor type
- Timestamps must be monotonic and within acceptable time windows
- Covariance values must be non-negative

### Robot State Validation
- Joint positions must be within defined limits
- Joint velocities must not exceed maximum values
- TF transforms must represent valid rotations (unit quaternions)
- Robot must not intersect with environment obstacles

## State Transitions

### Simulation States
- **STOPPED**: Simulation is not running
- **INITIALIZING**: Simulation is loading world and models
- **RUNNING**: Simulation is actively computing physics
- **PAUSED**: Simulation is temporarily suspended
- **ERROR**: Simulation encountered an unrecoverable error

### Sensor States
- **UNCONFIGURED**: Sensor plugin is loaded but not initialized
- **CONFIGURING**: Sensor parameters are being set
- **RUNNING**: Sensor is actively publishing data
- **ERROR**: Sensor encountered an error during operation

### Bridge States
- **DISCONNECTED**: No communication established
- **CONNECTING**: Attempting to establish connection
- **CONNECTED**: Communication established but not synchronized
- **SYNCHRONIZED**: Full bidirectional communication operational
- **FAULT**: Communication error detected