# Autonomous Line Following and Navigation

An autonomous navigation system for **TurtleBot3** robots built on **ROS 2 Humble**. This project implements a complete pipeline for line following, waypoint recording, and autonomous navigation using computer vision, SLAM, and Nav2.

## 🚀 Overview

This project enables a TurtleBot3 robot to:
1. **Follow a red line** using camera-based computer vision
2. **Record waypoints** automatically during line following
3. **Navigate autonomously** to recorded waypoints or predefined green points using Nav2

The system is fully integrated with **Gazebo** simulation and can be adapted for real hardware deployment.

## 🔧 ROS 2 Node Architecture

### line_follow_node

**Node Class**: `LineFollower : public rclcpp::Node`

**Timers**:
- `start_timer_`: 100ms wall timer (one-shot, initialization)
- `countdown_timer_`: 1s wall timer (shutdown countdown)

**Subscribers**:
```cpp
/camera/image_raw (sensor_msgs/Image)
  → image_callback() → HSV processing → cmd_vel

/odom (nav_msgs/Odometry)
  → odom_callback() → distance check → waypoint save
```

**Publishers**:
```cpp
/cmd_vel (geometry_msgs/Twist)
  → linear.x ∈ [0, LINEAR_SPEED]
  → angular.z ∈ [-Kp×width/2, Kp×width/2]

/is_task (std_msgs/Bool)
  → true when line lost for 10s
```

**Threading Model**: Single-threaded executor (default)

### navigator_node

**Node Class**: `Navigator : public rclcpp::Node`

**Action Client**:
```cpp
follow_waypoints (nav2_msgs/action/FollowWaypoints)
  → Goal: vector<PoseStamped>
  → Feedback: current_waypoint_index
  → Result: missed_waypoints[]
```

**State Management**:
- `is_started`: Prevents multiple task executions
- File I/O: Synchronous read operation
- Action: Asynchronous send_goal with default options

## 📋 Requirements

### System Requirements
- **ROS 2**: Humble Hawksbill
- **Ubuntu**: 22.04 (recommended)
- **Gazebo**: Ignition Gazebo (Fortress or later)

### ROS 2 Packages
- `rclcpp` - ROS 2 C++ client library
- `sensor_msgs` - Camera and sensor message types
- `geometry_msgs` - Twist and pose messages
- `nav_msgs` - Navigation messages
- `nav2_msgs` - Nav2 action messages
- `cv_bridge` - OpenCV-ROS bridge
- `slam_toolbox` - SLAM implementation
- `nav2_bringup` - Nav2 navigation stack
- `ros_gz_sim` - Gazebo-ROS bridge

### External Libraries
- **OpenCV** (4.x recommended) - Computer vision library
- **CMake** 3.8+

## 🏗️ System Architecture

### Node Graph

```
┌─────────────────┐
│  Gazebo Sim     │
│  (gz_sim)       │
└────────┬────────┘
         │
    ┌────┴────┬──────────────┬─────────────┐
    │         │              │             │
┌───▼───┐ ┌──▼───┐    ┌─────▼─────┐  ┌───▼────┐
│Camera │ │LIDAR │    │ Odometry  │  │Physics │
│Sensor │ │Sensor│    │           │  │Engine  │
└───┬───┘ └──┬───┘    └─────┬─────┘  └────────┘
    │        │              │
    │        │              │
┌───▼────────▼──────────────▼──────────────┐
│         line_follow_node                  │
│  ┌────────────────────────────────────┐  │
│  │ Image Processing Pipeline          │  │
│  │  • HSV Conversion                  │  │
│  │  • Dual-Range Masking              │  │
│  │  • Morphological Ops               │  │
│  │  • Contour Analysis                │  │
│  │  • PID Control                     │  │
│  └────────────────────────────────────┘  │
│  ┌────────────────────────────────────┐  │
│  │ Waypoint Recording                 │  │
│  │  • Distance Filtering               │  │
│  │  • File I/O                        │  │
│  └────────────────────────────────────┘  │
└───┬──────────────────────────────────┬───┘
    │                                  │
    │ /cmd_vel                         │ /is_task
    │                                  │
┌───▼──────────┐              ┌───────▼────────┐
│  Robot Base  │              │ navigator_node │
│  (base_link) │              │                 │
└──────────────┘              │  • File Reader  │
                              │  • Random Select│
                              │  • Nav2 Client  │
                              └────────┬────────┘
                                       │
                              ┌────────▼────────┐
                              │   Nav2 Stack    │
                              │  • Planner      │
                              │  • Controller   │
                              │  • Recovery     │
                              └────────┬────────┘
                                       │
                              ┌────────▼────────┐
                              │  SLAM Toolbox   │
                              │  • Map Builder  │
                              │  • Localization │
                              └─────────────────┘
```

### ROS 2 Communication Graph

#### Topics

| Topic | Type | Publisher | Subscriber | Rate | Description |
|-------|------|-----------|------------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo | `line_follow_node` | 30 Hz | RGB camera feed (640×480) |
| `/odom` | `nav_msgs/Odometry` | Gazebo | `line_follow_node` | 50 Hz | Wheel odometry |
| `/cmd_vel` | `geometry_msgs/Twist` | `line_follow_node` | Robot controller | 10 Hz | Velocity commands |
| `/is_task` | `std_msgs/Bool` | `line_follow_node` | `navigator_node` | Event-based | Task completion signal |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo | SLAM Toolbox | 10 Hz | LIDAR scan data |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox | Nav2, RViz | 1 Hz | Occupancy grid map |
| `/tf` | `tf2_msgs/TFMessage` | Multiple | All nodes | 100 Hz | Transform tree |

#### Actions

| Action | Type | Client | Server | Description |
|--------|------|--------|--------|-------------|
| `follow_waypoints` | `nav2_msgs/FollowWaypoints` | `navigator_node` | Nav2 | Sequential waypoint navigation |

#### Services

| Service | Type | Provider | Description |
|---------|------|----------|-------------|
| `/map_saver/save_map` | `nav2_msgs/SaveMap` | Nav2 Map Server | Save map to file |

### Data Flow

**Line Following Loop** (30 Hz):
```
Camera Image (BGR) 
  → cv_bridge::toCvCopy() 
  → ROI Extraction 
  → BGR2HSV 
  → Dual-Range Masking 
  → Morphological Operations 
  → Moments Calculation 
  → Error = center_x - cx 
  → ω = Kp × error 
  → Twist{linear.x=0.15, angular.z=ω} 
  → /cmd_vel
```

**Waypoint Recording** (Event-driven):
```
Odometry Message 
  → Extract Pose 
  → Calculate Distance from Last Waypoint 
  → If distance ≥ 0.5m: 
      → Format: "x y z qx qy qz qw\n" 
      → Append to /tmp/line_follow_waypoints.txt
```

**Navigation Pipeline**:
```
/is_task = true 
  → Read waypoints file 
  → Parse poses 
  → Select 3 random from [N/2, N) 
  → Create FollowWaypoints Goal 
  → Send to Nav2 Action Server 
  → Nav2 Plans Path (SmacPlanner2D) 
  → Nav2 Executes (DWB Controller) 
  → Feedback: current_waypoint_index 
  → Result: success/failure
```

## ⚙️ Configuration Parameters

### Line Following Constants

**Velocity Parameters**:
```cpp
const double LINEAR_SPEED = 0.15;      // m/s, forward velocity during line tracking
const double SEARCH_SPEED = 0.05;      // m/s, forward velocity when line lost
```

**Control Parameters**:
```cpp
const double Kp = 0.003;               // Proportional gain (rad/s per pixel error)
                                       // Angular velocity: ω = Kp × (center_x - cx)
```

**Image Processing Parameters**:
```cpp
const int ROI_HEIGHT_RATIO = 5;        // ROI = bottom 1/5 of image
const double MIN_CONTOUR_AREA = 1000.0; // pixels², minimum blob size for detection
```

**HSV Color Ranges** (Red detection):
```cpp
// Range 1: Low hue values (0-10°)
LOWER_RED_1 = [0, 100, 100]    // [H, S, V]
UPPER_RED_1 = [10, 255, 255]

// Range 2: High hue values (160-180°)
LOWER_RED_2 = [160, 100, 100]
UPPER_RED_2 = [180, 255, 255]
```

**State Machine Parameters**:
```cpp
const int LOG_INTERVAL = 150;                    // Log every N frames
const int MAX_MISSING_COUNT_BEFORE_STOP = 300;   // ~10s at 30 FPS
const int shutdown_countdown_ = 5;               // seconds before exit
```

**Waypoint Recording**:
```cpp
double min_distance_between_waypoints_ = 0.5;    // meters
std::string waypoints_file_path_ = "/tmp/line_follow_waypoints.txt";
```

### Nav2 Configuration

**Planner Parameters** (`config/nav2_params.yaml`):
- **Planner Type**: `SmacPlanner2D`
- **Tolerance**: `xy_goal_tolerance: 0.25`, `yaw_goal_tolerance: 0.25`
- **Costmap Resolution**: `0.05 m/pixel`
- **Inflation Radius**: `0.55 m`
- **Obstacle Inflation**: `0.1 m`

**Controller Parameters**:
- **Controller Type**: `DWBController`
- **Max Velocity**: `vx_max: 0.26 m/s`, `vy_max: 0.0 m/s`
- **Max Acceleration**: `acc_lim_x: 2.5 m/s²`
- **Sim Time**: `use_sim_time: true`

**Recovery Behaviors**:
- `Spin`: 360° rotation
- `BackUp`: Reverse 0.15m
- `Wait`: 5s pause

### SLAM Configuration

**SLAM Toolbox Parameters** (`config/slam_params.yaml`):
```yaml
slam_toolbox:
  ros__parameters:
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    map_update_interval: 5.0
    resolution: 0.05
    max_laser_range: 20.0
    minimum_time_interval: 0.5
    transform_publish_period: 0.02
```

**Map Server**:
- **Topic**: `/map` (nav_msgs/OccupancyGrid)
- **Frame**: `map`
- **Update Rate**: 1 Hz (map updates)

### Transform Tree

```
map (SLAM frame)
 └── odom (odometry frame, drift-free)
      └── base_link (robot base)
           ├── camera_link
           ├── laser_link
           └── wheel_left/right_link
```

**TF Publishing**:
- `robot_state_publisher`: URDF → TF tree
- `slam_toolbox`: `map → odom` transform
- `odometry`: `odom → base_link` transform

## 📁 Project Structure

```
line_follow/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata
├── config/                 # Configuration files
│   ├── nav2_params.yaml    # Nav2 parameters
│   └── slam_params.yaml    # SLAM parameters
├── launch/                 # Launch files
│   ├── bring_up.launch.py  # Main launch file
│   ├── robot_state_publisher.launch.py
│   └── spawn_entity.launch.py
├── src/                    # Source code
│   ├── line_follow_node.cpp    # Line following node
│   └── navigator_node.cpp      # Navigation node
├── worlds/                 # Gazebo world files
│   └── line_and_points.world
├── models/                 # Robot models
├── urdf/                   # Robot URDF files
└── rviz/                   # RViz configurations
    └── rviz_config.rviz
```

## 🔧 Advanced Customization

### Color Detection Tuning

**HSV Color Space Selection**:
- **Hue (H)**: 0-179 (wraps at 180°)
- **Saturation (S)**: 0-255 (0=grayscale, 255=fully saturated)
- **Value (V)**: 0-255 (brightness)

**Calibration Procedure**:
1. Capture sample images under target lighting
2. Use OpenCV trackbars to find optimal ranges:
```cpp
cv::createTrackbar("H Min", "Control", &hMin, 179);
cv::createTrackbar("H Max", "Control", &hMax, 179);
// ... similar for S and V
cv::inRange(img_hsv, cv::Scalar(hMin, sMin, vMin), 
            cv::Scalar(hMax, sMax, vMax), mask);
```

**Common Color Ranges**:
```cpp
// Green
LOWER = [40, 50, 50], UPPER = [80, 255, 255]

// Blue  
LOWER = [100, 50, 50], UPPER = [130, 255, 255]

// Yellow
LOWER = [20, 100, 100], UPPER = [30, 255, 255]
```

### PID Tuning Methodology

**Ziegler-Nichols Method**:
1. Set Kp = 0, increase until sustained oscillation
2. Record Ku (ultimate gain) and Tu (oscillation period)
3. For P-controller: `Kp = 0.5 × Ku`

**Empirical Tuning**:
- Start with `Kp = 0.001`
- Increase until overshoot appears
- Reduce by 20% for stability margin
- Test with varying line curvatures

**Performance Metrics**:
- **Steady-state error**: Should converge to < 5 pixels
- **Overshoot**: < 10% of image width
- **Settling time**: < 0.5s for step input

### Real Hardware Deployment

**Camera Calibration**:
```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.024 \
  image:=/camera/image_raw \
  camera:=/camera
```

**URDF Modifications**:
- Update `robot.urdf.xacro` with actual camera intrinsics
- Adjust `base_link → camera_link` transform
- Verify camera frame rate matches code expectations

**Performance Optimization**:
- Reduce image resolution: `640×480 → 320×240`
- Downsample processing: Process every Nth frame
- Use ROS 2 QoS profiles for lower latency:
```cpp
rclcpp::QoS qos(10);
qos.best_effort();
subscription_ = create_subscription<sensor_msgs::msg::Image>(
  "/camera/image_raw", qos, callback);
```

**Hardware-Specific Tuning**:
- **Wheel Encoders**: Verify odometry accuracy
- **Motor Dynamics**: Adjust `LINEAR_SPEED` based on max velocity
- **Battery Voltage**: Compensate speed for voltage drop
- **Lighting**: Use auto-exposure or manual camera settings
