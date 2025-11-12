## Project Description

This project is an autonomous navigation system developed for a **TurtleBot3 Waffle Pi** robot based on **ROS2 Humble**. The robot follows a route using a camera-based **red line following** (line following) algorithm, records **waypoints** (intermediate points) during the route. It then navigates to a randomly selected waypoint from the recorded ones or to predefined **green points**. The system is tested in a **Gazebo** simulation environment and integrates **SLAM** (Simultaneous Localization and Mapping) and **Nav2** (Navigation2) stacks.

### Key Features

- **Line Following**: Detects the red line in HSV color space and follows it with PID-like control. Enters search mode if the line is lost.
- **Waypoint Recording**: Automatically records intermediate points from odometry data during following (/tmp/line_follow_waypoints.txt).
- **Autonomous Navigation**:
    - Selects a random waypoint from the middle 50% of the recorded route and navigates using Nav2.
    - Sequential navigation to 3 predefined green points (GreenPoint_1, _2, _3).
- **Simulation Support**: Integrated with a custom Gazebo world (line_and_points.world), including traffic signs and models.
- **Visualization**: RViz2 for map, pose, and navigation visualization.
- **Automatic Map Creation**: Dynamically creates and saves maps using SLAM Toolbox.

The project can be run on real TurtleBot3 hardware but is optimized for simulation.

## Contribution & Support

- Report issues: [Issues](mailto:muammersonmezofficial@gmail.com)
