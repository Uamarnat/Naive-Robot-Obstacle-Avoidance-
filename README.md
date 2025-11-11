# Obstacle Avoidance with Differential Drive Robot Simulation using ROS and Gazebo

This project demonstrates a differential-drive robot equipped with a LiDAR sensor, simulated in **Gazebo** with **ROS**.  
The setup includes a URDF-defined robot, a custom Gazebo world, a bridge connecting topics between Gazebo and ROS 2, and a Python obstacle avoidance node.

---

## Requirements

- **Operating System:** Ubuntu 22.04  
- **ROS 2 Distribution:** Humble  
- **Simulator:** Gazebo Ignition Fortress  
- **Bridge Package:** ros_gz_bridge  

---

## Setup

1. **Copy the package into your ROS 2 workspace**
   ```bash
   cd ~/ros2_ws/src
   copy and paste assignment
   ```

2. **Build the package**
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

3. **Source the workspace environment**
   ```bash
   source install/setup.bash
   ```

4. **Launch the Gazebo world and spawn the robot**
   ```bash
   ros2 launch assignment world.launch.py
   ```

5. **In another terminal, run the obstacle avoidance node**
   ```bash
   ros2 run assignment obstacle_avoidance
   ```

---

## Working

### Robot URDF

The robot model is defined in a URDF file and consists of:

- **Base chassis:** `base_link`  
- **Left and right wheels:** `left_wheel_link`, `right_wheel_link`  
- **Caster wheel:** `caster_link`  
- **LiDAR sensor:** `lidar_link`  

Each component includes corresponding joints and inertial parameters.

**Gazebo plugins:**
- **Diff-drive plugin** – Controls wheel motion and publishes odometry data.  
- **LiDAR plugin** – Simulates a 2D laser scanner that provides obstacle range data.


### Gazebo World

The simulation world file (`world.world`) defines the physical environment for the robot.  
It includes:

- A **flat ground plane** spanning a 6m × 6m area.  
- **Walls** surrounding the environment.  
- **Static obstacles** such as boxes and cylinders to test LiDAR sensing and avoidance behavior.


### ROS–Gazebo Bridge

The **ROS–Gazebo bridge** connects simulation data from Gazebo to ROS 2 nodes.  
This allows the robot to interact with ROS-based topics and tools such as RViz.

**Bridged topics:**
- Gazebo `/model/robot/cmd_vel` → ROS 2 `/cmd_vel` 
  Command velocity input controlling linear and angular motion.

- Gazebo `/model/robot/odom` → ROS 2 `/odom` 
  Odometry feedback data published by the diff-drive plugin.

- Gazebo `/world/simple_world/model/robot/link/base_link/sensor/lidar/scan` → ROS 2 `/scan`
  LiDAR range data used for obstacle detection.


The bridge converts Ignition messages into standard ROS 2 message types (`geometry_msgs/Twist`, `nav_msgs/Odometry`, and `sensor_msgs/LaserScan`).

---

### Obstacle Avoidance Node

The Obstacle Avoidance Node implements a simple reactive control strategy using LiDAR data.

#### Summary
- If no obstacle is detected: The robot moves forward at a constant linear speed.  
- If an obstacle is detected within a threshold distance: The robot stops and turns until the path is clear.

#### Node Architecture

1. **Subscription:**
   The node subscribes to `/scan`, which provides 2D LiDAR scan data including:
   - `ranges[]`: distance readings for each angular step.
   - `angle_min`, `angle_max`: limits of the scan.
   - `angle_increment`: angle between consecutive laser beams.

2. **Publisher:**
   The node creates a publisher to `/cmd_vel`, which sends velocity commands of type `geometry_msgs/Twist` to control the robot.

3. **Scan Callback Function:**
   - Triggered each time a new LiDAR scan is received.
   - The front sector (±30°) of the scan is analyzed for obstacles.
   - The node calculates the minimum distance within this range.
   - Based on this value:
     - If the minimum distance > threshold (0.5 m):  
       The robot moves forward (`Twist.linear.x > 0`).
     - If the minimum distance < threshold:  
       The robot stops and rotates in place (`Twist.linear.x = 0` and`Twist.angular.z > 0`).

4. **Main Execution:**
   - Initializes the ROS 2 node (`rclpy.init()`).
   - Creates an instance of the `ObstacleAvoidance` class.
   - Spins the node so that the callback continuously processes LiDAR data.
   - Publishes control commands to `/cmd_vel` for real-time movement.



This process repeats continuously, resulting in a robot that can navigate around obstacles using only LiDAR sensing.

---

### SIDE NOTE

Due to recent updates and version changes in Gazebo, I had several compatibility issues were encountered during implementation of the URDF and ROS. Some of the Gazebo plugin configurations and dependencies in the URDF file were incomplete or deprecated. To ensure functionality, the simulation was developed and tested using Gazebo Fortress and its ROS-Gazebo bridge, which is the recommended version for ROS 2 Humble.
While integrating the ROS-Gazebo bridge, there were TF frame conflicts between Gazebo’s coordinate system and ROS 2’s TF tree.  These conflicts caused visualization issues in RViz and did not work. Despite these limitations, the core simulation and obstacle avoidance functionalities can be observed in Gazbeo.



