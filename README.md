# Custom DWA Local Planner for ROS2 Humble
![ROS Humble](https://img.shields.io/badge/ROS-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.10-blueviolet)

A custom Dynamic Window Approach (DWA) local planner implemented from scratch for TurtleBot3 in ROS2 Humble Gazebo simulation. Meets all assignment requirements for obstacle avoidance, goal navigation, and RViz visualization.

---

## Features

- Deterministic grid search velocity sampling (no random sampling)
- Proper LaserScan obstacle avoidance with world coordinates
- Dynamic window constraints (acceleration limits)
- Multi-objective cost function (goal distance + obstacles + speed)
- RViz MarkerArray visualization of planned trajectories
- ROS2 parameters for easy tuning
- Comprehensive logging for debugging

---
## Demo Video
<video autoplay muted loop playsinline width="800" height="450">
  <source src="media/demo.mp4" type="video/mp4">
  Your browser does not support the video tag. [Download demo.mp4](media/demo.mp4)
</video>

[![Download Video](https://img.shields.io/badge/Download%20Video-56MB-blue?style=for-the-badge&logo=download&logoColor=white)](media/demo.mp4)
</div>

---

## Prerequisites
-   ROS 2 Humble
-   Gazebo
-   Python 3
-   `turtlebot3`, `turtlebot3_msgs`, `turtlebot3_simulations` (cloned manually from source)

---

### ROS 2 Setup
```bash
sudo apt update && sudo apt install -y \
  ros-humble-rclpy \
  ros-humble-geometry-msgs \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-visualization-msgs \
  ros-humble-std-msgs
```

### 🔧 Quick Setup Instructions
1. Clone Repositories
Create a workspace and clone this repository along with the required TurtleBot3 packages.
```bash
# Create a Workspace
mkdir -p ~/dwa_ws
cd ~/dwa_ws

# Clone the DWA Repository
git clone https://github.com/Dev584/custom_dwa_planner.git

# Clone TurtleBot3 packages
cd src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

2. Install ROS 2 Dependencies
Navigate to your workspace root and let rosdep install any remaining dependencies.
```bash
cd ~/dwa_ws
rosdep install --from-paths src --ignore-src -r -y
```
3. Build the Workspace
Build the packages using colcon and source the setup file.
```bash
colcon build --symlink-install
source install/setup.bash
```
(Optional) Add the source command to your .bashrc for convenience:
```bash
echo "source ~/dwa_ws/install/setup.bash" >> ~/.bashrc
```
4. Set TurtleBot3 Model
Export the TURTLEBOT3_MODEL environment variable. This project uses burger.
```bash
export TURTLEBOT3_MODEL=burger
```
(Optional) Add this to your .bashrc as well:
```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```
---

### 📡 Running the Simulation
1. Launch the Gazebo world
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
2. Launch DWA Planner
```bash
ros2 launch custom_dwa_planner dwa_planner.launch.py
```
3. Launch & Configure RViz2 (Manual Setup)
```bash
ros2 run rviz2 rviz2
```
RViz2 Configuration (Add these displays in order):
1. Global Options → Fixed Frame: odom
2. Add → RobotModel → Description Topic: /robot_description
3. Add → LaserScan → Topic: /scan
4. Add → MarkerArray → Marker Topic: /dwa_paths

---

### 🎮 Testing
1. Set goal: Click-drag in RViz2 → 2D Pose Goal → Double-click for orientation
2. Watch: Green trajectory appears → Robot navigates avoiding obstacles
3. Expected logs:
```text
[INFO] New goal received: (2.00, 1.00)
[INFO] Cmd: v=0.18, ω=0.45, Dist: 2.28
[INFO] Goal reached! Distance: 0.12m
```
---

### ⚙️ Parameters (Live Tuning)
```bash
# During execution
ros2 param set /dwa_planner_node robot_radius 0.25
ros2 param set /dwa_planner_node max_speed 0.4
ros2 param set /dwa_planner_node goal_threshold 0.2
ros2 param list /dwa_planner_node
```

| Parameter  | Default | Description |
| ---------- | ------- | ----------- |
| robot_radius | 0.22m | TurtleBot3 Burger footprint |
| max_speed	 | 0.3 m/s	 | Maximum linear velocity |
| max_yaw_rate | 2.5 rad/s | Maximum angular velocity |
| goal_threshold | 0.15m | Stop distance to goal |
| predict_time | 1.5s | Trajectory prediction horizon |

---

### 🛠️ Algorithm Details
DWA Pipeline
```text
1. Dynamic Window: [v±accel×dt, ω±Δω×dt]
2. Grid Search: v∈[0.02], ω∈[0.2] resolution  
3. Trajectory Prediction: Kinematic model (15 steps)
4. Cost Function: 1.0×goal_dist + 2.0×obs_cost + 0.3×speed_cost
5. Publish: Best (v,ω) → /cmd_vel + trajectory → /dwa_paths
```

Cost Functions
- Goal Cost: Euclidean distance from trajectory end to goal
- Obstacle Cost: Min distance to LaserScan points (1/dist)
- Speed Cost: Encourage max_speed (max_v - v)

---

## 🙏 Acknowledgments
- ROS2 Humble documentation
- TurtleBot3 Gazebo simulation

---

- Repository: https://github.com/Dev584/custom_dwa_planner
- ROS2 Version: Humble
- Tested: Ubuntu 22.04 + WSL2