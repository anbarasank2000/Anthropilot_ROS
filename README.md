# Trial 1: TurtleBot3 in Custom World with SLAM and Navigation

This package spawns a TurtleBot3 in a custom Gazebo world, runs SLAM to build a map, and uses Nav2 for autonomous navigation.

---

## 📦 Prerequisites
- ROS 2 Humble
- `turtlebot3`, `turtlebot3_gazebo`, `turtlebot3_description`
- `slam_toolbox`
- `nav2_bringup`

Make sure your workspace is sourced:
```bash
colcon build
source ~/Anthropilot_ROS/install/setup.bash
export TURTLEBOT3_MODEL=burger
```
## 🚀 Usage
- 1. Launch Gazebo with TurtleBot3
```bash
ros2 launch trial_1 my_custom_world.launch.py
```
- 2. Control the robot (teleoperation)
```bash
ros2 launch trial_1 teleop_keyboard.launch.py
```
- 3. Run SLAM (to build the map)
```bash
ros2 launch trial_1 slam_toolbox.launch.py
```
- 4. Save the map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/anthropilot/src/map/
```

This will generate:

map.yaml

map.pgm

in your home directory.

- 5. Launch Nav2 for navigation
```bash
ros2 launch trial_1 nav2_navigation.launch.py
```
## 🗺️ Workflow

Spawn TurtleBot3 in custom world.

Drive the robot using keyboard teleop.

SLAM builds the environment map.

Save the generated map.

Load the map and run Nav2 for autonomous navigation.

## 📝 Notes

Default robot: TurtleBot3 Burger.

Make sure to set the correct environment variable before running:

Use RViz2 to visualize /map, /scan, and the robot model.
