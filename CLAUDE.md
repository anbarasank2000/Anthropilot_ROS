# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a ROS 2 Humble workspace for TurtleBot3 autonomous navigation using SLAM and Nav2. The project spawns a TurtleBot3 Burger robot in custom Gazebo worlds, builds maps using SLAM Toolbox, and performs autonomous navigation using Nav2.

## Environment Setup

**Critical:** Before running any commands, ensure these environment variables are set:

```bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/rrnitish/Anthropilot_ROS/src
source install/setup.bash
```

The GAZEBO_MODEL_PATH must include the src directory to load custom world models (Room, map1, map2).

## Build System

This is a colcon-based ROS 2 workspace.

**Build the workspace:**
```bash
colcon build
```

**Build specific package:**
```bash
colcon build --packages-select trial_1
```

**After building, source the workspace:**
```bash
source install/setup.bash
```

## Architecture

### Package Structure

- **trial_1**: Main ROS 2 package (ament_cmake)
  - `launch/`: Python launch files for different subsystems
  - `worlds/`: Gazebo world files (my_custom_world.world, world2.world, world3.world)
  - `urdf/`: Robot description (tb3_fixed.urdf)
  - `map/`: Saved SLAM maps (map.yaml, map.pgm)

- **src/**: Additional resources
  - `map1/`, `map2/`: Alternative saved maps
  - `Room/`: Custom Gazebo model (model.sdf, model.config)
  - `rviz_settings/`: Custom RViz configuration (rviz.rviz)

### Launch File Architecture

The system is split into separate launch files for modularity:

1. **my_custom_world.launch.py**: Spawns robot in Gazebo
   - Launches Gazebo with custom world (world3.world)
   - Starts robot_state_publisher with tb3_fixed.urdf
   - Spawns TurtleBot3 Burger at origin
   - Launches RViz2 with slam_toolbox config

2. **teleop_keyboard.launch.py**: Keyboard control for manual driving

3. **slam_toolbox.launch.py**: SLAM mapping
   - Runs sync_slam_toolbox_node
   - Uses sim_time=True for Gazebo synchronization

4. **nav2_navigation.launch.py**: Autonomous navigation
   - Includes nav2_bringup with pre-built map
   - Runs AMCL for localization
   - Uses map from trial_1/map/map.yaml

### Simulation Time

All nodes must use `use_sim_time: True` when working with Gazebo simulation. This ensures proper time synchronization between Gazebo and ROS 2 nodes.

## Development Workflow

### Complete SLAM and Navigation Workflow

**1. Launch simulation:**
```bash
ros2 launch trial_1 my_custom_world.launch.py
```

**2. Start teleoperation (in new terminal):**
```bash
ros2 launch trial_1 teleop_keyboard.launch.py
```

**3. Start SLAM (in new terminal):**
```bash
ros2 launch trial_1 slam_toolbox.launch.py
```

**4. Drive robot to map environment, then save map:**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/anthropilot/src/map/
```
This generates map.yaml and map.pgm files.

**5. Launch Nav2 navigation:**
```bash
ros2 launch trial_1 nav2_navigation.launch.py
```

### Installing Package Resources

The CMakeLists.txt installs these directories to share/${PROJECT_NAME}:
- launch/
- map/
- worlds/
- urdf/

After modifying these resources, rebuild with `colcon build` to update installed files.

## Key Dependencies

- turtlebot3
- turtlebot3_gazebo
- turtlebot3_description
- slam_toolbox
- nav2_bringup
- nav2_amcl
- robot_state_publisher
- gazebo_ros

## RViz Visualization

RViz2 is automatically launched with the simulation. Key topics to visualize:
- `/map` - SLAM-generated or loaded map
- `/scan` - LiDAR sensor data
- Robot model via robot_state_publisher

Custom RViz config available at: `src/rviz_settings/rviz.rviz`

## World Files

The launch file currently uses `world3.world`. To switch worlds, modify the world_file path in `my_custom_world.launch.py`:
- `my_custom_world.world`
- `world2.world`
- `world3.world`

Custom world models are loaded from the Room/ directory via GAZEBO_MODEL_PATH.
