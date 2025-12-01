# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a ROS 2 Humble workspace for TurtleBot3 autonomous navigation using SLAM and Nav2 with a custom teleop recovery system. The project spawns a TurtleBot3 Burger robot in custom Gazebo worlds, builds maps using SLAM Toolbox, performs autonomous navigation using Nav2, and provides a state machine for seamless teleop recovery when navigation fails.

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

- **trial_1**: Main ROS 2 package (hybrid ament_cmake + ament_python)
  - `trial_1/`: Python module with recovery nodes
    - `teleop_recovery_node.py`: State machine for recovery behavior
    - `teleop_keyboard_recovery.py`: Keyboard teleop with f/r/c controls
    - `teleop_trigger.py`: CLI tool for triggering recovery
  - `launch/`: Python launch files for different subsystems
  - `config/`: Nav2 parameter files
  - `worlds/`: Gazebo world files (my_custom_world.world, world2.world, world3.world)
  - `urdf/`: Robot description (tb3_fixed.urdf)
  - `map/`: Saved SLAM maps (map.yaml, map.pgm)
  - `setup.py`: Python package setup with console_scripts entry points

- **src/**: Additional resources
  - `map1/`, `map2/`: Alternative saved maps
  - `Room/`: Custom Gazebo model (model.sdf, model.config)
  - `rviz_settings/`: Custom RViz configuration (rviz.rviz)

**Note:** The package uses a hybrid build system (ament_python build type with ament_cmake dependency) that installs both Python modules and traditional ROS resources.

### Launch File Architecture

The system is split into separate launch files for modularity:

1. **my_custom_world.launch.py**: Spawns robot in Gazebo
   - Launches Gazebo with custom world (world3.world)
   - Starts robot_state_publisher with tb3_fixed.urdf
   - Spawns TurtleBot3 Burger at origin
   - Launches RViz2 with slam_toolbox config

2. **teleop_keyboard.launch.py**: Basic keyboard control for manual driving

3. **slam_toolbox.launch.py**: SLAM mapping
   - Runs sync_slam_toolbox_node
   - Uses sim_time=True for Gazebo synchronization

4. **nav2_navigation.launch.py**: Autonomous navigation
   - Includes nav2_bringup with pre-built map
   - Runs AMCL for localization
   - Uses map from trial_1/map/map.yaml

5. **teleop_recovery.launch.py**: Teleop recovery system
   - Runs teleop_recovery_node (state machine)
   - Used alongside nav2_navigation.launch.py
   - Requires separate keyboard node: `ros2 run trial_1 teleop_keyboard_recovery`

6. **nav2_with_recovery.launch.py**: Combined Nav2 + recovery (alternative to separate launch)

### Teleop Recovery State Machine

The teleop recovery system provides seamless switching between autonomous navigation and manual control:

**States:**
- **IDLE**: No navigation goal active
- **NAVIGATING**: Robot autonomously navigating to goal
- **TELEOP_RECOVERY**: User has taken manual control
- **PAUSED**: Navigation paused, awaiting user input

**State Transitions:**
- IDLE → NAVIGATING: When goal_pose is received
- NAVIGATING → TELEOP_RECOVERY: On navigation failure, 'f' key press, or manual trigger
- TELEOP_RECOVERY → NAVIGATING: 'r' key press to resume with last goal
- Any state → IDLE: 'c' key press to cancel

**Key Topics:**
- `/trigger_teleop_recovery` (std_msgs/Empty): Manually trigger teleop mode
- `/resume_navigation` (std_msgs/Empty): Resume autonomous navigation
- `/cancel_navigation` (std_msgs/Empty): Cancel current goal
- `/teleop_recovery/state` (std_msgs/String): Current state (IDLE/NAVIGATING/TELEOP_RECOVERY)
- `/teleop_recovery/teleop_active` (std_msgs/Bool): Whether teleop is active
- `/teleop_cmd_vel` (geometry_msgs/Twist): Teleop velocity input
- `/cmd_vel` (geometry_msgs/Twist): Output velocity commands

The node intercepts `/goal_pose` to track the last navigation goal for resume functionality.

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

### Navigation with Teleop Recovery Workflow

For autonomous navigation with manual recovery capability, use **4 terminals**:

**Terminal 1: Gazebo simulation**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch trial_1 my_custom_world.launch.py
```

**Terminal 2: Nav2 navigation**
```bash
ros2 launch trial_1 nav2_navigation.launch.py
```

**Terminal 3: Teleop recovery system**
```bash
ros2 launch trial_1 teleop_recovery.launch.py
```

**Terminal 4: Keyboard teleop (must have focus for keyboard input)**
```bash
ros2 run trial_1 teleop_keyboard_recovery
```

**Keyboard controls in Terminal 4:**
- `f` = FAIL/takeover (force teleop mode during navigation)
- `r` = RESUME navigation (return to autonomous mode with last goal)
- `c` = CANCEL navigation (abort current goal)
- `w`/`x` = increase/decrease linear velocity
- `a`/`d` = increase/decrease angular velocity
- `s` = STOP all movement
- `q` = quit teleop

**Recovery triggers:**
- Automatic: Navigation goal is aborted by Nav2
- Manual: Press 'f' key during navigation
- Programmatic: Publish to `/trigger_teleop_recovery`

**Alternative:** Use `teleop_trigger` CLI tool:
```bash
ros2 run trial_1 trigger_teleop           # Trigger recovery
ros2 run trial_1 trigger_teleop --resume  # Resume navigation
ros2 run trial_1 trigger_teleop --cancel  # Cancel navigation
ros2 run trial_1 trigger_teleop --status  # Check status
```

### Installing Package Resources

The setup.py installs these resources to share/${PROJECT_NAME}:
- launch/ (Python launch files)
- config/ (Nav2 parameters)
- map/ (SLAM maps)
- worlds/ (Gazebo worlds)
- urdf/ (Robot descriptions)

Python modules are installed as console_scripts entry points:
- `teleop_recovery_node` → trial_1.teleop_recovery_node:main
- `teleop_keyboard_recovery` → trial_1.teleop_keyboard_recovery:main
- `trigger_teleop` → trial_1.teleop_trigger:main

After modifying Python code or resources, rebuild with `colcon build --packages-select trial_1` and source the workspace.

## Key Dependencies

**C++/Launch dependencies:**
- turtlebot3
- turtlebot3_gazebo
- turtlebot3_description
- slam_toolbox
- nav2_bringup (includes bt_navigator, controller, planner, behaviors)
- nav2_amcl
- nav2_map_server
- nav2_lifecycle_manager
- robot_state_publisher
- gazebo_ros
- teleop_twist_keyboard

**Python dependencies:**
- rclpy
- std_msgs
- geometry_msgs
- nav2_msgs
- action_msgs

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
