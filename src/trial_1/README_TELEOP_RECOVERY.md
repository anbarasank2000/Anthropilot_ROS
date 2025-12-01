# TurtleBot3 Navigation with Teleop Recovery Behavior

This package extends your existing TurtleBot3 navigation setup with a custom teleop recovery system. When navigation fails, you can take over control using teleoperation and then resume autonomous navigation.

## 🎯 Features

1. **Automatic Recovery**: When Nav2 navigation fails (goal aborted), automatically switches to teleop mode
2. **Manual Takeover**: Press 'f' (fail) during navigation to manually trigger teleop recovery
3. **Seamless Resume**: Resume navigation to the last goal after manually controlling the robot
4. **State Management**: Clear state machine managing IDLE, NAVIGATING, TELEOP_RECOVERY, and PAUSED states

## 📁 New Files

```
trial_1/
├── trial_1/
│   ├── __init__.py
│   ├── teleop_recovery_node.py      # Main recovery state machine
│   ├── teleop_keyboard_recovery.py  # Keyboard teleop with recovery integration
│   └── teleop_trigger.py            # CLI tool for triggering recovery
├── launch/
│   ├── teleop_recovery.launch.py    # Launch recovery system alone
│   └── nav2_with_recovery.launch.py # Launch Nav2 + recovery together
├── config/
│   └── nav2_params.yaml             # Nav2 parameters
├── setup.py
├── setup.cfg
└── package.xml
```

## 🚀 Installation

1. Copy the new files to your existing `trial_1` package:

```bash
# Copy the trial_1 Python module
cp -r trial_1/ ~/Anthropilot_ROS/src/trial_1/

# Copy launch files
cp launch/teleop_recovery.launch.py ~/Anthropilot_ROS/src/trial_1/launch/
cp launch/nav2_with_recovery.launch.py ~/Anthropilot_ROS/src/trial_1/launch/

# Copy config
mkdir -p ~/Anthropilot_ROS/src/trial_1/config
cp config/nav2_params.yaml ~/Anthropilot_ROS/src/trial_1/config/

# Copy package files (backup your existing ones first!)
cp setup.py setup.cfg package.xml ~/Anthropilot_ROS/src/trial_1/

# Create resource directory
mkdir -p ~/Anthropilot_ROS/src/trial_1/resource
touch ~/Anthropilot_ROS/src/trial_1/resource/trial_1
```

2. Build the workspace:

```bash
cd ~/Anthropilot_ROS
colcon build --packages-select trial_1
source install/setup.bash
```

## 📖 Usage

### Option 1: Full System Launch (Gazebo + Nav2 + Recovery)

```bash
# Terminal 1: Launch Gazebo with TurtleBot3
ros2 launch trial_1 my_custom_world.launch.py

# Terminal 2: Launch Nav2 with teleop recovery
ros2 launch trial_1 nav2_with_recovery.launch.py
```

### Option 2: Add Recovery to Existing Navigation

If you already have Gazebo and Nav2 running:

```bash
# Terminal: Launch just the teleop recovery system
ros2 launch trial_1 teleop_recovery.launch.py
```

### Option 3: Individual Components

```bash
# Run the recovery node
ros2 run trial_1 teleop_recovery_node

# Run the keyboard teleop (in a separate terminal)
ros2 run trial_1 teleop_keyboard_recovery
```

## 🎮 Keyboard Controls

When running `teleop_keyboard_recovery`:

| Key | Action |
|-----|--------|
| **Movement** ||
| `w` | Increase linear velocity |
| `x` | Decrease linear velocity |
| `a` | Increase angular velocity (turn left) |
| `d` | Decrease angular velocity (turn right) |
| `s` | Stop |
| `↑` | Move forward |
| `↓` | Move backward |
| `←` | Turn left |
| `→` | Turn right |
| **Recovery** ||
| `f` | **Force FAIL** - Trigger teleop recovery |
| `r` | **Resume** - Continue autonomous navigation |
| `c` | **Cancel** - Cancel current navigation |
| **Other** ||
| `h` | Show help |
| `q` | Quit |

## 🔄 Workflow

1. **Start Navigation**: Use RViz to send a goal pose or publish to `/goal_pose`
2. **If Navigation Succeeds**: Robot arrives at goal, state returns to IDLE
3. **If Navigation Fails**: Automatically switches to TELEOP_RECOVERY mode
4. **Manual Intervention**: Press `f` at any time to take over control
5. **Resume Navigation**: Press `r` to resume autonomous navigation to the last goal

## 📡 ROS Topics

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/teleop_recovery/state` | `std_msgs/String` | Current state (IDLE, NAVIGATING, TELEOP_RECOVERY) |
| `/teleop_recovery/teleop_active` | `std_msgs/Bool` | True when teleop mode is active |
| `/cmd_vel` | `geometry_msgs/Twist` | Robot velocity commands |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/trigger_teleop_recovery` | `std_msgs/Empty` | Manually trigger teleop mode |
| `/resume_navigation` | `std_msgs/Empty` | Resume autonomous navigation |
| `/cancel_navigation` | `std_msgs/Empty` | Cancel current navigation goal |
| `/teleop_cmd_vel` | `geometry_msgs/Twist` | Teleop velocity commands |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Navigation goal |

## 🛠️ CLI Tools

Trigger recovery from command line:

```bash
# Trigger teleop recovery
ros2 run trial_1 trigger_teleop

# Resume navigation
ros2 run trial_1 trigger_teleop --resume

# Cancel navigation
ros2 run trial_1 trigger_teleop --cancel

# Check status
ros2 run trial_1 trigger_teleop --status
```

Or use topics directly:

```bash
# Trigger teleop recovery
ros2 topic pub --once /trigger_teleop_recovery std_msgs/Empty

# Resume navigation
ros2 topic pub --once /resume_navigation std_msgs/Empty

# Cancel navigation
ros2 topic pub --once /cancel_navigation std_msgs/Empty
```

## ⚙️ Parameters

The `teleop_recovery_node` accepts these parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `teleop_timeout` | 30.0 | Timeout in teleop mode before prompting |
| `cmd_vel_topic` | `/cmd_vel` | Output velocity topic |
| `teleop_cmd_vel_topic` | `/teleop_cmd_vel` | Input from teleop |
| `nav_cmd_vel_topic` | `/cmd_vel_nav` | Input from Nav2 |
| `auto_resume_on_teleop_idle` | false | Auto-resume when teleop is idle |
| `teleop_idle_timeout` | 5.0 | Seconds before auto-resume |

## 🔧 State Machine

```
                    ┌─────────────────┐
                    │                 │
         ┌──────────│      IDLE       │◄──────────┐
         │          │                 │           │
         │          └────────┬────────┘           │
         │                   │                    │
         │            goal_pose                   │
         │            received                    │
         │                   │                    │
         │                   ▼                    │
         │          ┌─────────────────┐           │
   navigation       │                 │    navigation
   cancelled        │   NAVIGATING    │    succeeded
         │          │                 │           │
         │          └────────┬────────┘           │
         │                   │                    │
         │     ┌─────────────┼─────────────┐      │
         │     │             │             │      │
         │   'f' key    navigation    teleop   goal
         │   pressed    failed        input   reached
         │     │             │             │      │
         │     │             ▼             │      │
         │     │    ┌─────────────────┐    │      │
         │     └───►│                 │◄───┘      │
         │          │ TELEOP_RECOVERY │           │
         │          │                 │           │
         └──────────└────────┬────────┘───────────┘
                             │
                        'r' key
                        (resume)
                             │
                             ▼
                    (back to NAVIGATING)
```

## 📝 Notes

- The system intercepts `/goal_pose` commands to track the last goal for resume functionality
- When in TELEOP_RECOVERY mode, Nav2 velocity commands are ignored
- The system publishes state at 2Hz for monitoring
- Compatible with ROS 2 Humble and Nav2

## 🐛 Troubleshooting

**Q: Keyboard teleop doesn't work?**
- Make sure the terminal running keyboard teleop has focus
- Check that `/teleop_cmd_vel` is being published: `ros2 topic echo /teleop_cmd_vel`

**Q: Navigation doesn't resume?**
- Check state: `ros2 topic echo /teleop_recovery/state`
- Ensure a goal was set before triggering recovery
- Check Nav2 is running: `ros2 lifecycle list`

**Q: Robot doesn't move?**
- Check `/cmd_vel` is being published: `ros2 topic echo /cmd_vel`
- Verify the robot is receiving commands: check Gazebo or hardware

## 📧 Support

For issues or questions, check the state of the system:

```bash
ros2 topic echo /teleop_recovery/state
ros2 topic echo /teleop_recovery/teleop_active
```
