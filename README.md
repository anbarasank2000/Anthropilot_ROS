# TurtleBot3 Navigation with Teleop Recovery Behavior

This package extends your existing TurtleBot3 navigation setup with a custom teleop recovery system. When navigation fails, you can take over control using teleoperation and then resume autonomous navigation.

## 🎯 Features

1. **Automatic Recovery**: When Nav2 navigation fails (goal aborted), automatically switches to teleop mode
2. **Manual Takeover**: Press `f` (fail) during navigation to manually trigger teleop recovery
3. **Seamless Resume**: Press `r` to resume navigation to the last goal after manual control
4. **State Management**: Clear state machine managing IDLE, NAVIGATING, TELEOP_RECOVERY states

## 📁 New Files

```
trial_1/
├── trial_1/                          # Python module
│   ├── __init__.py
│   ├── teleop_recovery_node.py       # Main recovery state machine
│   ├── teleop_keyboard_recovery.py   # Keyboard teleop with recovery integration
│   └── teleop_trigger.py             # CLI tool for triggering recovery
├── launch/
│   ├── teleop_recovery.launch.py     # Launch recovery system (use this!)
│   └── nav2_with_recovery.launch.py  # Combined Nav2 + recovery
├── config/
│   └── nav2_params.yaml              # Nav2 parameters (reference)
├── resource/
│   └── trial_1                       # Ament resource marker
├── setup.py
├── setup.cfg
└── package.xml
```

## 🚀 Installation

1. **Copy the new files to your existing `trial_1` package:**

```bash
# Backup your existing package first!
cp -r ~/Anthropilot_ROS/src/trial_1 ~/Anthropilot_ROS/src/trial_1_backup

# Extract the new files (adjust path as needed)
unzip trial_1_teleop_recovery.zip -d /tmp/
cp -r /tmp/trial_1/trial_1 ~/Anthropilot_ROS/src/trial_1/
cp /tmp/trial_1/launch/teleop_recovery.launch.py ~/Anthropilot_ROS/src/trial_1/launch/
cp /tmp/trial_1/setup.py ~/Anthropilot_ROS/src/trial_1/
cp /tmp/trial_1/setup.cfg ~/Anthropilot_ROS/src/trial_1/
cp /tmp/trial_1/package.xml ~/Anthropilot_ROS/src/trial_1/

# Create resource directory if it doesn't exist
mkdir -p ~/Anthropilot_ROS/src/trial_1/resource
touch ~/Anthropilot_ROS/src/trial_1/resource/trial_1
```

2. **Build the workspace:**

```bash
cd ~/Anthropilot_ROS
colcon build --packages-select trial_1
source install/setup.bash
```

## 📖 Usage - CORRECT LAUNCH SEQUENCE

You need **4 terminals**. The keyboard input happens in **Terminal 4**.

### Terminal 1: Launch Gazebo with TurtleBot3
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch trial_1 my_custom_world.launch.py
```

### Terminal 2: Launch Nav2 Navigation (your existing working launch)
```bash
ros2 launch trial_1 nav2_navigation.launch.py
```

### Terminal 3: Launch Teleop Recovery System
```bash
ros2 launch trial_1 teleop_recovery.launch.py
```

### Terminal 4: Run Keyboard Teleop (TYPE HERE!)
```bash
ros2 run trial_1 teleop_keyboard_recovery
```

> ⚠️ **IMPORTANT**: You must click on Terminal 4 and keep it focused to send keyboard commands!

## 🎮 Keyboard Controls

When running `teleop_keyboard_recovery` in Terminal 4:

| Key | Action |
|-----|--------|
| **Recovery Controls** ||
| `f` | **FAIL** - Force teleop takeover during navigation |
| `r` | **RESUME** - Continue autonomous navigation to last goal |
| `c` | **CANCEL** - Cancel current navigation completely |
| **Movement** ||
| `w` | Increase linear velocity (forward) |
| `x` | Decrease linear velocity (backward) |
| `a` | Increase angular velocity (turn left) |
| `d` | Decrease angular velocity (turn right) |
| `s` | **STOP** - Stop all movement |
| `↑` | Move forward at set velocity |
| `↓` | Move backward at set velocity |
| `←` | Turn left at set angular velocity |
| `→` | Turn right at set angular velocity |
| **Other** ||
| `h` | Show help |
| `q` | Quit keyboard teleop |

## 🔄 Typical Workflow

1. **Set a Goal**: Use RViz "2D Goal Pose" tool to send a navigation goal
2. **Navigation Starts**: Robot begins autonomous navigation (state: NAVIGATING)
3. **Option A - Navigation Succeeds**: Robot arrives, state returns to IDLE
4. **Option B - Navigation Fails**: Automatically switches to TELEOP_RECOVERY
5. **Option C - Manual Takeover**: Press `f` anytime to take control
6. **Manual Control**: Use WASD keys to drive the robot manually
7. **Resume**: Press `r` to resume autonomous navigation to the last goal

## 📡 ROS Topics

### Published by teleop_recovery_node

| Topic | Type | Description |
|-------|------|-------------|
| `/teleop_recovery/state` | `std_msgs/String` | Current state: IDLE, NAVIGATING, TELEOP_RECOVERY |
| `/teleop_recovery/teleop_active` | `std_msgs/Bool` | True when teleop mode is active |
| `/cmd_vel` | `geometry_msgs/Twist` | Robot velocity commands (when in teleop mode) |

### Subscribed by teleop_recovery_node

| Topic | Type | Description |
|-------|------|-------------|
| `/trigger_teleop_recovery` | `std_msgs/Empty` | Manually trigger teleop mode |
| `/resume_navigation` | `std_msgs/Empty` | Resume autonomous navigation |
| `/cancel_navigation` | `std_msgs/Empty` | Cancel current navigation goal |
| `/teleop_cmd_vel` | `geometry_msgs/Twist` | Teleop velocity commands input |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Navigation goals (intercepted to track last goal) |

## 🛠️ CLI Tools

You can also trigger recovery from any terminal:

```bash
# Trigger teleop recovery (same as pressing 'f')
ros2 run trial_1 trigger_teleop

# Resume navigation (same as pressing 'r')
ros2 run trial_1 trigger_teleop --resume

# Cancel navigation (same as pressing 'c')
ros2 run trial_1 trigger_teleop --cancel

# Check current status
ros2 run trial_1 trigger_teleop --status
```

Or use topics directly:

```bash
# Trigger teleop recovery
ros2 topic pub --once /trigger_teleop_recovery std_msgs/msg/Empty

# Resume navigation
ros2 topic pub --once /resume_navigation std_msgs/msg/Empty

# Cancel navigation  
ros2 topic pub --once /cancel_navigation std_msgs/msg/Empty

# Monitor state
ros2 topic echo /teleop_recovery/state
```

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
        (press 'c')      │                 │           │
              │          └────────┬────────┘           │
              │                   │                    │
              │     ┌─────────────┼─────────────┐      │
              │     │             │             │      │
              │  'f' key     navigation    teleop      │
              │  pressed      failed       input       │
              │     │             │             │      │
              │     │             ▼             │      │
              │     │    ┌─────────────────┐    │      │
              │     └───►│                 │◄───┘      │
              │          │ TELEOP_RECOVERY │           │
              │          │                 │           │
              └──────────┴────────┬────────┴───────────┘
                                  │
                             'r' key
                             (resume)
                                  │
                                  ▼
                         (back to NAVIGATING
                          with last goal)
```

## ⚙️ Parameters

The `teleop_recovery_node` accepts these parameters (set in launch file):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_sim_time` | True | Use simulation time |
| `teleop_timeout` | 30.0 | Timeout before prompting in teleop mode |
| `cmd_vel_topic` | `/cmd_vel` | Output velocity topic |
| `teleop_cmd_vel_topic` | `/teleop_cmd_vel` | Input from keyboard teleop |
| `auto_resume_on_teleop_idle` | False | Auto-resume when teleop is idle |
| `teleop_idle_timeout` | 5.0 | Seconds of idle before auto-resume |

## 🐛 Troubleshooting

### "Empty Tree" Error from Nav2
```
[ERROR] [BehaviorTreeEngine]: Behavior tree threw exception: Empty Tree
```
**Cause**: Nav2 behavior tree XML not configured properly.  
**Solution**: Use your original `nav2_navigation.launch.py` that was working, not `nav2_with_recovery.launch.py`.

### Keyboard doesn't respond
**Cause**: Terminal 4 (keyboard teleop) doesn't have focus.  
**Solution**: Click on Terminal 4 window to give it focus. The keyboard only works when that terminal is active.

### Robot doesn't move in teleop mode
1. Check state is TELEOP_RECOVERY:
   ```bash
   ros2 topic echo /teleop_recovery/state
   ```
2. Check teleop commands are being sent:
   ```bash
   ros2 topic echo /teleop_cmd_vel
   ```
3. Check cmd_vel is being published:
   ```bash
   ros2 topic echo /cmd_vel
   ```

### Navigation doesn't resume
1. Check a goal was set before triggering recovery
2. Check state:
   ```bash
   ros2 topic echo /teleop_recovery/state
   ```
3. Make sure Nav2 is still running:
   ```bash
   ros2 lifecycle list /bt_navigator
   ```

### "Goal rejected" when resuming
**Cause**: Nav2 may have timed out or crashed.  
**Solution**: Restart Nav2 or set a new goal manually in RViz.

## 📊 Monitoring

View all relevant info at once:

```bash
# In a new terminal, monitor state
watch -n 0.5 'ros2 topic echo /teleop_recovery/state --once 2>/dev/null'

# Or use rqt_graph to visualize node connections
rqt_graph
```

## 📝 Quick Reference Card

```
┌────────────────────────────────────────────────────────┐
│              TELEOP RECOVERY QUICK REFERENCE           │
├────────────────────────────────────────────────────────┤
│  LAUNCH SEQUENCE (4 terminals):                        │
│    T1: ros2 launch trial_1 my_custom_world.launch.py   │
│    T2: ros2 launch trial_1 nav2_navigation.launch.py   │
│    T3: ros2 launch trial_1 teleop_recovery.launch.py   │
│    T4: ros2 run trial_1 teleop_keyboard_recovery       │
├────────────────────────────────────────────────────────┤
│  KEYBOARD (focus Terminal 4):                          │
│    f = FAIL (take over)    r = RESUME navigation       │
│    c = CANCEL goal         s = STOP robot              │
│    w/x = forward/back      a/d = turn left/right       │
│    q = quit teleop                                     │
├────────────────────────────────────────────────────────┤
│  STATES:                                               │
│    IDLE → NAVIGATING → TELEOP_RECOVERY → NAVIGATING    │
└────────────────────────────────────────────────────────┘
```

## 📧 Notes

- The system intercepts `/goal_pose` to track the last goal for resume functionality
- When in TELEOP_RECOVERY mode, Nav2 continues running but teleop has priority
- Compatible with ROS 2 Humble and Nav2
- Works with TurtleBot3 Burger (default) or other models
