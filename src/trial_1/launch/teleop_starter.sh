#!/bin/bash
# Teleop Keyboard Recovery Starter Script

source /opt/ros/humble/setup.bash
source ~/Work/Anthropilot_ROS/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

echo "=== KEYBOARD TELEOP ==="
echo ""
echo "Controls:"
echo "  f = Force teleop mode (takeover)"
echo "  r = Resume navigation"
echo "  c = Cancel navigation"
echo "  w/x = Linear velocity +/-"
echo "  a/d = Angular velocity +/-"
echo "  s = Stop"
echo "  q = Quit"
echo ""
echo "Waiting for systems to initialize..."
sleep 20

ros2 run trial_1 teleop_keyboard_recovery
