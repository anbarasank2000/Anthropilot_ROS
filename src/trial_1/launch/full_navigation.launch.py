"""
Full Navigation Launch File
Launches all necessary components for TurtleBot3 navigation with teleop recovery
in separate terminal windows with a single command.

Terminals launched:
1. Gazebo simulation with TurtleBot3
2. Nav2 navigation stack
3. Teleop recovery state machine
4. Keyboard teleop (for manual control)
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('trial_1')

    # Common environment setup commands
    env_setup = (
        'source /opt/ros/humble/setup.bash && '
        'source ~/Work/Anthropilot_ROS/install/setup.bash && '
        'export TURTLEBOT3_MODEL=burger && '
        'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/Work/Anthropilot_ROS/src'
    )

    # Terminal 1: Gazebo simulation
    gazebo_cmd = (
        f'{env_setup} && '
        'echo "=== GAZEBO SIMULATION ===" && '
        'ros2 launch trial_1 my_custom_world.launch.py'
    )

    gazebo_terminal = ExecuteProcess(
        cmd=['xterm', '-title', 'Gazebo Simulation', '-fa', 'Monospace', '-fs', '10',
             '-bg', 'black', '-fg', 'green', '-e', 'bash', '-c', gazebo_cmd],
        output='screen'
    )

    # Terminal 2: Nav2 navigation (delayed to let Gazebo start)
    nav2_cmd = (
        f'{env_setup} && '
        'echo "=== NAV2 NAVIGATION ===" && '
        'echo "Waiting for Gazebo to initialize..." && '
        'sleep 10 && '
        'ros2 launch trial_1 nav2_navigation.launch.py'
    )

    nav2_terminal = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['xterm', '-title', 'Nav2 Navigation', '-fa', 'Monospace', '-fs', '10',
                     '-bg', 'black', '-fg', 'cyan', '-e', 'bash', '-c', nav2_cmd],
                output='screen'
            )
        ]
    )

    # Terminal 3: Teleop recovery system (delayed further)
    recovery_cmd = (
        f'{env_setup} && '
        'echo "=== TELEOP RECOVERY SYSTEM ===" && '
        'echo "Waiting for Nav2 to initialize..." && '
        'sleep 20 && '
        'ros2 launch trial_1 teleop_recovery.launch.py'
    )

    recovery_terminal = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['xterm', '-title', 'Teleop Recovery', '-fa', 'Monospace', '-fs', '10',
                     '-bg', 'black', '-fg', 'yellow', '-e', 'bash', '-c', recovery_cmd],
                output='screen'
            )
        ]
    )

    # Terminal 4: Keyboard teleop (delayed, needs user focus)
    teleop_cmd = (
        f'{env_setup} && '
        'echo "=== KEYBOARD TELEOP ===" && '
        'echo "" && '
        'echo "Controls:" && '
        'echo "  f = Force teleop mode (takeover)" && '
        'echo "  r = Resume navigation" && '
        'echo "  c = Cancel navigation" && '
        'echo "  w/x = Linear velocity +/-" && '
        'echo "  a/d = Angular velocity +/-" && '
        'echo "  s = Stop" && '
        'echo "  q = Quit" && '
        'echo "" && '
        'echo "Waiting for recovery system..." && '
        'sleep 25 && '
        'ros2 run trial_1 teleop_keyboard_recovery'
    )

    teleop_terminal = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['xterm', '-title', 'Keyboard Teleop - FOCUS HERE', '-fa', 'Monospace', '-fs', '12',
                     '-bg', '#1a1a2e', '-fg', 'white', '-geometry', '80x30',
                     '-e', 'bash', '-c', teleop_cmd],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo_terminal,
        nav2_terminal,
        recovery_terminal,
        teleop_terminal,
    ])
