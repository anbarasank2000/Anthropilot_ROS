"""
Teleop Recovery Launch - Run alongside your existing Nav2 setup

This launch file starts ONLY the teleop recovery system.
Use this with your existing nav2_navigation.launch.py

Usage:
    Terminal 1: ros2 launch trial_1 my_custom_world.launch.py
    Terminal 2: ros2 launch trial_1 nav2_navigation.launch.py  (your existing)
    Terminal 3: ros2 launch trial_1 teleop_recovery.launch.py  (this file)
    Terminal 4: ros2 run trial_1 teleop_keyboard_recovery      (keyboard control)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    
    auto_resume_arg = DeclareLaunchArgument(
        'auto_resume',
        default_value='False',
        description='Auto-resume navigation when teleop is idle'
    )
    
    # Teleop Recovery Node
    teleop_recovery_node = Node(
        package='trial_1',
        executable='teleop_recovery_node',
        name='teleop_recovery_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'teleop_timeout': 30.0,
            'cmd_vel_topic': '/cmd_vel',
            'teleop_cmd_vel_topic': '/teleop_cmd_vel',
            'nav_cmd_vel_topic': '/nav_cmd_vel',
            'auto_resume_on_teleop_idle': LaunchConfiguration('auto_resume'),
            'teleop_idle_timeout': 5.0
        }]
    )
    
    # Log info
    info_msg = LogInfo(
        msg="\n\n" + "="*70 + "\n" +
            "  TELEOP RECOVERY NODE STARTED\n\n" +
            "  Run keyboard teleop in a NEW terminal:\n" +
            "      ros2 run trial_1 teleop_keyboard_recovery\n\n" +
            "  Keyboard controls:\n" +
            "      f = FAIL/takeover (switch to manual control)\n" +
            "      r = RESUME navigation\n" +
            "      c = CANCEL navigation\n" +
            "      w/x = forward/backward\n" +
            "      a/d = turn left/right\n" +
            "      s = stop\n" +
            "      q = quit\n" +
            "="*70 + "\n"
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        auto_resume_arg,
        info_msg,
        teleop_recovery_node,
    ])
