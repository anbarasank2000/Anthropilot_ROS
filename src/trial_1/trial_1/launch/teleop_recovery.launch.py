"""
Launch file for Teleop Recovery System

This launch file starts:
1. The teleop recovery node that manages navigation/teleop switching
2. The keyboard teleop node with recovery integration

Usage:
    ros2 launch trial_1 teleop_recovery.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    
    teleop_timeout_arg = DeclareLaunchArgument(
        'teleop_timeout',
        default_value='30.0',
        description='Timeout in teleop mode before prompting'
    )
    
    auto_resume_arg = DeclareLaunchArgument(
        'auto_resume_on_teleop_idle',
        default_value='False',
        description='Automatically resume navigation when teleop is idle'
    )
    
    # Teleop Recovery Node
    teleop_recovery_node = Node(
        package='trial_1',
        executable='teleop_recovery_node',
        name='teleop_recovery_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'teleop_timeout': LaunchConfiguration('teleop_timeout'),
            'cmd_vel_topic': '/cmd_vel',
            'teleop_cmd_vel_topic': '/teleop_cmd_vel',
            'nav_cmd_vel_topic': '/cmd_vel_nav',
            'auto_resume_on_teleop_idle': LaunchConfiguration('auto_resume_on_teleop_idle'),
            'teleop_idle_timeout': 5.0
        }]
    )
    
    # Keyboard Teleop in separate terminal
    teleop_keyboard_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'trial_1', 'teleop_keyboard_recovery'],
        output='screen',
        shell=False
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        teleop_timeout_arg,
        auto_resume_arg,
        teleop_recovery_node,
        teleop_keyboard_node
    ])
