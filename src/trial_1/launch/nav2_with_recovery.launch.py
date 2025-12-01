"""
Nav2 Navigation Launch with Teleop Recovery Integration

This launch file starts:
1. Nav2 navigation stack (using default Nav2 params with proper BT XML)
2. Teleop recovery system

Usage:
    ros2 launch trial_1 nav2_with_recovery.launch.py

IMPORTANT: For keyboard teleop, run in a SEPARATE terminal:
    ros2 run trial_1 teleop_keyboard_recovery
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    trial_1_share = get_package_share_directory('trial_1')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(trial_1_share, 'map', 'map.yaml'),
        description='Full path to map yaml file'
    )
    
    auto_resume_arg = DeclareLaunchArgument(
        'auto_resume',
        default_value='False',
        description='Auto-resume navigation when teleop is idle'
    )
    
    # Nav2 bringup - DO NOT override params_file to use Nav2 defaults with proper BT XML
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam': 'False',
            # Using Nav2 default params which have proper behavior tree XML paths
        }.items(),
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
            'nav_cmd_vel_topic': '/cmd_vel_nav',
            'auto_resume_on_teleop_idle': LaunchConfiguration('auto_resume'),
            'teleop_idle_timeout': 5.0
        }]
    )
    
    # Log info about keyboard teleop
    keyboard_info = LogInfo(
        msg="\n\n" + "="*70 + "\n" +
            "  KEYBOARD TELEOP: Run this in a SEPARATE terminal:\n\n" +
            "      ros2 run trial_1 teleop_keyboard_recovery\n\n" +
            "  Keys: f=FAIL/takeover, r=RESUME, c=CANCEL, wasd=move, q=quit\n" +
            "="*70 + "\n"
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        map_file_arg,
        auto_resume_arg,
        keyboard_info,
        nav2_launch,
        teleop_recovery_node,
    ])
