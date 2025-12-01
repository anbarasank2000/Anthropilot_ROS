"""
Nav2 Navigation Launch with Teleop Recovery Integration

This launch file starts:
1. Nav2 navigation stack
2. Teleop recovery system
3. Remaps cmd_vel topics appropriately

Usage:
    ros2 launch trial_1 nav2_with_recovery.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
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
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(trial_1_share, 'config', 'nav2_params.yaml'),
        description='Full path to nav2 params file'
    )
    
    auto_resume_arg = DeclareLaunchArgument(
        'auto_resume',
        default_value='False',
        description='Auto-resume navigation when teleop is idle'
    )
    
    # Nav2 bringup - with remapped cmd_vel to cmd_vel_nav
    # The teleop_recovery_node will manage switching between nav and teleop
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam': 'False',
            'params_file': LaunchConfiguration('params_file')
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
    
    # Keyboard Teleop in separate terminal (optional - can be started separately)
    teleop_keyboard = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--title=Teleop Recovery Keyboard', '--',
            'bash', '-c',
            'sleep 2 && ros2 run trial_1 teleop_keyboard_recovery; exec bash'
        ],
        output='screen',
        shell=False
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        map_file_arg,
        params_file_arg,
        auto_resume_arg,
        nav2_launch,
        teleop_recovery_node,
        teleop_keyboard
    ])
