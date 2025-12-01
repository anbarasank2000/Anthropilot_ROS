# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# import os

# def generate_launch_description():
#     map_file = os.path.join(
#         get_package_share_directory('trial_1'),
#         'map',
#         'map.yaml'
#     )

#     nav2_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
#         ]),
#         launch_arguments={
#             'map': map_file,
#             'use_sim_time': 'True',
#             'slam': 'False'
#         }.items(),
#     )

#     return LaunchDescription([
#         nav2_launch
#     ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Map file path (required by Nav2 bringup even in SLAM mode, but won't be used for localization)
    map_file = os.path.join(
        get_package_share_directory('trial_1'),
        'map',
        'map.yaml'
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'map': map_file,  # Required parameter, but not used in SLAM mode
            'use_sim_time': 'True',
            'slam': 'True'  # Enable SLAM mode for simultaneous mapping and navigation
        }.items(),
    )

    # SLAM Toolbox node for online SLAM
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        nav2_launch,
        slam_toolbox_node  # AMCL removed - SLAM handles localization
    ])
