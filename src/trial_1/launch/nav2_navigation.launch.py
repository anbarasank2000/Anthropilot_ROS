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
            'map': map_file,
            'use_sim_time': 'True',
            'slam': 'False'
        }.items(),
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        nav2_launch,
        amcl
    ])
