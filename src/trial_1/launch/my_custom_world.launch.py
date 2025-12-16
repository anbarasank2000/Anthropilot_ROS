from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    world_file = os.path.join(
        get_package_share_directory('trial_1'),
        'worlds',
        'world3.world'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file}.items(),
    )

    # Add robot_state_publisher
    robot_description_file = os.path.join(
        get_package_share_directory('trial_1'),
        'urdf',
        'tb3_fixed.urdf'
    )

    with open(robot_description_file, 'r') as infp:
        robot_description = infp.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
        }]
    )



    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'spawn_turtlebot3.launch.py')
        ]),
        launch_arguments={
            'x_pose': '0.0',
            'y_pose': '0.0',
            'z_pose': '0.0',
            'model': 'waffle_pi'
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('slam_toolbox'), 'rviz', 'slam_toolbox.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        rsp_node,        # ✅ Added this
        spawn_turtlebot,
        rviz_node
    ])
