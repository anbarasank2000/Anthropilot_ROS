"""
Full Navigation Launch File
Launches TurtleBot3 Waffle Pi with Nav2, teleop recovery, and camera view.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('trial_1')

    # === 1. Gazebo simulation (from my_custom_world.launch.py) ===
    world_file = os.path.join(pkg_share, 'worlds', 'world3.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    robot_description_file = os.path.join(pkg_share, 'urdf', 'tb3_fixed.urdf')
    with open(robot_description_file, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}]
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pose': '0.0', 'y_pose': '0.0', 'z_pose': '0.0', 'model': 'waffle_pi'}.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('slam_toolbox'), 'rviz', 'slam_toolbox.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    # === 2. Nav2 + SLAM (delayed start via shell) ===
    nav2_cmd = ExecuteProcess(
        cmd=['bash', '-c',
             'sleep 12 && ros2 launch nav2_bringup bringup_launch.py '
             'use_sim_time:=True slam:=True '
             f'map:={os.path.join(pkg_share, "map", "map.yaml")}'],
        output='screen'
    )

    slam_node = ExecuteProcess(
        cmd=['bash', '-c',
             'sleep 12 && ros2 run slam_toolbox sync_slam_toolbox_node --ros-args -p use_sim_time:=true'],
        output='screen'
    )

    # === 3. Teleop recovery node (delayed) ===
    teleop_recovery = ExecuteProcess(
        cmd=['bash', '-c',
             'sleep 20 && ros2 run trial_1 teleop_recovery_node --ros-args -p use_sim_time:=true'],
        output='screen'
    )

    # === 4. Interactive keyboard teleop (xterm window) ===
    teleop_script = os.path.join(pkg_share, 'launch', 'teleop_starter.sh')
    keyboard_teleop = ExecuteProcess(
        cmd=['bash', '-c', f'sleep 5 && xterm -title "Keyboard Teleop" -fa Monospace -fs 12 -geometry 60x25 -e bash {teleop_script}'],
        output='screen'
    )

    # === 5. Camera view (delayed GUI) ===
    camera_view = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 15 && ros2 run rqt_image_view rqt_image_view /camera/image_raw'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        rviz,
        nav2_cmd,
        slam_node,
        teleop_recovery,
        keyboard_teleop,
        camera_view,
    ])
