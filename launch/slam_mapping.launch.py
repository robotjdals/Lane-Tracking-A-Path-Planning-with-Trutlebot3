#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    SLAM Toolboxë¥¼ ì‚¬ìš©í•˜ì—¬ ë§µì„ ìƒì„±í•˜ëŠ” launch íŒŒì¼

    ì‚¬ìš©ë²•:
    1. í„°ë¯¸ë„ 1: ros2 launch turtlebot3_gazebo turtlebot3_autorace_2020.launch.py
    2. í„°ë¯¸ë„ 2: ros2 launch min_22_pkg slam_mapping.launch.py
    3. í„°ë¯¸ë„ 3: ros2 run turtlebot3_teleop teleop_keyboard (ë§µ íƒìƒ‰)
    4. í„°ë¯¸ë„ 4: ros2 run nav2_map_server map_saver_cli -f ~/colcon_ws/src/.../maps/my_map
    """

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # SLAM Toolbox ì‹¤í–‰
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'base_frame': 'base_footprint',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'mapping',
            }
        ],
    )

    # RViz ì‹¤í–‰ (ë§µ ì‹œê°í™”)
    rviz_config_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),

        slam_toolbox_node,
        rviz_node,
    ])
