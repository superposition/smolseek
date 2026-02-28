"""Master launch file for autonomous VSLAM mapping.

Launches all nodes with timed sequencing:
  t=0s:  stella_vslam + Cartographer + map_bridge + point_cloud_ws
  t=10s: Nav2 + nav2_goal_sender (waits for initial map)
  t=20s: explore_lite (waits for Nav2)
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Arguments
    timeout_minutes = LaunchConfiguration('timeout_minutes', default='10')
    map_db_path = LaunchConfiguration(
        'map_db_path', default='/home/ws/ugv_ws/maps/current_map.msg'
    )
    vocab_path = LaunchConfiguration(
        'vocab_path', default='/home/ws/ugv_ws/vocab/orb_vocab.fbow'
    )
    config_path = LaunchConfiguration(
        'config_path', default='/home/ws/ugv_ws/src/smolROS/config/ugv_monocam.yaml'
    )

    # ---- t=0s: Core SLAM nodes ----

    # stella_vslam ROS2 node (uses CLI args, not ROS params)
    stella_vslam_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'stella_vslam_ros', 'run_slam',
            '-v', '/home/ws/ugv_ws/vocab/orb_vocab.fbow',
            '-c', '/home/ws/ugv_ws/src/smolROS/config/ugv_monocam.yaml',
            '-o', '/home/ws/maps/current_map.msg',
            '--viewer', 'none',
        ],
        output='screen',
    )

    # Cartographer for 2D occupancy grid
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', '/home/ws/ugv_ws/src/ugv_else/cartographer/config',
            '-configuration_basename', 'mapping_2d.lua',
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
            ('imu', '/imu/data'),
        ],
    )

    # Cartographer occupancy grid node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'resolution': 0.05,
        }],
    )

    # Map bridge node
    map_bridge_node = Node(
        package='ugv_map_bridge',
        executable='map_bridge_node',
        name='map_bridge_node',
        output='screen',
        parameters=[{
            'map_db_path': map_db_path,
            'extract_interval': 5.0,
            'frame_id': 'map',
        }],
    )

    # WebSocket server (now with bidirectional game commands)
    ws_server_node = Node(
        package='ugv_map_bridge',
        executable='point_cloud_ws',
        name='point_cloud_ws',
        output='screen',
        parameters=[{
            'ws_port': 9090,
            'ws_host': '0.0.0.0',
            'full_refresh_interval': 30.0,
        }],
    )

    # ---- t=10s: Nav2 + Nav2 goal sender ----
    nav2_bringup = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='Starting Nav2...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'navigation_launch.py',
                    )
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': '/home/ws/ugv_ws/src/smolROS/config/nav2_exploration.yaml',
                }.items(),
            ),
            # Nav2 goal sender — bridges /smolseek/goal to Nav2 NavigateToPose
            LogInfo(msg='Starting nav2_goal_sender...'),
            Node(
                package='ugv_map_bridge',
                executable='nav2_goal_sender',
                name='nav2_goal_sender',
                output='screen',
            ),
        ],
    )

    # ---- t=20s: explore_lite ----
    explore_lite = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg='Starting frontier exploration...'),
            Node(
                package='explore_lite',
                executable='explore',
                name='explore_lite',
                output='screen',
                parameters=[
                    '/home/ws/ugv_ws/src/smolROS/config/explore_lite.yaml'
                ],
            ),
        ],
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('timeout_minutes', default_value='10'),
        DeclareLaunchArgument('map_db_path', default_value='/home/ws/ugv_ws/maps/current_map.msg'),
        DeclareLaunchArgument('vocab_path', default_value='/home/ws/ugv_ws/vocab/orb_vocab.fbow'),
        DeclareLaunchArgument('config_path', default_value='/home/ws/ugv_ws/src/smolROS/config/ugv_monocam.yaml'),

        # Static TF: bridge pt_camera_link -> camera frame for stella_vslam
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'pt_camera_link', 'camera'],
        ),

        # t=0s: Core nodes
        LogInfo(msg='Starting stella_vslam + Cartographer + map_bridge...'),
        stella_vslam_node,
        cartographer_node,
        occupancy_grid_node,
        map_bridge_node,
        ws_server_node,

        # t=10s: Navigation + goal sender
        nav2_bringup,

        # t=20s: Exploration
        explore_lite,
    ])
