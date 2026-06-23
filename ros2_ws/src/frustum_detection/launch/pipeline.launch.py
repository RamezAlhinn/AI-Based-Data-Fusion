"""
Full pipeline launch file.

Starts:
  1. painting_node    — YOLO segmentation + PointPainting
  2. frustum_node     — Frustum 3D detection + AB3DMOT tracking
  3. metrics_logger_node — CSV metrics logger
  4. ros2 bag play    — replays the student dataset bag

Usage (inside devcontainer, after colcon build + source install/setup.bash):

    ros2 launch frustum_detection pipeline.launch.py

Override any argument:
    ros2 launch frustum_detection pipeline.launch.py \
        bag_path:=/workspace/studentProject1 \
        calib_file:=/workspace/calib.txt \
        bag_rate:=0.1 \
        output_dir:=/workspace/metrics
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/workspace/studentProject1',
        description='Path to the ROS 2 bag directory',
    )
    calib_arg = DeclareLaunchArgument(
        'calib_file',
        default_value='/workspace/calib.txt',
        description='Path to KITTI-format calib.txt',
    )
    bag_rate_arg = DeclareLaunchArgument(
        'bag_rate',
        default_value='0.1',
        description='Bag playback rate (0.1 = 10x slower for CPU-only YOLO)',
    )
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/workspace/metrics',
        description='Directory to write metrics CSV files',
    )

    # ── painting_node ─────────────────────────────────────────────────────────
    painting_node = Node(
        package='point_painting',
        executable='painting_node',
        name='painting_node',
        output='screen',
        parameters=[{
            'calib_file':     LaunchConfiguration('calib_file'),
            'use_sim_time':   True,
        }],
    )

    # ── frustum_node ──────────────────────────────────────────────────────────
    frustum_node = Node(
        package='frustum_detection',
        executable='frustum_node',
        name='frustum_node',
        output='screen',
        parameters=[{
            'calib_file':   LaunchConfiguration('calib_file'),
            'use_sim_time': True,
        }],
    )

    # ── bev_recorder_node ─────────────────────────────────────────────────────
    bev_recorder_node = Node(
        package='frustum_detection',
        executable='bev_recorder_node',
        name='bev_recorder_node',
        output='screen',
        parameters=[{
            'output_dir':   '/workspace/video_output',
            'use_sim_time': True,
        }],
    )

    # ── metrics_logger_node ───────────────────────────────────────────────────
    metrics_node = Node(
        package='frustum_detection',
        executable='metrics_logger_node',
        name='metrics_logger_node',
        output='screen',
        parameters=[{
            'output_dir':   LaunchConfiguration('output_dir'),
            'use_sim_time': True,
        }],
    )

    # ── bag play (delayed 3 s to let nodes initialise first) ─────────────────
    bag_play = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'play',
                    LaunchConfiguration('bag_path'),
                    '--clock',
                    '--loop',
                    '--rate', LaunchConfiguration('bag_rate'),
                ],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        bag_path_arg,
        calib_arg,
        bag_rate_arg,
        output_dir_arg,
        painting_node,
        frustum_node,
        metrics_node,
        bev_recorder_node,
        bag_play,
    ])
