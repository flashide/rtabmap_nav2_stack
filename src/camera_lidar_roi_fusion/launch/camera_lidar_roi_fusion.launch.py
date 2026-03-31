#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument('pointcloud_topic', default_value='/cloud_registered_body'),
        DeclareLaunchArgument('camera_info_topic', default_value='/sensors/camera/rgb/camera_info'),
        DeclareLaunchArgument('detections_topic', default_value='/detections'),
        DeclareLaunchArgument('fused_detections_topic', default_value='/perception/fused_detections'),
        DeclareLaunchArgument('fused_cloud_topic', default_value='/perception/fused_cloud'),
        DeclareLaunchArgument('marker_topic', default_value='/perception/fusion_markers'),
        DeclareLaunchArgument('min_depth_m', default_value='0.5'),
        DeclareLaunchArgument('max_depth_m', default_value='60.0'),
        DeclareLaunchArgument('distance_tolerance_m', default_value='1.5'),
        DeclareLaunchArgument('depth_quantile', default_value='0.35'),
        DeclareLaunchArgument('min_points_per_roi', default_value='3'),
        DeclareLaunchArgument('sync_slop_s', default_value='0.25'),
    ]

    node = Node(
        package='camera_lidar_roi_fusion',
        executable='camera_lidar_roi_fusion.py',
        name='camera_lidar_roi_fusion',
        output='screen',
        parameters=[{
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'detections_topic': LaunchConfiguration('detections_topic'),
            'fused_detections_topic': LaunchConfiguration('fused_detections_topic'),
            'fused_cloud_topic': LaunchConfiguration('fused_cloud_topic'),
            'marker_topic': LaunchConfiguration('marker_topic'),
            'min_depth_m': LaunchConfiguration('min_depth_m'),
            'max_depth_m': LaunchConfiguration('max_depth_m'),
            'distance_tolerance_m': LaunchConfiguration('distance_tolerance_m'),
            'depth_quantile': LaunchConfiguration('depth_quantile'),
            'min_points_per_roi': LaunchConfiguration('min_points_per_roi'),
            'sync_slop_s': LaunchConfiguration('sync_slop_s'),
        }],
    )

    ld = LaunchDescription(args)
    ld.add_action(node)
    return ld
