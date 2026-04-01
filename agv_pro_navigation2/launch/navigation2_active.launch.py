import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    fastlio_config_file = LaunchConfiguration('fastlio_config_file')
    fastlio_config_path = LaunchConfiguration('fastlio_config_path')
    scan_min_height = LaunchConfiguration('scan_min_height')
    scan_max_height = LaunchConfiguration('scan_max_height')

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('agv_pro_navigation2'),
            'map',
            'map.yaml'))

    param_file_name = 'agvpro.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('agv_pro_navigation2'),
            'param',
            param_file_name))

    fast_lio_dir = get_package_share_directory('fast_lio')
    livox_dir = get_package_share_directory('livox_ros_driver2')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    fastlio_launch = os.path.join(fast_lio_dir, 'launch', 'mapping.launch.py')
    livox_config = os.path.join(livox_dir, 'config', 'MID360_config.json')

    rviz_config_dir = os.path.join(
        get_package_share_directory('agv_pro_navigation2'),
        'rviz',
        'agvpro_navigation2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),
    
        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'fastlio_config_path',
            default_value=os.path.join(fast_lio_dir, 'config'),
            description='Full path to FAST-LIO config directory'),

        DeclareLaunchArgument(
            'fastlio_config_file',
            default_value='mid360.yaml',
            description='FAST-LIO config file name'),

        DeclareLaunchArgument(
            'scan_min_height',
            default_value='-0.15',
            description='Minimum height kept when projecting point cloud to /scan'),

        DeclareLaunchArgument(
            'scan_max_height',
            default_value='0.60',
            description='Maximum height kept when projecting point cloud to /scan'),

        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[{
                'xfer_format': 0,
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'output_data_type': 0,
                'frame_id': 'laser_link',
                'lvx_file_path': '/home/livox/livox_test.lvx',
                'user_config_path': livox_config,
                'cmdline_input_bd_code': 'livox0000000001',
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0.12', '0.0', '0.10', '0', '0', '0', 'base_link', 'laser_link'],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fastlio_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'config_path': fastlio_config_path,
                'config_file': fastlio_config_file,
                'rviz': 'false'}.items(),
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='mid360_pointcloud_to_laserscan',
            output='screen',
            remappings=[
                ('cloud_in', '/cloud_registered_body'),
                ('scan', '/scan')
            ],
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.05,
                'min_height': scan_min_height,
                'max_height': scan_max_height,
                'angle_min': -3.141592653589793,
                'angle_max': 3.141592653589793,
                'angle_increment': 0.004363323129985824,
                'scan_time': 0.1,
                'range_min': 0.25,
                'range_max': 30.0,
                'use_inf': True,
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'),
    ])
