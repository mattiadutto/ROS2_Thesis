#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    # Specify the name of the package
    pkg_name = 'scout_navigation'

    # Namespace definition
    namespace = 'scout_mini'

    # Rviz directory path of the Rviz Launch File (part of nav2_bringup package)
    rviz_dir = os.path.join(get_package_share_directory('nav2_bringup'))

    rviz_dir_local = os.path.join(get_package_share_directory('scout_navigation'))

    # Config file directory path of the scout_navigation package
    config_file_dir = os.path.join(get_package_share_directory(pkg_name), 'config')

    # Nav2 launch directory path (part of nav2_bringup pacakge)
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # Map launch config with the default map to be used
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory(pkg_name),
            'maps',
            'my_map.yaml'))
    
    # Nav2 parameters launch config with the default yaml file 
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory(pkg_name),
            'config',
            'nav2_params.yaml'))
 
    # Sim time config 
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Specifying the given namespace in EKF nodes
    namespaced_ekf_localization_params = ReplaceString(
        source_file=os.path.join(config_file_dir, 'ekf_localization_with_gps.yaml'), 
        replacements={'namespace': namespace})
    
    # Launch arguments 
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
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'),
        # Launch bringup_launch.py to bringup NAV2 nodes
        # (yaml file contains <robot_namespace> to be replaced with actual namespace according to bringup_launch.py)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_namespace': 'True',
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        # Launch rviz_launch.py (part of nav2_bringup package)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/rviz_launch.py']),
            launch_arguments={
                'use_namespace': 'True',
                'namespace': namespace,
                 #'rviz_config': os.path.join(rviz_dir_local, 'rviz', 'nav2_namespaced_view.rviz')}.items(),
                 'rviz_config': os.path.join(rviz_dir_local, 'rviz', 'config.rviz')}.items(),
        
        
        ),
        # Launch EKF nodes
        Node(
         namespace=namespace,
         package='robot_localization',
         executable='ekf_node',
         name='ekf_local_filter_node',
         output='screen',
         parameters=[namespaced_ekf_localization_params, {'use_sim_time': use_sim_time}],
         remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('odometry/filtered', 'odometry/filtered/local')
         ]
        ),

         Node(
         namespace=namespace,
         package='robot_localization',
         executable='ekf_node',
         name='ekf_global_filter_node',
         output='screen',
         parameters=[namespaced_ekf_localization_params, {'use_sim_time': use_sim_time}],
         remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('odometry/filtered', 'odometry/filtered/global')
         ]
    )           
    ]) 


