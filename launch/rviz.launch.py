# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os
from pathlib import Path
from launch.conditions import IfCondition
from nav2_common.launch import ReplaceString

def generate_launch_description():
    namespace = LaunchConfiguration('robot_namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default='False')
    
    rviz_config_dir = os.path.join(
        get_package_share_directory('neo_mp_400-2'),
        'configs/rviz',
        'test_rviz.rviz')

    rviz_namespaced_config_dir = os.path.join(
        get_package_share_directory('neo_mp_400-2'),
        'configs/rviz',
        'test_namespaced_rviz.rviz')

    namespaced_rviz_config_file = ReplaceString(
        source_file=rviz_namespaced_config_dir,
        replacements={'<robot_namespace>': ('/', namespace)})

    rviz =   Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen',
            condition=IfCondition(PythonExpression(['not ', use_namespace])))

    rviz_namespaced =  Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace= namespace,
            arguments=['-d', namespaced_rviz_config_file],
            output='screen',
            condition=IfCondition(use_namespace))

    return LaunchDescription([rviz, rviz_namespaced])
