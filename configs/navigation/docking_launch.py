# Author: Pradheep Padmanabhan

import launch
import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    namespace = launch.substitutions.LaunchConfiguration('namespace', default="")

    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    neo_docking2 = os.path.join(get_package_share_directory('neo_docking2'), 'launch')

    docking = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([neo_docking2, '/docking_launch.py']),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time}.items(),
        )

    return LaunchDescription([
        docking
    ])
