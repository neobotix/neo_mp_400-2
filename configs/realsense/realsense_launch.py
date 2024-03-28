# Author: Pradheep Padmanabhan

import launch
import os

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    d435 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('realsense2_camera'),
                    'launch',
                    'rs_launch.py')
            )
        )

    return LaunchDescription([
        d435
    ])
