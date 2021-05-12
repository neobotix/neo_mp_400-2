# Neobotix GmbH

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node
import os
from pathlib import Path

def generate_launch_description():
    neo_mp_400 = get_package_share_directory('neo_mp_400')

    urdf = os.path.join(get_package_share_directory('neo_mp_400'), 'robot_model/mp_400', 'mp_400.urdf')

    with open(urdf, 'r') as infp:  
        robot_desc = infp.read() # Dummy to use parameter instead of using argument=[urdf] in Node. Reference page: https://github.com/ros2/demos/pull/426/commits/a35a25732159e4c8b5655755ce31ec4c3e6e7975

    rsp_params = {'robot_description': robot_desc}

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[rsp_params])

    relayboard = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(neo_mp_400, 'configs/relayboard_v2', 'relayboard_v2.launch.py')
            )
        )

    kinematics = IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 os.path.join(neo_mp_400, 'configs/kinematics', 'kinematics.launch.py')
             )
         )
    teleop = IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 os.path.join(neo_mp_400, 'configs/teleop', 'teleop.launch.py')
             )
         )

    laser = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(neo_mp_400, 'configs/lidar/sick/s300', 'sick_s300.launch.py')
            )
        )


    return LaunchDescription([relayboard, start_robot_state_publisher_cmd, laser, kinematics, teleop])
