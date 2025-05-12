# Neobotix GmbH
# Author: Pradheep Padmanabhan
# Contributor: Adarsh Karan K P

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    OpaqueFunction
    )
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def execution_stage(context: LaunchContext,
                    robot_namespace,
                    world,
                    imu_enable,
                    d435_enable,
                    uss_enable,
                    scanner_type):

    launch_actions = []

    robot_type = "mp_400"
    world_name = str(world.perform(context))

    # Launch bringup_sim file from mp_bringup package
    bringup_sim_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mp_bringup'), 'launch', 'bringup_sim.launch.py')
        ),
        launch_arguments={
            'robot_namespace': robot_namespace,
            'robot_type': robot_type,
            'world': world_name,
            'imu_enable': imu_enable,
            'd435_enable': d435_enable,
            'use_uss': uss_enable,
            'scanner_type': scanner_type,
        }.items(),
    )

    # Add the launch command to the launch actions
    launch_actions.append(bringup_sim_launch_cmd)

    return launch_actions

def generate_launch_description():

    declare_namespace_cmd = DeclareLaunchArgument(
            'robot_namespace', default_value='', description='Top-level namespace'
        )

    declare_world_name_arg = DeclareLaunchArgument(
            'world',
            default_value='neo_workshop',
            choices=['', 'neo_workshop'],
            description='Simulation world to load'
        )

    declare_imu_cmd = DeclareLaunchArgument(
            'imu_enable', default_value='False',
            description='Enable IMU - Options: True/False'
        )

    declare_realsense_cmd = DeclareLaunchArgument(
            'd435_enable', default_value='False',
            description='Enable Intel RealSense D435 camera if true'
        )

    declare_use_uss_cmd = DeclareLaunchArgument(
            'use_uss', default_value='False',
            description='Enable Ultrasonic sensors if true'
        )

    declare_scanner_type_cmd = DeclareLaunchArgument(
            'scanner_type', default_value='sick_s300',
            choices=['', 'sick_s300', 'sick_microscan3'],
            description='Type of laser scanner to use\n\t'
        )

    opq_function = OpaqueFunction(
        function=execution_stage,
        args=[
            LaunchConfiguration('robot_namespace'),
            LaunchConfiguration('world'),
            LaunchConfiguration('imu_enable'),
            LaunchConfiguration('d435_enable'),
            LaunchConfiguration('use_uss'),
            LaunchConfiguration('scanner_type'),
        ])

    return LaunchDescription([
        declare_namespace_cmd,
        declare_world_name_arg,
        declare_imu_cmd,
        declare_realsense_cmd,
        declare_use_uss_cmd,
        declare_scanner_type_cmd,
        opq_function
    ])
