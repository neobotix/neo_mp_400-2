# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
import xacro
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  IncludeLaunchDescription,
  ExecuteProcess,
  OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

from launch.launch_context import LaunchContext
from launch.conditions import IfCondition

def execution_stage(context: LaunchContext,
                    robot_namespace,
                    imu_enable,
                    d435_enable,
                    uss_enable):
    
    imu_enabl = str(imu_enable.perform(context))
    d435_enabl = str(d435_enable.perform(context))
    uss_enabl = str(uss_enable.perform(context))

    rp_ns = ""
    if (robot_namespace.perform(context) != "/"):
        rp_ns = robot_namespace.perform(context) + "/"

    launches = []

    # Setting up the URDF
    urdf = os.path.join(get_package_share_directory('neo_mp_400-2'),
        'robot_model/mp_400',
        'mp_400.urdf.xacro')
    
    # Start robot state publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace=robot_namespace,
        parameters=[{
            'robot_description': Command([
            "xacro", " ", urdf,
            " ", 'use_imu:=',
            imu_enabl,
            " ", 'use_d435:=',
            d435_enabl,
            " ", 'use_uss:=',
            uss_enabl
            ]), 'frame_prefix': rp_ns}],
		arguments=[urdf])

    launches.append(start_robot_state_publisher_cmd)

    # 5. IMU
    imu = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('neo_mp_400-2'),
                    'configs/phidget_imu',
                    'imu_launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items(),
            condition=IfCondition(imu_enable)
        )
    
    launches.append(imu)

    # 6. D435
    # TODO: Add support for namespacing
    d435 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('neo_mp_400-2'),
                    'configs/realsense',
                    'realsense_launch.py')
            ),
            condition=IfCondition(d435_enable)
        )

    launches.append(d435)

    return launches

def generate_launch_description():
    neo_mp_400 = get_package_share_directory('neo_mp_400-2')

    # Launch configurations
    robot_namespace = LaunchConfiguration('robot_namespace')
    imu_enable = LaunchConfiguration('imu_enable')
    realsense_enable = LaunchConfiguration('d435_enable')
    uss_enable = LaunchConfiguration('uss_enable')

    context_arguments = [robot_namespace, imu_enable, realsense_enable, uss_enable]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
            'robot_namespace', default_value='', description='Top-level namespace'
        )
    
    declare_imu_cmd = DeclareLaunchArgument(
            'imu_enable', default_value='False',
            description='Enable IMU - Options: True/False'
        )
    
    declare_realsense_cmd = DeclareLaunchArgument(
            'd435_enable', default_value='False',
            description='Enable Realsense - Options: True/False'
        )
    
    declare_uss_cmd = DeclareLaunchArgument(
            'uss_enable', default_value='False',
            description='Enable uss - Options: True/False'
    )
    
    #  Launch hardware nodes
    # 1. Relayboard
    relayboard = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(neo_mp_400, 'configs/relayboard_v2', 'relayboard_v2.launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items()
        )

    # 2. Kinematics
    kinematics = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(neo_mp_400, 'configs/kinematics', 'kinematics.launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items()
        )

    # 3. Teleop
    teleop = IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 os.path.join(neo_mp_400, 'configs/teleop', 'teleop.launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items()
        )

    # 4. Laser
    # TODO: Add support for MicroScan3
    laser = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(neo_mp_400, 'configs/lidar/sick/s300', 'sick_s300.launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items()
        )
    
    # Opaque function for configuring URDF, IMU, Realsense and the USBoard
    opq_function = OpaqueFunction(function=execution_stage, args=context_arguments)

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_imu_cmd)
    ld.add_action(declare_realsense_cmd)
    ld.add_action(declare_uss_cmd)
    ld.add_action(relayboard)
    ld.add_action(kinematics)
    ld.add_action(teleop)
    ld.add_action(opq_function)
    ld.add_action(laser)

    return ld
