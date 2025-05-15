# Neobotix GmbH
# Contributor: Adarsh Karan K P

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
  DeclareLaunchArgument,
  IncludeLaunchDescription,
  OpaqueFunction
)
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_context import LaunchContext
import os
from pathlib import Path
import xacro
"""
This code is used for debugging, quick testing, and visualization of the robot in Rviz. 
"""

def execution_stage(context: LaunchContext, 
                    use_sim_time, 
                    use_joint_state_publisher_gui, 
                    imu_enable,
                    d435_enable,
                    uss_enable,
                    scanner_type,
                    display_mode,
                    rviz_config):

    launch_actions = []

    neo_mp_400 = get_package_share_directory('neo_mp_400-2')

    # Resolve launch arguments
    imu_enabl = str(imu_enable.perform(context))
    d435_enabl = str(d435_enable.perform(context))
    uss_enabl = str(uss_enable.perform(context))
    scanner_typ = str(scanner_type.perform(context))
    use_sim_tim = use_sim_time.perform(context)
    rviz_config_file = str(rviz_config.perform(context))
    use_joint_state_publisher_gui = use_joint_state_publisher_gui.perform(context)
    display_mod = display_mode.perform(context)

    # Robot description package for mp_400 robot
    urdf = os.path.join(neo_mp_400,
        'robot_model',
        'mp_400.urdf.xacro')

    xacro_args = [
        "xacro", " ", urdf,
        " ", 'use_gz:=true',
        " ", 'use_imu:=', imu_enabl,
        " ", 'use_d435:=', d435_enabl,
        " ", 'use_uss:=', uss_enabl,
        " ", 'scanner_type:=', scanner_typ,
    ]

    # Launch bringup_sim file from mp_bringup package
    rviz_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mp_rviz'), 'launch', 'rviz.launch.py')
        ),
        launch_arguments={
        'use_sim_time': use_sim_tim,
        'use_joint_state_publisher_gui': use_joint_state_publisher_gui,
        'display_mode': display_mod,
        'rviz_config': rviz_config_file,
        'robot_description_content': Command(xacro_args)
        }.items(),
    )

    # Add the launch command to the launch actions
    launch_actions.append(rviz_bringup_cmd)

    return launch_actions

def generate_launch_description():

    # Declare launch arguments with default values and descriptions
    declare_use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time', default_value='False',
            description='Use simulation clock if True (True/False)'
        )

    declare_use_joint_state_publisher_gui_arg = DeclareLaunchArgument(
            'use_joint_state_publisher_gui', default_value='True',
            description='Use joint state publisher gui if True (True/False)'
        )

    declare_use_imu_cmd = DeclareLaunchArgument(
            'use_imu', default_value='False',
            description='Enable IMU sensors if true'
        )

    declare_realsense_cmd = DeclareLaunchArgument(
            'use_d435', default_value='False',
            description='Enable Intel RealSense D435 camera if true'
        )

    declare_uss_cmd = DeclareLaunchArgument(
            'uss_enable', default_value='False',
            description='Enable uss - Options: True/False'
        )

    declare_scanner_type_cmd = DeclareLaunchArgument(
            'scanner_type', default_value='sick_s300',
            choices=['sick_s300', 'sick_microscan3'],
            description='Type of laser scanner to use\n\t'
        )

    declare_use_display_mode_cmd = DeclareLaunchArgument(
            'display_mode', default_value='True',
            description='Disable robot and joint state publishers if true (True/False)'
        )

    declare_rviz_cfg_arg = DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
            get_package_share_directory('neo_mp_400-2'),
            'configs', 'rviz', 'robot_description_rviz.rviz'),
            description='Full path to an RViz config file'
        )

    opq_function = OpaqueFunction(
        function=execution_stage,
        args=[
            LaunchConfiguration('use_sim_time'),
            LaunchConfiguration('use_joint_state_publisher_gui'),
            LaunchConfiguration('use_imu'),
            LaunchConfiguration('use_d435'),
            LaunchConfiguration('uss_enable'),
            LaunchConfiguration('scanner_type'),
            LaunchConfiguration('display_mode'),
            LaunchConfiguration('rviz_config')
        ])

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_use_joint_state_publisher_gui_arg,
        declare_use_imu_cmd,
        declare_realsense_cmd,
        declare_uss_cmd,
        declare_scanner_type_cmd,
        declare_use_display_mode_cmd,
        declare_rviz_cfg_arg,
        opq_function
    ])
