import launch
import launch.actions
import launch.substitutions
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_context import LaunchContext

def generate_launch_description():
    robot_namespace = launch.substitutions.LaunchConfiguration('namespace', default="")
    context = LaunchContext()

    config = os.path.join(get_package_share_directory('neo_mp_400-2'),'configs/lidar/sick/s300','s300_1.yaml')
    config1 = os.path.join(get_package_share_directory('neo_mp_400-2'),'configs/lidar/sick/s300','s300_filter.yaml')


    return launch.LaunchDescription([
        GroupAction([
        PushRosNamespace(
            namespace=robot_namespace),

     launch_ros.actions.Node(
            package='neo_sick_s300-2', 
            namespace = robot_namespace.perform(context) + "lidar_1", 
            executable='neo_sick_s300_node', output='screen',
            name='neo_sick_s300_node', 
            parameters = [config])
    ,
     launch_ros.actions.Node(
            package='neo_sick_s300-2',
            namespace = robot_namespace.perform(context) + 'lidar_1',
            executable='neo_scan_filter_node', output='screen',
            name='neo_scan_filter_node',
            parameters = [config1]),
        ])
    ])
