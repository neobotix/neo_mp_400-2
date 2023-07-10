import launch
import launch.actions
import launch.substitutions
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions

def generate_launch_description():
    config = os.path.join(get_package_share_directory('neo_mp_400-2'),'configs/kinematics','kinematics.yaml')
    robot_namespace = launch.substitutions.LaunchConfiguration('namespace', default="")

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='neo_kinematics_differential',
            executable='neo_differential_node',
            output='screen',
            namespace = robot_namespace,
            name='neo_differential_node',
            parameters = [config])
    ])
