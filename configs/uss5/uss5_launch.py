import launch
import launch.actions
import launch.substitutions
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions

def generate_launch_description():
    config = os.path.join(get_package_share_directory('neo_mp_400-2'),'configs','uss5/uss5.yaml')
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='neo_usboard_v2', executable='neo_usboard_v2_node', output='screen', parameters = [config])
    ])