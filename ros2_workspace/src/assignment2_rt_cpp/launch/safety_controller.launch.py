from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('assignment2_rt_cpp')
    params_file = os.path.join(pkg_share, 'config', 'safety_controller.yaml')

    return LaunchDescription([
        Node(
            package='assignment2_rt_cpp',
            executable='safety_controller',
            name='safety_controller',
            output='screen',
            parameters=[params_file]
        )
    ])
