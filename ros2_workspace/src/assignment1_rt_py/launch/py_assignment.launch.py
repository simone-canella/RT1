from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # === TURTLESIM NODE ===
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        # === TURTLE SPAWNER NODE ===
        Node(
            package='turtle_spawner_py',
            executable='turtle_spawn',
            name='turtle_spawner'
        ),

        # === PYTHON DISTANCE NODE ===
        Node(
            package='assignment1_rt_py',
            executable='distance_py',
            name='distance_node_py'
        ),
    ])
