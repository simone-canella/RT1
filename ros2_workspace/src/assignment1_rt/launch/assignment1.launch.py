from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # Start turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        # Spawn turtle2 (your Python package)
        Node(
            package='turtle_spawner_py',
            executable='turtle_spawn',
            name='turtle_spawner'
        ),

        # Distance node (C++)
        Node(
            package='assignment1_rt',
            executable='distance_node',
            name='distance_node'
        ),
    ])
