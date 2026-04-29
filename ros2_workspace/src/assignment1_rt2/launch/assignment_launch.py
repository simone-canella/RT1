from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Action Server
        Node(
            package='assignment1_rt2',
            executable='action_server_node',
            name='robot_server'
        ),
        
        # The Bridge for TF 
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
            ],
            output='screen'
        )
    ])