from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tspc_controller',
            executable='tspc_node',
            name='tspc_node',
            output='screen',
            parameters=['config/tspc_params.yaml']
        )
    ])
