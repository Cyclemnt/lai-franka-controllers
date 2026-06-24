import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to your parameter file (Change name/path to match your choice above)
    config = os.path.join(
        get_package_share_directory('my_franka_controllers'),
        'config',
        'hqp_node_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_franka_controllers',
            executable='hqp_reference_generator_node',
            name='hqp_reference_generator_node',
            output='screen',
            parameters=[config]
        )
    ])