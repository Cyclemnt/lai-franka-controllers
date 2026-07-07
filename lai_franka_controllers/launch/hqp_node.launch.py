'''
@file hqp_node.launch.py
@brief Launch configuration file starting the primary HQP solver node.
@author Clement Lamouller
@date 2026
'''

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('lai_franka_controllers')
    hqp_config = os.path.join(pkg_share_dir, 'config', 'hqp_node_params.yaml')

    # Force colored logging format outputs across tracking streams
    force_color_env = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')

    # Primary Multi-Priority Workspace Optimization Solver Module
    hqp_reference_generator_node = Node(
        package='lai_franka_controllers',
        executable='hqp_reference_generator_node',
        name='hqp_reference_generator_node',
        parameters=[hqp_config],
        output='screen'
    )

    return LaunchDescription([
        force_color_env,
        hqp_reference_generator_node
    ])