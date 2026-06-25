import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('lai_franka_controllers')
    teleop_params = os.path.join(pkg_share_dir, 'config', 'teleop_params.yaml')

    force_color_env = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')

    # Native ROS 2 Joy Node Driver
    joy_driver_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'deadzone': 0.05,
            'autorepeat_rate': 50.0
        }],
        output='screen'
    )

    # or custom compiled Libusb binary
    # joy_driver_node = Node(
    #     package='lai_franka_controllers',
    #     executable='raw_usb_joy_node',
    #     name='joy_node',
    #     output='screen'
    # )

    # Custom Teleop Translator Node
    joy_teleop_node = Node(
        package='lai_franka_controllers',
        executable='joy_teleop_node',
        name='joy_teleop_node',
        parameters=[teleop_params],
        output='screen'
    )

    return LaunchDescription([
        force_color_env,
        joy_driver_node,
        joy_teleop_node
    ])