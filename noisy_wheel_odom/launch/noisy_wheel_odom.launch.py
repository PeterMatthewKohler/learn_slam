from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('noisy_wheel_odom')
    params_file = os.path.join(package_share, 'params', 'noisy_wheel_odom.yaml')

    noisy_wheel_odom_node = Node(
        package='noisy_wheel_odom',
        executable='NoisyWheelOdomNode_exe',
        name='noisy_wheel_odom_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        noisy_wheel_odom_node
    ])


