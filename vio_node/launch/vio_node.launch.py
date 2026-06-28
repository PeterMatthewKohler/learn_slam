from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('vio_node')
    params_file = os.path.join(package_share, 'params', 'vio_node_params.yaml')

    noisy_wheel_odom_node = Node(
        package='vio_node',
        executable='VIONode_exe',
        name='vio_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        noisy_wheel_odom_node
    ])