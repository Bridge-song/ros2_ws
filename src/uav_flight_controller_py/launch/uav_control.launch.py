from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share_dir = get_package_share_directory('uav_flight_controller_py')
    params_file = os.path.join(package_share_dir, 'config', 'uav_params.yaml')

    return LaunchDescription([
        Node(
            package='uav_flight_controller_py',
            executable='flight_control_node',
            name='uav_controller',
            output='screen',
            emulate_tty=True,  # 让print输出到终端
            parameters=[params_file]
        )
    ])
