import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(get_package_share_directory('arva_sim'), 'cfg', 'arva_sim.yaml')
    return LaunchDescription([
        Node(
            package="arva_sim",
            executable="arva_sim_node",
            name="arva_sim_node",
            output="screen",
            parameters=[
                config_file,
                {"use_sim_time": True}
            ]
        )
    ])
