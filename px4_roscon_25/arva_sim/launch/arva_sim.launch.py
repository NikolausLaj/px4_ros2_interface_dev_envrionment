from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package="arva_sim",
            executable="arva_sim_node",
            name="arva_sim_node",
            output="screen",
            parameters=[
                {"use_sim_time": True}
            ]
        )
    ])