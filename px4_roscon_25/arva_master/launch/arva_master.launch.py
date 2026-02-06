import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory("arva_master")
    
    clock_bridge_config_file = os.path.join(pkg_share,"cfg","clock_bridge.yaml")

    run_uxrcedds_agent_arg = DeclareLaunchArgument(
        "run_uxrcedds_agent",
        default_value="false",
        description="Whether to run the MicroXRCEdds Agent",
    )

    run_gz_clock_bridge_arg = DeclareLaunchArgument(
        "run_gz_clock_bridge",
        default_value="false",
        description="Whether to run the Gazebo clock bridge",
    )

    aruco_tracker = get_package_share_directory("aruco_tracker")
    aruco_tracker_launch_file = os.path.join(aruco_tracker, "launch", "aruco_tracker.launch.py")

    return LaunchDescription([
        run_uxrcedds_agent_arg,
        run_gz_clock_bridge_arg,
        Node(
            package="arva_master",
            executable="arva_master",
            name="arva_master",
            output="screen",
            parameters=[
                {"use_sim_time": True}
            ]
        ),
        Node(
            package="arva_master",
            executable="terrain_follow_controller",
            name="terrain_follow_controller",
            output="screen",
            parameters=[
                {"use_sim_time": True},
                os.path.join(pkg_share, "cfg", "terrain_follow.yaml")
            ]
        ),
        Node(
            package="arva_master",
            executable="fluxline_follow_controller",
            name="fluxline_follow_controller",
            output="screen",
            parameters=[
                {"use_sim_time": True},
                os.path.join(pkg_share, "cfg", "fluxline_follow.yaml")
            ]
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_clock_bridge",
            parameters=[
                {"config_file": clock_bridge_config_file}
            ],
            condition=IfCondition(LaunchConfiguration("run_gz_clock_bridge"))
        ),
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", "8888", "-v", "3"],
            output="screen",
            condition=IfCondition(LaunchConfiguration("run_uxrcedds_agent"))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(aruco_tracker_launch_file),
            launch_arguments={"world_name": "walls", "model_name":"x500_mono_cam_down_0"}.items(),)
    ])
