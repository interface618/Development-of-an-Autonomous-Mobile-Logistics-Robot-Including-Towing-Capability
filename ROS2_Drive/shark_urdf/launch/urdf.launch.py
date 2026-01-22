#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_path   = get_package_share_directory("shark_urdf")
    xacro_file = PathJoinSubstitution([pkg_path, "urdf", "shark.urdf.xacro"])
    xacro_exe  = FindExecutable(name="xacro")

    # xacro 전개 + prefix 하드코딩(shark1/)
    robot_description = ParameterValue(
        Command([xacro_exe, " ", xacro_file]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false", description="use sim time"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace="shark1",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
            
            
        ),
    ])