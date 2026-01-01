"""
A launch file for running the motion planning python api tutorial
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.substitutions import FindPackageShare

from pathlib import Path


def generate_launch_description():

    declare_joint_targets_dir_arg = DeclareLaunchArgument(
        'joint_targets_dir',
        default_value= get_package_share_directory("ur_handeye_app") + '/data',
        description='Directory to save/load joint target data'
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_handeye_moveit_config")
        .moveit_cpp(
            file_path=get_package_share_directory("ur_handeye_app")
            + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
    )


    moveit_py_node = Node(
        name="moveit_py",
        package="ur_handeye_app",
        executable="data_control",
        output="both",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                'joint_targets_dir': LaunchConfiguration('joint_targets_dir'),
            },
        ]
    )

    return LaunchDescription(
        [
            declare_joint_targets_dir_arg,
            moveit_py_node,
        ]
    )
