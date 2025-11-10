from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

"""
Quick test for spawing SDF in Gazebo e ROS2
"""

def generate_launch_description():

    simulation_package = FindPackageShare('ur_handeye_simulation')

    file = PathJoinSubstitution([simulation_package, "models", "checker_default", "model.sdf"])


    spawn_model_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch',
                                   'gz_spawn_model.launch.py'])]),
        launch_arguments=[('file', file),
                          ('entity_name', "pattern"),
                          ('allow_renaming', "true"),
        ])
    
    return LaunchDescription([spawn_model_description])
    