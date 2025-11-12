from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

    # General arguments
    ur_type = "ur10"
    #moveit_launch_file = PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])
    moveit_launch_file = PathJoinSubstitution([FindPackageShare("ur_handeye_moveit_config"), "launch", "ur_handeye_moveit.launch.py"])

    # Use the argument value
    handeye_setup = LaunchConfiguration("handeye_setup")

    # Gazebo simulation
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_handeye_simulation"), "launch", "sim_control.launch.py"]
            )
        ),
        launch_arguments={
            "launch_rviz": "false",
            "handeye_setup": handeye_setup,
        }.items(),
    )

    # MoveIt configuration
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": "true",
            "launch_rviz": "true",
        }.items(),
    )

    nodes_to_start = [
        ur_control_launch,
        ur_moveit_launch,
    ]

    declared_arguments = [        
        DeclareLaunchArgument(
            "handeye_setup",
            default_value="eye_to_hand",
            description="Select hand-eye setup: 'none', 'eye_in_hand' or 'eye_to_hand'",
        )
    ]


    return LaunchDescription(declared_arguments + nodes_to_start)