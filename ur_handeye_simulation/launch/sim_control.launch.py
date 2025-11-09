from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, IfElseSubstitution, LaunchConfiguration, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


"""
Start the robot with simulated hardware provided by Gazebo simulator
"""

def generate_launch_description():
    simulation_package = FindPackageShare('ur_handeye_simulation')
    description_package = FindPackageShare("ur_handeye_description")

    # General arguments
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration('world')
    launch_rviz = LaunchConfiguration("launch_rviz")

    rviz_config_file = PathJoinSubstitution([simulation_package, "rviz", "urdf.rviz"])
    description_file = PathJoinSubstitution([simulation_package, "urdf", "ur_handeye_workcell.urdf.xacro"])

    # Load robot description from URDF with default arguments
    #robot_description_content = Command(["xacro", " ", description_file])
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    ###############
    # GZ nodes
    ###############
    
    # Spawn robot into Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            '-string', robot_description_content,
            '-name', 'ur_handeye_workcell',
            '-allow_renaming', 'true'
        ]
    )
    
    # Start Gazebo with its world
    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": IfElseSubstitution(
                gazebo_gui,
                if_value=[" -r -v 4 ", world_file],
                else_value=[" -s -r -v 4 ", world_file],
            ),
            'on_exit_shutdown': 'True'
        }.items(),
    )

    # Bridging and remapping Gazebo topics to ROS 2
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    nodes_to_start = [
        rviz_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        gz_spawn_entity,
        gz_launch_description,
        gz_sim_bridge,
    ]

    ###############
    # Define the launch argument that can be passed
    ###############
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([simulation_package, 'worlds', 'empty.sdf']),
            description='Name of the world file to load (inside the worlds/ folder of the simulation package)'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui", default_value="true", description="Start gazebo with GUI?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )



    return LaunchDescription(declared_arguments + nodes_to_start)