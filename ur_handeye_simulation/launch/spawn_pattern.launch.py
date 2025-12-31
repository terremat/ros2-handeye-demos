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

    # ---- Declare launch arguments ----
    declare_x = DeclareLaunchArgument('x', default_value='1.8', description='Spawn position X')
    declare_y = DeclareLaunchArgument('y', default_value='0.4', description='Spawn position Y')
    declare_z = DeclareLaunchArgument('z', default_value='1.01', description='Spawn position Z')
    declare_roll  = DeclareLaunchArgument('roll', default_value='0.0', description='Spawn position roll')
    declare_pitch = DeclareLaunchArgument('pitch', default_value='0.0', description='Spawn position pitch')
    declare_yaw   = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn position yaw')


    spawn_model_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_spawn_model.launch.py'
                ])
        ]),
        launch_arguments={
            'file': file,
            'entity_name': 'pattern',
            'allow_renaming': 'true',
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'roll': LaunchConfiguration('roll'),
            'pitch': LaunchConfiguration('pitch'),
            'yaw': LaunchConfiguration('yaw'),
        }.items()
    )
    
    return LaunchDescription([
        declare_x,
        declare_y,
        declare_z,
        declare_roll,
        declare_pitch,
        declare_yaw,
        spawn_model_description
    ])
    