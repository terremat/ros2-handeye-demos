import rclpy
from rclpy.node import Node

from ament_index_python import get_package_share_directory
from pathlib import Path

# moveit python library
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped


class MotionTester(Node):

    def __init__(self):
        super().__init__('motion_tester')



def main(args=None):
    rclpy.init(args=args)
    
    move_group_name = "ur_manipulator"  # Name of the MoveIt planning group
    target_frame = "tool0" 
    world_frame = "base_link"

    moveit_config_builder = MoveItConfigsBuilder(
        robot_name="ur_handeye", 
        package_name="ur_handeye_moveit_config")
    
    moveit_config_builder.robot_description_semantic(Path("config") / "ur_handeye_workcell.srdf")
    #moveit_config_builder.moveit_cpp(file_path=get_package_share_directory("pipeline") + "/config/moveit_config.yaml")
    
    node_name="moveit_node"
    ur_robot = MoveItPy(node_name, config_dict=moveit_config_builder.to_moveit_configs().to_dict())

    # instantiate MoveItPy instance and get planning component
    print("MoveItPy instance created")


    rclpy.shutdown()


if __name__ == '__main__':
    main()