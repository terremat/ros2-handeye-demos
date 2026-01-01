import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.logging import get_logger

from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped

from std_srvs.srv import Trigger
from moveit_msgs.msg import RobotTrajectory, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def load_joint_target(file_path):
    """Utility function to load joint targets from a file."""
    with open(file_path, 'r') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    joint_names = data['joints']['names']
    joint_positions = data['joints']['position']
    return joint_names, joint_positions

class HandEyeDataControlNode(Node):
    def __init__(self):
        super().__init__('handeye_data_control_node')
        self.logger = get_logger('handeye_data_control_node')

        # Parameters
        self.declare_parameter('joint_targets_dir', None)
        self.joint_targets_dir = self.get_parameter('joint_targets_dir').value
        print("Data directory:", self.joint_targets_dir)

        self.jt_filenames = [os.path.join(self.joint_targets_dir, f) for f in sorted(os.listdir(self.joint_targets_dir))]
        self.next_target_idx = 0
        self.logger.info(f"Found {len(self.jt_filenames)} joint configurations.")


        # Initialize MoveItPy
        self.moveit = MoveItPy(node_name="moveit_py")

        self.group_name = "ur_manipulator"
        self.world_frame = 'base_link'
        self.target_frame = 'tool0'

        # Robot model & state
        self.arm = self.moveit.get_planning_component(self.group_name)
        self.robot_model = self.moveit.get_robot_model()
        self.robot_state = RobotState(self.robot_model)

        # Service to execute joint targets
        self.srv = self.create_service(
            Trigger,
            "execute_joint_targets",
            self.execute_joint_targets_cb
        )

        # Action client for FollowJointTrajectory
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        self.logger.info("Waiting for controller action server...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.logger.error("Controller action server not available!")
        else:
            self.logger.info("Controller action server ready.")

        self.logger.info("HandEyeDataControlNode has been initialized.")

    # ----------------------------------------------------------------------
    # Utility methods
    # ----------------------------------------------------------------------
    def get_current_joint_values(self):
        """Return current joint configuration of the robot."""
        self.arm.set_start_state_to_current_state()
        state = self.arm.get_start_state()
        joints = state.get_joint_group_positions(self.group_name)
        return joints

    def get_current_pose(self, frame="tool0"):
        """Return current end-effector pose (Pose object)."""
        pass
    
    # ----------------------------------------------------------------------
    # Planning methods
    # ----------------------------------------------------------------------
    def plan_to_joint_target(self, joint_values):
        """Plan to a specific list of joint target values."""
        self.arm.set_start_state_to_current_state()

        # Create a state based on the current robot model
        target_state = RobotState(self.robot_model)
        target_state.set_joint_group_positions(self.group_name, joint_values)

        # Set that state as the target
        self.arm.set_goal_state(robot_state=target_state)

        # Plan
        plan = self.arm.plan()
        return plan
    

    def plan_to_pose_target(self, pose, frame_id=None):
        """Plan to a specific goal pose for the robot's end effector.
        
        Args:
            pose: A geometry_msgs/Pose object representing the target pose.
        """
        self.arm.set_start_state_to_current_state()

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = self.world_frame if frame_id is None else frame_id
        pose_goal.pose = pose
        self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=self.target_frame)

        # Plan
        plan = self.arm.plan()
        return plan

    def plan_to_named_target(self, name):
        """Plan to a predefined MoveIt configuration."""
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name=name)
        return self.arm.plan()

    # ----------------------------------------------------------------------
    # Execution methods
    # ----------------------------------------------------------------------
    def execute(self, plan_result):
        """Execute a MoveIt plan via MoveItPy."""
        if plan_result is None:
            self.logger.error("No plan to execute")
            return False
        
        result = self.moveit.execute(plan_result.trajectory, controllers=[])
        return bool(result)

    # Combined convenience function
    def move_to_joint_target(self, joint_values):
        plan = self.plan_to_joint_target(joint_values)
        if not plan:
            self.logger.error("Planning failed.")
            return False
        return self.execute(plan)

    def move_to_named_target(self, name):
        plan = self.plan_to_named_target(name)
        if not plan:
            self.logger.error("Planning failed.")
            return False
        return self.execute(plan)
    
    # ----------------------------------------------------------------------
    # Server methods
    # ----------------------------------------------------------------------
    def move_to_next_target(self):
        """Move to the next joint target in the list."""
        if self.next_target_idx >= len(self.jt_filenames):
            self.logger.info("All joint targets have been processed.")
            return False

        jt_filename = self.jt_filenames[self.next_target_idx]
        joint_names, joint_positions = load_joint_target(jt_filename)

        # joint_names: [elbow_joint, shoulder_lift_joint,shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
        # joint_plans: [shoulder_pan_joint, shoulder_lift_joint,elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
        joint_commands = joint_positions.copy()
        joint_commands[0] = joint_positions[2]  # shoulder_pan_joint
        joint_commands[2] = joint_positions[0]  # elbow_joint

        success = self.move_to_joint_target(joint_commands)
        if success:
            self.logger.info(f"Moved to joint target from file: {jt_filename}")
            self.next_target_idx += 1
        else:
            self.logger.error(f"Failed to move to joint target from file: {jt_filename}")

        return success
    
    def execute_joint_targets_cb(self, request, response):
        self.logger.info("Service request received: execute_joint_targets")

        success = self.move_to_next_target()

        response.success = success
        response.message = (
            "Joint targets executed successfully"
            if success else
            "Failed to execute joint targets"
        )
        return response
    

def main(args=None):
    rclpy.init()

    robot = HandEyeDataControlNode()

    # Get current robot configuration
    joints = robot.get_current_joint_values()
    robot.logger.info(f"Current joints: {joints}")

    # Plan and move to "home" position
    success = robot.move_to_named_target("home")
    if success:
        robot.logger.info("Moved to 'home' position successfully.")
    else:
        robot.logger.error("Failed to move to 'home' position.")

    """
    # Move robot according to predefined joint configuration
    if robot.joint_targets_dir is not None:
        joint_targets = os.listdir(robot.joint_targets_dir)
        robot.logger.info(f"Found {len(joint_targets)} joint configurations.")
        robot.logger.info("Moving robot to predefined joint configuration...")

        for jt_file in joint_targets:
            jt_path = os.path.join(robot.joint_targets_dir, jt_file)
            joint_names, joint_positions = load_joint_target(jt_path)

            # joint_names: [elbow_joint, shoulder_lift_joint,shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
            # joint_plans: [shoulder_pan_joint, shoulder_lift_joint,elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
            joint_commands = joint_positions.copy()
            joint_commands[0] = joint_positions[2]  # shoulder_pan_joint
            joint_commands[2] = joint_positions[0]  # elbow_joint
            robot.move_to_joint_target(joint_commands)

    else:
        robot.logger.error("No joint targets directory specified.")
    """

    robot.logger.info("HandEyeDataControlNode is spinning...")
    rclpy.spin(robot)
        
    # Destroy node
    robot.destroy_node()

    # Shutdown rclpy
    rclpy.shutdown()


    
if __name__ == "__main__":
    main()
