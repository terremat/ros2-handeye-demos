import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import time

import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import TransformStamped

class MoveRobotClient(Node):
    def __init__(self):
        super().__init__('move_robot_client')
        self._client = ActionClient(self, MoveGroup, 'move_action')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        time.sleep(1.0)  # sleeps for 1 second

        print("READY")

    def send_goal(self, target_pose: PoseStamped):
        self.get_logger().info("Waiting for move_group action server...")
        self._client.wait_for_server()

        goal_msg = MoveGroup.Goal()
        # Build the goal request
        # NOTE: This is a simplified example; normally you need to fill:
        #   goal.request.workspace_parameters
        #   goal.request.start_state
        #   goal.request.goal_constraints
        # For simple pose target, fill goal.request.goal_constraints with target_pose

        self.get_logger().info("Sending goal...")
        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server")
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.get_logger().info(f"Result: {result}")

    def get_transform(self, target_frame, source_frame, timeout=5.0):
        start_time = time.time()
        while rclpy.ok():
            try:
                trans: TransformStamped = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time()
                )
                print("Tool0 position:", trans.transform.translation)
                print("Tool0 orientation:", trans.transform.rotation)
                return trans
            except Exception:
                if time.time() - start_time > timeout:
                    raise TimeoutError(f"Transform {source_frame} -> {target_frame} not available after {timeout}s")
                time.sleep(0.05)
        




def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotClient()

    node.get_logger().info("Waiting for transform base_link -> tool0...")
    #node.get_transform('tool0', 'base_link')

    # ros2 run tf2_ros tf2_echo  world base_link
    #- Translation: [2.500, 0.500, 1.000]
    # - Rotation: in Quaternion [0.000, 0.000, 1.000, 0.000]

    # ros2 run tf2_ros tf2_echo  world tool0
    #- Translation: [2.093, 0.336, 1.950]
    #- Rotation: in Quaternion [-0.498, -0.498, 0.502, 0.502]

    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = 2.093
    pose.pose.position.y = 0.336
    pose.pose.position.z = 1.950
    pose.pose.orientation.x = -0.498
    pose.pose.orientation.x = -0.498
    pose.pose.orientation.x = 0.502
    pose.pose.orientation.w = 0.502

    node.send_goal(pose)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
