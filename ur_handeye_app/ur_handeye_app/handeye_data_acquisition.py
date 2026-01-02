#!/usr/bin/env python3
import os
import struct
import time
import yaml
import json
import threading
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from message_filters import Subscriber, ApproximateTimeSynchronizer

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge
from std_srvs.srv import Trigger

######################################################
# Utilities
######################################################
def transform_to_matrix(t, r):
    """Convert ROS2 translation + quaternion to 4×4 homogeneous matrix."""
    
    # Quaternion order for SciPy is (x, y, z, w)
    quat = [r.x, r.y, r.z, r.w]

    # SciPy builds the rotation matrix directly
    rot = R.from_quat(quat)
    R_mat = rot.as_matrix()   # 3×3 rotation

    # Build homogeneous transform
    T = np.eye(4)
    T[0:3, 0:3] = R_mat
    T[0:3, 3] = [t.x, t.y, t.z]

    return T


def create_output_dirs(base_dir: str):
    """
    Create the structured output directories for camera and robot data.
    
    Returns a dictionary with paths for easy access.
    """
    dirs = {}

    # Camera directories
    dirs['camera'] = os.path.join(base_dir, 'camera1')
    dirs['cam_color'] = os.path.join(dirs['camera'], 'color')
    dirs['cam_depth'] = os.path.join(dirs['camera'], 'depth')
    dirs['cam_cloud'] = os.path.join(dirs['camera'], 'cloud')
    dirs['cam_color_pose'] = os.path.join(dirs['camera'], 'color_pose')
    dirs['cam_cloud_pose'] = os.path.join(dirs['camera'], 'cloud_pose')

    # Robot directories
    dirs['robot'] = os.path.join(base_dir, 'robot')
    dirs['robot_pose'] = os.path.join(dirs['robot'], 'pose')
    dirs['robot_joints'] = os.path.join(dirs['robot'], 'joint_states')

    # Create all directories
    for path in dirs.values():
        os.makedirs(path, exist_ok=True)

    return dirs

######################################################
# HandEye Data Acquisition Node
######################################################
class HandEyeDataAcquisitionNode(Node):

    def __init__(self):

        super().__init__('handeye_data_acquisition_node')

        # Output directories
        self.output_dir = 'data_captures'
        self.dirs = create_output_dirs(self.output_dir)
        
        # --- Topics ---
        self.cam_color_topic = '/camera/color/image_raw'
        self.cam_cinfo_topic = '/camera/color/camera_info'
        self.cam_depth_topic = '/camera/depth/image_raw'
        self.cam_cloud_topic = '/camera/points'
        self.robot_joints_topic = '/joint_states'

        # --- Reference frames ---
        self.base_frame = 'base_link'
        self.tool_frame = 'tool0'
        self.cam1_frame = 'eye_in_hand_camera_color_optical_frame'
        self.pcd1_frame = 'eye_in_hand_camera_color_frame'

        # -- Parameters ---
        self.is_ready = False
        self.use_robot = True
        self.use_input = True
        self.sample_counter = 0

        # Thread-safe storage
        self.data_lock = threading.Lock()
        self.latest_color = None
        self.latest_depth = None
        self.latest_cinfo = None
        self.latest_cloud = None
        self.latest_joints = None

        # --- Subscribers ---
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cam_cinfo_sub = self.create_subscription(CameraInfo, self.cam_cinfo_topic, self.cinfo_cb, 10)
        self.create_subscription(JointState, self.robot_joints_topic, self.joints_cb, 10)
        # Message filters for synchronized subscriptions
        self.color_sub = Subscriber(self, Image, self.cam_color_topic)
        self.depth_sub = Subscriber(self, Image, self.cam_depth_topic)
        self.cloud_sub = Subscriber(self, PointCloud2, self.cam_cloud_topic)
        self.ts = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.cloud_sub],
            queue_size=10,
            slop=0.05  # 50ms tolerance for approximate sync
        )
        self.ts.registerCallback(self.synced_cb)

        # --- Service Clients ---
        self.move_robot_srv = self.create_client(Trigger, 'execute_joint_targets')
        if self.use_robot:
            self.get_logger().info("Wait for 'execute_joint_targets' service to be available...")
            while not self.move_robot_srv.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.get_logger().info("'execute_joint_targets' service is now available.")

        # Wait for TFs and services to be ready
        self.wait_for_ready()

        # Start input thread
        self.input_thread = threading.Thread(target=self.wait_for_input, daemon=True)
        self.input_thread.start()

    # --- Callbacks ---
    def cinfo_cb(self, msg):
        with self.data_lock:
            self.latest_cinfo = msg
        self.get_logger().debug(f"CameraInfo messages received at {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")

    def joints_cb(self, msg):
        with self.data_lock:
            self.latest_joints = msg
        self.get_logger().debug(f"JointState messages received at {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")

    def synced_cb(self, color_msg, depth_msg, cloud_msg):
        with self.data_lock:
            self.latest_color = color_msg
            self.latest_depth = depth_msg
            self.latest_cloud = cloud_msg
        self.get_logger().debug(f"Synced messages received at {color_msg.header.stamp.sec}.{color_msg.header.stamp.nanosec}")

    def get_transform_matrix(self, target_frame, source_frame, timestamp=rclpy.time.Time()):
        try:
            # Lookup transform between frames, provides "T_target_source"
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                timestamp,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            t = transform.transform.translation
            r = transform.transform.rotation
            matrix = transform_to_matrix(t, r)
            timestamp = transform.header.stamp
            return matrix, timestamp

        except Exception as e:
            self.get_logger().warn(f"Could not transform {target_frame} -> {source_frame}: {e}")
            return None, None
        

    # --- Saving utilities ---
    def save_color_image(self, image_msg: Image, sample_name: str):
        try:
            filepath = os.path.join(self.dirs['cam_color'], f"{sample_name}.png")
            image_cv = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            cv2.imwrite(filepath, image_cv)
        except Exception as e:
            self.get_logger().error(f"Error saving color image: {e}")

    def save_depth_image(self, image_msg: Image, sample_name: str):
        try:
            filepath = os.path.join(self.dirs['cam_depth'], f"{sample_name}.tiff")
            depth_raw = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
            # Handle encodings
            if image_msg.encoding == '32FC1':
                # meters → millimeters → uint16
                depth_mm = np.clip(depth_raw * 1000.0, 0, 65535).astype(np.uint16)
            elif image_msg.encoding == '16UC1':
                depth_mm = depth_raw.astype(np.uint16)
            else:
                self.get_logger().error(f'Unsupported encoding: {image_msg.encoding}')
                return
            cv2.imwrite(filepath, depth_mm)
        except Exception as e:
            self.get_logger().error(f"Error saving depth image: {e}")

    def save_camerainfo(self, cinfo_msg: CameraInfo):
        K = np.array(cinfo_msg.k).reshape([3,3])
        D = np.array(cinfo_msg.d)
   
        # Build a dictionary to save
        cam_info_dict = {
            "fx": float(K[0, 0]),
            "fy": float(K[1, 1]),
            "cx": float(K[0, 2]),
            "cy": float(K[1, 2]),
            "has_dist_coeff": 1 if D.any() else 0,
            "dist_k0": float(D[0]),
            "dist_k1": float(D[1]),
            "dist_px": float(D[2]),
            "dist_py": float(D[3]),
            "dist_k2": float(D[4]),
            "dist_k3": 0.0,
            "dist_k4": 0.0,
            "dist_k5": 0.0,
            "img_width": cinfo_msg.width,
            "img_height": cinfo_msg.height
        }

        filename = os.path.join(self.dirs['camera'], 'intrinsic_pars_file.yaml')
        with open(filename, 'w') as f:
            yaml.dump(cam_info_dict, f, sort_keys=False)

    def save_pointcloud(self, cloud_msg: PointCloud2, sample_name: str):
        points = []
        colors = []
        for p in point_cloud2.read_points(cloud_msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True):
            x, y, z, rgb = p
            # reinterpret float32 bits as uint32
            rgb_uint32 = struct.unpack('I', struct.pack('f', rgb))[0]
            r = (rgb_uint32 >> 16) & 0xFF
            g = (rgb_uint32 >> 8) & 0xFF
            b = rgb_uint32 & 0xFF
            points.append([float(x), float(y), float(z)])
            colors.append([r / 255.0, g / 255.0, b / 255.0])

        if len(points) == 0:
            self.get_logger().warn('Empty point cloud')
            return
        
        # Convert to NumPy arrays
        points_array = np.array(points, dtype=np.float32)
        colors_array = np.array(colors, dtype=np.float32)
        np.save(os.path.join(self.dirs['cam_cloud'], f"{sample_name}_points.npy"), points_array)
        np.save(os.path.join(self.dirs['cam_cloud'], f"{sample_name}_colors.npy"), colors_array)

    def save_joint_states(self, joints_msg: JointState, sample_name: str):
        joints_dict = {
            "header": {
                "stamp": {
                    "sec": joints_msg.header.stamp.sec,
                    "nanosec": joints_msg.header.stamp.nanosec
                },
                "frame_id": joints_msg.header.frame_id
            },
            "joints": { 
                "name": list(joints_msg.name),
                "position": list(joints_msg.position),
                "velocity": list(joints_msg.velocity),
                "effort": list(joints_msg.effort),
            }
        }
        filepath = os.path.join(self.dirs['robot_joints'], f"{sample_name}.yaml")
        with open(filepath, 'w') as f:
            yaml.dump(joints_dict, f, indent=4)
    
    
    # --- Node execution ---
    def wait_for_ready(self):
        self.get_logger().info(f"Waiting for TF from '{self.base_frame}' to '{self.tool_frame}'...")
        while rclpy.ok():
            try:
                self.tf_buffer.lookup_transform(
                    self.tool_frame,
                    self.base_frame,
                    rclpy.time.Time()
                )
                self.is_ready = True
                self.get_logger().info("TF is ready!")
                self.get_logger().info("👉 Press ENTER to start collecting data (images + robot state).")
                break
            except (LookupException, ConnectivityException, ExtrapolationException):
                self.get_logger().debug("TF not ready yet. Retrying in 0.5s...")
                rclpy.spin_once(self, timeout_sec=0.5)

    # Thread function to wait for Enter
    def wait_for_input(self):
        while rclpy.ok():
            try:
                input()  # Wait for Enter key

                if self.use_robot:
                    self.get_logger().info("Moving robot to a new pose...")
                    request = Trigger.Request()
                    future = self.move_robot_srv.call_async(request)
                    future.add_done_callback(self.trigger_response_cb)
                else:
                    self.take_sample()
            except KeyboardInterrupt:
                break

    def trigger_response_cb(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"Service response: success={response.success}, "
                f"message='{response.message}'"
            )
            if response.success:
                self.take_sample()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            
    def take_sample(self):
        sample_time = self.get_clock().now() # rclpy.time.Time (use t.to_msg() for ROS2 Time msg)
        sample_stamp = sample_time.to_msg()
        self.get_logger().info(f"Taking sample at time:\n{sample_stamp.sec}.{sample_stamp.nanosec}...")

        # Acquire lock to safely access latest messages
        with self.data_lock:
            color = self.latest_color
            depth = self.latest_depth
            cloud = self.latest_cloud
            joints = self.latest_joints

        self.get_logger().debug(f"Color msg: {color.header.stamp.sec}.{color.header.stamp.nanosec}" if color else "No color msg")
        self.get_logger().debug(f"Depth msg: {depth.header.stamp.sec}.{depth.header.stamp.nanosec}" if depth else "No depth msg")
        self.get_logger().debug(f"Cloud msg: {cloud.header.stamp.sec}.{cloud.header.stamp.nanosec}" if cloud else "No cloud msg")
        self.get_logger().debug(f"Joints msg: {joints.header.stamp.sec}.{joints.header.stamp.nanosec}" if joints else "No joints msg")

        color_time = rclpy.time.Time.from_msg(color.header.stamp)
        cloud_time = rclpy.time.Time.from_msg(cloud.header.stamp)
        tcp_robot_frame_pose, tcp_robot_frame_stamp = self.get_transform_matrix(self.base_frame, self.tool_frame, color_time)
        cam_color_frame_pose, cam_color_frame_stamp = self.get_transform_matrix(self.base_frame, self.cam1_frame, color_time)
        cam_cloud_frame_pose, cam_cloud_frame_stamp = self.get_transform_matrix(self.base_frame, self.pcd1_frame, cloud_time)
        self.get_logger().debug(f"Tcp robot pose: {tcp_robot_frame_stamp.sec}.{tcp_robot_frame_stamp.nanosec}")
        self.get_logger().debug(f"Cam color pose: {cam_color_frame_stamp.sec}.{cam_color_frame_stamp.nanosec}")
        self.get_logger().debug(f"Cam cloud pose: {cam_cloud_frame_stamp.sec}.{cam_cloud_frame_stamp.nanosec}")
        
        # Save data to files
        sample_name = f"{self.sample_counter:04d}"
        self.save_color_image(color, sample_name)
        self.save_depth_image(depth, sample_name)
        self.save_pointcloud(cloud, sample_name)
        self.save_joint_states(joints, sample_name)

        filepath = os.path.join(self.dirs['robot_pose'], f"{sample_name}.csv")
        np.savetxt(filepath, tcp_robot_frame_pose, delimiter=" ")
        filepath = os.path.join(self.dirs['cam_color_pose'], f"{sample_name}.csv")
        np.savetxt(filepath, cam_color_frame_pose, delimiter=" ")
        filepath = os.path.join(self.dirs['cam_cloud_pose'], f"{sample_name}.csv")
        np.savetxt(filepath, cam_cloud_frame_pose, delimiter=" ")

        if self.sample_counter == 0:
            # Save also the static transforms once
            tcp2cam_transform, _ = self.get_transform_matrix(self.tool_frame, self.cam1_frame)
            pcd2cam_transform, _ = self.get_transform_matrix(self.pcd1_frame, self.cam1_frame)
            np.savetxt(os.path.join(self.output_dir, 'tcp2cam.csv'), tcp2cam_transform, delimiter=" ")
            np.savetxt(os.path.join(self.output_dir, 'pcd2cam.csv'), pcd2cam_transform, delimiter=" ")

            # Save camera info once and remove subscriber
            self.save_camerainfo(self.latest_cinfo)
            self.destroy_subscription(self.cam_cinfo_sub)
            self.get_logger().debug("Camera info saved! Subscriber removed.")

        # Sample collected
        self.get_logger().info(f"Sample '{sample_name}' saved.")
        self.sample_counter += 1

        # Final check on timestamps
        self.get_logger().info(f"Enter time: {sample_stamp.sec}.{sample_stamp.nanosec}")
        self.get_logger().info(f"Image data: {color.header.stamp.sec}.{color.header.stamp.nanosec}")
        self.get_logger().info(f"Cloud data: {cloud.header.stamp.sec}.{cloud.header.stamp.nanosec}")
        self.get_logger().info(f"Robot Pose: {tcp_robot_frame_stamp.sec}.{tcp_robot_frame_stamp.nanosec}")
        self.get_logger().info(f"Color Pose: {cam_color_frame_stamp.sec}.{cam_color_frame_stamp.nanosec}")
        self.get_logger().info(f"Cloud Pose: {cam_cloud_frame_stamp.sec}.{cam_cloud_frame_stamp.nanosec}")  

        self.get_logger().info("\n👉 Press ENTER to take the next sample.") 


######################################################
# Main function
######################################################
def main(args=None):

    rclpy.init(args=args)
    node = HandEyeDataAcquisitionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':

    main()