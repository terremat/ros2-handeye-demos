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
from rclpy.logging import get_logger
from ament_index_python.packages import get_package_share_directory
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


def create_output_dirs(base_dir: str, camera_names: list):
    """
    Create the structured output directories for camera and robot data.
    
    Returns a dictionary with paths for easy access.
    """
    dirs = {}

    # Camera directories
    # dirs['camera'] = os.path.join(base_dir, 'camera1')
    # dirs['cam_color'] = os.path.join(dirs['camera'], 'color')
    # dirs['cam_depth'] = os.path.join(dirs['camera'], 'depth')
    # dirs['cam_cloud'] = os.path.join(dirs['camera'], 'cloud')
    # dirs['cam_color_pose'] = os.path.join(dirs['camera'], 'color_pose')
    # dirs['cam_cloud_pose'] = os.path.join(dirs['camera'], 'cloud_pose')
    for camera_name in camera_names:
        dirs[f'{camera_name}'] = os.path.join(base_dir, camera_name)
        dirs[f'{camera_name}_color'] = os.path.join(dirs[f'{camera_name}'], 'color')
        dirs[f'{camera_name}_depth'] = os.path.join(dirs[f'{camera_name}'], 'depth')
        dirs[f'{camera_name}_cloud'] = os.path.join(dirs[f'{camera_name}'], 'cloud')
        dirs[f'{camera_name}_color_pose'] = os.path.join(dirs[f'{camera_name}'], 'color_pose')
        dirs[f'{camera_name}_cloud_pose'] = os.path.join(dirs[f'{camera_name}'], 'cloud_pose')
        dirs[f'{camera_name}_robot_pose'] = os.path.join(dirs[f'{camera_name}'], 'robot_pose')

    # Robot directories
    dirs['robot'] = os.path.join(base_dir, 'robot')
    dirs['robot_pose'] = os.path.join(dirs['robot'], 'pose')
    dirs['robot_joints'] = os.path.join(dirs['robot'], 'joint_states')

    # Create all directories
    for path in dirs.values():
        os.makedirs(path, exist_ok=True)

    return dirs

######################################################
# Camera Handler: creates subscriptions and latest synchronized messages
######################################################
class CameraHandler:
    def __init__(self, node: Node, cam_cfg: dict):
        self.node = node
        self.name = cam_cfg['name']

        self.color_topic = cam_cfg.get('color_image_topic')
        self.cinfo_topic = cam_cfg.get('color_cinfo_topic')
        self.depth_topic = cam_cfg.get('depth_image_topic', None)
        self.cloud_topic = cam_cfg.get('depth_cloud_topic', None)

        # TODO: None for color info means no data to acquire, raise error?

        self.color_frame = cam_cfg.get('color_frame')
        self.cloud_frame = cam_cfg.get('cloud_frame', None)

        self.lock = threading.Lock()
        self.latest_color = None
        self.latest_depth = None
        self.latest_cloud = None
        self.latest_cinfo = None

        self.cinfo_sub = node.create_subscription(CameraInfo, self.cinfo_topic, self._cinfo_cb, 10)
        
        self.color_sub = Subscriber(node, Image, self.color_topic)
        self.depth_sub = Subscriber(node, Image, self.depth_topic) if self.depth_topic else None
        self.cloud_sub = Subscriber(node, PointCloud2, self.cloud_topic) if self.cloud_topic else None
        self.subs = [self.color_sub, self.depth_sub, self.cloud_sub]

        # Message filters for synchronized subscriptions
        self.ts = ApproximateTimeSynchronizer(
            [s for s in self.subs if s is not None],
            queue_size=10,
            slop=0.05  # 50ms tolerance for approximate sync
        )
        self.ts.registerCallback(self._synced_cb)

    # Use *msgs to accept variable number of messages to be parsed
    def _synced_cb(self, *msgs):
        idx = 0
        color = msgs[idx]; idx += 1
        depth = msgs[idx] if self.depth_sub else None
        if self.depth_sub: idx += 1
        cloud = msgs[idx] if self.cloud_sub else None

        with self.lock:
            self.latest_color = color
            self.latest_depth = depth
            self.latest_cloud = cloud

    def _cinfo_cb(self, msg):
        with self.lock:
            self.latest_cinfo = msg
        # Once we have the camera info, we can unsubscribe
        self.node.destroy_subscription(self.cinfo_sub)
        self.node.get_logger().debug(f"[{self.name}] Camera info saved! Subscriber removed.")
    
    def get_latest(self):
        with self.lock:
            return self.latest_color, self.latest_depth, self.latest_cloud, self.latest_cinfo



######################################################
# HandEye Data Acquisition Node
######################################################
class HandEyeDataAcquisitionNode(Node):

    def __init__(self):

        super().__init__('handeye_data_acquisition_node')
        self.logger = get_logger('handeye_data_control_node')

        # Read config filepath from ROS2 parameter 'config_filepath' (set with --ros-args -p config_filepath:=/path/to/file.yaml)
        default_config = os.path.join(get_package_share_directory("ur_handeye_app"), 'config', 'handeye_setup.yaml')
        config_filepath = self.declare_parameter('config_filepath', default_config).value
        self.get_logger().info(f"Using config file: {config_filepath}")
        with open(config_filepath, 'r') as f:
            self.config = yaml.safe_load(f)

        self.camera_nodes = []
        for cam_cfg in self.config.get('cameras', []):
            cam = CameraHandler(self, cam_cfg)
            self.camera_nodes.append(cam)
        self.camera_names = [cam.name for cam in self.camera_nodes]

        self.get_logger().info(f"Loaded {len(self.camera_names)} cameras: {self.camera_names}")

        # Output directories
        self.output_dir = 'data_captures'
        self.dirs = create_output_dirs(self.output_dir, self.camera_names)

        # --- Topics ---
        # self.cam_color_topic = '/camera/color/image_raw'
        # self.cam_cinfo_topic = '/camera/color/camera_info'
        # self.cam_depth_topic = '/camera/depth/image_raw'
        # self.cam_cloud_topic = '/camera/points'
        self.robot_joints_topic = '/joint_states'

        # --- Reference frames ---
        self.base_frame = self.config.get('base_frame', 'base_link')
        self.tool_frame = self.config.get('tool_frame', 'tool0')
        # self.cam1_frame = self.config.get('cam1_frame', 'eye_in_hand_camera_color_optical_frame')
        # self.pcd1_frame = self.config.get('pcd1_frame', 'eye_in_hand_camera_color_frame')

        # -- Parameters ---
        self.is_ready = False
        self.use_robot = False
        self.use_input = True
        self.sample_counter = 0

        # Thread-safe storage
        self.data_lock = threading.Lock()
        self.latest_joints = None

        # --- Subscribers ---
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(JointState, self.robot_joints_topic, self.joints_cb, 10)

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
    def save_color_image(self, image_msg: Image, sample_name: str, output_dir: str):
        try:
            filepath = os.path.join(output_dir, f"{sample_name}.png")
            image_cv = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            cv2.imwrite(filepath, image_cv)
        except Exception as e:
            self.get_logger().error(f"Error saving color image: {e}")

    def save_depth_image(self, image_msg: Image, sample_name: str, output_dir: str):
        try:
            filepath = os.path.join(output_dir, f"{sample_name}.tiff")
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

    def save_camerainfo(self, cinfo_msg: CameraInfo, output_dir: str):
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

        filename = os.path.join(output_dir, 'intrinsic_pars_file.yaml')
        with open(filename, 'w') as f:
            yaml.dump(cam_info_dict, f, sort_keys=False)

    def save_pointcloud(self, cloud_msg: PointCloud2, sample_name: str, output_dir: str):
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
        np.save(os.path.join(output_dir, f"{sample_name}_points.npy"), points_array)
        np.save(os.path.join(output_dir, f"{sample_name}_colors.npy"), colors_array)

    def save_joint_states(self, joints_msg: JointState, sample_name: str, output_dir: str):
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
        filepath = os.path.join(output_dir, f"{sample_name}.yaml")
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
            #color = self.latest_color
            #depth = self.latest_depth
            #cloud = self.latest_cloud
            joints = self.latest_joints

        self.get_logger().debug(f"Joints msg: {joints.header.stamp.sec}.{joints.header.stamp.nanosec}" if joints else "No joints msg")

        camera_samples = []
        for camera_node in self.camera_nodes:
            color, depth, cloud, cinfo = camera_node.get_latest()
            self.get_logger().debug(f"[{camera_node.name}] Color msg: {color.header.stamp.sec}.{color.header.stamp.nanosec}" if color else "No color msg")
            self.get_logger().debug(f"[{camera_node.name}] Depth msg: {depth.header.stamp.sec}.{depth.header.stamp.nanosec}" if depth else "No depth msg")
            self.get_logger().debug(f"[{camera_node.name}] Cloud msg: {cloud.header.stamp.sec}.{cloud.header.stamp.nanosec}" if cloud else "No cloud msg")

            color_time = rclpy.time.Time.from_msg(color.header.stamp)
            cloud_time = rclpy.time.Time.from_msg(cloud.header.stamp)
            tcp_robot_frame_pose, tcp_robot_frame_stamp = self.get_transform_matrix(self.base_frame, self.tool_frame, color_time)
            cam_color_frame_pose, cam_color_frame_stamp = self.get_transform_matrix(self.base_frame, camera_node.color_frame, color_time)
            cam_cloud_frame_pose, cam_cloud_frame_stamp = self.get_transform_matrix(self.base_frame, camera_node.cloud_frame, cloud_time)
            self.get_logger().debug(f"Tcp robot pose: {tcp_robot_frame_stamp.sec}.{tcp_robot_frame_stamp.nanosec}")
            self.get_logger().debug(f"Cam color pose: {cam_color_frame_stamp.sec}.{cam_color_frame_stamp.nanosec}")
            self.get_logger().debug(f"Cam cloud pose: {cam_cloud_frame_stamp.sec}.{cam_cloud_frame_stamp.nanosec}")
            camera_samples.append({
                'camera_name': camera_node.name,
                'color': color,
                'depth': depth,
                'cloud': cloud,
                'cinfo': cinfo,
                'tcp_robot_frame_pose': tcp_robot_frame_pose,
                'tcp_robot_frame_stamp': tcp_robot_frame_stamp,
                'cam_color_frame_pose': cam_color_frame_pose,
                'cam_cloud_frame_pose': cam_cloud_frame_pose
            })
        
        
        # Save data to files
        sample_name = f"{self.sample_counter:04d}"
        
        self.save_joint_states(joints, sample_name, self.dirs['robot_joints'])

        for camera_sample in camera_samples:
            self.save_color_image(camera_sample['color'], sample_name, self.dirs[f"{camera_sample['camera_name']}_color"])
            self.save_depth_image(camera_sample['depth'], sample_name, self.dirs[f"{camera_sample['camera_name']}_depth"])
            self.save_pointcloud(camera_sample['cloud'], sample_name, self.dirs[f"{camera_sample['camera_name']}_cloud"])

            filepath = os.path.join(self.dirs[f"{camera_sample['camera_name']}_robot_pose"], f"{sample_name}.csv")
            np.savetxt(filepath, camera_sample['tcp_robot_frame_pose'], delimiter=" ")
            filepath = os.path.join( self.dirs[f"{camera_sample['camera_name']}_color_pose"], f"{sample_name}.csv")
            np.savetxt(filepath, camera_sample['cam_color_frame_pose'], delimiter=" ")
            filepath = os.path.join(self.dirs[f"{camera_sample['camera_name']}_cloud_pose"], f"{sample_name}.csv")
            np.savetxt(filepath, camera_sample['cam_cloud_frame_pose'], delimiter=" ")

            if self.sample_counter == 0:
            # Save camera info once and remove subscriber
                self.save_camerainfo(camera_sample['cinfo'], self.dirs[f"{camera_sample['camera_name']}"])

        # Use last camera robot pose as overall pose
        filepath = os.path.join(self.dirs['robot_pose'], f"{sample_name}.csv")
        np.savetxt(filepath, camera_samples[-1]['tcp_robot_frame_pose'], delimiter=" ")


        # Sample collected
        self.get_logger().info(f"Sample '{sample_name}' saved.")
        self.sample_counter += 1

        # Final check on timestamps
        self.get_logger().info(f"Enter time: {sample_stamp.sec}.{sample_stamp.nanosec}")
        self.get_logger().info(f"Image data: {camera_samples[0]['color'].header.stamp.sec}.{camera_samples[0]['color'].header.stamp.nanosec}")
        self.get_logger().info(f"Cloud data: {camera_samples[0]['cloud'].header.stamp.sec}.{camera_samples[0]['cloud'].header.stamp.nanosec}")
        self.get_logger().info(f"Robot Pose: {camera_samples[0]['tcp_robot_frame_stamp'].sec}.{camera_samples[0]['tcp_robot_frame_stamp'].nanosec}")

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