#!/usr/bin/env python3
import os
import json
import yaml
import threading
import datetime
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage, PointCloud2
from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
#from image_transport import ImageTransport

import tf2_ros
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

#import tifffile
import cv2
from cv_bridge import CvBridge

 
# helper function that converts quaternion + translation → 4×4 matrix
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
 
class HandEyeDataAcquisitionNode(Node):

    def __init__(self):

        super().__init__('handeye_data_acquisition_node')

        # State variables
        self.joint_state = None
        self.current_state  = None
        self.current_image1 = None
        self.counter = 0
        self.is_ready = False
        self.verbose = False
        self.gui = True

        # Create TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to query TF periodically
        self.timer = self.create_timer(0.1, self.lookup_transform)
        self.base_frame = 'base_link'
        self.tool_frame = 'tool0'
        self.cam1_frame = 'eye_in_hand_camera_color_optical_frame'

        # Robot data
        self.create_subscription(JointState, "/joint_states", self.joint_cb, 10)

        # TODO: generalize to N cameras
        self.bridge = CvBridge()
        self.image1_topic = '/camera/color/image_raw'
        self.image1_cinfo = '/camera/color/camera_info'
        self.depth1_topic = '/camera/depth/image_raw'
        self.points_topic = '/camera/depth/points'

        self.get_logger().info(f"Subscribe to {self.image1_topic}...")
        self.image1_sub = self.create_subscription(Image, self.image1_topic, self.image1_callback, 10)
        
        # Depth and pointcloud subscribers
        self.get_logger().info(f"Subscribe to {self.depth1_topic}...")
        self.depth1_sub = self.create_subscription(Image, self.depth1_topic, self.depth1_callback, 10)

        self.get_logger().info(f"Subscribe to {self.points_topic}...")
        self.points_sub = self.create_subscription(PointCloud2, self.points_topic, self.points_callback, 10)
        
        # Check on timestamp
        self.current_state_t  = None
        self.current_image1_t = None
        self.current_depth1 = None
        self.current_depth1_t = None
        self.current_pcd = {'points': None, 'colors': None}
        self.current_pcd_t = None

 
        # Save directory for data
        self.save_dir = 'data_captures'
        self.save_dir_camera1 = os.path.join(self.save_dir, "camera1")
        self.save_dir_tcppose = os.path.join(self.save_dir, 'tcp_pose')
        self.save_dir_joints  = os.path.join(self.save_dir, 'joint_states')
        
        os.makedirs(self.save_dir, exist_ok=True)
        os.makedirs( os.path.join(self.save_dir_joints), exist_ok=True)
        os.makedirs( os.path.join(self.save_dir_tcppose, "pose"), exist_ok=True)
        os.makedirs( os.path.join(self.save_dir_camera1, "image"), exist_ok=True)
        os.makedirs( os.path.join(self.save_dir_camera1, "pose"),  exist_ok=True)
        os.makedirs( os.path.join(self.save_dir_camera1, "clouds"), exist_ok=True)
        os.makedirs( os.path.join(self.save_dir_camera1, "depth"), exist_ok=True)

        timestamp_file = open(os.path.join(self.save_dir, "timestamps.txt"), "w")
        timestamp_file.write(f"Image ID, Current state t, Current_image1_t\n")
        timestamp_file.close()
 
        self.get_logger().info("Start collecting TFs...")

    def get_transform_matrix(self, target_frame, source_frame):
        try:
            # Lookup transform between frames
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame,         # target frame
                source_frame,         # source frame
                rclpy.time.Time(),    # latest available
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            t = transform.transform.translation
            r = transform.transform.rotation

            matrix = transform_to_matrix(t, r)
            timestamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9

            return matrix, timestamp

        except Exception as e:
            self.get_logger().warn(f"Could not transform {target_frame} -> {source_frame}: {e}")
            return None, None

    def lookup_transform(self):
        try:
            self.current_state, self.current_state_t = self.get_transform_matrix(self.base_frame, self.tool_frame)
            self.current_cam_pose, self.current_cam_pose_t = self.get_transform_matrix(self.base_frame, self.cam1_frame)

            # TFBuffer ready to provide transformations
            if not self.is_ready:
                self.is_ready = True
                self.get_logger().info("✔ First valid TF received.")
                self.get_logger().info("👉 Press ENTER to start collecting data (images + robot state).")

                # Start the input thread only now
                self.input_thread = threading.Thread(target=self.wait_for_input, daemon=True)
                self.input_thread.start()
        except Exception as e:
            self.get_logger().error(f"Errore lookup_transform: {e}")

 
    def joint_cb(self, msg):
        self.joint_state = msg

    def image1_callback(self, msg):
        try:
            self.current_image1 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.current_image1_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception as e:
            self.get_logger().error(f"Errore conversione immagine: {e}")

    def depth1_callback(self, msg):
        try:
            depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            # Handle encodings
            if msg.encoding == '32FC1':
                # meters → millimeters → uint16
                depth_mm = np.clip(depth_raw * 1000.0, 0, 65535).astype(np.uint16)
            elif msg.encoding == '16UC1':
                depth_mm = depth_raw.astype(np.uint16)
            else:
                self.get_logger().error(f'Unsupported encoding: {msg.encoding}')
                return

            self.current_depth1 = depth_mm
            self.current_depth1_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


        except Exception as e:
            self.get_logger().error(f"Errore conversione immagine: {e}")



    def points_callback(self, msg):
        # Convert PointCloud2 → numpy array
        points = []
        colors = []

        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True):
            x, y, z, rgb = p
            r = (int(rgb) >> 16) & 0x0000ff
            g = (int(rgb) >> 8) & 0x0000ff
            b = (int(rgb)) & 0x0000ff
            points.append([x, y, z])
            colors.append([r / 255.0, g / 255.0, b / 255.0])

        if len(points) == 0:
            self.get_logger().warn('Empty point cloud')
            return
        

        # Convert to NumPy arrays
        points_array = np.array(points, dtype=np.float32)
        colors_array = np.array(colors, dtype=np.float32)
        
        # Create Open3D point cloud
        #self.current_pcd = o3d.geometry.PointCloud()
        #self.current_pcd.points = o3d.utility.Vector3dVector(points)
        #self.current_pcd.colors = o3d.utility.Vector3dVector(colors)
        
        self.current_pcd['points'] = points_array
        self.current_pcd['colors'] = colors_array
        self.current_pcd_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


    def image1_info_callback(self, msg):
        # Get the parameters info
        camera_info_K = np.array(msg.k).reshape([3,3])
        camera_info_D = np.array(msg.d)
        camera_info_height = msg.height
        camera_info_width = msg.width

        # Create the file yml
        file_content = "fx: {}\nfy: {}\ncx: {}\ncy: {}\nhas_dist_coeff: {}\ndist_k0: {}\ndist_k1: {}\ndist_px: {}\ndist_py: {}\ndist_k2: {}\ndist_k3: {}\ndist_k4: {}\ndist_k5: {}\nimg_width: {}\nimg_height: {}".format(camera_info_K[0,0],camera_info_K[1,1],camera_info_K[0,2],camera_info_K[1,2],1,camera_info_D[0],camera_info_D[1],camera_info_D[2],camera_info_D[3],camera_info_D[4],0,0,0,camera_info_width,camera_info_height)

        with open( os.path.join(self.save_dir_camera1, 'intrinsic_pars_file.yaml'), 'w') as file:
            file.write(file_content)
        self.destroy_subscription(self.image1_info_sub)
        self.get_logger().info("Subscriber camera info 1 removed!")
       

    def wait_for_input(self):
        while rclpy.ok():
            input()  # attende pressione INVIO
            self.save_data()

 
    def save_data(self):

        if self.current_state is None or self.current_image1 is None:
            self.get_logger().warn("Data not yet available!")
            return
 
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        timestamp = self.get_clock().now().nanoseconds * 1e-9

        # Camera 1 -------------------
        image1_filename = os.path.join(self.save_dir_camera1, "image", f'{self.counter:04d}.png')
        cv2.imwrite(image1_filename, self.current_image1)


        depth1_filename = os.path.join(self.save_dir_camera1, "depth", f'{self.counter:04d}.tiff')
        cv2.imwrite(depth1_filename, self.current_depth1)
        #tifffile.imwrite(depth1_filename, self.current_depth1)

        #cloud1_filename = os.path.join(self.save_dir_camera1, "clouds", f'{self.counter:04d}.pcd')
        #o3d.io.write_point_cloud(cloud1_filename, self.current_pcd, write_ascii=False)
        for key, item in self.current_pcd.items():
            cloud1_filename = os.path.join(self.save_dir_camera1, "clouds", f'{self.counter:04d}_{key}.npy')
            np.save(cloud1_filename, item)    
        

        state_filename = os.path.join(self.save_dir_camera1, 'pose', f'{self.counter:04d}.csv')
        np.savetxt(state_filename, self.current_cam_pose, delimiter=" ")

        state_filename = os.path.join(self.save_dir_tcppose, 'pose', f'{self.counter:04d}.csv')
        np.savetxt(state_filename, self.current_state, delimiter=" ")

        # Keep joints
        joint_filename = os.path.join(self.save_dir_joints, f'{self.counter:04d}.yaml')

        data = {
            "timestamp": self.joint_state.header.stamp,
            "joints": {
                "names": list(self.joint_state.name),
                "position": list(self.joint_state.position),
                "velocity": list(self.joint_state.velocity),
                "effort": list(self.joint_state.effort),
            }
        }
        with open(joint_filename, "w") as f:
            yaml.dump(data, f, sort_keys=False)

        self.get_logger().info(f"Data saved:\n - {image1_filename}\n - {state_filename}\n")

        # Check timestamps
        self.get_logger().info(f"Current : {timestamp}")
        self.get_logger().info(f"Robot TF: {self.current_state_t}")
        self.get_logger().info(f"Camera 1: {self.current_image1_t}")

        timestamp_file = open(os.path.join(self.save_dir, "timestamps.txt"), "a")
        timestamp_file.write(f"{self.counter:04d}, {self.current_state_t:.9f}, {self.current_image1_t:.9f}\n")
        timestamp_file.close()

        self.counter += 1


        


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

 