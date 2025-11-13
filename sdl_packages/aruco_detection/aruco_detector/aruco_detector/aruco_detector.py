#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import cv2
import cv2.aruco as aruco

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
import tf2_geometry_msgs

from aruco_interfaces.srv import ArucoDetect


# =====================================================================
# Aruco detector class
# =====================================================================
class ArucoDetector:
    def __init__(self, node: Node):
        self.node = node
        self.bridge = CvBridge()

        # Image topic (adjust if needed)
        self.image_topic = "/camera/camera/color/image_raw"

        # Your RealSense intrinsics

        self.camera_matrix = np.array([
            [907.1922312613871, 0.0,                646.733835502371],
            [0.0,                907.7231897656274, 392.47462991366143],
            [0.0,                0.0,                1.0]
        ], dtype=np.float32)

        self.dist_coeffs = np.array([
            0.11066023092792425,
           -0.16898972182470387,
            0.0013485937713551353,
            0.0003857067833756188,
            0.0
        ], dtype=np.float32)

        # Marker size in meters (ADJUST THIS!)
        self.marker_size = 0.05

        # Number of measurements to average
        self.num_scans = 20

        # Calibrated camera frame name
        self.camera_frame = "camera_link_calibrated"
        self.base_frame = "panda_link0"

        # ArUco dictionary
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # TF setup 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        # Static TF broadcaster
        self.static_broadcaster = StaticTransformBroadcaster(node)



    # -----------------------------------------------------------------
    def wait_for_image(self, timeout=1.0):
        msg_container = {"msg": None}

        def cb(msg):
            msg_container["msg"] = msg

        sub = self.node.create_subscription(Image, self.image_topic, cb, 10)

        end_time = self.node.get_clock().now() + rclpy.duration.Duration(seconds=timeout)
        while rclpy.ok() and self.node.get_clock().now() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if msg_container["msg"] is not None:
                break

        self.node.destroy_subscription(sub)
        return msg_container["msg"]



    # -----------------------------------------------------------------
    def detect_single(self, image):
        """Detect a single ArUco marker and return its ID, rvec, tvec."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.parameters
        )

        if ids is None or len(ids) == 0:
            return None

        marker_id = int(ids[0][0])

        # Estimate pose in OpenCV optical frame
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
            corners[0],
            self.marker_size,
            self.camera_matrix,
            self.dist_coeffs
        )

        rvec = rvec[0][0]
        tvec = tvec[0][0]

        rvec, tvec = self.transform_pose_to_world(rvec, tvec)

        return marker_id, rvec, tvec

    # -----------------------------------------------------------------
    def rvec_to_quaternion(self, rvec):
        """Convert Rodrigues rotation vector to quaternion [x, y, z, w]."""
        R, _ = cv2.Rodrigues(rvec)

        # Build 4x4 transformation matrix for quaternion extraction
        T = np.eye(4)
        T[:3, :3] = R

        # Extract quaternion using the standard algorithm
        q = np.empty((4,), dtype=np.float64)
        t = np.trace(R)

        if t > 0:
            s = np.sqrt(1.0 + t) * 2
            q[3] = 0.25 * s  # w
            q[0] = (R[2,1] - R[1,2]) / s  # x
            q[1] = (R[0,2] - R[2,0]) / s  # y
            q[2] = (R[1,0] - R[0,1]) / s  # z
        else:
            i = np.argmax([R[0,0], R[1,1], R[2,2]])
            if i == 0:
                s = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
                q[3] = (R[2,1] - R[1,2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0,1] + R[1,0]) / s
                q[2] = (R[0,2] + R[2,0]) / s
            elif i == 1:
                s = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
                q[3] = (R[0,2] - R[2,0]) / s
                q[0] = (R[0,1] + R[1,0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1,2] + R[2,1]) / s
            else:
                s = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
                q[3] = (R[1,0] - R[0,1]) / s
                q[0] = (R[0,2] + R[2,0]) / s
                q[1] = (R[1,2] + R[2,1]) / s
                q[2] = 0.25 * s

        # Normalize
        norm = np.linalg.norm(q)
        if norm > 0:
            q = q / norm

        return q  # [x, y, z, w]

    # -----------------------------------------------------------------
    def transform_pose_to_world(self, rvec_cam, tvec_cam):
        """
        Transform from OpenCV optical frame to a frame where
        Z points into the marker and rotated −90° around Z.
        """
        # Flip Z (180° about Y)
        R_flip = np.array([
            [-1,  0,  0],
            [ 0,  1,  0],
            [ 0,  0, -1]
        ], dtype=float)

        # −90° rotation around Z
        theta = -np.pi / 2
        R_z_neg90 = np.array([
            [ np.cos(theta),  np.sin(theta), 0],
            [-np.sin(theta),  np.cos(theta), 0],
            [ 0,              0,             1]
        ], dtype=float)

        # Combine the rotations
        R_adjust = R_flip @ R_z_neg90

        # Apply to camera rotation
        R_cam, _ = cv2.Rodrigues(rvec_cam.reshape(3,))
        R_world = R_cam @ R_adjust

        # Back to rvec
        rvec_world, _ = cv2.Rodrigues(R_world)

        # Keep same translation
        t_world = tvec_cam.reshape(3,)

        return rvec_world.reshape(3,), t_world


    # -----------------------------------------------------------------
    def average_quaternions(self, quaternions):
        """
        Average multiple quaternions using the method from:
        "Averaging Quaternions" by F. Landis Markley et al.
        
        Args:
            quaternions: list of numpy arrays [x, y, z, w]
        
        Returns:
            averaged quaternion [x, y, z, w]
        """
        if len(quaternions) == 1:
            return quaternions[0]

        # Build matrix M
        M = np.zeros((4, 4))
        for q in quaternions:
            q = q.reshape(4, 1)
            M += q @ q.T

        M = M / len(quaternions)

        # Find eigenvector corresponding to largest eigenvalue
        eigenvalues, eigenvectors = np.linalg.eigh(M)
        max_idx = np.argmax(eigenvalues)
        q_avg = eigenvectors[:, max_idx]

        # Normalize
        q_avg = q_avg / np.linalg.norm(q_avg)

        return q_avg

    # -----------------------------------------------------------------
    def publish_marker_frame(self, marker_pose, frame_name):
        """Publish static TF transform for the detected marker."""
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = marker_pose.header.frame_id
        t.child_frame_id = frame_name

        t.transform.translation.x = marker_pose.pose.position.x
        t.transform.translation.y = marker_pose.pose.position.y
        t.transform.translation.z = marker_pose.pose.position.z
        t.transform.rotation = marker_pose.pose.orientation
        
        self.static_broadcaster.sendTransform(t)
        self.node.get_logger().info(f"Published static TF: {frame_name}")

    # -----------------------------------------------------------------
    def detect_and_average(self):
        """
        Take multiple measurements of ArUco marker pose and average them.
        Returns averaged PoseStamped and marker ID, or None if no detection.
        """
        poses = []
        ids_detected = []

        self.node.get_logger().info(f"Taking {self.num_scans} measurements...")

        for i in range(self.num_scans):
            # Wait for image
            msg = self.wait_for_image(timeout=1.0)
            if msg is None:
                self.node.get_logger().warn(f"Scan {i+1}/{self.num_scans}: No image received")
                continue

            # Convert to OpenCV
            try:
                img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                self.node.get_logger().warn(f"Scan {i+1}/{self.num_scans}: Image conversion failed: {e}")
                continue

            # Detect marker
            result = self.detect_single(img)
            if result is None:
                self.node.get_logger().warn(f"Scan {i+1}/{self.num_scans}: No marker detected")
                continue

            marker_id, rvec, tvec = result
            
            # Convert to quaternion
            quat = self.rvec_to_quaternion(rvec)

            # Store results
            poses.append({
                'position': tvec,
                'quaternion': quat
            })
            ids_detected.append(marker_id)
            
            self.node.get_logger().info(f"Scan {i+1}/{self.num_scans}: Marker ID {marker_id} detected")

        # Check if we got any detections
        if not poses:
            self.node.get_logger().warn("No markers detected in any scan")
            return None, -1

        # Average positions (simple median)
        positions = np.array([p['position'] for p in poses])
        median_pos = np.median(positions, axis=0)

        # Average quaternions
        quaternions = [p['quaternion'] for p in poses]
        avg_quat = self.average_quaternions(quaternions)

        # Calculate offset to move from corner to center of marker
        # ArUco detection gives pose at top-left corner, we want center
        offset_x = self.marker_size / 2.0  # Half marker size in X
        offset_y = self.marker_size / 2.0  # Half marker size in Y
        
        # Convert quaternion to rotation matrix to apply offset in marker frame
        qw, qx, qy, qz = avg_quat[3], avg_quat[0], avg_quat[1], avg_quat[2]
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
        ])
        
        # Apply offset in marker's local frame
        offset_local = np.array([offset_x, offset_y, 0.0])
        offset_world = R @ offset_local
        
        # Build final PoseStamped with centered position
        final_pose = PoseStamped()
        final_pose.header.stamp = rclpy.time.Time().to_msg()
        final_pose.header.frame_id = self.camera_frame

        final_pose.pose.position.x = float(median_pos[0] + offset_world[0])
        final_pose.pose.position.y = float(median_pos[1] + offset_world[1])
        final_pose.pose.position.z = float(median_pos[2] + offset_world[2])
        
        final_pose.pose.orientation.x = float(avg_quat[0])
        final_pose.pose.orientation.y = float(avg_quat[1])
        final_pose.pose.orientation.z = float(avg_quat[2])
        final_pose.pose.orientation.w = float(avg_quat[3])

        marker_id = ids_detected[0]  # Assume same marker detected each time

        self.node.get_logger().info(f"Averaged {len(poses)} measurements for marker ID {marker_id}")
        self.node.get_logger().info(f"Position: [{median_pos[0]:.3f}, {median_pos[1]:.3f}, {median_pos[2]:.3f}]")

        # Set frame_id to base frame for consistency
        try:
            final_pose = self.tf_buffer.transform(
                final_pose, 
                self.base_frame, 
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            self.node.get_logger().info(f"Transformed to {self.base_frame}: [{final_pose.pose.position.x:.3f}, {final_pose.pose.position.y:.3f}, {final_pose.pose.position.z:.3f}]")
        except Exception as e:
            self.node.get_logger().error(f"Transform failed: {e}")
            return None, -1

        return final_pose, marker_id


# =====================================================================
# ROS2 Node
# =====================================================================
class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__("aruco_detector_node")

        self.detector = ArucoDetector(self)

        # Create service
        self.srv = self.create_service(
            ArucoDetect,
            "aruco_detect",
            self.handle_service,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info("    ArUco Detection Service Ready")
        self.get_logger().info(f"   Service: /aruco_detect")
        self.get_logger().info(f"   Camera frame: {self.detector.camera_frame}")
        self.get_logger().info(f"   Image topic: {self.detector.image_topic}")
        self.get_logger().info(f"   Marker size: {self.detector.marker_size} m")
        self.get_logger().info(f"   Averaging {self.detector.num_scans} measurements")

    # -----------------------------------------------------------------
    def handle_service(self, request, response):
        """
        Service callback: detect ArUco marker, average multiple measurements,
        publish TF, and return the pose.
        """
        self.get_logger().info("ArUco detection service called")

        # Detect and average multiple measurements
        final_pose, marker_id = self.detector.detect_and_average()

        if final_pose is None:
            # No detection
            response.id = -1
            response.pose = PoseStamped()
            self.get_logger().warn("No ArUco marker detected")
            return response



        # Publish static TF transform
        frame_name = f"aruco_marker_{marker_id}"
        self.detector.publish_marker_frame(final_pose, frame_name)

        # Return response
        response.id = marker_id
        response.pose = final_pose
        
        self.get_logger().info(f"Detection complete: Marker ID {marker_id}, TF frame: {frame_name}")
        
        return response


# =====================================================================
def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
