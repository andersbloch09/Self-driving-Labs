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
from geometry_msgs.msg import PoseStamped, Quaternion

from aruco_interfaces.srv import ArucoDetect


# =====================================================================
# Aruco detector class
# =====================================================================
class ArucoDetector:
    def __init__(self, node: Node):
        self.node = node
        self.bridge = CvBridge()

        # ✅ Image topic (adjust if needed)
        self.image_topic = "/camera/camera/color/image_raw"

        # ✅ Your RealSense intrinsics
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

        # ✅ Marker size in meters
        self.marker_size = 0.05   # CHANGE THIS!

        # ✅ ArUco dictionary
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

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
        """Detect ID + 6DOF pose."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.parameters
        )

        if ids is None or len(ids) == 0:
            return None

        marker_id = int(ids[0][0])

        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
            corners[0],
            self.marker_size,
            self.camera_matrix,
            self.dist_coeffs
        )

        rvec = rvec[0][0]
        tvec = tvec[0][0]

        return marker_id, rvec, tvec

    # -----------------------------------------------------------------
    def rvec_to_quaternion(self, rvec):
        """Convert Rodrigues rvec to quaternion."""
        R, _ = cv2.Rodrigues(rvec)

        q = np.empty((4,), dtype=np.float64)
        t = np.trace(R)

        if t > 0:
            s = np.sqrt(1.0 + t) * 2
            q[3] = 0.25 * s
            q[0] = (R[2,1] - R[1,2]) / s
            q[1] = (R[0,2] - R[2,0]) / s
            q[2] = (R[1,0] - R[0,1]) / s
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

        return q

    # -----------------------------------------------------------------
    def process(self):
        msg = self.wait_for_image(timeout=1.0)
        if msg is None:
            return None

        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        return self.detect_single(img)


# =====================================================================
# ROS2 Node
# =====================================================================
class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__("arucodetector_node")

        self.detector = ArucoDetector(self)

        self.srv = self.create_service(
            ArucoDetect,
            "aruco_detect",
            self.handle_service,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info("✅ ArucoDetect service ready (FULL 6DOF POSE).")

    # -----------------------------------------------------------------
    def handle_service(self, request, response):
        result = self.detector.process()

        if result is None:
            response.id = -1
            response.pose = PoseStamped()
            return response

        marker_id, rvec, tvec = result

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "camera_link_calibrated"

        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = float(tvec[2])

        q = self.detector.rvec_to_quaternion(rvec)
        pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        response.id = marker_id
        response.pose = pose
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
