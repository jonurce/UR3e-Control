#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ur_custom_driver.srv import CameraProcess, ArucoPose
import cv2
from scipy.spatial.transform import Rotation as R
import numpy as np
from ultralytics import YOLO
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import math

class CameraServer(Node):
    def __init__(self):
        super().__init__('camera_server')
        self.srv = self.create_service(CameraProcess, '/camera_process', self.process_image_callback)
        self.aruco_srv = self.create_service(ArucoPose, '/aruco_pose', self.aruco_pose_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info('CameraServer initialized.')

        # Hardcoded parameters for ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.marker_size = 0.024  # meters
        self.camera_matrix = np.array([[1.39870845e+03, 0.00000000e+00, 9.66861109e+02], [0.00000000e+00, 1.40794892e+03, 5.41442327e+02], [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]], dtype=np.float32)
        self.dist_coeffs = np.array([[-8.78837077e-03, 1.52017955e+00, 7.77842959e-04, -3.30963514e-03, -5.42404330e+00]], dtype=np.float32)


    def process_image_callback(self, request, response):
        try:
            model = YOLO('/home/jon/Workspace/swap_ws/src/ur_custom/ur_custom_driver/scripts/yolov8n_NN2/weights/best.pt')
            cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            if not cap.isOpened():
                self.get_logger().error('Failed to open camera.')
                response.value1 = -1
                response.value2 = -1
                return response

            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            cap.set(cv2.CAP_PROP_FPS, 15)

            ret, frame = cap.read()
            if not ret:
                self.get_logger().error('Failed to capture image.')
                response.value1 = -1
                response.value2 = -1
                return response

            results = model.predict(frame, conf=0.7, iou=0.5, device="cpu")

            max_odd_score = -1
            max_odd_class = None
            max_even_score = -1
            max_even_class = None
            # Placeholder: Extract two integers (one even, one odd) from YOLO results
            for result in results:
                scores = result.boxes.conf.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy()
                # Choose odd class with highest score and even class with highest score
                for score, cls in zip(scores, classes):
                    cls_int = int(cls)
                    if cls_int in range(2, 10):
                        if cls_int % 2 == 1:  # Odd
                            if score > max_odd_score:
                                max_odd_score = score
                                max_odd_class = cls_int
                        else:  # Even
                            if score > max_even_score:
                                max_even_score = score
                                max_even_class = cls_int
            cap.release()

            # Assign values (odd and even)
            value1 = max_odd_class if max_odd_class is not None else -1
            value2 = max_even_class if max_even_class is not None else -1
            response.value1 = value1
            response.value2 = value2
            self.get_logger().info(f'Returning values: {value1}, {value2}')
            return response


        except Exception as e:
            self.get_logger().error(f'Image capture or processing failed: {str(e)}')
            response.value1 = -1
            response.value2 = -1

        finally:
            if cap:
                cap.release()

        return response

    def aruco_pose_callback(self, request, response):
        try:
            # Capture image
            cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            if not cap.isOpened():
                response.success = False
                response.message = "Failed to open camera."
                return response

            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            cap.set(cv2.CAP_PROP_FPS, 15)

            ret, frame = cap.read()
            if not ret:
                response.success = False
                response.message = "Failed to capture image."
                cap.release()
                return response

            # Detect ArUco marker
            detector = cv2.aruco.ArucoDetector(self.aruco_dict)
            corners, ids, rejected = detector.detectMarkers(frame)
            self.get_logger().info(f"Detected IDs: {ids}, Rejected: {len(rejected)}")

            if ids is None or len(ids) == 0:
                response.success = False
                response.message = "No marker detected."
                cap.release()
                return response

            # Estimate pose in camera frame
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

            # Convert quaternion to roll, pitch, yaw
            def quaternion_to_euler(x, y, z, w):
                t0 = 2.0 * (w * x + y * z)
                t1 = 1.0 - 2.0 * (x * x + y * y)
                roll = math.atan2(t0, t1)
                t2 = 2.0 * (w * y - z * x)
                t2 = 1.0 if t2 > 1.0 else t2
                t2 = -1.0 if t2 < -1.0 else t2
                pitch = math.asin(t2)
                t3 = 2.0 * (w * z + x * y)
                t4 = 1.0 - 2.0 * (y * y + z * z)
                yaw = math.atan2(t3, t4)
                return roll, pitch, yaw

            for i, id in enumerate(ids):

                # Transform: marker to camera (Marker frame represented in camera frame)
                marker_pose_camera = np.eye(4, dtype=np.float32)
                marker_pose_camera[0:3, 0:3] = cv2.Rodrigues(np.array(rvec[i][0]))[0]
                marker_pose_camera[0:3, 3:4] = tvec[i][0].reshape(3, 1)

                # Transform: camera to tool_tip (Camera frame represented in tool_tip frame)
                # Camera frame: x right - y down - z out (same as tool_tip)
                # No rotation, just translation
                camera_pose_tool = np.eye(4, dtype=np.float32)
                camera_pose_tool[0:3, 3:4] = np.array([[0.0], [-0.075], [-0.16]], dtype=np.float32)

                # Transform: tool_tip to world (Tool_tip frame represented in world frame)
                # Convert roll, pitch, yaw to quaternion
                cy = math.cos(request.yaw)
                sy = math.sin(request.yaw)
                cp = math.cos(request.pitch)
                sp = math.sin(request.pitch)
                cr = math.cos(request.roll)
                sr = math.sin(request.roll)
                rotation_matrix = np.array([
                    [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                    [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                    [-sp, cp*sr, cp*cr]
                ], dtype=np.float32)
                translation = np.array([[request.x], [request.y], [request.z]], dtype=np.float32)
                tool_to_world = np.eye(4, dtype=np.float32)
                tool_to_world[0:3, 0:3] = rotation_matrix
                tool_to_world[0:3, 3:4] = translation

                # Transform: marker to world frame (Marker frame represented in world frame)
                marker_pose_world = tool_to_world @ camera_pose_tool @ marker_pose_camera

                # Extract position and orientation
                world_pos = marker_pose_world[0:3, 3]
                r = R.from_matrix(marker_pose_world[0:3, 0:3])
                quat = r.as_quat()
                world_pose = PoseStamped()
                world_pose.header.frame_id = "world"
                world_pose.pose.position.x = float(world_pos[0])
                world_pose.pose.position.y = float(world_pos[1])
                world_pose.pose.position.z = float(world_pos[2])
                world_pose.pose.orientation.x = quat[0]
                world_pose.pose.orientation.y = quat[1]
                world_pose.pose.orientation.z = quat[2]
                world_pose.pose.orientation.w = quat[3]

                # Euler angle format in radians
                roll, pitch, yaw = quaternion_to_euler(world_pose.pose.orientation.x, world_pose.pose.orientation.y, world_pose.pose.orientation.z, world_pose.pose.orientation.w)

                # Fill response
                response.x = world_pose.pose.position.x
                response.y = world_pose.pose.position.y
                response.z = world_pose.pose.position.z
                response.roll = roll
                response.pitch = pitch
                response.yaw = yaw
                response.success = True
                response.message = "Marker pose detected."
                break

            cap.release()
            return response

        except Exception as e:
            self.get_logger().error(f'ArUco processing failed: {str(e)}')
            response.success = False
            response.message = f"Error: {str(e)}"
            if cap:
                cap.release()
            return response


def main():
    rclpy.init()
    node = CameraServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()