#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ur_custom_driver.srv import CameraProcess, ArucoPose
import cv2
import numpy as np
import torch
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
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.marker_size = 0.05  # meters
        self.camera_matrix = np.array([[1000, 0, 640], [0, 1000, 360], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

    def process_image_callback(self, request, response):
        try:
            model = YOLO('/home/jon/Workspace/swap_ws/src/ur_custom/ur_custom_driver/scripts/yolov8n_NN2/weights/best.pt')
            cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            if not cap.isOpened():
                self.get_logger().error('Failed to open camera.')
                response.value1 = 0
                response.value2 = 0
                return response

            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            cap.set(cv2.CAP_PROP_FPS, 15)

            ret, frame = cap.read()
            if not ret:
                self.get_logger().error('Failed to capture image.')
                response.value1 = 0
                response.value2 = 0
                return response

            results = model.predict(frame, conf=0.7, iou=0.5, device="cpu")

            max_odd_score = -1
            max_odd_class = None
            max_even_score = -1
            max_even_class = None
            # Placeholder: Extract two integers (one even, one odd) from YOLO results
            for result in results:
                boxes = result.boxes.xyxy.cpu().numpy()
                scores = result.boxes.conf.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy()
                class_names = result.names
                print(classes)

                # Choose odd class with highest score and even class with highest score
                for box, score, cls in zip(boxes, scores, classes):
                    x1, y1, x2, y2 = map(int, box)
                    label = f"{class_names[int(cls)]} {score:.2f}"
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
            value1 = max_odd_class if max_odd_class is not None else 0
            value2 = max_even_class if max_even_class is not None else 0

            # Validate values
            if (value1 in range(2, 10) and value2 in range(2, 10) and value1 % 2 != value2 % 2):
                response.value1 = value1
                response.value2 = value2
                self.get_logger().info(f'Returning values: {value1}, {value2}')
            else:
                self.get_logger().error('YOLO output invalid: values must be 2-9, one even, one odd')
                self.get_logger().error(f'Obtained values: {value1}, {value2}')
                response.value1 = 0
                response.value2 = 0

        except Exception as e:
            self.get_logger().error(f'Image capture or processing failed: {str(e)}')
            response.value1 = 0
            response.value2 = 0

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
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, _ = detector.detectMarkers(frame)

            if ids is None or len(ids) == 0:
                response.success = False
                response.message = "No marker detected."
                cap.release()
                return response

            # Estimate pose in camera frame
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[0], self.marker_size, self.camera_matrix, self.dist_coeffs)

            # Convert rotation vector to rotation matrix
            rmat, _ = cv2.Rodrigues(rvec[0])

            # Create pose in camera frame
            pose = PoseStamped()
            pose.header.frame_id = "camera_frame"
            pose.pose.position.x = float(tvec[0][0])
            pose.pose.position.y = float(tvec[0][1])
            pose.pose.position.z = float(tvec[0][2])

            # Convert rotation matrix to quaternion
            def rotation_matrix_to_quaternion(rmat):
                trace = np.trace(rmat)
                if trace > 0:
                    S = np.sqrt(trace + 1.0) * 2.0
                    w = 0.25 * S
                    x = (rmat[2, 1] - rmat[1, 2]) / S
                    y = (rmat[0, 2] - rmat[2, 0]) / S
                    z = (rmat[1, 0] - rmat[0, 1]) / S
                elif rmat[0, 0] > rmat[1, 1] and rmat[0, 0] > rmat[2, 2]:
                    S = np.sqrt(1.0 + rmat[0, 0] - rmat[1, 1] - rmat[2, 2]) * 2.0
                    w = (rmat[2, 1] - rmat[1, 2]) / S
                    x = 0.25 * S
                    y = (rmat[0, 1] + rmat[1, 0]) / S
                    z = (rmat[0, 2] + rmat[2, 0]) / S
                elif rmat[1, 1] > rmat[2, 2]:
                    S = np.sqrt(1.0 + rmat[1, 1] - rmat[0, 0] - rmat[2, 2]) * 2.0
                    w = (rmat[0, 2] - rmat[2, 0]) / S
                    x = (rmat[0, 1] + rmat[1, 0]) / S
                    y = 0.25 * S
                    z = (rmat[1, 2] + rmat[2, 1]) / S
                else:
                    S = np.sqrt(1.0 + rmat[2, 2] - rmat[0, 0] - rmat[1, 1]) * 2.0
                    w = (rmat[1, 0] - rmat[0, 1]) / S
                    x = (rmat[0, 2] + rmat[2, 0]) / S
                    y = (rmat[1, 2] + rmat[2, 1]) / S
                    z = 0.25 * S
                return np.array([x, y, z, w])

            quat = rotation_matrix_to_quaternion(rmat)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            # Create transform from camera to world
            camera_transform = tf2_geometry_msgs.TransformStamped()
            camera_transform.header.frame_id = "world"
            camera_transform.child_frame_id = "camera_frame"
            camera_transform.transform.translation.x = request.x
            camera_transform.transform.translation.y = request.y
            camera_transform.transform.translation.z = request.z

            # Convert roll, pitch, yaw to quaternion
            cy = math.cos(request.yaw * 0.5)
            sy = math.sin(request.yaw * 0.5)
            cp = math.cos(request.pitch * 0.5)
            sp = math.sin(request.pitch * 0.5)
            cr = math.cos(request.roll * 0.5)
            sr = math.sin(request.roll * 0.5)
            camera_transform.transform.rotation.w = cr * cp * cy + sr * sp * sy
            camera_transform.transform.rotation.x = sr * cp * cy - cr * sp * sy
            camera_transform.transform.rotation.y = cr * sp * cy + sr * cp * sy
            camera_transform.transform.rotation.z = cr * cp * sy - sr * sp * cy

            # Transform pose to world frame
            world_pose = self.tf_buffer.transform(pose, "world", timeout=rclpy.duration.Duration(seconds=1.0))

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

            roll, pitch, yaw = quaternion_to_euler(
                world_pose.pose.orientation.x,
                world_pose.pose.orientation.y,
                world_pose.pose.orientation.z,
                world_pose.pose.orientation.w
            )

            # Fill response
            response.x = world_pose.pose.position.x
            response.y = world_pose.pose.position.y
            response.z = world_pose.pose.position.z
            response.roll = roll
            response.pitch = pitch
            response.yaw = yaw
            response.success = True
            response.message = "Marker pose detected."

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