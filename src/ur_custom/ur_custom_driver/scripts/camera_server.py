#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ur_custom_driver.srv import CameraProcess
import cv2
import numpy as np
# from ultralytics import YOLO

class CameraServer(Node):
    def __init__(self):
        super().__init__('camera_server')
        self.srv = self.create_service(CameraProcess, '/camera_process', self.process_image_callback)
        #self.yolo_model = YOLO('yolov8s.pt')  # Replace with your YOLO model path
        self.get_logger().info('CameraServer initialized.')

    def process_image_callback(self, request, response):
        # Open L515 camera with OpenCV
        cap = None
        try:
            cap = cv2.VideoCapture(0)  # Adjust index if needed (e.g., 0 for default USB camera)
            if not cap.isOpened():
                self.get_logger().error('Failed to open camera.')
                response.value1 = 0
                response.value2 = 0
                return response

            # Set resolution to 1280x720, RGB format
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

            # Capture one frame
            ret, image = cap.read()
            if not ret:
                self.get_logger().error('Failed to capture image.')
                response.value1 = 0
                response.value2 = 0
                return response

            # Convert BGR to RGB
            #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # Process image with YOLO
            #results = self.yolo_model(image)
            # Placeholder: Extract two integers (one even, one odd) from YOLO results
            # Replace with your actual YOLO output processing logic
            value1 = 3  # Odd (placeholder)
            value2 = 4  # Even (placeholder)

            # Validate values
            if (value1 in range(2, 10) and value2 in range(2, 10) and
                    value1 % 2 != value2 % 2):
                response.value1 = value1
                response.value2 = value2
                self.get_logger().info(f'Returning values: {value1}, {value2}')
            else:
                self.get_logger().error('YOLO output invalid: values must be 2-9, one even, one odd')
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