#!/usr/bin/env python3
"""
YOLO Object Detection Service for MIE443 Contest 2
Detects objects using YOLOv8, annotates the image, and saves it to disk.
"""

import rclpy
from rclpy.node import Node
from mie443_contest2.srv import DetectObject
import cv2
import numpy as np
from ultralytics import YOLO
import os

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Load YOLO model (yolov8n is fast for CPU inference)
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLO model loaded')
        
        # Confidence threshold to ignore weak detections
        self.confidence_threshold = 0.45
        
        # Define the specific contest objects to look for
        # These correspond to COCO classes: 'cup', 'bottle', 'clock', 'potted plant', 'motorcycle'
        self.target_classes = ["cup", "bottle", "clock", "potted plant", "motorcycle"]
        
        # Create service
        self.service = self.create_service(
            DetectObject,
            'detect_object',
            self.detect_callback
        )
        
        self.get_logger().info('YOLO Detector Service ready with Auto-Annotation')

    def detect_callback(self, request, response):
        """Process image, save annotated version, and return highest confidence detection."""
        
        # 1. Decode compressed image
        np_arr = np.frombuffer(request.image.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if image is None:
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = "Failed to decode image"
            return response
        
        # 2. Run YOLO inference
        results = self.model(image, verbose=False, device='cpu')
        boxes = results[0].boxes

        # 3. Find the best detection among the contest-specific target classes
        best_box = None
        best_conf = 0.0

        for box in boxes:
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            name = self.model.names[cls_id]

            # Filter for objects relevant to Contest 2
            if name in self.target_classes and conf > best_conf:
                best_conf = conf
                best_box = box

        # 4. Handle no detection or low confidence
        if best_box is None or best_conf < self.confidence_threshold:
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = best_conf if best_box else 0.0
            response.message = "No target objects detected above threshold"
            return response

        # 5. Extract Final Detection Info
        class_id = int(best_box.cls[0])
        class_name = self.model.names[class_id]

        # 6. ANNOTATE AND SAVE (Satisfies Contest Requirement for Image Output)
        # results[0].plot() creates an image with boxes and labels already drawn
        annotated_img = results[0].plot()
        
        # Save the file to the current directory (likely your ros2_ws root or package folder)
        filename = f"detection_{class_name}.jpg"
        cv2.imwrite(filename, annotated_img)
        self.get_logger().info(f"Saved annotated image: {filename} (Conf: {best_conf:.2f})")

        # 7. Fill Response
        response.success = True
        response.class_id = class_id
        response.class_name = class_name
        response.confidence = best_conf
        response.message = f"Detected {class_name}"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()