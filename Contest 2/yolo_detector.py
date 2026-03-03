#!/usr/bin/env python3
"""
YOLO Object Detection Service for MIE443 Contest 2
Detects objects using YOLOv8/v13 and returns the highest confidence detection.
"""

import rclpy
from rclpy.node import Node
from mie443_contest2.srv import DetectObject
import cv2
import numpy as np
from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Load YOLO model
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLO model loaded')
        
        # Confidence threshold
        self.confidence_threshold = 0.5
        
        # Create service
        self.service = self.create_service(
            DetectObject,
            'detect_object',
            self.detect_callback
        )
        
        self.get_logger().info('YOLO Detector Service ready')

    def detect_callback(self, request, response):
        """Process image and return highest confidence detection."""
        
        # Decode compressed image
        np_arr = np.frombuffer(request.image.data, np.uint8)
        save_detected_image = request.save_detected_image
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if image is None:
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = "Failed to decode image"
            return response
        
        # Run YOLO inference
        results = self.model(image, verbose=False, device='cpu')
        boxes = results[0].boxes

        ### YOUR CODE HERE ###
        # Define the 5 target object categories (COCO class names and IDs)
        target_classes = {
            "cup":          41,   # 1. plastic cup
            "bottle":       39,   # 2. water bottle
            "clock":        74,   # 3. clock
            "potted plant": 58,   # 4. potted plant
            "motorcycle":    3,   # 5. toy motorcycle
        }

        # Find the highest confidence detection above threshold
        best_conf = 0.0
        best_class_id = -1
        best_class_name = ""

        if boxes is not None and len(boxes) > 0:
            for i in range(len(boxes)):
                conf = float(boxes.conf[i])
                cls_id = int(boxes.cls[i])
                cls_name = self.model.names[cls_id]

                if conf >= self.confidence_threshold and conf > best_conf and cls_name in target_classes:
                    best_conf = conf
                    best_class_id = cls_id
                    best_class_name = cls_name

        if best_class_id != -1:
            response.success = True
            response.class_id = best_class_id
            response.class_name = best_class_name
            response.confidence = best_conf
            response.message = f"Detected {best_class_name} with confidence {best_conf:.2f}"

            # Save annotated image if requested
            if save_detected_image:
                annotated = results[0].plot()
                cv2.imwrite('/home/admin1/detected_object.jpg', annotated)
                self.get_logger().info("Saved annotated image to /home/admin1/detected_object.jpg")
        else:
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = "No detection above confidence threshold"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
