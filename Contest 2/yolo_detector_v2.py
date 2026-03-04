#!/usr/bin/env python3
"""
YOLO Object Detection Service for MIE443 Contest 2
Detects objects using YOLOv8/v13 and returns the highest confidence detection.
Also publishes live annotated streams for OAK-D and wrist cameras.
"""

import rclpy
from rclpy.node import Node
from mie443_contest2.srv import DetectObject, CaptureImage
from sensor_msgs.msg import CompressedImage
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

        # Target object categories (COCO class names and IDs)
        self.target_classes = {
            "cup":          41,
            "bottle":       39,
            "clock":        74,
            "potted plant": 58,
            "motorcycle":    3,
        }

        # Create detect_object service
        self.service = self.create_service(
            DetectObject,
            'detect_object',
            self.detect_callback
        )

        # Publishers for annotated camera streams
        self.oakd_annotated_pub = self.create_publisher(
            CompressedImage, '/yolo/oakd/compressed', 10)
        self.wrist_annotated_pub = self.create_publisher(
            CompressedImage, '/yolo/wrist/compressed', 10)

        # Subscribe to OAK-D camera stream
        self.oakd_sub = self.create_subscription(
            CompressedImage,
            '/oakd/rgb/preview/image_raw/compressed',
            self.oakd_callback,
            10
        )

        # Wrist camera service client
        self.wrist_capture_client = self.create_client(CaptureImage, 'wrist_camera/capture_image')

        # Timer to poll wrist camera at 5 Hz
        self.wrist_timer = self.create_timer(0.2, self.wrist_timer_callback)

        self.get_logger().info('YOLO Detector Service ready')
        self.get_logger().info('  - OAK-D annotated stream: /yolo/oakd/compressed')
        self.get_logger().info('  - Wrist annotated stream:  /yolo/wrist/compressed')

    def _run_yolo_and_annotate(self, image_bgr):
        """Run YOLO on a BGR image and return annotated image."""
        results = self.model(image_bgr, verbose=False, device='cpu')
        boxes = results[0].boxes
        annotated = image_bgr.copy()

        if boxes is not None and len(boxes) > 0:
            for i in range(len(boxes)):
                conf = float(boxes.conf[i])
                cls_id = int(boxes.cls[i])
                cls_name = self.model.names[cls_id]
                if conf < self.confidence_threshold or cls_name not in self.target_classes:
                    continue
                x1, y1, x2, y2 = map(int, boxes.xyxy[i].tolist())
                label = f"{cls_name} {conf:.2f}"
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated, label, (x1, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return annotated

    def _publish_annotated(self, publisher, annotated_bgr, frame_id):
        """Compress and publish an annotated BGR image."""
        _, buf = cv2.imencode('.jpg', annotated_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        publisher.publish(msg)

    def oakd_callback(self, msg):
        """Decode OAK-D frame, run YOLO, publish annotated image."""
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is None:
            return
        annotated = self._run_yolo_and_annotate(image)
        self._publish_annotated(self.oakd_annotated_pub, annotated, 'oakd')

    def wrist_timer_callback(self):
        """Poll wrist camera service and publish annotated image."""
        if not self.wrist_capture_client.service_is_ready():
            return
        req = CaptureImage.Request()
        future = self.wrist_capture_client.call_async(req)
        future.add_done_callback(self._wrist_capture_done)

    def _wrist_capture_done(self, future):
        """Handle wrist capture response."""
        try:
            response = future.result()
        except Exception:
            return
        if not response.success:
            return
        np_arr = np.frombuffer(response.image.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is None:
            return
        annotated = self._run_yolo_and_annotate(image)
        self._publish_annotated(self.wrist_annotated_pub, annotated, 'wrist_camera')

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

        # Find the highest confidence detection above threshold
        best_conf = 0.0
        best_class_id = -1
        best_class_name = ""

        if boxes is not None and len(boxes) > 0:
            for i in range(len(boxes)):
                conf = float(boxes.conf[i])
                cls_id = int(boxes.cls[i])
                cls_name = self.model.names[cls_id]

                if conf >= self.confidence_threshold and conf > best_conf and cls_name in self.target_classes:
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
                cv2.imwrite('/home/turtlebot/detected_object.jpg', annotated)
                self.get_logger().info("Saved annotated image to /home/turtlebot/detected_object.jpg")
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
