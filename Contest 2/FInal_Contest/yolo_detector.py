#!/usr/bin/env python3
"""
YOLO Object Detection Service for MIE443 Contest 2
Detects objects using YOLOv8 and returns the highest confidence detection.
Also publishes live annotated streams for OAK-D and wrist cameras.
"""

import os
import time

import rclpy
from rclpy.node import Node
from mie443_contest2.srv import DetectObject, CaptureImage
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from ultralytics import YOLO
import torch

SCREENSHOT_DIR_WRIST = '/home/turtlebot/yolo_screenshots/wrist'
SCREENSHOT_DIR_OAKD  = '/home/turtlebot/yolo_screenshots/oakd'


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Select compute device: use GPU if available for faster inference
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Using compute device: {self.device}')

        # Load YOLO model — yolov8s gives meaningfully better accuracy than yolov8n
        # with acceptable inference speed on a laptop CPU/GPU
        self.model = YOLO('yolov8m.pt')
        self.get_logger().info('YOLO model (yolov8m) loaded')

        # Create screenshot folders once at startup
        os.makedirs(SCREENSHOT_DIR_WRIST, exist_ok=True)
        os.makedirs(SCREENSHOT_DIR_OAKD, exist_ok=True)
        self._frame_counter = 0
        self.get_logger().info(f'Wrist screenshots: {SCREENSHOT_DIR_WRIST}/')
        self.get_logger().info(f'OAK-D screenshots: {SCREENSHOT_DIR_OAKD}/')

        # Per-class confidence thresholds.
        # Cup/bottle are common COCO classes — moderate threshold is fine.
        # Clock/motorcycle are rare in indoor COCO training data so need a lower
        # threshold to fire reliably. Potted plant sits in between.
        self.class_thresholds = {
            "cup":          0.30,
            "bottle":       0.45,
            "clock":        0.35,
            "potted plant": 0.40,
            "motorcycle":   0.35,
        }
        # Fallback for any class not in the table
        self.confidence_threshold = 0.45

        # Pre-create CLAHE object once to avoid per-call allocation overhead
        # (Bug #7 fix: was recreated on every _preprocess call)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # Flag to suppress wrist timer polling while a detect_callback is in progress.
        # Both this timer and detect_callback call wrist_camera/capture_image on the Pi;
        # suppressing the timer avoids queuing simultaneous requests to that service.
        # (Bug #3 fix: race condition between wrist live-stream timer and detection calls)
        self._suppress_wrist_timer = False

        # Target object categories (COCO class names and IDs)
        # These are the five scene objects used in the contest
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

        # Publishers for annotated camera streams (visualization only)
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

        # Timer to poll wrist camera at 5 Hz for live annotation stream
        self.wrist_timer = self.create_timer(0.2, self.wrist_timer_callback)

        self.get_logger().info('YOLO Detector Service ready')
        self.get_logger().info('  - OAK-D annotated stream: /yolo/oakd/compressed')
        self.get_logger().info('  - Wrist annotated stream:  /yolo/wrist/compressed')

    def _preprocess(self, image_bgr):
        """
        Apply CLAHE (Contrast Limited Adaptive Histogram Equalisation) to the
        luminance channel to improve detection under variable indoor lighting.
        Operates in LAB colour space so colour hue is preserved.
        """
        lab = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = self.clahe.apply(l)
        enhanced = cv2.merge((l, a, b))
        return cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)

    def _run_yolo_and_annotate(self, image_bgr):
        """Run YOLO on a BGR image and return annotated image (for live streams)."""
        results = self.model(image_bgr, verbose=False, device=self.device, iou=0.45)
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
        # Skip poll while a detection call is using or about to use the wrist camera service
        if self._suppress_wrist_timer:
            return
        if not self.wrist_capture_client.service_is_ready():
            return
        # Suppress further timer calls while this async request is in flight
        self._suppress_wrist_timer = True
        req = CaptureImage.Request()
        future = self.wrist_capture_client.call_async(req)
        future.add_done_callback(self._wrist_capture_done)

    def _wrist_capture_done(self, future):
        """Handle wrist capture response."""
        self._suppress_wrist_timer = False  # release so next timer tick can proceed
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
        self.get_logger().info(f'detect_callback reached — camera_source: "{request.camera_source}"')
        # Cancel the wrist live-stream timer for the duration of this call so it
        # cannot compete with contest2's wrist_camera/capture_image service call.
        self.wrist_timer.cancel()
        try:
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



            # Rotate wrist camera 180° — mounted upside-down on the arm
            if request.camera_source == "wrist":
                image = cv2.rotate(image, cv2.ROTATE_180)

            # Enhance contrast for variable indoor lighting before inference
            image = self._preprocess(image)

            # OAK-D images are upscaled to 640x640 for optimal YOLO input resolution.
            # Wrist camera images are passed at native resolution to preserve quality
            # (upscaling a small wrist image reduces effective detail).
            if request.camera_source == "oakd":
                image = cv2.resize(image, (640, 640), interpolation=cv2.INTER_LINEAR)

            # Run YOLO inference with tighter NMS IoU for cleaner single-object results
            results = self.model(image, verbose=False, device=self.device, iou=0.45)
            boxes = results[0].boxes

            # Find the highest confidence detection above threshold among target classes
            best_conf = 0.0
            best_class_id = -1
            best_class_name = ""
            best_bbox_center_norm = 0.0  # horizontal offset: -1=far left, 0=centre, +1=far right

            if boxes is not None and len(boxes) > 0:
                for i in range(len(boxes)):
                    conf = float(boxes.conf[i])
                    cls_id = int(boxes.cls[i])
                    cls_name = self.model.names[cls_id]

                    thresh = self.class_thresholds.get(cls_name, self.confidence_threshold)
                    if (conf >= thresh
                            and conf > best_conf
                            and cls_name in self.target_classes):
                        best_conf = conf
                        best_class_id = cls_id
                        best_class_name = cls_name
                        x1, y1, x2, y2 = boxes.xyxy[i].tolist()
                        bbox_cx = (x1 + x2) / 2.0
                        best_bbox_center_norm = (bbox_cx - 320.0) / 320.0  # image is 640px wide

            # Draw only target-class detections on the saved image
            annotated = image.copy()
            if boxes is not None and len(boxes) > 0:
                for i in range(len(boxes)):
                    conf = float(boxes.conf[i])
                    cls_id = int(boxes.cls[i])
                    cls_name = self.model.names[cls_id]
                    thresh = self.class_thresholds.get(cls_name, self.confidence_threshold)
                    if conf < thresh or cls_name not in self.target_classes:
                        continue
                    x1, y1, x2, y2 = map(int, boxes.xyxy[i].tolist())
                    label_text = f"{cls_name} {conf:.2f}"
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(annotated, label_text, (x1, y1 - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # Overlay extra_label (e.g. AprilTag ID) on the image if provided
            if request.extra_label:
                cv2.putText(annotated, request.extra_label, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

            ts = time.strftime('%Y%m%d_%H%M%S')
            label = request.extra_label if request.extra_label else (best_class_name if best_class_id != -1 else "none")
            save_dir = SCREENSHOT_DIR_OAKD if request.camera_source == "oakd" else SCREENSHOT_DIR_WRIST
            frame_path = os.path.join(save_dir, f'frame_{self._frame_counter:04d}_{ts}_{label}.jpg')
            cv2.imwrite(frame_path, annotated)
            self._frame_counter += 1
            if best_class_id != -1:
                response.success = True
                response.class_id = best_class_id
                response.class_name = best_class_name
                response.confidence = best_conf
                response.bbox_center_norm = best_bbox_center_norm
                response.message = f"Detected {best_class_name} with confidence {best_conf:.2f}"

                # Also save to fixed path if explicitly requested
                if save_detected_image:
                    cv2.imwrite('/home/turtlebot/detected_object.jpg', annotated)
                    self.get_logger().info("Saved annotated image to /home/turtlebot/detected_object.jpg")
            else:
                response.success = False
                response.class_id = -1
                response.class_name = ""
                response.confidence = 0.0
                response.bbox_center_norm = 0.0
                response.message = "No target-class detection above confidence threshold"

            return response
        finally:
            self._suppress_wrist_timer = False
            self.wrist_timer.reset()  # restart the live-stream timer


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
