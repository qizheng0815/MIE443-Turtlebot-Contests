#include "mie443_contest2/yoloInterface.h"
#include <chrono>
#include <fstream>

using namespace std::chrono_literals;

YoloInterface::YoloInterface(std::shared_ptr<rclcpp::Node> node)
    : node_(node), has_detection_(false), latest_confidence_(0.0), latest_class_id_(-1), latest_bbox_center_norm_(0.0) {

    // Create service clients
    capture_client_oakd_ = node_->create_client<mie443_contest2::srv::CaptureImage>("oakd_camera/capture_image");
    capture_client_wrist_ = node_->create_client<mie443_contest2::srv::CaptureImage>("wrist_camera/capture_image");
    detect_client_ = node_->create_client<mie443_contest2::srv::DetectObject>("detect_object");

    // Check service availability
    if (capture_client_oakd_->wait_for_service(5s)) {
        RCLCPP_INFO(node_->get_logger(), "YoloInterface: Connected to OAK-D camera");
    } else {
        RCLCPP_WARN(node_->get_logger(), "OAK-D camera service not available");
    }

    if (capture_client_wrist_->wait_for_service(5s)) {
        RCLCPP_INFO(node_->get_logger(), "YoloInterface: Connected to wrist camera");
    } else {
        RCLCPP_WARN(node_->get_logger(), "Wrist camera service not available");
    }

    if (detect_client_->wait_for_service(5s)) {
        RCLCPP_INFO(node_->get_logger(), "YoloInterface: Connected to YOLO detector");
    } else {
        RCLCPP_WARN(node_->get_logger(), "YOLO detection service not available");
    }
}

bool YoloInterface::captureAndDetect(CameraSource camera, bool save_image, const std::string& extra_label) {
    has_detection_ = false;

    // Select camera
    auto capture_client = (camera == CameraSource::OAKD) ? capture_client_oakd_ : capture_client_wrist_;
    std::string camera_name = (camera == CameraSource::OAKD) ? "OAK-D" : "Wrist";

    // Capture image
    if (!capture_client->wait_for_service(2s)) {
        RCLCPP_ERROR(node_->get_logger(), "%s camera service not available", camera_name.c_str());
        return false;
    }

    auto capture_request = std::make_shared<mie443_contest2::srv::CaptureImage::Request>();
    auto capture_future = capture_client->async_send_request(capture_request);

    if (rclcpp::spin_until_future_complete(node_, capture_future, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to capture from %s camera", camera_name.c_str());
        return false;
    }

    auto capture_response = capture_future.get();
    if (!capture_response->success) {
        RCLCPP_ERROR(node_->get_logger(), "%s capture failed: %s", camera_name.c_str(), capture_response->message.c_str());
        return false;
    }


    // Run YOLO detection
    if (!detect_client_->wait_for_service(2s)) {
        RCLCPP_ERROR(node_->get_logger(), "YOLO detection service not available");
        return false;
    }

    auto detect_request = std::make_shared<mie443_contest2::srv::DetectObject::Request>();
    detect_request->image = capture_response->image;
    detect_request->save_detected_image = save_image;
    detect_request->camera_source = (camera == CameraSource::OAKD) ? "oakd" : "wrist";
    detect_request->extra_label = extra_label;
    auto detect_future = detect_client_->async_send_request(detect_request);

    if (rclcpp::spin_until_future_complete(node_, detect_future, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "YOLO detection call failed");
        return false;
    }

    auto detect_response = detect_future.get();
    if (!detect_response->success) {
        RCLCPP_WARN(node_->get_logger(), "No detection: %s", detect_response->message.c_str());
        return false;
    }

    // Store results
    latest_class_name_ = detect_response->class_name;
    latest_confidence_ = detect_response->confidence;
    latest_class_id_ = detect_response->class_id;
    latest_bbox_center_norm_ = detect_response->bbox_center_norm;
    has_detection_ = true;

    RCLCPP_INFO(node_->get_logger(), "Detected: %s (%.2f) from %s camera",
                latest_class_name_.c_str(), latest_confidence_, camera_name.c_str());
    return true;
}

std::string YoloInterface::getObjectName(CameraSource camera, bool save_image) {
    if (captureAndDetect(camera, save_image)) {
        return latest_class_name_;
    }
    return "";
}

float YoloInterface::getConfidence() {
    return has_detection_ ? latest_confidence_ : -1.0f;
}

int YoloInterface::getClassId() {
    return has_detection_ ? latest_class_id_ : -1;
}

std::string YoloInterface::getLatestClassName() {
    return latest_class_name_;
}

float YoloInterface::getBboxCenterNorm() {
    return has_detection_ ? latest_bbox_center_norm_ : 0.0f;
}

bool YoloInterface::hasDetection() {
    return has_detection_;
}

void YoloInterface::clearDetection() {
    has_detection_ = false;
    latest_class_name_ = "";
    latest_confidence_ = 0.0;
    latest_class_id_ = -1;
    latest_bbox_center_norm_ = 0.0;
}
