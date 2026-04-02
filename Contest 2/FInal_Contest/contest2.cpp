#include "mie443_contest2/boxes.h"
#include "mie443_contest2/navigation.h"
#include "mie443_contest2/robot_pose.h"
#include "mie443_contest2/yoloInterface.h"
#include <mie443_contest2/srv/detect_object.hpp>
#include "mie443_contest2/arm_controller.h"
#include "mie443_contest2/apriltag_detector.h"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>

void dropoff(ArmController& armController, rclcpp::Node::SharedPtr node);

// Clear both local and global Nav2 costmaps to remove stale obstacle data.
void clearCostmaps(std::shared_ptr<rclcpp::Node> node) {
    for (const auto& svc : {
            "/local_costmap/clear_entirely_local_costmap",
            "/global_costmap/clear_entirely_global_costmap"}) {
        auto client = node->create_client<nav2_msgs::srv::ClearEntireCostmap>(svc);
        if (!client->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(node->get_logger(), "Costmap clear service %s not available", svc);
            continue;
        }
        auto req = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
        auto future = client->async_send_request(req);
        rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(3));
        RCLCPP_INFO(node->get_logger(), "Cleared costmap: %s", svc);
    }
}

// Drive backward at low speed for backup_sec seconds to escape a stuck position.
void backUp(std::shared_ptr<rclcpp::Node> node, double speed = 0.12, double backup_sec = 2.0) {
    auto vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = -speed;
    auto start = node->now();
    RCLCPP_INFO(node->get_logger(), "Backing up %.1f m/s for %.1f s...", speed, backup_sec);
    while ((node->now() - start).seconds() < backup_sec && rclcpp::ok()) {
        vel_pub->publish(cmd);
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    cmd.linear.x = 0.0;
    vel_pub->publish(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

// Navigate to goal with a recovery sequence on failure:
//   1. Try direct navigation
//   2. Clear costmaps → retry
//   3. Back up 2s → retry
//   4. Keep robot_radius, reduce inflation to 0.05m → retry → restore inflation
//   5. Reduce robot radius to 0.12m → retry → restore radius
// time_budget_sec caps total time across all attempts so a stuck bin
// cannot prevent remaining bins from being visited.
// Returns true if any attempt succeeded.
bool navigateWithRecovery(Navigation& nav, std::shared_ptr<rclcpp::Node> node,
                          double x, double y, double phi, int time_budget_sec = 105) {
    auto budget_start = std::chrono::steady_clock::now();
    auto time_used = [&]() {
        return (int)std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - budget_start).count();
    };
    // Each attempt gets an equal share of the remaining budget (5 attempts total)
    auto attempt_timeout = [&](int attempt_num) -> int {
        int remaining = time_budget_sec - time_used();
        int attempts_left = 6 - attempt_num;  // attempts_left including this one
        return std::max(5, remaining / attempts_left);
    };

    // Attempt 1 — normal navigation
    if (nav.moveToGoalWithTimeout(x, y, phi, attempt_timeout(1))) return true;
    if (time_used() >= time_budget_sec) {
        RCLCPP_WARN(node->get_logger(), "Nav budget exhausted — skipping waypoint");
        return false;
    }
    RCLCPP_WARN(node->get_logger(), "Nav failed — clearing costmaps and retrying...");

    // Attempt 2 — clear costmaps + retry
    clearCostmaps(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (nav.moveToGoalWithTimeout(x, y, phi, attempt_timeout(2))) return true;
    if (time_used() >= time_budget_sec) {
        RCLCPP_WARN(node->get_logger(), "Nav budget exhausted — skipping waypoint");
        return false;
    }
    RCLCPP_WARN(node->get_logger(), "Nav failed after costmap clear — backing up and retrying...");

    // Attempt 3 — back up + retry
    backUp(node);
    clearCostmaps(node);
    if (nav.moveToGoalWithTimeout(x, y, phi, attempt_timeout(3))) return true;
    if (time_used() >= time_budget_sec) {
        RCLCPP_WARN(node->get_logger(), "Nav budget exhausted — skipping waypoint");
        return false;
    }
    RCLCPP_WARN(node->get_logger(), "Nav failed after backup — reducing inflation radius and retrying...");

    // Attempt 4 — keep robot_radius at 0.19m but drop inflation padding to 0.05m so the
    // planner can thread narrow gaps while still respecting the robot's physical footprint
    auto param_client_lc = std::make_shared<rclcpp::AsyncParametersClient>(node, "local_costmap");
    auto param_client_gc = std::make_shared<rclcpp::AsyncParametersClient>(node, "global_costmap");
    for (auto& pc : {param_client_lc, param_client_gc}) {
        if (pc->wait_for_service(std::chrono::seconds(2))) {
            auto f = pc->set_parameters({
                rclcpp::Parameter("inflation_layer.inflation_radius", 0.05)
            });
            rclcpp::spin_until_future_complete(node, f, std::chrono::seconds(3));
        }
    }
    RCLCPP_INFO(node->get_logger(), "Inflation radius temporarily reduced to 0.05 m");
    clearCostmaps(node);
    bool reached_inflation = nav.moveToGoalWithTimeout(x, y, phi, attempt_timeout(4));

    // Always restore inflation before proceeding — attempt 5 sets robot_radius to 0.12m
    // which would violate Nav2's constraint that inflation_radius >= circumscribed_radius
    for (auto& pc : {param_client_lc, param_client_gc}) {
        if (pc->wait_for_service(std::chrono::seconds(2))) {
            auto f = pc->set_parameters({
                rclcpp::Parameter("inflation_layer.inflation_radius", 0.24)
            });
            rclcpp::spin_until_future_complete(node, f, std::chrono::seconds(3));
        }
    }
    RCLCPP_INFO(node->get_logger(), "Inflation radius restored to 0.24 m");
    if (reached_inflation) return true;
    if (time_used() >= time_budget_sec) {
        RCLCPP_WARN(node->get_logger(), "Nav budget exhausted — skipping waypoint");
        return false;
    }
    RCLCPP_WARN(node->get_logger(), "Nav failed with reduced inflation — shrinking robot radius and retrying...");

    // Attempt 5 — shrink robot radius to 0.12m so planner finds a path, then restore
    for (auto& pc : {param_client_lc, param_client_gc}) {
        if (pc->wait_for_service(std::chrono::seconds(2))) {
            auto f = pc->set_parameters({rclcpp::Parameter("robot_radius", 0.12)});
            rclcpp::spin_until_future_complete(node, f, std::chrono::seconds(3));
        }
    }
    RCLCPP_INFO(node->get_logger(), "Robot radius temporarily reduced to 0.12 m");
    clearCostmaps(node);
    bool reached = nav.moveToGoalWithTimeout(x, y, phi, attempt_timeout(5));

    // Restore robot radius and inflation radius
    for (auto& pc : {param_client_lc, param_client_gc}) {
        if (pc->wait_for_service(std::chrono::seconds(2))) {
            auto f = pc->set_parameters({
                rclcpp::Parameter("robot_radius", 0.19),
                rclcpp::Parameter("inflation_layer.inflation_radius", 0.24)
            });
            rclcpp::spin_until_future_complete(node, f, std::chrono::seconds(3));
        }
    }
    RCLCPP_INFO(node->get_logger(), "Robot radius and inflation radius restored");

    if (!reached) {
        RCLCPP_ERROR(node->get_logger(),
                     "All recovery attempts failed — skipping waypoint (%.2f, %.2f)", x, y);
        // Clear costmaps after full failure to reset Nav2's state for the next waypoint.
        // This prevents a corrupted/stale costmap from causing immediate code-6 rejections
        // on subsequent goals.
        clearCostmaps(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    return reached;
}

// Set robot_radius on both global and local costmaps so the planner keeps the
// robot further from walls, avoiding paths through corridors that are too tight.
// Originally 0.17m (TurtleBot4 Lite physical radius); increased to avoid narrow-corridor stuck behaviour.
void setRobotRadius(std::shared_ptr<rclcpp::Node> node, double radius) {
    for (const auto& costmap : {"global_costmap", "local_costmap"}) {
        auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, costmap);
        if (!client->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_WARN(node->get_logger(), "%s not available — robot_radius not updated", costmap);
            continue;
        }
        // inflation_radius must be >= robot circumscribed radius to avoid planning errors
        auto future = client->set_parameters({
            rclcpp::Parameter("robot_radius", radius),
            rclcpp::Parameter("inflation_layer.inflation_radius", radius + 0.05)
        });
        if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(3)) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "%s robot_radius=%.2f m, inflation_radius=%.2f m",
                        costmap, radius, radius + 0.05);
        } else {
            RCLCPP_WARN(node->get_logger(), "Failed to set robot_radius on %s", costmap);
        }
    }
}

// Set Nav2 controller_server goal tolerances dynamically.
// xy_tol  : acceptable radius around the target x,y (metres)
// yaw_tol : acceptable yaw deviation (radians)
void setNavTolerances(std::shared_ptr<rclcpp::Node> node, double xy_tol, double yaw_tol) {
    auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "controller_server");
    if (!param_client->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_WARN(node->get_logger(), "controller_server not available — using default tolerances");
        return;
    }
    auto future = param_client->set_parameters({
        rclcpp::Parameter("general_goal_checker.xy_goal_tolerance",  xy_tol),
        rclcpp::Parameter("general_goal_checker.yaw_goal_tolerance", yaw_tol)
    });
    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(3)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(),
                    "Nav tolerances set — xy: %.2f m (circle), yaw: %.1f deg (cone)",
                    xy_tol, yaw_tol * 180.0 / M_PI);
    } else {
        RCLCPP_WARN(node->get_logger(), "Failed to set nav tolerances");
    }
}

int main(int argc, char** argv) {
    // Setup ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("contest2");

    // Load the arm URDF and SRDF directly as node parameters so that
    // MoveGroupInterface builds the SO-ARM101 model
    {
        std::string desc_dir = ament_index_cpp::get_package_share_directory("lerobot_description");
        std::ifstream urdf_file(desc_dir + "/urdf/so101.urdf");
        if (urdf_file.is_open()) {
            std::stringstream ss;
            ss << urdf_file.rdbuf();
            node->declare_parameter("robot_description", ss.str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Could not open arm URDF file");
        }

        std::string moveit_dir = ament_index_cpp::get_package_share_directory("lerobot_moveit");
        std::ifstream srdf_file(moveit_dir + "/config/so101.srdf");
        if (srdf_file.is_open()) {
            std::stringstream ss;
            ss << srdf_file.rdbuf();
            node->declare_parameter("robot_description_semantic", ss.str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Could not open arm SRDF file");
        }
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node started");

    // Robot pose object + subscriber
    RobotPose robotPose(0, 0, 0);
    // TRANSIENT_LOCAL matches AMCL's publisher durability — ensures the last published
    // pose is delivered even if the pose was set before this node subscribed.
    auto amcl_qos = rclcpp::QoS(10).transient_local();
    auto amclSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        amcl_qos,
        std::bind(&RobotPose::poseCallback, &robotPose, std::placeholders::_1)
    );

    // Initialize box coordinates
    Boxes boxes;
    if(!boxes.load_coords()) {
        RCLCPP_ERROR(node->get_logger(), "ERROR: could not load box coordinates");
        return -1;
    }

    for(size_t i = 0; i < boxes.coords.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "Box %zu coordinates: x=%.2f, y=%.2f, phi=%.2f",
                    i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
    }

    // Initialize arm controller
    ArmController armController(node);
    YoloInterface yolo(node);
    AprilTagDetector tagDetector(node);

    // Block until yolo_detector.py is ready (detect_object service available)
    {
        auto detectClient = node->create_client<mie443_contest2::srv::DetectObject>("detect_object");
        RCLCPP_INFO(node->get_logger(), "Waiting for yolo_detector to become available...");
        while (!detectClient->wait_for_service(std::chrono::seconds(2)) && rclcpp::ok()) {
            RCLCPP_WARN(node->get_logger(), "detect_object service not yet available — retrying...");
            rclcpp::spin_some(node);
        }
        RCLCPP_INFO(node->get_logger(), "yolo_detector ready.");
    }
/*
    // OAK-D test detection at contest start (saved to yolo_screenshots/oakd/)
    RCLCPP_INFO(node->get_logger(), "=== OAK-D YOLO test detection ===");
    if (yolo.captureAndDetect(CameraSource::OAKD, true)) {
        RCLCPP_INFO(node->get_logger(), "OAK-D detected: %s (ID: %d, conf: %.2f)",
                    yolo.getLatestClassName().c_str(), yolo.getClassId(), yolo.getConfidence());
    } else {
        RCLCPP_WARN(node->get_logger(), "OAK-D test detection: nothing detected");
    }
    */
/*
    // AprilTag test — scan with OAK-D and save screenshot labelled with detected tag ID
    RCLCPP_INFO(node->get_logger(), "=== AprilTag test detection ===");
    {
        std::vector<int> testTags = {0, 1, 2, 3, 4};  // 36H11 family, IDs 0-4
        // Spin briefly to populate TF before querying
        for (int s = 0; s < 20; ++s) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::vector<int> found = tagDetector.getVisibleTags(testTags, 500);
        if (!found.empty()) {
            std::string tagLabel = "AprilTag";
            for (int id : found) tagLabel += " " + std::to_string(id);
            RCLCPP_INFO(node->get_logger(), "AprilTag test: detected tag(s): %s", tagLabel.c_str());
            yolo.captureAndDetect(CameraSource::OAKD, true, tagLabel);
        } else {
            RCLCPP_WARN(node->get_logger(), "AprilTag test: no tags detected");
            yolo.captureAndDetect(CameraSource::OAKD, true, "AprilTag_none");
        }
    }
*/

/*
    //Initialize home position
    RCLCPP_INFO(node->get_logger(), "=== INITIAL ARM POSITION");
    bool success = armController.moveToCartesianPose(0.036, -0.174, 0.190,
                                                     -0.029, -0.004, -0.041, 0.999);
    if(success) {
        RCLCPP_INFO(node->get_logger(), "Arm moved successfully!, arm initialized");
        std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
        RCLCPP_ERROR(node->get_logger(), "Arm movement failed - pose may be unreachable");
    }
*/
    // Object Detection Pose
    bool success = armController.moveToCartesianPose(0.165, 0.003, 0.181,
                                                     0.314, 0.002, 1.757);

    if(success) {
        RCLCPP_INFO(node->get_logger(), "Arm moved successfully!");
        //std::this_thread::sleep_for(std::chrono::seconds(2));

    } else {
        RCLCPP_ERROR(node->get_logger(), "Arm movement failed Object detection pose is unreachable");
    }


    // Test arm movement with pre-OBjectDetection pose
    success = armController.moveToCartesianPose(0.165, 0.003, 0.181,
                                                     0.093, 0.112, 0.757, 0.637);
    if(success) {
        RCLCPP_INFO(node->get_logger(), "Arm returned to home position successfully!");
        //std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
        RCLCPP_ERROR(node->get_logger(), "Arm movement failed - pose4 may be unreachable");
    }

    armController.openGripper(); //OPEN GRIPPER FUll

    // Object detected by wrist camera (used to match against bins later)
    std::string detectedObject;
    int objectId = -1;
    float bestWristConf = -1.0f;  // tracks highest confidence across all wrist detection attempts

    // Open results log file (timestamped so each run creates a new file)
    auto log_ts = std::chrono::system_clock::now();
    std::time_t log_t = std::chrono::system_clock::to_time_t(log_ts);
    char log_ts_buf[32];
    std::strftime(log_ts_buf, sizeof(log_ts_buf), "%Y%m%d_%H%M%S", std::localtime(&log_t));
    std::string log_path = std::string("/home/turtlebot/contest2_results_") + log_ts_buf + ".txt";
    std::ofstream log_file(log_path);
    RCLCPP_INFO(node->get_logger(), "Logging results to %s", log_path.c_str());
    log_file << "=== Contest 2 Results (" << log_ts_buf << ") ===\n\n";
/*
// Test YOLO detection
    if (yolo.captureAndDetect(CameraSource::WRIST, true)) {
        detectedObject = yolo.getLatestClassName();
        objectId = yolo.getClassId();
        RCLCPP_INFO(node->get_logger(), "YOLO detection successful: %s | ID: %d | confidence: %.2f",
                    detectedObject.c_str(), objectId, yolo.getConfidence());
    } else {
        detectedObject = "cup";
        objectId = 41;
        RCLCPP_WARN(node->get_logger(), "YOLO detection failed -- defaulting to cup (ID: 41)");
    }
*/
    // Test arm movement with pre-Object detection pose
    success = armController.moveToCartesianPose(0.192, -0.056, 0.158,
                                                     0.168, 0.142, 0.616, 0.757);
    if(success) {
        RCLCPP_INFO(node->get_logger(), "Arm returned to home position successfully!");
        //std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
        RCLCPP_ERROR(node->get_logger(), "Arm movement failed - pose4 may be unreachable");
    }

  
// Wrist detection attempt 1
    if (yolo.captureAndDetect(CameraSource::WRIST, true)) {
        float conf = yolo.getConfidence();
        if (conf > bestWristConf) {
            bestWristConf  = conf;
            detectedObject = yolo.getLatestClassName();
            objectId       = yolo.getClassId();
        }
        RCLCPP_INFO(node->get_logger(), "Wrist attempt 1: %s | ID: %d | conf: %.2f",
                    yolo.getLatestClassName().c_str(), yolo.getClassId(), conf);
        log_file << "[Wrist attempt 1] " << yolo.getLatestClassName()
                 << " | ID: " << yolo.getClassId()
                 << " | Confidence: " << conf << "\n";
    } else {
        RCLCPP_WARN(node->get_logger(), "Wrist attempt 1: no detection");
        log_file << "[Wrist attempt 1] No detection\n";
    }
    log_file.flush();

    // TEST arm movement with pose 1
    success = armController.moveToCartesianPose(0.114, 0.009, 0.193,
                                                0.006, 0.009, 0.811, 0.585);
    
    if(success) {  
        RCLCPP_INFO(node->get_logger(), "Arm moved successfully!");
        //std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
        RCLCPP_ERROR(node->get_logger(), "Arm movement failed - pose1 may be unreachable");
    }

/*
    // Test YOLO detection
    if (yolo.captureAndDetect(CameraSource::WRIST, true)) {
        detectedObject = yolo.getLatestClassName();
        objectId = yolo.getClassId();
        RCLCPP_INFO(node->get_logger(), "YOLO detection successful: %s | ID: %d | confidence: %.2f",
                    detectedObject.c_str(), objectId, yolo.getConfidence());
    } else {
        detectedObject = "cup";
        objectId = 41;
        RCLCPP_WARN(node->get_logger(), "YOLO detection failed -- defaulting to cup (ID: 41)");
    }

*/
/*
    // Test arm movement with Object detection pose
    success = armController.moveToCartesianPose(0.145, -0.031, 0.146,
                                                     0.204, 0.249, 0.552, 0.769);
    if(success) {
        RCLCPP_INFO(node->get_logger(), "Arm returned to home position successfully!");
        //std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
        RCLCPP_ERROR(node->get_logger(), "Arm movement failed - pose4 may be unreachable");
    }

*/
    // Wrist detection attempt 2
    if (yolo.captureAndDetect(CameraSource::WRIST, true)) {
        float conf = yolo.getConfidence();
        if (conf > bestWristConf) {
            bestWristConf  = conf;
            detectedObject = yolo.getLatestClassName();
            objectId       = yolo.getClassId();
        }
        RCLCPP_INFO(node->get_logger(), "Wrist attempt 2: %s | ID: %d | conf: %.2f",
                    yolo.getLatestClassName().c_str(), yolo.getClassId(), conf);
        log_file << "[Wrist attempt 2] " << yolo.getLatestClassName()
                 << " | ID: " << yolo.getClassId()
                 << " | Confidence: " << conf << "\n";
    } else {
        RCLCPP_WARN(node->get_logger(), "Wrist attempt 2: no detection");
        log_file << "[Wrist attempt 2] No detection\n";
    }

    // If neither attempt detected anything, default to cup
    if (bestWristConf < 0.0f) {
        detectedObject = "cup";
        objectId = 41;
        RCLCPP_WARN(node->get_logger(), "All wrist attempts failed — defaulting to cup (ID: 41)");
        log_file << "[Wrist Camera] All attempts failed — defaulting to: cup (ID: 41)\n";
    } else {
        RCLCPP_INFO(node->get_logger(), "Best wrist detection: %s (ID: %d, conf: %.2f)",
                    detectedObject.c_str(), objectId, bestWristConf);
        log_file << "[Wrist Camera] Best detection: " << detectedObject
                 << " | ID: " << objectId << " | Confidence: " << bestWristConf << "\n";
    }
    log_file << "\n";
    log_file.flush();

/*
    // Test arm movement with pose 2
    success = armController.moveToCartesianPose(0.113, 0.008, 0.161,
                                                0.011, 0.015, 0.810, 0.585);

    if(success) {
        RCLCPP_INFO(node->get_logger(), "Arm moved successfully!"); 
        

        //std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
        RCLCPP_ERROR(node->get_logger(), "Arm movement failed - pose2 may be unreachable");
    }


*/
    //Test arm movement with pose 3
    success = armController.moveToCartesianPose(0.113, 0.008, 0.161,
                                                0.011, 0.015, 0.810, 0.585);
            
    if(success) {
        RCLCPP_INFO(node->get_logger(), "Arm moved successfully!");

        //std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
        RCLCPP_ERROR(node->get_logger(), "Arm movement failed - pose3 may be unreachable");
    }

    armController.closeGripper();

    // Test arm movement with pose 4
    success = armController.moveToCartesianPose(0.165, 0.003, 0.181,
                                                     0.093, 0.112, 0.757, 0.637);
    if(success) {
        RCLCPP_INFO(node->get_logger(), "Arm returned to home position successfully!");
        //std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
        RCLCPP_ERROR(node->get_logger(), "Arm movement failed - pose4 may be unreachable");
    }

    // Return to home position (90 degrees clockwise around Z-axis)
    success = armController.moveToCartesianPose(0.036, -0.174, 0.190,
                                                     -0.029, -0.004, -0.041, 0.999);
    if(success) {
        RCLCPP_INFO(node->get_logger(), "Arm returned to home position successfully!");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Arm movement failed - failed to return to home position");
    }

    //dropoff(armController, node); //testing drop off sequence



    uint64_t secondsElapsed = 0;

    // --- Wait until AMCL publishes at least one real pose (30s timeout) ---
    // Done BEFORE starting the contest timer so this wait does not eat into the 300s window.
    // The subscription uses TRANSIENT_LOCAL so it will receive the pose even if
    // the 2D Pose Estimate was set in RViz before this node started.
    RCLCPP_INFO(node->get_logger(), "Waiting for AMCL pose (set 2D Pose Estimate in RViz)...");
    {
        auto amcl_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
        while (rclcpp::ok() && !robotPose.received &&
               std::chrono::steady_clock::now() < amcl_deadline) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    if (robotPose.received) {
        RCLCPP_INFO(node->get_logger(), "AMCL pose received — waiting 2s for particle filter to stabilize...");
        auto stable = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (rclcpp::ok() && std::chrono::steady_clock::now() < stable) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } else {
        RCLCPP_WARN(node->get_logger(),
                    "AMCL pose timed out — navigation will be unreliable without initial pose!");
        RCLCPP_WARN(node->get_logger(),
                    "Assuming home (0, 0, 0) — set 2D Pose Estimate in RViz before running!");
    }

    // --- Initialize navigation ---
    Navigation nav(node);

    // 50 cm position circle, ±10° yaw cone
    setNavTolerances(node, 0.50, 6.0 * M_PI / 180.0);

    // Wait for costmap parameter services to be ready before setting robot radius.
    // Done BEFORE the timer so this overhead doesn't reduce navigation time.
    {
        auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, "global_costmap");
        for (int retry = 0; retry < 10 && !client->wait_for_service(std::chrono::seconds(2)); ++retry) {
            RCLCPP_WARN(node->get_logger(), "Costmap not ready — retry %d/10", retry + 1);
            rclcpp::spin_some(node);
        }
    }
    // Increase robot radius to 0.19m (originally 0.17m) so the planner avoids
    // paths through narrow corridors the robot physically cannot navigate cleanly
    setRobotRadius(node, 0.19);

    // --- Store home position (after AMCL has settled) ---
    double homeX   = robotPose.x;
    double homeY   = robotPose.y;
    double homePhi = robotPose.phi;
    RCLCPP_INFO(node->get_logger(),
                "Home stored: x=%.3f  y=%.3f  phi=%.3f rad", homeX, homeY, homePhi);

    // Contest countdown timer — starts AFTER all initialization so the full 300s
    // is available for navigation, not consumed by AMCL/costmap startup waits.
    auto start = std::chrono::system_clock::now();
    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    // Execute strategy
    while(rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);

        // Calculate elapsed time
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

        /***YOUR CODE HERE***/

        // --- Waypoints from coords.xml ---
        const int NUM_WP = 5;
        double wp[NUM_WP][3] = {};
        {
            const std::string COORDS_XML =
                "/home/turtlebot/ros2_ws/src/mie443_contest2/mie443_contest2/boxes_database/coords.xml";
            std::ifstream xmlFile(COORDS_XML);
            if (!xmlFile.is_open()) {
                RCLCPP_ERROR(node->get_logger(), "Failed to open coords.xml at %s", COORDS_XML.c_str());
                break;
            }
            std::string content((std::istreambuf_iterator<char>(xmlFile)),
                                 std::istreambuf_iterator<char>());
            xmlFile.close();

            for (int i = 0; i < NUM_WP; ++i) {
                std::string tag = "coordinate" + std::to_string(i + 1);
                size_t start = content.find("<" + tag + ">") + tag.size() + 2;
                size_t end   = content.find("</" + tag + ">");
                std::string block = content.substr(start, end - start);
                // Strip XML comments
                while (true) {
                    size_t cs = block.find("<!--");
                    if (cs == std::string::npos) break;
                    size_t ce = block.find("-->", cs) + 3;
                    block.erase(cs, ce - cs);
                }
                std::istringstream iss(block);
                iss >> wp[i][0] >> wp[i][1] >> wp[i][2];
                wp[i][2] += M_PI;
                RCLCPP_INFO(node->get_logger(),
                            "Loaded coordinate%d: x=%.3f y=%.3f phi=%.3f",
                            i + 1, wp[i][0], wp[i][1], wp[i][2]);
            }
        }

        // --- Per-bin detection results ---
        std::string binObject[NUM_WP];
        int binObjectId[NUM_WP];
        for (int i = 0; i < NUM_WP; ++i) { binObject[i] = ""; binObjectId[i] = -1; }
        int lastVisitedBin = -1;   // tracks the last bin actually visited (not skipped)
        bool droppedOff    = false; // true once dropoff() has been called

        // Candidate AprilTag IDs to scan for at each bin (AprilTag disabled)
        // std::vector<int> tagCandidates = {0, 1, 2, 3, 4};  // 36H11 family, IDs 0-4

        // Publisher for AMCL relocalization — used to snap pose when YOLO centres on a known bin
        auto initialpose_pub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

        // Time constants for budget planning
        const int CONTEST_DURATION  = 300;  // seconds
        const int HOME_RESERVE      = 40;   // seconds reserved to return home
        const int MIN_TIME_PER_BIN  = 15;   // minimum seconds needed to attempt a bin

        // --- Sequential path planning ---
        for (int i = 0; i < NUM_WP && rclcpp::ok(); ++i) {
            bool isLastBin = (i == NUM_WP - 1);

            // Refresh elapsed time and check whether there is enough time budget remaining
            // to visit this bin AND all subsequent bins AND return home.
            {
                auto tNow = std::chrono::system_clock::now();
                secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                                     tNow - start).count();
                int time_remaining  = CONTEST_DURATION - (int)secondsElapsed;
                int bins_remaining  = NUM_WP - i;           // includes current bin
                int time_needed     = HOME_RESERVE + bins_remaining * MIN_TIME_PER_BIN;

                RCLCPP_INFO(node->get_logger(),
                            "Bin %d/%d — %ds remaining, need ~%ds for %d bins + home",
                            i + 1, NUM_WP, time_remaining, time_needed, bins_remaining);

                if (time_remaining < time_needed) {
                    RCLCPP_WARN(node->get_logger(),
                                "Skipping bin %d — only %ds left, not enough for %d bins + home reserve",
                                i + 1, time_remaining, bins_remaining);
                    continue;
                }
            }

            // Compute standoff position: 0.4m back from the bin along phi
            // wp[i][0..1] is the approximate bin position; wp[i][2] is the heading toward the bin
            double goal_x   = wp[i][0] - 0.4 * std::cos(wp[i][2]);
            double goal_y   = wp[i][1] - 0.4 * std::sin(wp[i][2]);
            double goal_phi = wp[i][2];

            // Budget for this bin: fair share of remaining time minus home reserve,
            // capped at 90s so one bin cannot starve the others.
            {
                auto tNow = std::chrono::system_clock::now();
                secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                                     tNow - start).count();
                int time_remaining = CONTEST_DURATION - (int)secondsElapsed;
                int bins_remaining = NUM_WP - i;
                int bin_budget = std::min(90, (time_remaining - HOME_RESERVE) / bins_remaining);

                RCLCPP_INFO(node->get_logger(),
                            "Step %d/%d — bin %d at (%.3f, %.3f), standoff goal (%.3f, %.3f, %.3f rad) [budget: %ds]",
                            i + 1, NUM_WP, i + 1,
                            wp[i][0], wp[i][1], goal_x, goal_y, goal_phi, bin_budget);

                bool nav_reached = navigateWithRecovery(nav, node, goal_x, goal_y, goal_phi, bin_budget);
                if (!nav_reached) {
                    RCLCPP_WARN(node->get_logger(),
                                "Bin %d: navigation failed — skipping detection and relocalization",
                                i + 1);
                    continue;  // Do NOT run YOLO or relocalization from the wrong position
                }
                lastVisitedBin = i;
            }

            // Wait for camera to stabilize after robot stops moving (avoids motion blur)
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            /*
            // Spin to refresh TF before AprilTag lookup
            for (int s = 0; s < 10; ++s) {
                rclcpp::spin_some(node);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            // --- AprilTag scan ---
            RCLCPP_INFO(node->get_logger(), "Bin %d: scanning for AprilTag...", i + 1);
            std::vector<int> visible = tagDetector.getVisibleTags(tagCandidates, 500);
            bool tagFound = !visible.empty();

            if (!tagFound) {
                if (!isLastBin) {
                    RCLCPP_WARN(node->get_logger(),
                                "Bin %d: no AprilTag detected — skipping to next bin", i + 1);
                    continue;
                } else {
                    RCLCPP_WARN(node->get_logger(),
                                "Last bin: no AprilTag detected — executing dropoff anyway");
                    dropoff(armController, node);
                }
            }

            // --- Reorient 0.4m from tag using map-frame AprilTag pose ---
            int tagId = visible[0];
            RCLCPP_INFO(node->get_logger(), "Bin %d: AprilTag %d found — repositioning 0.4m away", i + 1, tagId);

            // Query tag directly in map frame so the AprilTag pose is authoritative,
            // not derived from AMCL + base_link (avoids costmap/AMCL drift compounding)
            tagDetector.setReferenceFrame("map");
            auto tagPose = tagDetector.getTagPose(tagId, 500);
            tagDetector.setReferenceFrame("base_link");  // restore default

            if (tagPose.has_value()) {
                double tx = tagPose->position.x;
                double ty = tagPose->position.y;

                // Direction from tag back toward robot (use current AMCL pose as approach vector)
                rclcpp::spin_some(node);
                double dx = robotPose.x - tx;
                double dy = robotPose.y - ty;
                double dist = std::sqrt(dx * dx + dy * dy);
                double approach_angle = std::atan2(dy, dx);

                // Stand 0.4m from the tag along the approach vector
                double map_x   = tx + 0.4 * std::cos(approach_angle);
                double map_y   = ty + 0.4 * std::sin(approach_angle);
                double map_phi = approach_angle + M_PI;  // face the tag

                RCLCPP_INFO(node->get_logger(),
                            "Bin %d: tag at map (%.3f, %.3f) — moving to (%.3f, %.3f, %.3f rad)",
                            i + 1, tx, ty, map_x, map_y, map_phi);
                nav.moveToGoal(map_x, map_y, map_phi);
            }
            */

            // --- OAK-D YOLO detection with centering ---
            // OAK-D horizontal FOV ~69°; each unit of bbox_center_norm maps to ~34.5°
            const float CENTER_THRESHOLD = 0.15f;   // within 15% of centre is good enough
            const double HFOV_RAD = 69.0 * M_PI / 180.0;
            const int MAX_CENTER_ATTEMPTS = 3;

            // --- Fan scan: if object is outside the frame, sweep outward in 10° steps
            // up to ±30° (alternating left/right) until a detection is found.
            // This updates goal_phi so the centering loop below starts from the found heading.
            RCLCPP_INFO(node->get_logger(), "Bin %d: initial YOLO detection...", i + 1);
            if (!yolo.captureAndDetect(CameraSource::OAKD, false)) {
                RCLCPP_WARN(node->get_logger(),
                            "Bin %d: no detection at standoff heading — starting fan scan", i + 1);
                const double SCAN_STEP_RAD  = 10.0 * M_PI / 180.0;
                const int    SCAN_STEPS     = 3;   // ±10°, ±20°, ±30°
                bool scan_found = false;
                rclcpp::spin_some(node);
                double base_phi = robotPose.phi;

                for (int step = 1; step <= SCAN_STEPS && !scan_found; ++step) {
                    // Try left (+step), then right (-step)
                    for (int dir : {1, -1}) {
                        double scan_phi = base_phi + dir * step * SCAN_STEP_RAD;
                        RCLCPP_INFO(node->get_logger(),
                                    "Bin %d: fan scan step %d dir %+d — rotating to %.1f deg",
                                    i + 1, step, dir, scan_phi * 180.0 / M_PI);
                        setNavTolerances(node, 1.0, 2.0 * M_PI / 180.0);  // loose xy, tight yaw
                        nav.moveToGoalWithTimeout(robotPose.x, robotPose.y, scan_phi, 10);
                        setNavTolerances(node, 0.50, 6.0 * M_PI / 180.0);  // restore

                        if (yolo.captureAndDetect(CameraSource::OAKD, false)) {
                            RCLCPP_INFO(node->get_logger(),
                                        "Bin %d: fan scan found %s at step %d dir %+d",
                                        i + 1, yolo.getLatestClassName().c_str(), step, dir);
                            rclcpp::spin_some(node);
                            goal_phi = robotPose.phi;  // update heading for centering loop
                            scan_found = true;
                            break;
                        }
                    }
                }

                if (!scan_found) {
                    // Rotate back to original standoff heading before moving on
                    setNavTolerances(node, 1.0, 2.0 * M_PI / 180.0);
                    nav.moveToGoalWithTimeout(robotPose.x, robotPose.y, base_phi, 10);
                    setNavTolerances(node, 0.50, 6.0 * M_PI / 180.0);
                    RCLCPP_WARN(node->get_logger(),
                                "Bin %d: fan scan exhausted — no object found", i + 1);
                }
            }

            RCLCPP_INFO(node->get_logger(), "Bin %d: running OAK-D YOLO centering...", i + 1);
            for (int attempt = 0; attempt < MAX_CENTER_ATTEMPTS; ++attempt) {
                if (!yolo.captureAndDetect(CameraSource::OAKD, (attempt == MAX_CENTER_ATTEMPTS - 1))) {
                    RCLCPP_WARN(node->get_logger(),
                                "Bin %d attempt %d: YOLO failed — no detection", i + 1, attempt + 1);
                    break;
                }

                binObject[i]   = yolo.getLatestClassName();
                binObjectId[i] = yolo.getClassId();
                float offset   = yolo.getBboxCenterNorm();
                RCLCPP_INFO(node->get_logger(),
                            "Bin %d attempt %d: %s (conf: %.2f, offset: %.2f)",
                            i + 1, attempt + 1, binObject[i].c_str(), yolo.getConfidence(), offset);
                log_file << "[Bin " << (i + 1) << " attempt " << (attempt + 1) << "] "
                         << binObject[i] << " | ID: " << binObjectId[i]
                         << " | Confidence: " << yolo.getConfidence()
                         << " | Offset: " << offset << "\n";
                log_file.flush();

                // If centred enough, relocalize AMCL using the known bin position and stop adjusting
                if (std::abs(offset) <= CENTER_THRESHOLD) {
                    RCLCPP_INFO(node->get_logger(), "Bin %d: centred (offset %.2f) — relocalizing AMCL", i + 1, offset);

                    // Robot is now at the standoff position facing the bin — this is a known map-frame pose
                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, goal_phi);

                    geometry_msgs::msg::PoseWithCovarianceStamped initialpose;
                    initialpose.header.frame_id = "map";
                    initialpose.header.stamp = node->get_clock()->now();
                    initialpose.pose.pose.position.x = goal_x;
                    initialpose.pose.pose.position.y = goal_y;
                    initialpose.pose.pose.orientation.x = q.x();
                    initialpose.pose.pose.orientation.y = q.y();
                    initialpose.pose.pose.orientation.z = q.z();
                    initialpose.pose.pose.orientation.w = q.w();
                    // Tight covariance — we trust this estimate
                    initialpose.pose.covariance[0]  = 0.02;  // x variance
                    initialpose.pose.covariance[7]  = 0.02;  // y variance
                    initialpose.pose.covariance[35] = 0.02;  // yaw variance
                    initialpose_pub->publish(initialpose);

                    RCLCPP_INFO(node->get_logger(),
                                "Bin %d: AMCL snapped to (%.3f, %.3f, %.3f rad)",
                                i + 1, goal_x, goal_y, goal_phi);
                    break;
                }

                // Rotate in place to centre the detected object
                double correction = -static_cast<double>(offset) * (HFOV_RAD / 2.0);
                rclcpp::spin_some(node);
                double corrected_phi = robotPose.phi + correction;
                RCLCPP_INFO(node->get_logger(),
                            "Bin %d: offset %.2f — rotating %.1f deg to centre",
                            i + 1, offset, correction * 180.0 / M_PI);
                setNavTolerances(node, 1.0, 2.0 * M_PI / 180.0);  // loose xy, tight yaw
                nav.moveToGoalWithTimeout(robotPose.x, robotPose.y, corrected_phi, 15);
                setNavTolerances(node, 0.50, 6.0 * M_PI / 180.0);  // restore
            }

            // If this bin matches the carried object, drop off immediately
            if (!binObject[i].empty() && binObject[i] == detectedObject) {
                RCLCPP_INFO(node->get_logger(),
                            "Bin %d matches carried object (%s) — executing dropoff!",
                            i + 1, detectedObject.c_str());
                dropoff(armController, node);
                droppedOff = true;
            } else if (isLastBin) {
                // Reached bin 5 with no match found — drop off as fallback
                RCLCPP_WARN(node->get_logger(),
                            "Last bin: no matching bin found for %s — executing dropoff as fallback",
                            detectedObject.c_str());
                dropoff(armController, node);
                droppedOff = true;
            }

            // Update elapsed time
            auto tNow = std::chrono::system_clock::now();
            secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                                 tNow - start).count();
            if (secondsElapsed > 280) {
                RCLCPP_WARN(node->get_logger(), "Time running low — stopping early");
                break;
            }
        }

        // --- Fallback dropoff if loop ended without a match (e.g. last bin was time-skipped) ---
        if (!droppedOff && lastVisitedBin >= 0) {
            RCLCPP_WARN(node->get_logger(),
                        "No matching bin found for %s — dropping off at last visited bin (%d) as fallback",
                        detectedObject.c_str(), lastVisitedBin + 1);
            dropoff(armController, node);
        }

        // --- Summary of all bin detections ---
        log_file << "\n=== Final Summary ===\n";
        log_file << "Carried object (wrist): " << detectedObject << " (ID: " << objectId << ")\n";
        for (int i = 0; i < NUM_WP; ++i) {
            RCLCPP_INFO(node->get_logger(),
                        "Bin %d: %s (ID: %d)", i + 1, binObject[i].c_str(), binObjectId[i]);
            std::string bin_str = binObject[i].empty() ? "none" : binObject[i];
            log_file << "Bin " << (i + 1) << ": " << bin_str
                     << " (ID: " << binObjectId[i] << ")";
            if (!binObject[i].empty() && binObject[i] == detectedObject) {
                log_file << "  <-- MATCH (dropoff here)";
            }
            log_file << "\n";
        }
        log_file.flush();

        // --- Return to home ---
        RCLCPP_INFO(node->get_logger(), "All waypoints visited. Returning to home...");
        navigateWithRecovery(nav, node, homeX, homeY, homePhi);
        RCLCPP_INFO(node->get_logger(), "Returned to home position.");

        break;  // Contest sequence complete — exit while loop

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (secondsElapsed > 300) {
        RCLCPP_WARN(node->get_logger(), "Contest time limit reached!");
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node shutting down");
    rclcpp::shutdown();
    return 0;
}

void dropoff(ArmController& armController, rclcpp::Node::SharedPtr node) {
    // --- YOUR CODE HERE: implement box dropoff sequence ---
    // You can create additional functions in arm_controller.cpp and call them here
    // to keep your code organized. For example, you might have a function to move
    // to the dropoff location, and another function to open the gripper and release the box.
    bool success = armController.moveToCartesianPose(0.165, 0.003, 0.181,
                                                     0.314, 0.002, 1.757);
    if(success) {
        RCLCPP_INFO(node->get_logger(), "Arm moved successfully!");
        //std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
        RCLCPP_ERROR(node->get_logger(), "Arm movement failed Object detection pose is unreachable");
    }
    // TEST arm movement with dropoff pose 1
    success = armController.moveToCartesianPose(0.041, -0.323, 0.114,
                                                -0.176, -0.027, -0.080, 0.981);
    
    if(success) {  
        RCLCPP_INFO(node->get_logger(), "Arm moved successfully!");
        armController.openGripper(); //OPEN GRIPPER
        //std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
        RCLCPP_ERROR(node->get_logger(), "Arm movement failed - pose1 may be unreachable");

    }

    // TEST arm movement with returning position
    success = armController.moveToCartesianPose(0.025, -0.064, 0.223,
                                                     -0.389, -0.024, 0.042, 0.920);
    if(success) {
    RCLCPP_INFO(node->get_logger(), "Arm moved successfully!");
    armController.closeGripper(); //CLOSE GRIPPER
        //std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
        RCLCPP_ERROR(node->get_logger(), "Arm movement failed Object detection pose is unreachable");
    }
}