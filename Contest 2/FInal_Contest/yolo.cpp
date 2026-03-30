#include "mie443_contest2/boxes.h"
#include "mie443_contest2/navigation.h"
#include "mie443_contest2/robot_pose.h"
#include "mie443_contest2/yoloInterface.h"
#include "mie443_contest2/arm_controller.h"
#include "mie443_contest2/apriltag_detector.h"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>

void dropoff(ArmController& armController, rclcpp::Node::SharedPtr node);

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
    auto amclSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
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


    // Test YOLO detection
    std::string detectedObject;
    int objectId;
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
    

    // Contest countdown timer
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    // Execute strategy
    while(rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);

        // Calculate elapsed time
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

        /***YOUR CODE HERE***/

        // --- Wait briefly for AMCL to publish an initial pose ---
        for (int w = 0; w < 50 && rclcpp::ok(); ++w) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // --- Store home position ---
        double homeX   = robotPose.x;
        double homeY   = robotPose.y;
        double homePhi = robotPose.phi;
        RCLCPP_INFO(node->get_logger(),
                    "Home stored: x=%.3f  y=%.3f  phi=%.3f rad",
                    homeX, homeY, homePhi);

        // --- Initialize navigation ---
        Navigation nav(node);

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

        // --- Sequential path planning ---
        for (int i = 0; i < NUM_WP && rclcpp::ok(); ++i) {
            RCLCPP_INFO(node->get_logger(),
                        "Step %d/%d — waypoint %d (x=%.3f, y=%.3f, phi=%.3f rad)",
                        i + 1, NUM_WP, i + 1,
                        wp[i][0], wp[i][1], wp[i][2]);

            nav.moveToGoal(wp[i][0], wp[i][1], wp[i][2]);

            RCLCPP_INFO(node->get_logger(),
                        "Arrived at waypoint %d. Waiting 5 seconds...", i + 1);
            std::this_thread::sleep_for(std::chrono::seconds(5));

            // Update elapsed time
            auto tNow = std::chrono::system_clock::now();
            secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                                 tNow - start).count();
            if (secondsElapsed > 280) {
                RCLCPP_WARN(node->get_logger(), "Time running low — stopping early");
                break;
            }
        }

        // --- Return to home ---
        RCLCPP_INFO(node->get_logger(), "All waypoints visited. Returning to home...");
        nav.moveToGoal(homeX, homeY, homePhi);
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
    success = armController.moveToCartesianPose(0.042, -0.216, 0.274,
                                                -0.201, -0.012, 0.050, 0.978);
    
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