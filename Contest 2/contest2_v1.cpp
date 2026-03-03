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
#include <map>
#include <cmath>

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

        // =====================================================================
        // CONTEST 2 STRATEGY
        //
        // All heavy objects are static so they are constructed only once
        // on the first loop iteration.  Subsequent iterations just spin.
        //
        // Phase 0 — Pick-up:  detect the manipulable object on the robot's
        //                     top plate (wrist camera), then grab it
        // Phase 1 — Survey:   drive to all 5 scene object waypoints, identify
        //                     each with YOLO, find which matches the manipulable
        // Phase 2 — Deposit:  navigate to the matching bin and release
        // Phase 3 — Return:   drive back to the starting position
        // Output  — File:     write detection results to disk for scoring
        // =====================================================================

        // --- One-time initialization -------------------------------------------
        static std::shared_ptr<Navigation>       nav;
        static std::shared_ptr<YoloInterface>    yolo;
        static std::shared_ptr<ArmController>    arm;
        static std::shared_ptr<AprilTagDetector> aprilTag;
        static std::vector<std::string>          boxDetections;
        static bool                              contestDone = false;

        // Starting pose — filled in after AMCL publishes
        static double startX   = 0.0;
        static double startY   = 0.0;
        static double startPhi = 0.0;

        // Manipulable object (placed on robot by instructors before contest)
        static std::string manipulableName = "";
        static float       manipulableConf = 0.0f;

        // Index into boxes.coords of the scene object that matches the manipulable
        // (-1 = not yet found)
        static int matchedBoxIndex = -1;

        // *** TUNE: AprilTag bin ID for each scene object waypoint.
        //     Index i of this vector corresponds to boxes.coords[i].
        //     Set each value to the actual tag ID on the bin in front of scene object i. ***
        static const std::vector<int> binTagIds = {0, 1, 2, 3, 4};

        if (!nav) {
            RCLCPP_INFO(node->get_logger(), "Initializing contest components...");
            nav      = std::make_shared<Navigation>(node);
            yolo     = std::make_shared<YoloInterface>(node);
            arm      = std::make_shared<ArmController>(node);
            aprilTag = std::make_shared<AprilTagDetector>(node, "tag36h11:", "map");
            boxDetections.assign(boxes.coords.size(), "");

            // Wait up to 5 s for AMCL to publish the initial robot pose
            RCLCPP_INFO(node->get_logger(), "Waiting for AMCL pose...");
            for (int i = 0; i < 50; ++i) {
                rclcpp::spin_some(node);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            startX   = robotPose.x;
            startY   = robotPose.y;
            startPhi = robotPose.phi;
            RCLCPP_INFO(node->get_logger(),
                        "Start pose recorded: (%.2f, %.2f, %.2f)", startX, startY, startPhi);
            RCLCPP_INFO(node->get_logger(), "All components ready.");
        }

        // --- Main strategy (runs only once) ------------------------------------
        if (!contestDone) {
            contestDone = true;

            // ==================================================================
            // PHASE 0: Pick-up — detect the manipulable object sitting on the
            //          robot's top plate using the wrist camera, then grab it.
            //          save_image=true saves the annotated image required for scoring.
            // ==================================================================
            RCLCPP_INFO(node->get_logger(),
                        "=== PHASE 0: Detecting manipulable object on top plate ===");

            for (int attempt = 0; attempt < 3 && manipulableName.empty(); ++attempt) {
                RCLCPP_INFO(node->get_logger(), "Detection attempt %d/3...", attempt + 1);
                std::string name = yolo->getObjectName(CameraSource::WRIST, /*save_image=*/true);
                if (!name.empty()) {
                    manipulableName = name;
                    manipulableConf = yolo->getConfidence();
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    rclcpp::spin_some(node);
                }
                yolo->clearDetection();
            }

            if (!manipulableName.empty()) {
                RCLCPP_INFO(node->get_logger(),
                            "Manipulable object: '%s' (conf=%.2f)",
                            manipulableName.c_str(), manipulableConf);

                // Pick up from top plate
                arm->openGripper();
                // *** TUNE: adjust (x, y, z, roll, pitch, yaw) so the gripper
                //     lines up with the object on the robot's top plate.
                //     x/y/z in metres relative to arm_mount; angles in radians. ***
                if (arm->moveToCartesianPose(0.15, 0.0, 0.02, 0.0, M_PI / 2.0, 0.0)) {
                    arm->closeGripper();
                    // *** TUNE: adjust lift pose so the object clears the top plate. ***
                    arm->moveToCartesianPose(0.10, 0.0, 0.15, 0.0, M_PI / 2.0, 0.0);
                    RCLCPP_INFO(node->get_logger(), "Manipulable object picked up ✓");
                } else {
                    RCLCPP_WARN(node->get_logger(), "Arm pick failed — attempting grasp anyway");
                    arm->closeGripper();
                }
            } else {
                RCLCPP_WARN(node->get_logger(), "Could not detect manipulable object on top plate!");
            }

            // ==================================================================
            // PHASE 1: Survey — navigate to each of the 5 scene object locations,
            //          identify it with YOLO, check if it matches the manipulable.
            // ==================================================================
            RCLCPP_INFO(node->get_logger(), "=== PHASE 1: Surveying all scene objects ===");

            for (size_t i = 0; i < boxes.coords.size(); ++i) {
                // Abort early if fewer than 60 s remain
                auto tNow = std::chrono::system_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(tNow - start).count() > 240) {
                    RCLCPP_WARN(node->get_logger(), "Time limit approaching — aborting survey early");
                    break;
                }

                RCLCPP_INFO(node->get_logger(),
                            "--- Scene object %zu: navigating to (%.2f, %.2f, %.2f) ---",
                            i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);

                if (!nav->moveToGoal(boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2])) {
                    RCLCPP_WARN(node->get_logger(), "Scene object %zu: navigation failed", i);
                    continue;
                }

                // Let robot settle and process pending callbacks
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                rclcpp::spin_some(node);

                // Primary detection: OAK-D forward-facing camera
                RCLCPP_INFO(node->get_logger(), "Scene object %zu: detecting with OAK-D...", i);
                std::string detected = yolo->getObjectName(CameraSource::OAKD, /*save_image=*/false);

                if (detected.empty()) {
                    RCLCPP_WARN(node->get_logger(),
                                "Scene object %zu: OAK-D found nothing — retrying with wrist camera", i);
                    detected = yolo->getObjectName(CameraSource::WRIST, /*save_image=*/false);
                }

                if (!detected.empty()) {
                    boxDetections[i] = detected;
                    RCLCPP_INFO(node->get_logger(),
                                "Scene object %zu: DETECTED '%s' (conf=%.2f)",
                                i, detected.c_str(), yolo->getConfidence());

                    // Check for match with the manipulable object
                    if (matchedBoxIndex < 0 && detected == manipulableName) {
                        matchedBoxIndex = static_cast<int>(i);
                        RCLCPP_INFO(node->get_logger(),
                                    "*** MATCH: scene object %zu ('%s') = manipulable — target bin! ***",
                                    i, detected.c_str());
                    }
                } else {
                    RCLCPP_WARN(node->get_logger(), "Scene object %zu: no detection", i);
                }

                yolo->clearDetection();
            }

            // Survey summary
            RCLCPP_INFO(node->get_logger(), "=== Survey summary ===");
            RCLCPP_INFO(node->get_logger(), "Manipulable: '%s' (conf=%.2f)",
                        manipulableName.empty() ? "(none)" : manipulableName.c_str(),
                        manipulableConf);
            for (size_t i = 0; i < boxDetections.size(); ++i) {
                RCLCPP_INFO(node->get_logger(), "  Scene obj %zu @ (%.2f, %.2f): %s",
                            i, boxes.coords[i][0], boxes.coords[i][1],
                            boxDetections[i].empty() ? "(none)" : boxDetections[i].c_str());
            }

            // ==================================================================
            // PHASE 2: Deposit — navigate to the matching bin and release the
            //          manipulable object inside it.
            // ==================================================================
            RCLCPP_INFO(node->get_logger(), "=== PHASE 2: Depositing manipulable object ===");

            if (matchedBoxIndex >= 0) {
                int binTagId = binTagIds[static_cast<size_t>(matchedBoxIndex)];
                RCLCPP_INFO(node->get_logger(),
                            "Depositing '%s' in bin (tag %d) at scene object %d",
                            manipulableName.c_str(), binTagId, matchedBoxIndex);

                // Navigate back to the matched scene object location
                if (nav->moveToGoal(boxes.coords[matchedBoxIndex][0],
                                    boxes.coords[matchedBoxIndex][1],
                                    boxes.coords[matchedBoxIndex][2])) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(300));
                    rclcpp::spin_some(node);

                    // Locate the garbage bin with its AprilTag
                    auto tagPose = aprilTag->getTagPose(binTagId);
                    if (tagPose.has_value()) {
                        // *** TUNE: 0.5 m standoff keeps the robot from colliding with the bin.
                        //     Increase if it bumps the bin; decrease if the arm can't reach. ***
                        if (nav->moveToGoal(tagPose->position.x - 0.5,
                                            tagPose->position.y, 0.0)) {
                            std::this_thread::sleep_for(std::chrono::milliseconds(300));
                            rclcpp::spin_some(node);

                            // *** TUNE: extend arm over the bin opening before releasing. ***
                            arm->moveToCartesianPose(0.15, 0.0, 0.10, 0.0, M_PI / 2.0, 0.0);
                            arm->openGripper();
                            // *** TUNE: retract to safe pose that won't snag on the bin. ***
                            arm->moveToCartesianPose(0.05, 0.0, 0.20, 0.0, 0.0, 0.0);
                            RCLCPP_INFO(node->get_logger(),
                                        "'%s' deposited in bin %d ✓",
                                        manipulableName.c_str(), binTagId);
                        } else {
                            RCLCPP_WARN(node->get_logger(),
                                        "Could not reach bin %d — releasing in place", binTagId);
                            arm->openGripper();
                        }
                    } else {
                        RCLCPP_WARN(node->get_logger(),
                                    "Bin tag %d not visible — releasing in place", binTagId);
                        arm->openGripper();
                    }
                } else {
                    RCLCPP_WARN(node->get_logger(),
                                "Could not return to matched scene object — releasing");
                    arm->openGripper();
                }
            } else {
                RCLCPP_WARN(node->get_logger(),
                            "No scene object matched the manipulable — releasing");
                arm->openGripper();
            }

            // ==================================================================
            // PHASE 3: Return to starting position (worth 1 mark)
            // ==================================================================
            RCLCPP_INFO(node->get_logger(),
                        "=== PHASE 3: Returning to starting position (%.2f, %.2f, %.2f) ===",
                        startX, startY, startPhi);
            if (nav->moveToGoal(startX, startY, startPhi)) {
                RCLCPP_INFO(node->get_logger(), "Returned to starting position ✓");
            } else {
                RCLCPP_WARN(node->get_logger(), "Failed to return to starting position");
            }

            // ==================================================================
            // OUTPUT: Write results file required for scoring (Section 1.2).
            //   Contents: 1) manipulable object name + confidence score
            //             2) all detected scene objects + their locations
            // The annotated wrist-camera image is saved by yolo_detector.py
            // when save_image=true is passed during Phase 0.
            // ==================================================================
            RCLCPP_INFO(node->get_logger(), "=== Writing scoring output file ===");
            {
                std::ofstream outFile("/home/admin1/contest2_results.txt");
                if (outFile.is_open()) {
                    outFile << "MIE443 Contest 2 — Detection Results\n";
                    outFile << "======================================\n\n";
                    outFile << "Manipulable Object:\n";
                    outFile << "  name:       "
                            << (manipulableName.empty() ? "(not detected)" : manipulableName)
                            << "\n";
                    outFile << "  confidence: " << manipulableConf << "\n\n";
                    outFile << "Scene Objects Detected:\n";
                    for (size_t i = 0; i < boxDetections.size(); ++i) {
                        outFile << "  [" << i << "]"
                                << "  location=(" << boxes.coords[i][0]
                                << ", "           << boxes.coords[i][1] << ")"
                                << "  object="
                                << (boxDetections[i].empty() ? "(none)" : boxDetections[i])
                                << "\n";
                    }
                    outFile << "\nMatched scene object index: " << matchedBoxIndex << "\n";
                    outFile.close();
                    RCLCPP_INFO(node->get_logger(),
                                "Results written to /home/admin1/contest2_results.txt");
                } else {
                    RCLCPP_ERROR(node->get_logger(), "Failed to open results file for writing!");
                }
            }

            RCLCPP_INFO(node->get_logger(), "=== Contest 2 complete! ===");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (secondsElapsed > 300) {
        RCLCPP_WARN(node->get_logger(), "Contest time limit reached!");
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node shutting down");
    rclcpp::shutdown();
    return 0;
}
