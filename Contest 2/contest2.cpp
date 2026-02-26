#include "mie443_contest2/boxes.h"
#include "mie443_contest2/navigation.h"
#include "mie443_contest2/robot_pose.h"
#include "mie443_contest2/yoloInterface.h"
#include "mie443_contest2/arm_controller.h"
#include "mie443_contest2/apriltag_detector.h"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <fstream>
#include <cmath>
#include <limits>
#include <vector>

// Define states for the controller
enum RobotState {
    INIT,
    PICKUP_OBJECT,
    NAVIGATE_AND_DETECT,
    PLACE_OBJECT,
    RETURN_HOME,
    FINISHED
};

// Helper function to calculate Euclidean distance
double get_dist(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("contest2");

    // Initialize Subscribers and Helpers
    RobotPose robotPose(0, 0, 0);
    auto amclSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10, std::bind(&RobotPose::poseCallback, &robotPose, std::placeholders::_1));

    Boxes boxes;
    if(!boxes.load_coords()) {
        RCLCPP_ERROR(node->get_logger(), "ERROR: could not load box coordinates");
        return -1;
    }

    Navigation nav(node);
    YoloInterface yolo(node);
    ArmController arm(node);
    AprilTagDetector tags(node, "map"); 

    auto start_time = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    RobotState currentState = INIT;
    
    // Data Storage
    double home_x = 0, home_y = 0, home_phi = 0;
    std::string manipulable_obj_name = "";
    std::vector<std::vector<float>> sorted_coords;
    std::vector<std::string> results_log;
    size_t current_box_index = 0;
    bool object_placed = false;

    while(rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();

        switch(currentState) {
            
            case INIT: {
                RCLCPP_INFO(node->get_logger(), "Initializing: Optimizing path using Nearest Neighbor...");
                home_x = robotPose.x;
                home_y = robotPose.y;
                home_phi = robotPose.phi;

                // --- Nearest Neighbor Algorithm ---
                std::vector<std::vector<float>> remaining = boxes.coords;
                double cur_x = home_x;
                double cur_y = home_y;

                while (!remaining.empty()) {
                    int best_idx = 0;
                    double min_dist = std::numeric_limits<double>::max();
                    
                    for (size_t i = 0; i < remaining.size(); ++i) {
                        double d = get_dist(cur_x, cur_y, remaining[i][0], remaining[i][1]);
                        if (d < min_dist) {
                            min_dist = d;
                            best_idx = i;
                        }
                    }
                    sorted_coords.push_back(remaining[best_idx]);
                    cur_x = remaining[best_idx][0];
                    cur_y = remaining[best_idx][1];
                    remaining.erase(remaining.begin() + best_idx);
                }
                
                RCLCPP_INFO(node->get_logger(), "Path optimized. Moving to pick up object.");
                currentState = PICKUP_OBJECT;
                break;
            }

            case PICKUP_OBJECT: {
                // Detect using Wrist Camera; 'true' saves the annotated image for marks
                manipulable_obj_name = yolo.getObjectName(CameraSource::WRIST);
                
                if (!manipulable_obj_name.empty()) {
                    RCLCPP_INFO(node->get_logger(), "Target identified: %s. Executing arm pick...", manipulable_obj_name.c_str());
                    
                    // Physical arm movements (Replace these placeholder XYZ with your tested values)
                    arm.moveGripper(1.0); // Open
                    arm.moveToCartesianPose(0.18, 0.0, 0.05, 0.0, 1.57, 0.0); // Reach
                    arm.moveGripper(0.0); // Close
                    arm.moveToCartesianPose(0.15, 0.0, 0.25, 0.0, 1.57, 0.0); // Lift
                    
                    currentState = NAVIGATE_AND_DETECT;
                } else {
                    RCLCPP_WARN(node->get_logger(), "Manipulable object not found on plate. Retrying...");
                    rclcpp::sleep_for(std::chrono::seconds(1));
                }
                break;
            }

            case NAVIGATE_AND_DETECT: {
                if (current_box_index < sorted_coords.size()) {
                    double tx = sorted_coords[current_box_index][0];
                    double ty = sorted_coords[current_box_index][1];
                    double tp = sorted_coords[current_box_index][2];

                    RCLCPP_INFO(node->get_logger(), "Navigating to Location %zu...", current_box_index + 1);
                    if(nav.moveToGoal(tx, ty, tp)) {
                        // Scan with OAK-D
                        std::string scene_obj = yolo.getObjectName(CameraSource::OAKD);
                        if (scene_obj.empty()) scene_obj = "nothing";

                        results_log.push_back("Box at (" + std::to_string(tx) + ", " + std::to_string(ty) + "): " + scene_obj);

                        // If match found and object is still in hand
                        if (scene_obj == manipulable_obj_name && !object_placed) {
                            currentState = PLACE_OBJECT;
                        } else {
                            current_box_index++;
                        }
                    }
                } else {
                    currentState = RETURN_HOME;
                }
                break;
            }

            case PLACE_OBJECT: {
                RCLCPP_INFO(node->get_logger(), "Match found. Locating bin...");
                std::vector<int> visible = tags.getVisibleTags({0,1,2,3,4,5}, 1000);
                
                if (!visible.empty()) {
                    // Placeholder placement sequence
                    arm.moveToCartesianPose(0.20, 0.0, 0.10, 0.0, 0.0, 0.0); 
                    arm.moveGripper(1.0); // Release
                    arm.moveToCartesianPose(0.10, 0.0, 0.25, 0.0, 1.57, 0.0); // Retract
                    
                    object_placed = true;
                    current_box_index++;
                    currentState = NAVIGATE_AND_DETECT;
                } else {
                    RCLCPP_WARN(node->get_logger(), "Bin tag not seen. Adjusting position...");
                    // Optional: Small rotation if tag is missed
                    current_box_index++; // For now, skip to avoid getting stuck
                    currentState = NAVIGATE_AND_DETECT;
                }
                break;
            }

            case RETURN_HOME: {
                RCLCPP_INFO(node->get_logger(), "Mission successful. Returning to start.");
                nav.moveToGoal(home_x, home_y, home_phi);

                // FINAL REQUIREMENT: Output results to text file
                std::ofstream outfile("contest2_report.txt");
                outfile << "=== MIE443 Contest 2 Results ===\n";
                outfile << "Manipulable Object: " << manipulable_obj_name << "\n\n";
                outfile << "Detected Scene Objects:\n";
                for (const auto& line : results_log) outfile << "- " << line << "\n";
                outfile.close();

                currentState = FINISHED;
                break;
            }

            case FINISHED:
                RCLCPP_INFO_ONCE(node->get_logger(), "Contest tasks complete. Standing by.");
                break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    rclcpp::shutdown();
    return 0;
}