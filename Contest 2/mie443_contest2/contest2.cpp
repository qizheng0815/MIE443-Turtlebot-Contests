#include "mie443_contest2/boxes.h"
#include "mie443_contest2/navigation.h"
#include "mie443_contest2/robot_pose.h"
#include "mie443_contest2/yoloInterface.h"
#include "mie443_contest2/arm_controller.h"
#include "mie443_contest2/apriltag_detector.h"

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

// ─── Arm pose constants (arm_mount frame: x=forward, y=left, z=up) ─────────────
// RPY (0, 1.57, 0) = gripper pointing straight down
constexpr double CARRY_X = 0.15, CARRY_Y = 0.0, CARRY_Z = 0.25;  // holding object during nav
constexpr double STOW_X  = 0.12, STOW_Y  = 0.0, STOW_Z  = 0.18;  // compact, no object

// ─── State machine ────────────────────────────────────────────────────────────
enum RobotState {
    INIT,
    PICKUP_OBJECT,
    NAVIGATE_AND_DETECT,
    PLACE_OBJECT,
    RETURN_HOME,
    FINISHED
};

// ─── Helpers ──────────────────────────────────────────────────────────────────

double get_dist(double x1, double y1, double x2, double y2) {
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

/**
 * Spin in place to target_phi at (gx, gy) via Nav2, then settle for 1 s.
 */
void spinToAngle(Navigation &nav, double gx, double gy, double target_phi,
                 std::shared_ptr<rclcpp::Node> node)
{
    nav.moveToGoal(gx, gy, target_phi);
    for (int i = 0; i < 10; ++i) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/**
 * Run YOLO detection up to max_attempts times.
 * Returns the detection with the highest confidence above threshold.
 * If save_image=true, saves the annotated image on the first successful attempt.
 * Returns {class_name, confidence}; name is "" if nothing detected.
 */
std::pair<std::string, float> yoloRetry(
    YoloInterface &yolo,
    CameraSource cam,
    bool save_image,
    int max_attempts,
    std::shared_ptr<rclcpp::Node> node)
{
    std::string best_name = "";
    float best_conf = 0.0f;
    bool image_saved = false;

    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        bool do_save = save_image && !image_saved;
        std::string name = yolo.getObjectName(cam, do_save);
        if (!name.empty()) {
            float conf = yolo.getConfidence();
            if (do_save) image_saved = true;
            if (conf > best_conf) {
                best_conf = conf;
                best_name = name;
            }
            if (conf >= 0.70f) break;  // Confident enough — stop early
        }
        if (attempt < max_attempts - 1) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(400));
        }
    }
    return {best_name, best_conf};
}

/**
 * Try YOLO with OAK-D at the current heading (base_attempts), then if no
 * confident detection sweep ±SWEEP_RAD and retry.  Returns the best detection.
 * Always returns the robot to the original heading gp before returning.
 */
std::pair<std::string, float> yoloWithSweep(
    YoloInterface &yolo,
    Navigation &nav,
    double gx, double gy, double gp,
    int base_attempts,
    std::shared_ptr<rclcpp::Node> node)
{
    constexpr double SWEEP_RAD = 0.26;  // ≈ 15°

    // ── Primary detection at goal heading ────────────────────────────────────
    auto det = yoloRetry(yolo, CameraSource::OAKD, false, base_attempts, node);
    if (!det.first.empty() && det.second >= 0.65f) return det;

    // ── Left sweep (−15°) ────────────────────────────────────────────────────
    spinToAngle(nav, gx, gy, gp - SWEEP_RAD, node);
    auto det_left = yoloRetry(yolo, CameraSource::OAKD, false, 3, node);
    if (!det_left.first.empty() && det_left.second > det.second) det = det_left;

    if (det.second >= 0.65f) {
        spinToAngle(nav, gx, gy, gp, node);
        return det;
    }

    // ── Right sweep (+15°) ───────────────────────────────────────────────────
    spinToAngle(nav, gx, gy, gp + SWEEP_RAD, node);
    auto det_right = yoloRetry(yolo, CameraSource::OAKD, false, 3, node);
    if (!det_right.first.empty() && det_right.second > det.second) det = det_right;

    // Return to original heading for subsequent AprilTag detection
    spinToAngle(nav, gx, gy, gp, node);
    return det;
}

// Write the results file (called at RETURN_HOME and on timeout)
void writeResults(
    const std::string &manip_name,
    float manip_conf,
    const std::vector<std::vector<float>> &sorted_coords,
    const std::vector<std::string> &scene_results,
    bool placed,
    bool timed_out)
{
    std::ofstream f("/home/turtlebot/contest2_report.txt");
    if (timed_out) {
        f << "=== MIE443 Contest 2 Results (INCOMPLETE - TIMEOUT) ===\n\n";
    } else {
        f << "=== MIE443 Contest 2 Results ===\n\n";
    }

    f << "Manipulable Object Detected:\n";
    f << "  Name:       " << (manip_name.empty() ? "none" : manip_name) << "\n";
    f << "  Confidence: " << manip_conf << "\n\n";

    f << "Detected Scene Objects:\n";
    for (size_t i = 0; i < sorted_coords.size(); ++i) {
        std::string result = (i < scene_results.size()) ? scene_results[i] : "not_visited";
        f << "  Box " << (i + 1)
          << " (x=" << sorted_coords[i][0]
          << ", y=" << sorted_coords[i][1] << "): "
          << result << "\n";
    }

    f << "\nManipulable Object Placed in Bin: " << (placed ? "YES" : "NO") << "\n";
    f.close();
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("contest2");

    // ── Pose subscriber ──────────────────────────────────────────────────────
    RobotPose robotPose(0, 0, 0);
    auto amclSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        std::bind(&RobotPose::poseCallback, &robotPose, std::placeholders::_1));

    // Wait up to 4 s for first AMCL pose
    RCLCPP_INFO(node->get_logger(), "Waiting for AMCL pose...");
    for (int i = 0; i < 20; ++i) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (robotPose.x != 0.0 || robotPose.y != 0.0) break;
    }

    // ── Box coordinates ──────────────────────────────────────────────────────
    Boxes boxes;
    if (!boxes.load_coords()) {
        RCLCPP_ERROR(node->get_logger(), "ERROR: could not load box coordinates");
        return -1;
    }
    for (size_t i = 0; i < boxes.coords.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "Box %zu: x=%.2f  y=%.2f  phi=%.2f",
                    i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
    }

    // ── Components ───────────────────────────────────────────────────────────
    Navigation nav(node);
    YoloInterface yolo(node);
    ArmController arm(node);

    // AprilTag frames published by apriltag_ros: tag_0, tag_1, ..., tag_4
    // Reference frame: base_link (tag pose relative to robot body)
    AprilTagDetector tags(node, "tag_", "base_link");

    // ── Contest timer ────────────────────────────────────────────────────────
    auto start_time = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    constexpr uint64_t TIME_LIMIT = 300;  // 5 minutes

    RCLCPP_INFO(node->get_logger(), "Starting contest — 300-second timer begins now!");

    // ── State variables ──────────────────────────────────────────────────────
    RobotState state = INIT;
    double home_x = 0.0, home_y = 0.0, home_phi = 0.0;
    std::string manip_name = "";
    float manip_conf = 0.0f;
    std::vector<std::vector<float>> sorted_coords;
    std::vector<std::string> scene_results;   // detected name at each location
    size_t box_idx = 0;
    bool placed = false;
    int pickup_tries = 0;

    // ── Main loop ────────────────────────────────────────────────────────────
    while (rclcpp::ok() && secondsElapsed <= TIME_LIMIT) {
        rclcpp::spin_some(node);
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        uint64_t timeLeft = (secondsElapsed < TIME_LIMIT) ? (TIME_LIMIT - secondsElapsed) : 0;

        switch (state) {

        // ═══════════════════════════════════════════════════════════════════════
        case INIT: {
            home_x = robotPose.x;
            home_y = robotPose.y;
            home_phi = robotPose.phi;
            RCLCPP_INFO(node->get_logger(), "INIT: Home = (%.2f, %.2f, %.2f)",
                        home_x, home_y, home_phi);

            // Nearest-Neighbor TSP heuristic for path ordering
            std::vector<std::vector<float>> remaining = boxes.coords;
            double cx = home_x, cy = home_y;
            while (!remaining.empty()) {
                int best = 0;
                double min_d = std::numeric_limits<double>::max();
                for (size_t i = 0; i < remaining.size(); ++i) {
                    double d = get_dist(cx, cy, remaining[i][0], remaining[i][1]);
                    if (d < min_d) { min_d = d; best = static_cast<int>(i); }
                }
                sorted_coords.push_back(remaining[best]);
                cx = remaining[best][0];
                cy = remaining[best][1];
                remaining.erase(remaining.begin() + best);
            }
            scene_results.assign(sorted_coords.size(), "not_visited");

            RCLCPP_INFO(node->get_logger(), "INIT: Path optimised. Proceeding to PICKUP.");
            state = PICKUP_OBJECT;
            break;
        }

        // ═══════════════════════════════════════════════════════════════════════
        case PICKUP_OBJECT: {
            RCLCPP_INFO(node->get_logger(),
                        "PICKUP: Detecting manipulable object with wrist camera (try %d/5)...",
                        pickup_tries + 1);

            auto det = yoloRetry(yolo, CameraSource::WRIST, /*save_image=*/true, 3, node);
            manip_name = det.first;
            manip_conf = det.second;

            if (!manip_name.empty()) {
                RCLCPP_INFO(node->get_logger(),
                            "PICKUP: Detected '%s' (conf=%.2f). Executing pick sequence...",
                            manip_name.c_str(), manip_conf);

                // ── Arm pick sequence ─────────────────────────────────────────
                // NOTE: Cartesian coordinates are in arm_mount frame.
                //   x = forward (away from robot body)
                //   y = lateral
                //   z = up
                //   RPY (0, 1.57, 0) = gripper pointing straight down

                arm.openGripper();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));

                // Pre-grasp: hover above the object on the top plate
                arm.moveToCartesianPose(0.20, 0.0, 0.12, 0.0, 1.57, 0.0);
                std::this_thread::sleep_for(std::chrono::milliseconds(300));

                // Lower to object grasp height
                arm.moveToCartesianPose(0.20, 0.0, 0.04, 0.0, 1.57, 0.0);
                std::this_thread::sleep_for(std::chrono::milliseconds(200));

                // Close gripper to grasp
                arm.closeGripper();
                std::this_thread::sleep_for(std::chrono::milliseconds(700));

                // Lift to safe carry height
                arm.moveToCartesianPose(CARRY_X, CARRY_Y, CARRY_Z, 0.0, 1.57, 0.0);

                RCLCPP_INFO(node->get_logger(), "PICKUP: Complete. Navigating to scene objects.");
                state = NAVIGATE_AND_DETECT;

            } else {
                pickup_tries++;
                if (pickup_tries >= 5) {
                    RCLCPP_ERROR(node->get_logger(),
                                 "PICKUP: No object detected after 5 attempts. "
                                 "Skipping pickup — will still navigate for scene scoring.");
                    manip_name = "";
                    manip_conf = 0.0f;
                    state = NAVIGATE_AND_DETECT;
                } else {
                    RCLCPP_WARN(node->get_logger(), "PICKUP: No detection. Retrying in 1 s...");
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
            break;
        }

        // ═══════════════════════════════════════════════════════════════════════
        case NAVIGATE_AND_DETECT: {
            if (box_idx >= sorted_coords.size()) {
                RCLCPP_INFO(node->get_logger(),
                            "NAVIGATE: All %zu locations visited.", sorted_coords.size());
                state = RETURN_HOME;
                break;
            }

            // Bail out early if time is nearly up
            if (timeLeft < 55) {
                RCLCPP_WARN(node->get_logger(),
                            "NAVIGATE: Only %lus left — returning home early.", timeLeft);
                state = RETURN_HOME;
                break;
            }

            double gx = sorted_coords[box_idx][0];
            double gy = sorted_coords[box_idx][1];
            double gp = sorted_coords[box_idx][2];
            RCLCPP_INFO(node->get_logger(),
                        "NAVIGATE: → Location %zu/%zu  (%.2f, %.2f, %.2f)...",
                        box_idx + 1, sorted_coords.size(), gx, gy, gp);

            bool arrived = nav.moveToGoal(gx, gy, gp);

            // Let sensor data settle after arrival
            for (int i = 0; i < 15; ++i) {
                rclcpp::spin_some(node);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (arrived) {
                RCLCPP_INFO(node->get_logger(),
                            "NAVIGATE: Arrived at location %zu. Running OAK-D YOLO (with sweep)...",
                            box_idx + 1);

                // Detect scene object: 6 primary attempts + ±15° sweep if needed
                auto det = yoloWithSweep(yolo, nav, gx, gy, gp, 6, node);
                std::string scene_name = det.first.empty() ? "nothing" : det.first;
                float scene_conf = det.second;

                scene_results[box_idx] = scene_name;
                RCLCPP_INFO(node->get_logger(),
                            "NAVIGATE: Location %zu → '%s' (conf=%.2f)",
                            box_idx + 1, scene_name.c_str(), scene_conf);

                // Check if this scene object matches the manipulable object
                if (!placed &&
                    !manip_name.empty() &&
                    scene_name == manip_name) {
                    RCLCPP_INFO(node->get_logger(),
                                "NAVIGATE: ★ MATCH! '%s' matches manipulable object. "
                                "Moving to PLACE_OBJECT.",
                                scene_name.c_str());
                    state = PLACE_OBJECT;
                } else {
                    box_idx++;
                }
            } else {
                RCLCPP_WARN(node->get_logger(),
                            "NAVIGATE: Could not reach location %zu.", box_idx + 1);
                scene_results[box_idx] = "unreachable";
                box_idx++;
            }
            break;
        }

        // ═══════════════════════════════════════════════════════════════════════
        case PLACE_OBJECT: {
            RCLCPP_INFO(node->get_logger(),
                        "PLACE: Looking for garbage bin AprilTag (IDs 0-4)...");

            // Allow TF frames to update
            for (int i = 0; i < 20; ++i) {
                rclcpp::spin_some(node);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            // Retry AprilTag detection up to 3 times (1 s apart)
            std::vector<int> visible;
            for (int attempt = 0; attempt < 3 && visible.empty(); ++attempt) {
                visible = tags.getVisibleTags({0, 1, 2, 3, 4}, 500);
                if (visible.empty()) {
                    RCLCPP_WARN(node->get_logger(),
                                "PLACE: No tag on attempt %d/3, waiting 1 s...", attempt + 1);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    for (int k = 0; k < 10; ++k) rclcpp::spin_some(node);
                }
            }

            // Determine placement target in arm_mount frame
            double place_x = 0.25;   // default: straight forward
            double place_y = 0.0;
            double place_z = 0.18;   // above bin opening

            if (!visible.empty()) {
                int bin_tag = visible[0];
                RCLCPP_INFO(node->get_logger(), "PLACE: Tag %d visible.", bin_tag);

                auto pose_opt = tags.getTagPose(bin_tag, 500);
                if (pose_opt.has_value()) {
                    // Tag is mounted on the side of the bin.
                    // Bin opening is approximately 0.15 m above the tag face.
                    // Clamp to the arm's reachable workspace.
                    place_x = std::min(std::max(pose_opt->position.x, 0.12), 0.30);
                    place_y = std::min(std::max(pose_opt->position.y, -0.15), 0.15);
                    place_z = std::max(pose_opt->position.z + 0.15, 0.15);
                    RCLCPP_INFO(node->get_logger(),
                                "PLACE: Tag pose (%.2f, %.2f, %.2f) → "
                                "arm target (%.2f, %.2f, %.2f)",
                                pose_opt->position.x, pose_opt->position.y, pose_opt->position.z,
                                place_x, place_y, place_z);
                }
            } else {
                RCLCPP_WARN(node->get_logger(),
                            "PLACE: No tag visible after 3 attempts — using default placement.");
            }

            // Approach: hover above bin opening, then lower to release height
            arm.moveToCartesianPose(place_x, place_y, place_z + 0.05, 0.0, 1.57, 0.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            arm.moveToCartesianPose(place_x, place_y, place_z, 0.0, 1.57, 0.0);

            // Release object into bin
            arm.openGripper();
            std::this_thread::sleep_for(std::chrono::milliseconds(700));

            // Retract arm to compact stow (no object held)
            arm.moveToCartesianPose(STOW_X, STOW_Y, STOW_Z, 0.0, 1.57, 0.0);

            placed = true;
            RCLCPP_INFO(node->get_logger(), "PLACE: Object released into bin!");

            // Continue visiting remaining scene objects (for scene-identification marks)
            box_idx++;
            state = NAVIGATE_AND_DETECT;
            break;
        }

        // ═══════════════════════════════════════════════════════════════════════
        case RETURN_HOME: {
            // Stow arm safely before driving home
            if (placed || manip_name.empty()) {
                // No object in hand — compact stow
                RCLCPP_INFO(node->get_logger(), "RETURN: Stowing arm (no object in hand)...");
                arm.moveToCartesianPose(STOW_X, STOW_Y, STOW_Z, 0.0, 1.57, 0.0);
                arm.closeGripper();
            } else {
                // Unplaced object still in gripper — stay at carry pose
                RCLCPP_INFO(node->get_logger(),
                            "RETURN: Carrying unplaced object home (no match found).");
            }

            RCLCPP_INFO(node->get_logger(),
                        "RETURN: Navigating home (%.2f, %.2f, %.2f)...",
                        home_x, home_y, home_phi);
            nav.moveToGoal(home_x, home_y, home_phi);
            RCLCPP_INFO(node->get_logger(), "RETURN: Home reached.");

            writeResults(manip_name, manip_conf, sorted_coords, scene_results, placed, false);
            RCLCPP_INFO(node->get_logger(),
                        "RETURN: Results written to /home/turtlebot/contest2_report.txt");
            RCLCPP_INFO(node->get_logger(),
                        "RETURN: Annotated wrist image saved to /home/turtlebot/detected_object.jpg");
            state = FINISHED;
            break;
        }

        // ═══════════════════════════════════════════════════════════════════════
        case FINISHED:
            RCLCPP_INFO_ONCE(node->get_logger(),
                             "FINISHED: All tasks complete. Standing by.");
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // ── Timeout: write whatever data we have ──────────────────────────────────
    if (state != FINISHED) {
        RCLCPP_WARN(node->get_logger(), "TIME LIMIT reached — writing partial results.");
        writeResults(manip_name, manip_conf, sorted_coords, scene_results, placed, true);
    }

    rclcpp::shutdown();
    return 0;
}
