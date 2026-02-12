#include <chrono>
#include <memory>
#include <cmath>
#include <map>
#include <vector>
#include <deque>
#include <algorithm>
#include <string>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

// === UTILITY FUNCTIONS ===
inline double rad2deg(double rad) { return rad * (180.0 / M_PI); }
inline double deg2rad(double deg) { return deg * (M_PI / 180.0); }

inline double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// === STATE ENUMS ===
enum class State {
    UNDOCKING,
    EXPLORATION,
    WALL_FOLLOW,
    BUMPED, 
    ROTATE_BY_MARGIN, 
    MOVE_LINEAR_BY_MARGIN 
};

enum class WallFollowSubState {
    ALIGNING,
    FOLLOWING
};

enum class BumpSubState {
    CHECK_SENSORS,
    BACKING,
    TURNING,
    FORWARD_ARC
};

// === MAIN NODE ===
class Contest1Node : public rclcpp::Node
{
public:
    Contest1Node() : Node("contest1_node")
    {
        // Publishers & Subscribers
        vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::laserCallback, this, std::placeholders::_1));
        hazard_sub_ = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
            "/hazard_detection", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::hazardCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::odomCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(100ms, std::bind(&Contest1Node::controlLoop, this));

        // Initialize state
        start_time_ = this->now();
        current_state_ = State::EXPLORATION;
        undock_phase_ = 0;
        
        // Initialize odometry
        pos_x_ = 0.0; 
        pos_y_ = 0.0; 
        yaw_ = 0.0;
        stuck_count_ = 0;
        last_position_check_time_ = this->now();
        wall_lost_count_ = 0;
        
        // Initialize sensor values
        minLaserDistFront_ = std::numeric_limits<float>::infinity();
        minLaserDistFront_wider_ = std::numeric_limits<float>::infinity();
        minLaserDistRight_ = std::numeric_limits<float>::infinity();
        minLaserDistLeft_ = std::numeric_limits<float>::infinity();
        minLaserDistLeft_true_ = std::numeric_limits<float>::infinity();
        minLaserDistRight_true_ = std::numeric_limits<float>::infinity();
        minLaserDistLeft_medium_ = std::numeric_limits<float>::infinity();
        minLaserDistRight_medium_ = std::numeric_limits<float>::infinity();
        laser_left_front_ = std::numeric_limits<float>::infinity();
        laser_left_back_ = std::numeric_limits<float>::infinity();
        laser_right_front_ = std::numeric_limits<float>::infinity();
        laser_right_back_ = std::numeric_limits<float>::infinity();
        laser_back_ = std::numeric_limits<float>::infinity();

        minDistIndexFront_ = -1;
        minDistIndexFront_wider_ = -1;
        minDistIndexRight_ = -1;
        minDistIndexLeft_ = -1;
        minDistIndexRight_true_ = -1;
        minDistIndexLeft_true_ = -1;
        minDistIndexRight_medium_ = -1;
        minDistIndexLeft_medium_ = -1;
        
        // Initialize wall follow parameters
        alignment_locked_ = false;
        locked_target_distance_ = 0.0f;
        skip_alignment_ = false;

        // Initialize rotation parameters
        rotate_angular_speed_ = deg2rad(45);
        rotate_target_yaw_ = 0.0;
        rotate_start_yaw_ = 0.0;
        rotate_return_state_ = State::EXPLORATION;
        
        // Initialize linear movement parameters
        linear_start_x_ = 0.0;
        linear_start_y_ = 0.0;
        linear_target_distance_ = 0.0;
        linear_speed_ = 0.25;
        linear_return_state_ = State::EXPLORATION;
        
        last_wall_follow_exit_time_ = this->now();
        exploration_start_x_ = 0.0;
        exploration_start_y_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "Contest 1 node initialized. State: UNDOCKING.");
    }

    // Get minimum laser distance within angle range (handles ±180° wrap)
    float getMinLaserDistInRange(
        const sensor_msgs::msg::LaserScan::SharedPtr& scan,
        double start_angle_deg,
        double end_angle_deg,
        float min_valid_range = 0.15f,
        int* out_min_index = nullptr
    ) {
        auto searchRange = [&](double start_deg, double end_deg, float& min_dist, int& min_idx) {
            double start_angle_rad = deg2rad(start_deg);
            double end_angle_rad = deg2rad(end_deg);
            
            int start_idx = (int)((start_angle_rad - scan->angle_min) / scan->angle_increment);
            int end_idx = (int)((end_angle_rad - scan->angle_min) / scan->angle_increment);
            
            int num_readings = (int)scan->ranges.size();
            start_idx = std::clamp(start_idx, 0, num_readings - 1);
            end_idx = std::clamp(end_idx, 0, num_readings - 1);
            
            for (int i = start_idx; i <= end_idx; ++i) {
                float r = scan->ranges[i];
                if (std::isfinite(r) && r > min_valid_range && r < min_dist) {
                    min_dist = r;
                    min_idx = i;
                }
            }
        };
        
        float min_dist = std::numeric_limits<float>::infinity();
        int min_idx = -1;
        
        auto normalizeAngle = [](double angle) {
            while (angle > 180.0) angle -= 360.0;
            while (angle < -180.0) angle += 360.0;
            return angle;
        };
        
        start_angle_deg = normalizeAngle(start_angle_deg);
        end_angle_deg = normalizeAngle(end_angle_deg);
        
        if (start_angle_deg > end_angle_deg) {
            searchRange(start_angle_deg, 180.0, min_dist, min_idx);
            searchRange(-180.0, end_angle_deg, min_dist, min_idx);
        } else {
            searchRange(start_angle_deg, end_angle_deg, min_dist, min_idx);
        }
        
        if (out_min_index != nullptr) {
            *out_min_index = min_idx;
        }
        
        return min_dist;
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Reset distances
        minLaserDistFront_ = std::numeric_limits<float>::infinity();
        minLaserDistFront_wider_ = std::numeric_limits<float>::infinity();
        minLaserDistRight_ = std::numeric_limits<float>::infinity();
        minLaserDistLeft_ = std::numeric_limits<float>::infinity();
        minLaserDistLeft_true_ = std::numeric_limits<float>::infinity();
        minLaserDistRight_true_ = std::numeric_limits<float>::infinity();
        minLaserDistLeft_medium_ = std::numeric_limits<float>::infinity();
        minLaserDistRight_medium_ = std::numeric_limits<float>::infinity();
        laser_left_front_ = std::numeric_limits<float>::infinity();
        laser_left_back_ = std::numeric_limits<float>::infinity();
        laser_right_front_ = std::numeric_limits<float>::infinity();
        laser_right_back_ = std::numeric_limits<float>::infinity();
        laser_back_ = std::numeric_limits<float>::infinity();

        minDistIndexFront_ = -1;
        minDistIndexFront_wider_ = -1;
        minDistIndexRight_ = -1;
        minDistIndexLeft_ = -1;
        minDistIndexLeft_true_ = -1;
        minDistIndexRight_true_ = -1;
        minDistIndexRight_medium_ = -1;
        minDistIndexLeft_medium_ = -1;

        // Compute sensor readings
        minLaserDistFront_ = getMinLaserDistInRange(scan, -92.0, -88.0, 0.15f, &minDistIndexFront_);
        minLaserDistFront_wider_ = getMinLaserDistInRange(scan, -105.0, -75.0, 0.15f, &minDistIndexFront_);
        minLaserDistLeft_ = getMinLaserDistInRange(scan, -2.0, 2.0, 0.15f, &minDistIndexLeft_);
        minLaserDistLeft_true_ = getMinLaserDistInRange(scan, -90.0, 90.0, 0.15f, &minDistIndexLeft_true_);
        minLaserDistLeft_medium_ = getMinLaserDistInRange(scan, -15.0, 15.0, 0.15f, &minDistIndexLeft_medium_);
        minLaserDistRight_ = getMinLaserDistInRange(scan, 178.0, -178.0, 0.15f, &minDistIndexRight_);
        minLaserDistRight_true_ = getMinLaserDistInRange(scan, 90.0, -90.0, 0.15f, &minDistIndexRight_true_);
        minLaserDistRight_medium_ = getMinLaserDistInRange(scan, 165.0, -165.0, 0.15f, &minDistIndexRight_medium_);
        laser_left_front_ = getMinLaserDistInRange(scan, -46.0, -44.0);
        laser_left_back_ = getMinLaserDistInRange(scan, 44.0, 46.0);
        laser_right_front_ = getMinLaserDistInRange(scan, -136.0, -134.0);
        laser_right_back_ = getMinLaserDistInRange(scan, 134.0, 136.0);
        laser_back_ = getMinLaserDistInRange(scan, 86.0, 94.0);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "F=%.2f L=%.2f R=%.2f B=%.2f | LF=%.2f LB=%.2f RF=%.2f RB=%.2f | LW=%.2f RW=%.2f LM=%.2f RM=%.2f", 
            minLaserDistFront_, minLaserDistLeft_, minLaserDistRight_, laser_back_,
            laser_left_front_, laser_left_back_, laser_right_front_, laser_right_back_, 
            minLaserDistLeft_true_, minLaserDistRight_true_, minLaserDistLeft_medium_, minLaserDistRight_medium_
        );
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        pos_x_ = odom->pose.pose.position.x;
        pos_y_ = odom->pose.pose.position.y;
        yaw_ = tf2::getYaw(odom->pose.pose.orientation);
    }

    void hazardCallback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr hazard_vector)
    {
        bool any_bump = false;
        std::string bumped_frame;

        for (const auto& detection : hazard_vector->detections) {
            if (detection.type == irobot_create_msgs::msg::HazardDetection::BUMP) {
                bumped_frame = detection.header.frame_id;
                any_bump = true;
            }
        }

        if (any_bump) {
            last_bumped_frame_ = bumped_frame;
            
            if (current_state_ != State::BUMPED) {
                original_yaw_before_bump_sequence_ = yaw_;
                enterBumpedState();
            } else {
                bump_sub_state_ = BumpSubState::CHECK_SENSORS;
                stopRobot();
            }
        }
    }

    // === STATE TRANSITIONS ===
    
    void enterExplorationState() {
        RCLCPP_INFO(this->get_logger(), "State: EXPLORATION");
        current_state_ = State::EXPLORATION;
        alignment_locked_ = false;
        wall_lost_count_ = 0;  // Clear wall lost counter
        RCLCPP_INFO(this->get_logger(), "Wall lost counter cleared");
        stopRobot();
    }

    void enterWallFollowState() {
        RCLCPP_INFO(this->get_logger(), "State: WALL_FOLLOW");
        current_state_ = State::WALL_FOLLOW;
        
        if (skip_alignment_) {
            wf_sub_state_ = WallFollowSubState::FOLLOWING;
            skip_alignment_ = false;
            RCLCPP_INFO(this->get_logger(), "Resuming FOLLOWING (preserved origin)");
            stopRobot();
            return;
        }
        
        wf_sub_state_ = WallFollowSubState::ALIGNING;
        wall_follow_start_x_ = pos_x_;
        wall_follow_start_y_ = pos_y_;
        wall_follow_start_time_ = this->now();
        alignment_start_time_ = this->now();
        
        float left_true = minLaserDistLeft_true_;
        float right_true = minLaserDistRight_true_;
        
        if (left_true < right_true) {
            following_side_ = 1;
            locked_target_distance_ = left_true;
            RCLCPP_INFO(this->get_logger(), "Left Wall - TRUE min: %.3f", left_true);
        } else {
            following_side_ = -1;
            locked_target_distance_ = right_true;
            RCLCPP_INFO(this->get_logger(), "Right Wall - TRUE min: %.3f", right_true);
        }
        
        alignment_locked_ = true;
        position_history_.clear();
        stuck_count_ = 0;
        last_position_check_time_ = this->now();
        stopRobot();
    }
        
    void enterBumpedState() {
        RCLCPP_INFO(this->get_logger(), "State: BUMPED");
        current_state_ = State::BUMPED;
        bump_sub_state_ = BumpSubState::CHECK_SENSORS;
        stopRobot();
    }

    void enterRotateByMargin(double angle_deg, State return_state) {
        RCLCPP_INFO(this->get_logger(), "State: ROTATE_BY_MARGIN (%.1f degrees)", angle_deg);
        current_state_ = State::ROTATE_BY_MARGIN;
        rotate_return_state_ = return_state;
        rotate_start_yaw_ = yaw_;
        rotate_target_yaw_ = normalize_angle(yaw_ + deg2rad(angle_deg));
        stopRobot();
    }

    void enterMoveLinearByMargin(double distance_m, double speed_mps, State return_state) {
        RCLCPP_INFO(this->get_logger(), "State: MOVE_LINEAR_BY_MARGIN (%.2fm at %.2fm/s)", 
                    distance_m, speed_mps);
        current_state_ = State::MOVE_LINEAR_BY_MARGIN;
        linear_return_state_ = return_state;
        linear_start_x_ = pos_x_;
        linear_start_y_ = pos_y_;
        linear_target_distance_ = std::abs(distance_m);
        linear_speed_ = (distance_m >= 0) ? std::abs(speed_mps) : -std::abs(speed_mps);
        stopRobot();
    }

    // === CONTROL LOOP ===
    
    void controlLoop()
    {
        if ((this->now() - start_time_).seconds() >= 100000.0) {
            stopRobot();
            rclcpp::shutdown();
            return;
        }

        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();

        switch (current_state_) {
            case State::UNDOCKING:
            {
                static double start_x = pos_x_;
                static double start_y = pos_y_;
                static bool init = false;
                if (!init) { start_x = pos_x_; start_y = pos_y_; init = true; }

                if (undock_phase_ == 0) {
                    double dist = std::hypot(pos_x_ - start_x, pos_y_ - start_y);
                    if (dist < 0.3) {
                        vel.twist.linear.x = -0.5; 
                    } else {
                        undock_phase_ = 1;
                        undock_target_yaw_ = normalize_angle(yaw_ + M_PI);
                        stopRobot();
                    }
                } else {
                    double err = normalize_angle(undock_target_yaw_ - yaw_);
                    if (std::abs(err) > 0.1) {
                        vel.twist.angular.z = (err > 0) ? 0.5 : -0.5;
                    } else {
                        enterExplorationState();
                    }
                }
                break;
            }
            
            case State::EXPLORATION:
            {
                if (minLaserDistFront_wider_ < 0.5) {
                    if (minLaserDistLeft_true_ < minLaserDistRight_true_) {
                        RCLCPP_INFO(this->get_logger(), "Left Wall Selected");
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Right Wall Selected");
                    }
                    enterWallFollowState();
                } else {
                    vel.twist.linear.x = 0.25;
                    vel_pub_->publish(vel);
                }
                break;
            }

            case State::WALL_FOLLOW:
            {
                // Stuck detection
            double time_since_check = (this->now() - last_position_check_time_).seconds();

            if (time_since_check >= POSITION_CHECK_INTERVAL) {
                if (!position_history_.empty()) {
                    auto& zone_center = position_history_.front();
                    double dist_from_zone = std::hypot(pos_x_ - zone_center.first, pos_y_ - zone_center.second);
                    
                    // Still in same zone = stuck
                    if (dist_from_zone < STUCK_THRESHOLD) {
                        stuck_count_++;
                        RCLCPP_WARN(this->get_logger(), 
                            "STUCK! Still in zone (%.2fm from center) for %.1fs (count: %d/%d)",
                            dist_from_zone, stuck_count_ * POSITION_CHECK_INTERVAL, stuck_count_, MAX_STUCK_COUNT);
                        
                        if (stuck_count_ >= MAX_STUCK_COUNT) {
                            RCLCPP_ERROR(this->get_logger(), 
                                "STUCK FOR TOO LONG! Returning to EXPLORATION!");
                            stuck_count_ = 0;
                            position_history_.clear();
                            enterExplorationState();
                            break;
                        }
                    } else {
                        // Escaped zone - reset counter and update zone center
                        stuck_count_ = 0;
                        position_history_.clear();
                        position_history_.push_back({pos_x_, pos_y_});
                    }
                } else {
                    // First check - establish zone center
                    position_history_.push_back({pos_x_, pos_y_});
                }
                
                last_position_check_time_ = this->now();
            }
                
                float wall_center_instantaneous;
                if (following_side_ == 1) {
                    wall_center_instantaneous = minLaserDistLeft_medium_;
                } else {
                    wall_center_instantaneous = minLaserDistRight_medium_;
                }
                
                // Wall lost detection
                bool wall_lost = (wall_center_instantaneous > 2.5f);
                bool front_obstacle = (minLaserDistFront_ < 1.0f);
     
                if (wall_lost && !front_obstacle) {
                     wall_lost_count_++;  // Increment counter
                    RCLCPP_WARN(this->get_logger(), 
                    "Wall lost! (dist=%.2fm) [Count: %d]", 
                    wall_center_instantaneous, wall_lost_count_);
                    if (wall_lost_count_ >= 3) {
                        RCLCPP_ERROR(this->get_logger(), 
                            "Wall lost %d times! Giving up on wall follow, entering EXPLORATION", 
                            wall_lost_count_);
                        enterExplorationState();
                        break;
                    }
                    
                    skip_alignment_ = true;
                    double escape_angle = following_side_ * 60.0;
                    enterRotateByMargin(escape_angle, State::WALL_FOLLOW);
                    break;
                }
                
                // ALIGNING substate
                if (wf_sub_state_ == WallFollowSubState::ALIGNING) {
                    double alignment_duration = (this->now() - alignment_start_time_).seconds();
                    float current_perpendicular = (following_side_ == 1) ? minLaserDistLeft_ : minLaserDistRight_;
                    float distance_error = current_perpendicular - locked_target_distance_;

                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(), *this->get_clock(), 500,
                        "ALIGNING %s: Perp=%.3fm, Target=%.3fm, Error=%.3fm",
                        (following_side_ == 1) ? "LEFT" : "RIGHT",
                        current_perpendicular, locked_target_distance_, distance_error
                    );

                    if (alignment_duration > ALIGNMENT_TIMEOUT) {
                        RCLCPP_WARN(this->get_logger(), "⚠ Alignment timeout! Forcing FOLLOWING mode.");
                        wf_sub_state_ = WallFollowSubState::FOLLOWING;
                        locked_target_distance_ = current_perpendicular;
                        stopRobot();
                        break;
                    }
                    
                    if (std::abs(distance_error) < 0.05f) {
                        RCLCPP_INFO(this->get_logger(), 
                            "✓ ALIGNED! Perpendicular sensor matches target (%.3fm). Starting FOLLOWING.",
                            current_perpendicular
                        );
                        wf_sub_state_ = WallFollowSubState::FOLLOWING;
                        stopRobot();
                    } else {
                        vel.twist.linear.x = 0.0;
                        vel.twist.angular.z = -following_side_ * 1.0f;
                        vel.twist.angular.z = std::clamp(vel.twist.angular.z, -deg2rad(45), deg2rad(45));
                    }
                }
                // FOLLOWING substate
                else if (wf_sub_state_ == WallFollowSubState::FOLLOWING) 
                {
                    float wall_front, wall_back;
                    if (following_side_ == 1) {
                        wall_front = laser_left_front_;
                        wall_back = laser_left_back_;
                    } else {
                        wall_front = laser_right_front_;
                        wall_back = laser_right_back_;
                    }
                                
                    const float DESIRED_DISTANCE = 0.50f;
                    
                    if (minLaserDistFront_ < 0.5) {
                        RCLCPP_INFO(this->get_logger(), "Front obstacle!");
                        vel.twist.linear.x = 0.0;
                        vel.twist.angular.z = -following_side_ * 0.6;
                    } else {
                        float distance_error = wall_center_instantaneous - DESIRED_DISTANCE;
                        float parallelism_error = wall_front - wall_back;
                        
                        vel.twist.linear.x = 0.18;
                        
                        float Kp_distance = 0.6;
                        float Kp_parallel = 0.3;
                        
                        float angular_from_distance = following_side_ * Kp_distance * distance_error;
                        float angular_from_parallel = following_side_ * Kp_parallel * parallelism_error;
                        
                        vel.twist.angular.z = angular_from_distance + angular_from_parallel;
                        vel.twist.angular.z = std::clamp(vel.twist.angular.z, -1.6, 1.6);
                        
                        RCLCPP_INFO_THROTTLE(
                            this->get_logger(), *this->get_clock(), 1000,
                            "Following: Dist=%.2fm(err=%.2f), Parallel=%.3fm, ω=%.2f",
                            wall_center_instantaneous, distance_error, parallelism_error, vel.twist.angular.z
                        );
                    }
                    
                    // Loop closure detection
                    double elapsed_time = (this->now() - wall_follow_start_time_).seconds();
                    
                    if (elapsed_time > 12.0) {
                        double dist_to_start = std::hypot(
                            pos_x_ - wall_follow_start_x_, 
                            pos_y_ - wall_follow_start_y_
                        );
                        
                        if (dist_to_start < 1.2) {
                            RCLCPP_INFO(this->get_logger(), 
                                "Loop closure detected! Completed circuit in %.1f seconds. Escaping...", 
                                elapsed_time
                            );
                            double escape_angle = -following_side_ * 127.0;
                            enterRotateByMargin(escape_angle, State::EXPLORATION);
                            break;
                        }
                    }
                }
                break;
            }
                    
            case State::BUMPED:
            {
                switch (bump_sub_state_) {
                    case BumpSubState::CHECK_SENSORS:
                    {
                        if (minLaserDistFront_wider_ < 0.2f) {
                            skip_alignment_ = true;
                            enterMoveLinearByMargin(-0.5, 0.1, State::WALL_FOLLOW);
                        } else {
                            bump_start_x_ = pos_x_;
                            bump_start_y_ = pos_y_;
                            bump_sub_state_ = BumpSubState::BACKING;
                        }
                        break;
                    }

                    case BumpSubState::BACKING:
                    {
                        double dist = std::hypot(pos_x_ - bump_start_x_, pos_y_ - bump_start_y_);
                        if (dist < 0.1) {
                            vel.twist.linear.x = -0.1;
                        } else {
                            stopRobot();
                            double turn_rad = deg2rad(60.0);

                            if (last_bumped_frame_.find("right") != std::string::npos) {
                                bump_target_yaw_ = normalize_angle(yaw_ + turn_rad);
                                bump_turn_direction_ = 1.0; 
                            } else {
                                bump_target_yaw_ = normalize_angle(yaw_ - turn_rad);
                                bump_turn_direction_ = -1.0;
                            }
                            bump_sub_state_ = BumpSubState::TURNING;
                        }
                        break;
                    }

                    case BumpSubState::TURNING:
                    {
                        double err = normalize_angle(bump_target_yaw_ - yaw_);
                        if (std::abs(err) > 0.1) {
                            vel.twist.linear.x = 0.0;
                            vel.twist.angular.z = (bump_turn_direction_ > 0) ? 0.5 : -0.5;
                        } else {
                            bump_sub_state_ = BumpSubState::FORWARD_ARC;
                        }
                        break;
                    }

                    case BumpSubState::FORWARD_ARC:
                    {
                        double yaw_diff = std::abs(normalize_angle(yaw_ - original_yaw_before_bump_sequence_));
                        if (yaw_diff < 0.1) {
                            enterExplorationState();
                        } else {
                            vel.twist.linear.x = 0.1;
                            vel.twist.angular.z = -1.0 * bump_turn_direction_ * 0.5;
                        }
                        break;
                    }
                }
                break;
            }

            case State::ROTATE_BY_MARGIN:
            {
                double error = normalize_angle(rotate_target_yaw_ - yaw_);
                if (std::abs(error) < 0.1) {
                    RCLCPP_INFO(this->get_logger(), "Rotation complete");
                    current_state_ = rotate_return_state_;
                } else {
                    vel.twist.angular.z = (error > 0) ? rotate_angular_speed_ : -rotate_angular_speed_;
                }
                break;
            }

            case State::MOVE_LINEAR_BY_MARGIN:
            {
                double dist = std::hypot(pos_x_ - linear_start_x_, pos_y_ - linear_start_y_);
                if (dist >= linear_target_distance_) {
                    RCLCPP_INFO(this->get_logger(), "Movement complete");
                    current_state_ = linear_return_state_;
                } else {
                    vel.twist.linear.x = linear_speed_;
                }
                break;
            }
        }
        vel_pub_->publish(vel);
    }
    
    void stopRobot() {
        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        vel.twist.linear.x = 0.0; 
        vel.twist.angular.z = 0.0;
        vel_pub_->publish(vel);
    }

private:
    // ROS infrastructure
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr hazard_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    rclcpp::Time start_time_;
    State current_state_;

    // Odometry
    double pos_x_, pos_y_, yaw_;
    
    // Laser sensor data
    std::string last_bumped_frame_;
    float minLaserDistFront_, minLaserDistFront_wider_;
    float minLaserDistLeft_, minLaserDistRight_;
    float minLaserDistLeft_true_, minLaserDistRight_true_;
    float minLaserDistLeft_medium_, minLaserDistRight_medium_;
    int minDistIndexFront_, minDistIndexFront_wider_;
    int minDistIndexLeft_, minDistIndexRight_;
    int minDistIndexLeft_true_, minDistIndexRight_true_;
    int minDistIndexLeft_medium_, minDistIndexRight_medium_;
    float laser_left_front_, laser_left_back_;
    float laser_right_front_, laser_right_back_;
    float laser_back_;

    // Undocking
    int undock_phase_;
    double undock_target_yaw_;

    // Exploration
    rclcpp::Time last_wall_follow_exit_time_;
    double exploration_start_x_, exploration_start_y_;

    // Wall following
    bool alignment_locked_;
    float locked_target_distance_;
    WallFollowSubState wf_sub_state_;
    int following_side_; // 1=Left, -1=Right
    double wall_follow_start_x_, wall_follow_start_y_;
    rclcpp::Time wall_follow_start_time_;
    rclcpp::Time alignment_start_time_;
    const double ALIGNMENT_TIMEOUT = 5;
    bool skip_alignment_;

    // Bump handling
    BumpSubState bump_sub_state_;
    double bump_start_x_, bump_start_y_;
    double original_yaw_before_bump_sequence_;
    double bump_target_yaw_;
    double bump_turn_direction_;

    // Rotation actuation
    double rotate_target_yaw_;
    double rotate_start_yaw_;
    double rotate_angular_speed_;
    State rotate_return_state_;
    
    // Linear movement
    double linear_start_x_, linear_start_y_;
    double linear_target_distance_;
    double linear_speed_;
    State linear_return_state_;

    // Stuck detection
    std::deque<std::pair<double, double>> position_history_;
    rclcpp::Time last_position_check_time_;
    const double POSITION_CHECK_INTERVAL = 3.0;
    const double STUCK_THRESHOLD = 0.25;
    int stuck_count_;
    const int MAX_STUCK_COUNT = 5;
        
    // Wall lost counter
    int wall_lost_count_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
