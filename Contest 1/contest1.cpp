#include <chrono>
#include <memory>
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>
#include <string>
#include <limits>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

inline double rad2deg(double rad) { return rad * (180.0 / M_PI); }
inline double deg2rad(double deg) { return deg * (M_PI / 180.0); }

inline double normalize_angle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

enum class State {
    UNDOCKING,
    EXPLORATION,
    WALL_FOLLOW,
    BUMPED
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

class Contest1Node : public rclcpp::Node
{
public:
    Contest1Node()
        : Node("contest1_node")
    {
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

        timer_ = this->create_wall_timer(
            100ms, std::bind(&Contest1Node::controlLoop, this));

        start_time_ = this->now();
        current_state_ = State::UNDOCKING;
        undock_phase_ = 0;
        
        pos_x_ = 0.0; pos_y_ = 0.0; yaw_ = 0.0;
        
        // Initial sensor values
        minLaserDistFront_ = std::numeric_limits<float>::infinity();
        minLaserDistRight_ = std::numeric_limits<float>::infinity();
        minLaserDistLeft_  = std::numeric_limits<float>::infinity();
        laser_left_front_ = std::numeric_limits<float>::infinity();
        laser_left_back_ = std::numeric_limits<float>::infinity();
        laser_right_front_ = std::numeric_limits<float>::infinity();
        laser_right_back_ = std::numeric_limits<float>::infinity();
        laser_back_ =   std::numeric_limits<float>::infinity();
        
        minDistIndexRight_ = -1;
        minDistIndexLeft_ = -1;
        angle_increment__ = 0.0;



        RCLCPP_INFO(this->get_logger(), "Contest 1 node initialized. State: UNDOCKING.");
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        float angle_increment_ = scan->angle_increment;
        float angle_min_ = scan->angle_min;
        int num_readings = (int)scan->ranges.size();

        // Reset all minimum distances
        minLaserDistFront_ = std::numeric_limits<float>::infinity();
        minLaserDistRight_ = std::numeric_limits<float>::infinity();
        minLaserDistLeft_  = std::numeric_limits<float>::infinity();
        laser_left_front_  = std::numeric_limits<float>::infinity();
        laser_left_back_   = std::numeric_limits<float>::infinity();
        laser_right_front_ = std::numeric_limits<float>::infinity();
        laser_right_back_  = std::numeric_limits<float>::infinity();
        laser_back_        = std::numeric_limits<float>::infinity();

        minDistIndexFront_ = -1;
        minDistIndexRight_ = -1;
        minDistIndexLeft_  = -1;

        // ============================================
        // FRONT WINDOW: -94° to -86° (centered at -90°)
        // ============================================
        int front_start = (int)((deg2rad(-94.0) - angle_min_) / angle_increment_);
        int front_end   = (int)((deg2rad(-86.0) - angle_min_) / angle_increment_);
        
        if (front_start < 0) front_start = 0;
        if (front_end >= num_readings) front_end = num_readings - 1;
        //if (front_start > front_end) std::swap(front_start, front_end);

        for (int i = front_start; i <= front_end; ++i) {
            float r = scan->ranges[i];
            if (std::isfinite(r) && r > 0.05f && r < minLaserDistFront_) {
                minLaserDistFront_ = r;
                minDistIndexFront_ = i;
            }
        }

        // ============================================
        // LEFT WINDOW: -4° to +4° (centered at 0°)
        // ============================================
        int left_start_1 = (int)((deg2rad(-4.0) - angle_min_) / angle_increment_);
        int left_end_1   = (int)((deg2rad(0) - angle_min_) / angle_increment_);
        
        if (left_start_1 < 0) left_start_1 = 0;
        if (left_end_1 >= num_readings) left_end_1 = num_readings - 1;
        //if (left_start_1 > left_end_1) std::swap(left_start, left_end);

        for (int i = left_start_1; i <= left_end_1; ++i) {
            float r = scan->ranges[i];
            if (std::isfinite(r) && r > 0.05f && r < minLaserDistLeft_) {
                minLaserDistLeft_ = r;
                minDistIndexLeft_ = i;
            }
        }

        int left_start_2 = (int)((deg2rad(0) - angle_min_) / angle_increment_);
        int left_end_2  = (int)((deg2rad(4.0) - angle_min_) / angle_increment_);
        
        if (left_start_2 < 0) left_start_2 = 0;
        if (left_end_2 >= num_readings) left_end_2 = num_readings - 1;
        //if (left_start > left_end) std::swap(left_start, left_end);

        for (int i = left_start_2; i <= left_end_2; ++i) {
            float r = scan->ranges[i];
            if (std::isfinite(r) && r > 0.05f && r < minLaserDistLeft_) {
                minLaserDistLeft_ = r;
                minDistIndexLeft_ = i;
            }
        }

        // ============================================
        // RIGHT WINDOW: 175° to -175° (centered at ±180°)
        // SPECIAL: Split into two windows to handle wrapping
        // ============================================
        
        // RIGHT Part 1: 175° to 180°
        int right_start_1 = (int)((deg2rad(175.0) - angle_min_) / angle_increment_);
        int right_end_1   = (int)((deg2rad(180) - angle_min_) / angle_increment_);
        
        if (right_start_1 < 0) right_start_1 = 0;
        if (right_end_1 >= num_readings) right_end_1 = num_readings - 1;
        //if (right_start_1 > right_end_1) std::swap(right_start_1, right_end_1);
        
        for (int i = right_start_1; i <= right_end_1; ++i) {
            float r = scan->ranges[i];
            if (std::isfinite(r) && r > 0.05f && r < minLaserDistRight_) {
                minLaserDistRight_ = r;
                minDistIndexRight_ = i;
            }
        }

        // RIGHT Part 2: -180° to -175°
        int right_start_2 = (int)((deg2rad(-180) - angle_min_) / angle_increment_);
        int right_end_2   = (int)((deg2rad(-175.0) - angle_min_) / angle_increment_);
        
        if (right_start_2 < 0) right_start_2 = 0;
        if (right_end_2 >= num_readings) right_end_2 = num_readings - 1;
        //if (right_start_2 > right_end_2) std::swap(right_start_2, right_end_2);
        
        for (int i = right_start_2; i <= right_end_2; ++i) {
            float r = scan->ranges[i];
            if (std::isfinite(r) && r > 0.05f && r < minLaserDistRight_) {
                minLaserDistRight_ = r;
                minDistIndexRight_ = i;
            }
        }

        // ============================================
        // LEFT FRONT RAY: -50° to -40°
        // ============================================
        int lf_start = (int)((deg2rad(-50.0) - angle_min_) / angle_increment_);
        int lf_end   = (int)((deg2rad(-40.0) - angle_min_) / angle_increment_);
        
        if (lf_start < 0) lf_start = 0;
        if (lf_end >= num_readings) lf_end = num_readings - 1;
        //if (lf_start > lf_end) std::swap(lf_start, lf_end);
        
        for (int i = lf_start; i <= lf_end; ++i) {
            float r = scan->ranges[i];
            if (std::isfinite(r) && r > 0.05f && r < laser_left_front_) {
                laser_left_front_ = r;
            }
        }

        // ============================================
        // LEFT BACK RAY: +40° to +50°
        // ============================================
        int lb_start = (int)((deg2rad(40.0) - angle_min_) / angle_increment_);
        int lb_end   = (int)((deg2rad(50.0) - angle_min_) / angle_increment_);
        
        if (lb_start < 0) lb_start = 0;
        if (lb_end >= num_readings) lb_end = num_readings - 1;
        //if (lb_start > lb_end) std::swap(lb_start, lb_end);
        
        for (int i = lb_start; i <= lb_end; ++i) {
            float r = scan->ranges[i];
            if (std::isfinite(r) && r > 0.05f && r < laser_left_back_) {
                laser_left_back_ = r;
            }
        }

        // ============================================
        // RIGHT FRONT RAY: -140° to -130°
        // ============================================
        int rf_start = (int)((deg2rad(-140.0) - angle_min_) / angle_increment_);
        int rf_end   = (int)((deg2rad(-130.0) - angle_min_) / angle_increment_);
        
        if (rf_start < 0) rf_start = 0;
        if (rf_end >= num_readings) rf_end = num_readings - 1;
        //if (rf_start > rf_end) std::swap(rf_start, rf_end);
        
        for (int i = rf_start; i <= rf_end; ++i) {
            float r = scan->ranges[i];
            if (std::isfinite(r) && r > 0.05f && r < laser_right_front_) {
                laser_right_front_ = r;
            }
        }

        // ============================================
        // RIGHT BACK RAY: +130° to +140°
        // ============================================
        int rb_start = (int)((deg2rad(130.0) - angle_min_) / angle_increment_);
        int rb_end   = (int)((deg2rad(140.0) - angle_min_) / angle_increment_);
        
        if (rb_start < 0) rb_start = 0;
        if (rb_end >= num_readings) rb_end = num_readings - 1;
        //if (rb_start > rb_end) std::swap(rb_start, rb_end);
        
        for (int i = rb_start; i <= rb_end; ++i) {
            float r = scan->ranges[i];
            if (std::isfinite(r) && r > 0.05f && r < laser_right_back_) {
                laser_right_back_ = r;
            }
        }

        // ============================================
        // BACK RAY: +86° to +94° (centered at +90°)
        // ============================================
        int back_start = (int)((deg2rad(86.0) - angle_min_) / angle_increment_);
        int back_end   = (int)((deg2rad(94.0) - angle_min_) / angle_increment_);
        
        if (back_start < 0) back_start = 0;
        if (back_end >= num_readings) back_end = num_readings - 1;
        //if (back_start > back_end) std::swap(back_start, back_end);
       
        for (int i = back_start; i <= back_end; ++i) {
            float r = scan->ranges[i];
            if (std::isfinite(r) && r > 0.05f && r < laser_back_) {
                laser_back_ = r;
            }
        }

        // ============================================
        // Debug logging
        // ============================================
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "F=%.2f L=%.2f R=%.2f B=%.2f | LF=%.2f LB=%.2f RF=%.2f RB=%.2f",
            minLaserDistFront_, minLaserDistLeft_, minLaserDistRight_, laser_back_,
            laser_left_front_, laser_left_back_, laser_right_front_, laser_right_back_
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
            // Update bump memory
            last_bumped_frame_ = bumped_frame;
            
            // If strictly new bump or we were in a non-bump state
            if (current_state_ != State::BUMPED) {
                original_yaw_before_bump_sequence_ = yaw_;
                enterBumpedState();
            } else {
                // If we hit something *while* resolving a bump, restart the resolution
                bump_sub_state_ = BumpSubState::CHECK_SENSORS;
                stopRobot();
            }
        }
    }

    // --- State Transition Helpers ---
    void enterExplorationState() {
        RCLCPP_INFO(this->get_logger(), "State: EXPLORATION");
        current_state_ = State::EXPLORATION;
        stopRobot();
    }

    void enterWallFollowState() {
        RCLCPP_INFO(this->get_logger(), "State: WALL_FOLLOW");
        current_state_ = State::WALL_FOLLOW;
        wf_sub_state_ = WallFollowSubState::FOLLOWING;
        
        wall_follow_start_x_ = pos_x_;
        wall_follow_start_y_ = pos_y_;
        wall_follow_start_time_ = this->now();
        
        // Decide which wall to follow (closest one)
        if (minLaserDistLeft_ < minLaserDistRight_) {
            following_side_ = 1; // Left
            RCLCPP_INFO(this->get_logger(), "Target: Left Wall (Dist: %.2f)", minLaserDistLeft_);
        } else {
            following_side_ = -1; // Right
            RCLCPP_INFO(this->get_logger(), "Target: Right Wall (Dist: %.2f)", minLaserDistRight_);
        }
        stopRobot();
    }

    void enterBumpedState() {
        RCLCPP_INFO(this->get_logger(), "State: BUMPED");
        current_state_ = State::BUMPED;
        bump_sub_state_ = BumpSubState::CHECK_SENSORS;
        stopRobot();
    }

    void controlLoop()
    {
        if ((this->now() - start_time_).seconds() >= 1000.0) {
            stopRobot();
            rclcpp::shutdown();
            return;
        }

        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();

        switch (current_state_) {
            // ================= STATE 1: UNDOCKING =================
            case State::UNDOCKING:
            {
                static double start_x = pos_x_;
                static double start_y = pos_y_;
                static bool init = false;
                if (!init) { start_x = pos_x_; start_y = pos_y_; init = true; }

                if (undock_phase_ == 0) {
                    double dist = std::hypot(pos_x_ - start_x, pos_y_ - start_y);
                    if (dist < 0.3) {
                        vel.twist.linear.x = -0.1; 
                    } else {
                        undock_phase_ = 1;
                        undock_target_yaw_ = normalize_angle(yaw_ + M_PI);
                        stopRobot();
                    }
                } else {
                    double err = normalize_angle(undock_target_yaw_ - yaw_);
                    if (std::abs(err) > 0.1) vel.twist.angular.z = (err > 0) ? 0.5 : -0.5;
                    else enterExplorationState();
                }
                break;
            }

            // ================= STATE 2: EXPLORATION =================
            case State::EXPLORATION:
            {
                if (minLaserDistFront_ < 0.5) {
                    if (minLaserDistLeft_ < minLaserDistRight_) {
                        RCLCPP_INFO(this->get_logger(), "Obstacle ahead! Turning RIGHT");
                        vel.twist.angular.z = -deg2rad(90.0); // Turn in place
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Obstacle ahead! Turning LEFT");
                        vel.twist.angular.z = deg2rad(90.0); // Turn in place
                    }

                    enterWallFollowState();

                } else {
                    vel.twist.linear.x = 0.25;
                    
                }
                vel_pub_->publish(vel);
                break;
            }

            
            // ================= STATE 3: WALL FOLLOW =================
            case State::WALL_FOLLOW:
            {
                // =====================================================
                // STEP 1: Determine which wall we're following
                // =====================================================
                float wall_center;      // Direct perpendicular distance
                float wall_front;       // Front diagonal ray
                float wall_back;        // Back diagonal ray
                
                if (following_side_ == 1) {
                    // Following LEFT wall
                    wall_center = minLaserDistLeft_;
                    wall_front = laser_left_front_;
                    wall_back = laser_left_back_;
                } else {
                    // Following RIGHT wall
                    wall_center = minLaserDistRight_;
                    wall_front = laser_right_front_;
                    wall_back = laser_right_back_;
                }

                // =====================================================
                // STEP 2: Check if wall is still visible
                // =====================================================
                bool wall_exists = std::isfinite(wall_center) && wall_center < 1.5;
                
                if (!wall_exists) {
                    RCLCPP_WARN(this->get_logger(), "Wall lost! Returning to EXPLORATION");
                    enterExplorationState();
                    break;
                }

                // =====================================================
                // STEP 3: ALIGNING - Get parallel to wall
                // =====================================================
                /*
                if (wf_sub_state_ == WallFollowSubState::ALIGNING) {
                    
                    // Check if we have valid front and back readings
                    bool can_align = std::isfinite(wall_front) && std::isfinite(wall_back);
                    
                    if (!can_align) {
                        // Can't determine parallelism, skip to FOLLOWING
                        RCLCPP_WARN(this->get_logger(), "Cannot determine parallelism, starting FOLLOWING");
                        wf_sub_state_ = WallFollowSubState::FOLLOWING;
                        break;
                    }
                    
                    // Calculate parallelism error
                    // If front < back: we're angled INTO wall (need to turn away)
                    // If front > back: we're angled AWAY from wall (need to turn toward)
                    float parallelism_error = wall_front - wall_back;
                    
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(), *this->get_clock(), 500,
                        "ALIGNING %s: Front=%.2fm, Back=%.2fm, Error=%.3fm",
                        (following_side_ == 1) ? "LEFT" : "RIGHT",
                        wall_front, wall_back, parallelism_error
                    );
                    
                    // Threshold: within 5cm difference = good enough
                    if (std::abs(parallelism_error) > 0.05) {
                        // Still need to align
                        vel.twist.linear.x = 0.0;  // Don't move forward while aligning
                        
                        // Control logic:
                        // For LEFT wall (side=1):
                        //   - If front closer (error < 0): turn RIGHT (negative z)
                        //   - If back closer (error > 0): turn LEFT (positive z)
                        // For RIGHT wall (side=-1):
                        //   - If front closer (error < 0): turn LEFT (positive z)
                        //   - If back closer (error > 0): turn RIGHT (negative z)
                        
                        vel.twist.angular.z = -following_side_ * 3.0 * parallelism_error;
                        vel.twist.angular.z = std::clamp(vel.twist.angular.z, -0.5, 0.5);
                        
                    } else {
                        // Aligned!
                        RCLCPP_INFO(this->get_logger(), 
                            "✓ ALIGNED! Parallel to %s wall at %.2fm. Starting FOLLOWING.",
                            (following_side_ == 1) ? "LEFT" : "RIGHT",
                            wall_center
                        );
                        wf_sub_state_ = WallFollowSubState::FOLLOWING;
                        stopRobot();
                    }
                }
                */
                // =====================================================
                // STEP 4: FOLLOWING - Maintain distance while moving
                // =====================================================
                if (wf_sub_state_ == WallFollowSubState::FOLLOWING) 
                {
                    // Target distance from wall
                    const float DESIRED_DISTANCE = 0.40;  // 40cm from wall
                    
                    // ===== SUB-STEP 4A: Check front obstacle =====
                    if (minLaserDistFront_ < 0.35) {
                        // Obstacle ahead! Need to turn
                        RCLCPP_INFO(this->get_logger(), "Front obstacle! Turning away from wall");
                        
                        vel.twist.linear.x = 0.05;  // Slow forward movement
                        vel.twist.angular.z = following_side_ * 0.6;  // Turn away from wall
                        // (LEFT wall: turn right, RIGHT wall: turn left)
                    }
                    
                    // ===== SUB-STEP 4B: Detect corners =====
                    else if (std::isfinite(wall_back) && std::isinf(wall_front)) {
                        // OUTSIDE CORNER: Wall disappeared in front!
                        RCLCPP_INFO(this->get_logger(), "OUTSIDE CORNER detected! Turning toward wall");
                        
                        vel.twist.linear.x = 0.10;
                        vel.twist.angular.z = -following_side_ * 0.5;  // Turn toward wall
                        // (LEFT wall: turn left, RIGHT wall: turn right)
                    }
                    else if (std::isfinite(wall_front) && wall_front < 0.25 && wall_center > 0.35) {
                        // INSIDE CORNER: Wall suddenly very close in front!
                        RCLCPP_INFO(this->get_logger(), "INSIDE CORNER detected! Turning away from wall");
                        
                        vel.twist.linear.x = 0.10;
                        vel.twist.angular.z = following_side_ * 0.6;  // Turn away from wall
                    }
                    
                    // ===== SUB-STEP 4C: Normal wall following =====
                    else {
                        // Calculate control errors
                        float distance_error = DESIRED_DISTANCE - wall_center;
                        float parallelism_error = 0.0;
                        
                        // If we have front/back rays, use them for parallelism control
                        if (std::isfinite(wall_front) && std::isfinite(wall_back)) {
                            parallelism_error = wall_front - wall_back;
                        }
                        
                        // Move forward
                        vel.twist.linear.x = 0.15;
                        
                        // Combined P-controller:
                        // 1. Distance control: Keep DESIRED_DISTANCE from wall
                        // 2. Parallelism control: Stay parallel to wall
                        
                        float Kp_distance = 1.5;      // Gain for distance error
                        float Kp_parallel = 2.0;      // Gain for parallelism error
                        
                        // Distance control:
                        // - If too far (error > 0): turn toward wall
                        // - If too close (error < 0): turn away from wall
                        float angular_from_distance = -following_side_ * Kp_distance * distance_error;
                        
                        // Parallelism control:
                        // - If front closer (error < 0): turn away from wall
                        // - If back closer (error > 0): turn toward wall
                        float angular_from_parallel = -following_side_ * Kp_parallel * parallelism_error;
                        
                        // Combine both controllers
                        vel.twist.angular.z = angular_from_distance + angular_from_parallel;
                        vel.twist.angular.z = std::clamp(vel.twist.angular.z, -0.6, 0.6);
                        
                        RCLCPP_INFO_THROTTLE(
                            this->get_logger(), *this->get_clock(), 1000,
                            "Following: Dist=%.2fm(err=%.2f), Parallel=%.3fm, ω=%.2f",
                            wall_center, distance_error, parallelism_error, vel.twist.angular.z
                        );
                    }
                    
                    // ===== SUB-STEP 4D: Loop closure check =====
                    double elapsed_time = (this->now() - wall_follow_start_time_).seconds();
                    
                    if (elapsed_time > 10.0) {  // After 10 seconds minimum
                        double dist_to_start = std::hypot(
                            pos_x_ - wall_follow_start_x_, 
                            pos_y_ - wall_follow_start_y_
                        );
                        
                        if (dist_to_start < 0.4) {
                            RCLCPP_INFO(this->get_logger(), 
                                "Loop closure detected! Completed circuit in %.1f seconds", 
                                elapsed_time
                            );
                            
                            // Return to exploration (or stop, or turn, etc.)
                            enterExplorationState();
                        }
                    }
                }
                
                break;
            }
            /*
            case State::WALL_FOLLOW:
            {
                // Determine active distance and index
                float active_dist = (following_side_ == 1) ? minLaserDistLeft_ : minLaserDistRight_;
                int active_index  = (following_side_ == 1) ? minDistIndexLeft_ : minDistIndexRight_;

                if (wf_sub_state_ == WallFollowSubState::ALIGNING) {
                    // "Rotate such that the minimum distance is further minimized"
                    // Interpretation: Align robot so that the sensor reading the min distance is perpendicular (90 or -90).
                    
                    if (active_index == -1 || std::isinf(active_dist)) {
                        // Lost wall? Just switch to following to try and recover or drift
                        RCLCPP_WARN(this->get_logger(), "No wall detected, switching to FOLLOWING");
                        wf_sub_state_ = WallFollowSubState::FOLLOWING;
                        break;
                    }

                    // Calculate the angle of the minimum reading in the robot frame
                    double angle_of_min_scan = angle_min__ + active_index * angle_increment__;
                    
                    // Target: For left wall (0°), for right wall (-180° or π)
                    double target_angle;
                    if (following_side_ == 1) {
                        // Following LEFT wall - want minimum at 0° (directly left)
                        target_angle = 0.0;
                    } else {
                        // Following RIGHT wall - want minimum at ±π (directly right)
                        target_angle = M_PI;
                    }

                    double alignment_error = normalize_angle(target_angle - angle_of_min_scan);
    
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(), *this->get_clock(), 500,
                        "Aligning: min_angle=%.1f°, target=%.1f°, error=%.1f°",
                        rad2deg(angle_of_min_scan),
                        rad2deg(target_angle),
                        rad2deg(alignment_error)
                    );
    

                    if (std::abs(alignment_error) > deg2rad(5.0)) {
                        vel.twist.linear.x = 0.0;  // Don't move forward while aligning
                        vel.twist.angular.z = std::clamp(1.0 * alignment_error, -0.5, 0.5);
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Aligned to wall. Distance: %.2fm", active_dist);
                        wf_sub_state_ = WallFollowSubState::FOLLOWING;
                        stopRobot(); 
                    }
                } 
                else if (wf_sub_state_ == WallFollowSubState::FOLLOWING) 
                {
                    // Move forward and monitor target distance (0.5m)
                    double desired_dist = 0.5;
                    double error = desired_dist - active_dist;
                    if (std::isinf(active_dist)) error = 0.0;

                    vel.twist.linear.x = 0.15;
                    
                    // P-Controller
                    // If Left Wall (side=1): Too close (err>0) -> Turn Right (-z).
                    // If Right Wall (side=-1): Too close (err>0) -> Turn Left (+z).
                    // Formula: angular = -1 * side * Kp * error
                    
                    vel.twist.angular.z = -1.0 * following_side_ * 1.5 * error;
                    vel.twist.angular.z = std::clamp(vel.twist.angular.z, -0.5, 0.5);

                    // Loop Closure Check (after 5s)
                    if ((this->now() - wall_follow_start_time_).seconds() > 5.0) {
                        double d_start = std::hypot(pos_x_ - wall_follow_start_x_, pos_y_ - wall_follow_start_y_);
                        if (d_start < 0.3) {
                            RCLCPP_INFO(this->get_logger(), "Loop Closed. Resetting.");
                            // For simplicity, just enter exploration. (User asked for 160 deg turn previously, 
                            // assuming similar logic applies or just reset)
                            undock_phase_ = 1; // Hijack undock rotation logic
                            undock_target_yaw_ = normalize_angle(yaw_ + deg2rad(160.0));
                            current_state_ = State::UNDOCKING; // Go to rotation phase of undock
                        }
                    }
                }
                break;
            }
            */

            // ================= STATE 4: BUMPED =================
            case State::BUMPED:
            {
                switch (bump_sub_state_) {
                    case BumpSubState::CHECK_SENSORS:
                    {
                        if (minLaserDistFront_ < 0.1) {
                            enterWallFollowState();
                        } else {
                            // Prepare backing up
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
                            vel.twist.linear.x = -0.05; // Reverse
                        } else {
                            // Done backing up, calculate turn
                            stopRobot();
                            double turn_rad = deg2rad(60.0);

                            if (last_bumped_frame_.find("right") != std::string::npos) {
                                // Right Bumped -> Turn Left (+90)
                                bump_target_yaw_ = normalize_angle(yaw_ + turn_rad);
                                bump_turn_direction_ = 1.0; 
                            } else {
                                // Left/Center Bumped -> Turn Right (-90)
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
                        // "while moving forwards turn slight right" (if we turned left)
                        // Inverse logic: Arc direction = -1 * turn_direction
                        double yaw_diff = std::abs(normalize_angle(yaw_ - original_yaw_before_bump_sequence_));

                        if (yaw_diff < 0.1) {
                            enterExplorationState();
                        } else {
                            vel.twist.linear.x = 0.05;
                            // Apply slight turn in OPPOSITE direction of the 90 deg turn
                            vel.twist.angular.z = -1.0 * bump_turn_direction_ * 0.5;
                        }
                        break;
                    }
                }
                break;
            }
        }

        vel_pub_->publish(vel);
    }

    void stopRobot() {
        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        vel.twist.linear.x = 0.0; vel.twist.angular.z = 0.0;
        vel_pub_->publish(vel);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr hazard_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_;
    State current_state_;

    double pos_x_, pos_y_, yaw_;
    
    // Sensor Data
    std::string last_bumped_frame_;
    float minLaserDistFront_;
    float minLaserDistLeft_, minLaserDistRight_;
    int minDistIndexFront_;
    int minDistIndexLeft_, minDistIndexRight_;
    float angle_increment__, angle_min__;
    float laser_left_front_;
    float laser_left_back_;
    float laser_right_front_;
    float laser_right_back_;
    float laser_back_;
    // Undock
    int undock_phase_;
    double undock_target_yaw_;

    // Wall Follow
    WallFollowSubState wf_sub_state_;
    int following_side_; // 1 = Left, -1 = Right
    double wall_follow_start_x_, wall_follow_start_y_;
    rclcpp::Time wall_follow_start_time_;

    // Bump
    BumpSubState bump_sub_state_;
    double bump_start_x_, bump_start_y_;
    double original_yaw_before_bump_sequence_;
    double bump_target_yaw_;
    double bump_turn_direction_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
