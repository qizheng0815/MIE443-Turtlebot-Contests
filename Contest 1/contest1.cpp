#include <chrono>
#include <memory>
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

inline double rad2deg(double rad){return rad * (180.0 / M_PI);}

inline double deg2rad(double deg){return deg * (M_PI / 180.0);}


class Contest1Node : public rclcpp::Node
{
public:
    Contest1Node()
        : Node("contest1_node")
    {
        // Initialize publisher for velocity commands
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

        // Timer for main control loop at 10 Hz
        timer_ = this->create_wall_timer(
            100ms, std::bind(&Contest1Node::controlLoop, this));

        // Initialize variables
        start_time_ = this->now();
        angular_ = 0.0;
        linear_ = 0.0;

        bumpers_["bump_front_left"]   = false;
        bumpers_["bump_front_center"] = false;
        bumpers_["bump_front_right"]  = false;
        bumpers_["bump_left"]         = false;
        bumpers_["bump_right"]        = false;

        pos_x_ = 0.0;
        pos_y_ = 0.0;
        yaw_ = 0.0;
        minLaserDist_ = std::numeric_limits<float>::infinity();
        nLasers_ = 0;
        desiredNLasers_ = 0;
        desiredAngle_ = 5;

        RCLCPP_INFO(this->get_logger(), "Contest 1 node initialized. Running for 1000 seconds.");

        state_ = docked ? State::UNDOCKING : State::EXPLORATION;

        RCLCPP_INFO(this->get_logger(),
                "Initial state: %s",
                docked ? "UNDOCKING" : "EXPLORATION");
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "Laser FOV: angle_min=%.3f rad (%.1f deg), angle_max=%.3f rad (%.1f deg), inc=%.5f rad, N=%zu",
    scan->angle_min, rad2deg(scan->angle_min),
    scan->angle_max, rad2deg(scan->angle_max),
    scan->angle_increment, scan->ranges.size());
      //(void) scan;  // implement your code here
      nLasers_ = (scan->angle_max - scan->angle_min) / scan->angle_increment;
      laserRange_= scan->ranges;
      desiredNLasers_= deg2rad(desiredAngle_)/(scan->angle_increment);
      //RCLCPP_INFO(this->get_logger(), "Size of laser scan array: %d, and size of offset %d", nLasers_, desiredNLasers_);
        /*
      //Accounting for the LiDAR 90 degree offset
        float laser_offset = deg2rad(-90.0);
        uint32_t front_idx = (laser_offset - scan->angle_min) / scan->angle_increment;

        minLaserDist_ = std::numeric_limits<float>::infinity();

        // Find minimum distance in front center

        if (deg2rad(desiredAngle_) < scan-> angle_max && deg2rad(desiredAngle_) > scan-> angle_min)
        {
            for (uint32_t laser_idx = front_idx - desiredNLasers_; laser_idx <= front_idx + desiredNLasers_; ++laser_idx) {
                minLaserDist_ = std::min(minLaserDist_, laserRange_[laser_idx]);
            }
        }
        else 
        {
            for(uint32_t laser_idx = 0; laser_idx < nLasers_; ++laser_idx) {
                minLaserDist_ = std::min(minLaserDist_, laserRange_[laser_idx]);
            }
        }
            */
        laserRange_ = scan->ranges;
        int N = (int)laserRange_.size();
        if (N == 0) return;

        // window size from your desiredAngle_ (± desiredAngle_)
        int half_window = (int)(deg2rad(desiredAngle_) / scan->angle_increment);
        if (half_window < 1) half_window = 1;

        // We define "front" as angle 0 rad in the scan frame.
        // We define "right" as -90 deg in the scan frame.
        // (These are the *usual* conventions; if it’s flipped in your sim, we’ll swap signs.)
        float front_ang = deg2rad(-90.0f);
        float right_ang = deg2rad(-180.0f);  // or +180
        float left_ang  = deg2rad(0.0f);
        float back_ang  = deg2rad(90.0f);
        // Convert angle -> index
        int front_idx = (int)((front_ang - scan->angle_min) / scan->angle_increment);
        int right_idx = (int)((right_ang - scan->angle_min) / scan->angle_increment);
        int left_idx  = (int)((left_ang  - scan->angle_min) / scan->angle_increment);
        int back_idx  = (int)((back_ang  - scan->angle_min) / scan->angle_increment);

        // Clamp
        if (front_idx < 0) front_idx = 0;
        if (front_idx > N-1) front_idx = N-1;
        if (right_idx < 0) right_idx = 0;
        if (right_idx > N-1) right_idx = N-1;
        if (left_idx < 0) left_idx = 0;
        if (left_idx > N-1) left_idx = N-1;
        if (back_idx < 0) back_idx = 0;
        if (back_idx > N-1) back_idx = N-1;

        // Compute min in each window
        float frontDist = std::numeric_limits<float>::infinity();
        float rightDist = std::numeric_limits<float>::infinity();
        float leftDist  = std::numeric_limits<float>::infinity();
        float backDist  = std::numeric_limits<float>::infinity();

        int f0 = std::max(0, front_idx - half_window);
        int f1 = std::min(N-1, front_idx + half_window);
        int r0 = std::max(0, right_idx - half_window);
        int r1 = std::min(N-1, right_idx + half_window);
        int l0 = std::max(0, left_idx - half_window);
        int l1 = std::min(N-1, left_idx + half_window);
        int b0 = std::max(0, back_idx - half_window);
        int b1 = std::min(N-1, back_idx + half_window);


        for (int i = f0; i <= f1; i++) {
            float d = laserRange_[i];
            if (std::isfinite(d) && d > 0.01f) frontDist = std::min(frontDist, d);
        }
        for (int i = r0; i <= r1; i++) {
            float d = laserRange_[i];
            if (std::isfinite(d) && d > 0.01f) rightDist = std::min(rightDist, d);
        }
        for (int i = l0; i <= l1; i++) {
            float d = laserRange_[i];
            if (std::isfinite(d) && d > 0.01f) leftDist = std::min(leftDist, d);
        }
        for (int i = b0; i <= b1; i++) {
            float d = laserRange_[i];
            if (std::isfinite(d) && d > 0.01f) backDist = std::min(backDist, d);
        }

        // Store for control loop
        frontDist_ = frontDist;
        rightDist_ = rightDist;
        leftDist_  = leftDist;
        backDist_  = backDist;

        // Keep your old variable working
        minLaserDist_ = std::min(
            std::min(frontDist_, rightDist_),
            std::min(leftDist_, backDist_)
        ); 

        RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "Front=%.2f  Left=%.2f  Right=%.2f  Back=%.2f  MIN=%.2f",
        frontDist_, leftDist_, rightDist_, backDist, minLaserDist_);

        /*
        RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,   // ms
        "FOV deg: [%.1f, %.1f], frontDist=%.2f rightDist=%.2f",
        rad2deg(scan->angle_min),
        rad2deg(scan->angle_max),

        frontDist_,
        rightDist_);
        */
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        // implement your code here
        pos_x_ = odom->pose.pose.position.x;
        pos_y_ = odom->pose.pose.position.y;

        //Extract yaw from quaternion using tf2
        yaw_ = tf2::getYaw(odom->pose.pose.orientation);

        //RCLCPP_INFO(this->get_logger(), "Position: (%.2f, %.2f), Orientation: %f rad or %f deg", pos_x_, pos_y_, yaw_, rad2deg(yaw_));
        odom_ready_ = true;
    }

    void hazardCallback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr hazard_vector)
    {
        // implement your code here
        for (auto& [key, val] : bumpers_) {
            val = false; // reset all bumpers to false
        }

        for (const auto& detection : hazard_vector->detections) {
            if (detection.type == irobot_create_msgs::msg::HazardDetection::BUMP) {
                bumpers_[detection.header.frame_id] = true;
                //RCLCPP_INFO(this->get_logger(), "Bumper pressed: %s",
                            //detection.header.frame_id.c_str());
            }
        }
    }
    void controlLoop()
    {
        // Calculate elapsed time
        auto current_time = this->now();
        double seconds_elapsed = (current_time - start_time_).seconds();

        // Check if 480 seconds (8 minutes) have elapsed
        if (seconds_elapsed >= 10000000000.0) {
            RCLCPP_INFO(this->get_logger(), "Contest time completed (1000 seconds). Stopping robot.");

            // Stop the robot
            geometry_msgs::msg::TwistStamped vel;
            vel.header.stamp = this->now();
            vel.twist.linear.x = 0.0;
            vel.twist.angular.z = 0.0;
            vel_pub_->publish(vel);

            // Shutdown the node
            rclcpp::shutdown();
            return;
        }

        //
        // ==================== Exploration Code =====================
    linear_  = 0.0f;
    angular_ = 0.0f;

    // ------------------------------------------------------------
    // 2) STARTUP STATE: back off 1.0 m from dock using odom
    // ------------------------------------------------------------
    
    
    // Don’t do anything until odom is valid (prevents backing off from (0,0))
    if (!odom_ready_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Waiting for odom...");
        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        vel.twist.linear.x = 0.0;
        vel.twist.angular.z = 0.0;
        vel_pub_->publish(vel);
        return;
    }
    
    docked = false; // (you can implement dock detection if desired)

    if (docked == true) {
        state_ = State::UNDOCKING;
    }

    else {
        state_ = State::EXPLORATION;
    }

        // Latch starting pose ONCE
        if (!backoff_init_) {
            backoff_x0_ = pos_x_;
            backoff_y0_ = pos_y_;
            backoff_init_ = true;
            RCLCPP_INFO(this->get_logger(), "STARTUP_BACKOFF: begin (target = 1.0 m)");
        }

        // Compute traveled distance from starting pose
        double dx = pos_x_ - backoff_x0_;
        double dy = pos_y_ - backoff_y0_;
        double dist = std::sqrt(dx*dx + dy*dy);

        const double BACKOFF_DIST = 1.0;  // meters
        const float  V_START      = -0.20f; // m/s reverse (looks clear in sim)
        const float  W_START      = M_PI/16;   // not keep straight (or 0.2f for gentle arc)

        if (dist < BACKOFF_DIST) {
            // Keep backing up until we reach 1.0 m
            linear_  = V_START;
            angular_ = W_START;

            geometry_msgs::msg::TwistStamped vel;
            vel.header.stamp = this->now();
            vel.twist.linear.x = linear_;
            vel.twist.angular.z = angular_;
            vel_pub_->publish(vel);

            return; // IMPORTANT: don’t run obstacle logic yet
        }

        // Done: switch to exploration
        state_ = State::EXPLORATION;
        // RCLCPP_INFO(this->get_logger(), "STARTUP_BACKOFF done -> EXPLORE");
        // fall through into EXPLORE this tick (or you can return after publishing stop)
        
        //  ============================================================
        // Implement your exploration code here


        // Bumper handling
        bool any_bumper_pressed = false; 
        for (const auto& [key, val] : bumpers_) {
            //RCLCPP_INFO(this->get_logger(), "Position: (%.2f, %.2f), Orientation: %f rad or %f deg, Minimum laser distance: %.2f", pos_x_, pos_y_, yaw_, rad2deg(yaw_), minLaserDist_);
            if (val) {
                any_bumper_pressed = true;
                //angular_ = M_PI *10; // Turn left on bumper press
                //linear_ = -1000;
                //RCLCPP_INFO(this->get_logger(), "Bumper pressed, turning left.");
                break;
            }
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),10000,   // ms10000,
                "Pose:(%.2f, %.2f) yaw=%.2f rad (%.1f deg)  minLaser=%.2f  bumper=%d",
                pos_x_, pos_y_, yaw_, rad2deg(yaw_), minLaserDist_, any_bumper_pressed ? 1 : 0);


        const float V_FAST      = 0.30f; // m/s: cruise speed in open space
        const float V_CREEP     = 0.08f; // m/s: slow forward while turning near wall
        const float V_BACKOFF   = -0.15f;// m/s: reverse on bumper

        const float W_TURN      = 1.40f; // rad/s: quick turning while creeping
        const float W_SPIN      = 1.80f; // rad/s: fast spin-in-place when very close
        const float W_BUMPER    = 1.60f; // rad/s: strong turn when bumper hit

        // Distance thresholds (react early)
        const float D_SOFT      = 0.70f; // m: start steering away before wall
        const float D_HARD      = 0.35f; // m: too close -> spin in place
        //Added: 0.7 minimum Laser distance to move forward
        // ============================================================
        // Reactive decision logic
        
        if (any_bumper_pressed) {
            // ---- Bumper pressed: back off + turn hard (escape) ----
            // Note: This is still "reactive" (no duration). If you want
            
            linear_  = V_BACKOFF;
            angular_ = W_BUMPER;
        }
        else if (minLaserDist_ < D_HARD) {
            // ---- Very close to obstacle: rotate quickly in place ----
            linear_  = 0.0f;
            angular_ = W_SPIN;
        }
        else if (minLaserDist_ < D_SOFT) {
            // ---- Approaching obstacle: creep forward while turning away ----
            linear_  = V_CREEP;
            angular_ = W_TURN;
        }
        else {
            // ---- Clear: drive fast and straight ----
            linear_  = V_FAST;
            angular_ = 0.0f;
        }


        // Set velocity command
        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        vel.twist.linear.x = linear_;
        vel.twist.angular.z = angular_;

        // Publish velocity command
        vel_pub_->publish(vel);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr hazard_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    
    float angular_;
    float linear_;
    double pos_x_;
    double pos_y_;
    double yaw_;
    std::map<std::string, bool> bumpers_;
    float minLaserDist_;
    int32_t nLasers_;
    int32_t desiredNLasers_;
    int32_t desiredAngle_;
    std::vector<float> laserRange_;

    // --- State machine for startup ---
    enum class State { UNDOCKING, EXPLORATION, WALL_FOLLOW, BUMPED };
    State state_ = State::EXPLORATION;

    bool docked = true;
    bool odom_ready_ = false;

    // Lidar Distances
    float frontDist_ = std::numeric_limits<float>::infinity();
    float leftDist_ = std::numeric_limits<float>::infinity();
    float rightDist_ = std::numeric_limits<float>::infinity();
    float backDist_ = std::numeric_limits<float>::infinity();

    // undocking memory
    bool undock_init_ = false;
    double undock_x0_ = 0.0;
    double undock_y0_ = 0.0;

    //Bumped memory
    bool bumped_init_ = false;
    double loop_x0_ = 0.0;
    double loop_y0_ = 0.0;
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    double traveled_ = 0.0;
    bool backoff_init_ = false;
    double backoff_x0_ = 0.0;
    double backoff_y0_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
