// MTRX3760 2025 Project 2: Navigation Controller
// File: navigation_controller.hpp
// Author(s): Aryan Rai
//
// Handles path following and low-level robot control

#ifndef NAVIGATION_CONTROLLER_HPP
#define NAVIGATION_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace slam {

/**
 * @class NavigationController
 * @brief Handles robot motion control, path following, and pose tracking
 */
class NavigationController {
public:
    struct Config {
        double goal_tolerance;
        double origin_tolerance;
        double max_linear_velocity;
        double max_angular_velocity;
        double lookahead_distance;
        double stuck_distance_threshold;
        double stuck_timeout;
        
        Config() 
            : goal_tolerance(0.30),
              origin_tolerance(0.2),
              max_linear_velocity(0.12),
              max_angular_velocity(0.6),
              lookahead_distance(0.30),
              stuck_distance_threshold(0.1),
              stuck_timeout(6.0) {}
    };

    NavigationController(rclcpp::Node* node, const Config& config = Config());

    // Pose management
    bool updateCurrentPose();
    geometry_msgs::msg::PoseStamped getCurrentPose() const { return current_pose_; }
    geometry_msgs::msg::Point getCurrentPosition() const;
    double getCurrentYaw() const;
    
    // Path management
    void setCurrentPath(const nav_msgs::msg::Path& path);
    nav_msgs::msg::Path getCurrentPath() const { return current_path_; }
    bool hasPath() const { return !current_path_.poses.empty(); }
    void clearPath() { current_path_.poses.clear(); }
    
    // Goal management
    void setCurrentGoal(const geometry_msgs::msg::Point& goal);
    geometry_msgs::msg::Point getCurrentGoal() const { return current_goal_; }
    bool isGoalReached(const geometry_msgs::msg::Point& goal, double tolerance) const;
    bool isCurrentGoalReached() const;
    
    // Motion control
    void publishVelocityCommand(double linear, double angular);
    void stopRobot();
    bool followPath();
    
    // Stuck detection
    bool isRobotStuck();
    void resetStuckDetection();
    
    // Recovery behaviors
    void executeRecoveryBehavior();
    
    // Utility calculations
    double calculateDistance(const geometry_msgs::msg::Point& p1,
                           const geometry_msgs::msg::Point& p2) const;
    double calculateYaw(const geometry_msgs::msg::PoseStamped& pose) const;

private:
    rclcpp::Node* node_;
    Config config_;
    
    // ROS publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    
    // TF2 for pose tracking
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // Navigation state
    geometry_msgs::msg::PoseStamped current_pose_;
    nav_msgs::msg::Path current_path_;
    geometry_msgs::msg::Point current_goal_;
    
    // Stuck detection
    double last_distance_to_goal_;
    rclcpp::Time last_movement_time_;
    
    // Recovery state
    rclcpp::Time recovery_start_time_;
    int recovery_attempt_;
    
    // Pure pursuit control
    geometry_msgs::msg::Point calculateLookaheadPoint() const;
    void calculatePurePursuitControl(const geometry_msgs::msg::Point& lookahead);
    
    // Helper functions
    geometry_msgs::msg::Point poseToPoint(const geometry_msgs::msg::PoseStamped& pose) const;
};

} // namespace slam

#endif // NAVIGATION_CONTROLLER_HPP