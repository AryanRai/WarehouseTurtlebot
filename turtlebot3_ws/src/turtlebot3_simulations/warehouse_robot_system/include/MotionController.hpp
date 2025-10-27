// MTRX3760 2025 Project 2: Warehouse Robot
// File: MotionController.cpp
// Author(s): Aryan Rai
//
// Motion Controller - Follows planned paths using pure pursuit control

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

namespace slam {

class MotionController : public rclcpp::Node {
public:
    MotionController();

private:
    struct Config {
        double max_linear_velocity;
        double max_angular_velocity;
        double lookahead_distance;
        double goal_tolerance;
        double stuck_distance_threshold;
        double stuck_timeout;
    } config_;
    
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    void controlLoop();
    
    void followPath();
    
    geometry_msgs::msg::Point calculateLookaheadPoint();
    
    void calculatePurePursuitControl(const geometry_msgs::msg::Point& lookahead);
    
    bool isGoalReached();
    
    bool isRobotStuck();
    
    void updateProgressTracking();
    
    void performRecovery();
    
    void publishVelocityCommand(double linear, double angular);
    
    void stopRobot();
    
    void publishStatus(const std::string& status);
    
    double calculateDistance(const geometry_msgs::msg::Point& p1,
                           const geometry_msgs::msg::Point& p2);
    
    double calculateYawFromPose(const geometry_msgs::msg::PoseStamped::SharedPtr& pose);
    
    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // State
    nav_msgs::msg::Path::SharedPtr current_path_;
    geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
    geometry_msgs::msg::Point current_goal_;
    bool has_path_;
    bool goal_reached_;
    
    // Progress tracking
    double last_distance_to_goal_;
    rclcpp::Time last_movement_time_;
};

} // namespace slam