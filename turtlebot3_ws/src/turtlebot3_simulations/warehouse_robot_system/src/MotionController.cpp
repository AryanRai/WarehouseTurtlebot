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
#include "MotionController.hpp"
#include <cmath>

namespace slam {

MotionController::MotionController() : Node("motion_controller"),
                        has_path_(false),
                        goal_reached_(false) 
{
    RCLCPP_INFO(this->get_logger(), "Motion Controller starting...");
    
    // Configuration
    config_.max_linear_velocity = 0.12;
    config_.max_angular_velocity = 0.6;
    config_.lookahead_distance = 0.30;
    config_.goal_tolerance = 0.50;
    config_.stuck_distance_threshold = 0.03;
    config_.stuck_timeout = 4.0;
    
    // Subscribers - receive path and pose
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/planned_path", 10,
        std::bind(&MotionController::pathCallback, this, std::placeholders::_1));
    
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot_pose", 10,
        std::bind(&MotionController::poseCallback, this, std::placeholders::_1));
    
    // Publishers - send velocity commands and status
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/motion_status", 10);
    
    // Control timer
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&MotionController::controlLoop, this));
    
    last_movement_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Motion Controller initialized");
}


void MotionController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) 
{
    if (msg->poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty path");
        return;
    }
    
    current_path_ = msg;
    has_path_ = true;
    goal_reached_ = false;
    
    // Extract goal from last pose
    current_goal_ = msg->poses.back().pose.position;
    last_distance_to_goal_ = std::numeric_limits<double>::max();
    last_movement_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Received new path with %zu waypoints", msg->poses.size());
}

void MotionController::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{
    current_pose_ = msg;
}

void MotionController::controlLoop() 
{
    if (!has_path_ || !current_pose_ || goal_reached_) {
        stopRobot();
        return;
    }
    
    // Check if goal reached
    if (isGoalReached()) {
        stopRobot();
        goal_reached_ = true;
        has_path_ = false;
        publishStatus("goal_reached");
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        return;
    }
    
    // Check if stuck
    if (isRobotStuck()) {
        stopRobot();
        publishStatus("stuck");
        RCLCPP_WARN(this->get_logger(), "Robot stuck, attempting recovery");
        performRecovery();
        return;
    }
    
    // Follow path using pure pursuit
    followPath();
}

void MotionController::followPath() 
{
    if (!current_path_ || current_path_->poses.empty()) {
        return;
    }
    
    // Calculate lookahead point
    geometry_msgs::msg::Point lookahead = calculateLookaheadPoint();
    
    // Calculate pure pursuit control
    calculatePurePursuitControl(lookahead);
    
    // Update progress tracking
    updateProgressTracking();
}

geometry_msgs::msg::Point MotionController::calculateLookaheadPoint() 
{
    if (current_path_->poses.empty()) {
        return current_goal_;
    }
    
    geometry_msgs::msg::Point current_pos = current_pose_->pose.position;
    
    // Find closest point on path
    double min_distance = std::numeric_limits<double>::max();
    size_t nearest_index = 0;
    
    for (size_t i = 0; i < current_path_->poses.size(); ++i) {
        double dist = calculateDistance(current_pos, current_path_->poses[i].pose.position);
        if (dist < min_distance) {
            min_distance = dist;
            nearest_index = i;
        }
    }
    
    // Find lookahead point: first point beyond lookahead_distance
    size_t lookahead_index = nearest_index;
    for (size_t i = nearest_index; i < current_path_->poses.size(); ++i) {
        double dist = calculateDistance(current_pos, current_path_->poses[i].pose.position);
        if (dist >= config_.lookahead_distance) {
            lookahead_index = i;
            break;
        }
        lookahead_index = i;
    }
    
    return current_path_->poses[lookahead_index].pose.position;
}

void MotionController::calculatePurePursuitControl(const geometry_msgs::msg::Point& lookahead) 
{
    geometry_msgs::msg::Point current_pos = current_pose_->pose.position;
    double current_yaw = calculateYawFromPose(current_pose_);
    
    // Calculate vector to lookahead point
    double dx = lookahead.x - current_pos.x;
    double dy = lookahead.y - current_pos.y;
    double lookahead_dist = std::sqrt(dx*dx + dy*dy);
    
    if (lookahead_dist < 0.01) {
        stopRobot();
        return;
    }
    
    // Calculate target angle and alpha (angle error)
    double target_yaw = std::atan2(dy, dx);
    double alpha = target_yaw - current_yaw;
    
    // Normalize alpha to [-pi, pi]
    while (alpha > M_PI) alpha -= 2.0 * M_PI;
    while (alpha < -M_PI) alpha += 2.0 * M_PI;
    
    // Always drive forward
    double linear_vel = config_.max_linear_velocity;
    
    // Pure pursuit formula
    double sin_alpha = std::sin(alpha);
    if (std::abs(sin_alpha) < 0.01) {
        sin_alpha = (sin_alpha >= 0) ? 0.01 : -0.01;
    }
    
    double angular_vel = (2.0 * linear_vel * sin_alpha) / lookahead_dist;
    
    // Clamp angular velocity
    angular_vel = std::clamp(angular_vel, 
                            -config_.max_angular_velocity, 
                            config_.max_angular_velocity);
    
    // Adaptive speed reduction for sharp turns
    double abs_alpha = std::abs(alpha);
    if (abs_alpha > 1.2) {
        linear_vel *= 0.25;
    } else if (abs_alpha > 0.8) {
        linear_vel *= 0.4;
    } else if (abs_alpha > 0.5) {
        linear_vel *= 0.6;
    } else if (abs_alpha > 0.3) {
        linear_vel *= 0.8;
    }
    
    publishVelocityCommand(linear_vel, angular_vel);
    
    RCLCPP_DEBUG(this->get_logger(), "Pure pursuit: alpha=%.3f, linear=%.3f, angular=%.3f",
                alpha, linear_vel, angular_vel);
}

bool MotionController::isGoalReached() 
{
    if (!current_pose_) return false;
    
    double distance = calculateDistance(current_pose_->pose.position, current_goal_);
    return distance < config_.goal_tolerance;
}

bool MotionController::isRobotStuck() 
{
    if (!current_pose_) return false;
    
    double current_distance = calculateDistance(current_pose_->pose.position, current_goal_);
    
    // Check if robot has made progress
    if (std::abs(current_distance - last_distance_to_goal_) > config_.stuck_distance_threshold) {
        last_movement_time_ = this->now();
        last_distance_to_goal_ = current_distance;
        return false;
    }
    
    // Check if stuck for too long
    if ((this->now() - last_movement_time_).seconds() > config_.stuck_timeout) {
        RCLCPP_WARN(this->get_logger(), "Stuck detected: no progress for %.1f seconds",
                    config_.stuck_timeout);
        return true;
    }
    
    return false;
}

void MotionController::updateProgressTracking() 
{
    if (!current_pose_) return;
    
    double current_distance = calculateDistance(current_pose_->pose.position, current_goal_);
    
    // Update if making progress
    if (std::abs(current_distance - last_distance_to_goal_) > config_.stuck_distance_threshold) {
        last_movement_time_ = this->now();
    }
    
    last_distance_to_goal_ = current_distance;
}

void MotionController::performRecovery() 
{
    static rclcpp::Time recovery_start = this->now();
    static int recovery_phase = 0;
    const double recovery_duration = 3.0;
    
    double elapsed = (this->now() - recovery_start).seconds();
    
    if (recovery_phase == 0) {
        // Back up
        if (elapsed < recovery_duration * 0.4) {
            publishVelocityCommand(-0.15, 0.0);
        } else {
            recovery_phase = 1;
            recovery_start = this->now();
        }
    } else if (recovery_phase == 1) {
        // Turn
        if (elapsed < recovery_duration * 0.4) {
            publishVelocityCommand(0.0, 1.0);
        } else {
            recovery_phase = 2;
            recovery_start = this->now();
        }
    } else {
        // Move forward briefly
        if (elapsed < recovery_duration * 0.2) {
            publishVelocityCommand(0.1, 0.0);
        } else {
            stopRobot();
            recovery_phase = 0;
            last_movement_time_ = this->now();
            publishStatus("recovery_complete");
            RCLCPP_INFO(this->get_logger(), "Recovery complete");
        }
    }
}

void MotionController::publishVelocityCommand(double linear, double angular) 
{
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_footprint";
    cmd.twist.linear.x = linear;
    cmd.twist.angular.z = angular;
    cmd_vel_pub_->publish(cmd);
}

void MotionController::stopRobot() 
{
    publishVelocityCommand(0.0, 0.0);
}

void MotionController::publishStatus(const std::string& status) 
{
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
}

double MotionController::calculateDistance(const geometry_msgs::msg::Point& p1,
                        const geometry_msgs::msg::Point& p2) 
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

double MotionController::calculateYawFromPose(const geometry_msgs::msg::PoseStamped::SharedPtr& pose) 
{
    tf2::Quaternion q(
        pose->pose.orientation.x,
        pose->pose.orientation.y,
        pose->pose.orientation.z,
        pose->pose.orientation.w
    );
    
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}


} // namespace slam