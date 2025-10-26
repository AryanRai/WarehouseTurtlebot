// MTRX3760 2025 Project 2: Navigation Controller
// File: navigation_controller.cpp
// Author(s): Aryan Rai
//
// Implementation of navigation and motion control

#include "navigation_controller.hpp"
#include <cmath>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace slam {

NavigationController::NavigationController(rclcpp::Node* node, const Config& config)
    : node_(node),
      config_(config),
      last_distance_to_goal_(std::numeric_limits<double>::max()),
      recovery_attempt_(0)
{
    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Create velocity publisher
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", 10);
    
    // Initialize timestamps
    last_movement_time_ = node_->now();
    recovery_start_time_ = node_->now();
    
    current_goal_.x = 0.0;
    current_goal_.y = 0.0;
    current_goal_.z = 0.0;
}

bool NavigationController::updateCurrentPose() {
    try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            "map", "base_footprint", tf2::TimePointZero);
        
        current_pose_.header.frame_id = "map";
        current_pose_.header.stamp = node_->now();
        current_pose_.pose.position.x = transform.transform.translation.x;
        current_pose_.pose.position.y = transform.transform.translation.y;
        current_pose_.pose.position.z = transform.transform.translation.z;
        current_pose_.pose.orientation = transform.transform.rotation;
        
        return true;
    } catch (tf2::TransformException& ex) {
        RCLCPP_DEBUG(node_->get_logger(), "Could not get transform: %s", ex.what());
        return false;
    }
}

geometry_msgs::msg::Point NavigationController::getCurrentPosition() const {
    return poseToPoint(current_pose_);
}

double NavigationController::getCurrentYaw() const {
    return calculateYaw(current_pose_);
}

void NavigationController::setCurrentPath(const nav_msgs::msg::Path& path) {
    current_path_ = path;
}

void NavigationController::setCurrentGoal(const geometry_msgs::msg::Point& goal) {
    current_goal_ = goal;
    last_distance_to_goal_ = calculateDistance(getCurrentPosition(), goal);
    last_movement_time_ = node_->now();
}

bool NavigationController::isGoalReached(const geometry_msgs::msg::Point& goal, double tolerance) const {
    double distance = calculateDistance(getCurrentPosition(), goal);
    return distance < tolerance;
}

bool NavigationController::isCurrentGoalReached() const {
    return isGoalReached(current_goal_, config_.goal_tolerance);
}

void NavigationController::publishVelocityCommand(double linear, double angular) {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = node_->now();
    cmd.header.frame_id = "base_footprint";
    cmd.twist.linear.x = linear;
    cmd.twist.angular.z = angular;
    cmd_vel_pub_->publish(cmd);
    
    RCLCPP_DEBUG(node_->get_logger(), "Published velocity: linear=%.3f, angular=%.3f", 
                 linear, angular);
}

void NavigationController::stopRobot() {
    publishVelocityCommand(0.0, 0.0);
}

bool NavigationController::followPath() {
    if (current_path_.poses.empty()) {
        RCLCPP_WARN(node_->get_logger(), "No path to follow");
        return false;
    }

    geometry_msgs::msg::Point current_pos = getCurrentPosition();
    geometry_msgs::msg::Point lookahead = calculateLookaheadPoint();
    double dist_to_goal = calculateDistance(current_pos, current_goal_);
    
    calculatePurePursuitControl(lookahead);
    
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                "Following path: %zu poses, lookahead=(%.2f, %.2f), goal_dist=%.2f",
                current_path_.poses.size(), lookahead.x, lookahead.y, dist_to_goal);
    return true;
}

geometry_msgs::msg::Point NavigationController::calculateLookaheadPoint() const {
    if (current_path_.poses.empty()) {
        return current_goal_;
    }

    geometry_msgs::msg::Point current_pos = getCurrentPosition();
    
    // Find closest point on path
    double min_distance = std::numeric_limits<double>::max();
    size_t nearest_index = 0;
    
    for (size_t i = 0; i < current_path_.poses.size(); ++i) {
        double dist = calculateDistance(current_pos, current_path_.poses[i].pose.position);
        if (dist < min_distance) {
            min_distance = dist;
            nearest_index = i;
        }
    }

    // Find lookahead point
    size_t lookahead_index = nearest_index;
    for (size_t i = nearest_index; i < current_path_.poses.size(); ++i) {
        double dist = calculateDistance(current_pos, current_path_.poses[i].pose.position);
        if (dist >= config_.lookahead_distance) {
            lookahead_index = i;
            break;
        }
        lookahead_index = i;
    }

    return current_path_.poses[lookahead_index].pose.position;
}

void NavigationController::calculatePurePursuitControl(const geometry_msgs::msg::Point& lookahead) {
    geometry_msgs::msg::Point current_pos = getCurrentPosition();
    double current_yaw = getCurrentYaw();

    double dx = lookahead.x - current_pos.x;
    double dy = lookahead.y - current_pos.y;
    double lookahead_dist = std::sqrt(dx*dx + dy*dy);
    
    if (lookahead_dist < 0.01) {
        stopRobot();
        return;
    }
    
    double target_yaw = std::atan2(dy, dx);
    double alpha = target_yaw - current_yaw;
    
    // Normalize alpha to [-pi, pi]
    while (alpha > M_PI) alpha -= 2.0 * M_PI;
    while (alpha < -M_PI) alpha += 2.0 * M_PI;
    
    double linear_vel = config_.max_linear_velocity;
    double sin_alpha = std::sin(alpha);
    
    if (std::abs(sin_alpha) < 0.01) {
        sin_alpha = (sin_alpha >= 0) ? 0.01 : -0.01;
    }
    
    double angular_vel = (2.0 * linear_vel * sin_alpha) / lookahead_dist;
    angular_vel = std::clamp(angular_vel, -config_.max_angular_velocity, 
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
    
    RCLCPP_DEBUG(node_->get_logger(), 
                 "Pure pursuit: alpha=%.3f (%.1f°), dist=%.3f, linear=%.3f, angular=%.3f",
                 alpha, alpha * 180.0 / M_PI, lookahead_dist, linear_vel, angular_vel);
}

bool NavigationController::isRobotStuck() {
    double current_distance = calculateDistance(getCurrentPosition(), current_goal_);
    
    if (std::abs(current_distance - last_distance_to_goal_) > config_.stuck_distance_threshold) {
        last_movement_time_ = node_->now();
        last_distance_to_goal_ = current_distance;
        return false;
    }
    
    if ((node_->now() - last_movement_time_).seconds() > config_.stuck_timeout) {
        RCLCPP_WARN(node_->get_logger(), 
                    "Stuck detected: no progress for %.1f seconds (dist: %.3f)",
                    config_.stuck_timeout, current_distance);
        return true;
    }
    
    return false;
}

void NavigationController::resetStuckDetection() {
    last_movement_time_ = node_->now();
    last_distance_to_goal_ = calculateDistance(getCurrentPosition(), current_goal_);
}

void NavigationController::executeRecoveryBehavior() {
    const double recovery_duration = 3.0;
    double elapsed = (node_->now() - recovery_start_time_).seconds();
    
    if (elapsed < recovery_duration * 0.4) {
        publishVelocityCommand(-0.15, 0.0);
    } else if (elapsed < recovery_duration * 0.8) {
        publishVelocityCommand(0.0, 1.0);
    } else if (elapsed < recovery_duration) {
        publishVelocityCommand(0.1, 0.0);
    } else {
        stopRobot();
        recovery_attempt_++;
        recovery_start_time_ = node_->now();
        RCLCPP_INFO(node_->get_logger(), "Recovery complete (attempt %d)", recovery_attempt_);
    }
}

double NavigationController::calculateDistance(const geometry_msgs::msg::Point& p1,
                                               const geometry_msgs::msg::Point& p2) const {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

double NavigationController::calculateYaw(const geometry_msgs::msg::PoseStamped& pose) const {
    tf2::Quaternion q(
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w
    );
    
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

geometry_msgs::msg::Point NavigationController::poseToPoint(
    const geometry_msgs::msg::PoseStamped& pose) const {
    return pose.pose.position;
}

} // namespace slam