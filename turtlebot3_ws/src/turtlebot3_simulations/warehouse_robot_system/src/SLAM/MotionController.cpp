// ============================================================================
// MTRX3760 Project 2 - 
// File: MotionController.cpp
// Description: Implementation of MotionController. Provides low-level robot
//              motion control with obstacle avoidance, precise positioning,
//              and adaptive navigation for warehouse automation tasks.
// Author(s): Dylan George
// Last Edited: 2025-11-02
// ============================================================================

#include "SLAM/MotionController.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>

MotionController::MotionController(rclcpp::Node::SharedPtr node)
    : node_(node),
      has_path_(false),
      enabled_(true),
      reversed_(false),
      alpha_(0.0),
      closest_distance_(std::numeric_limits<double>::infinity()),
      debug_mode_(false),
      max_drive_speed_(INSPECTION_LINEAR_SPEED),   // Default to inspection speeds
      max_turn_speed_(INSPECTION_ANGULAR_SPEED) {
    
    // Check if debug mode is enabled
    if (!node_->has_parameter("debug")) {
        node_->declare_parameter("debug", false);
    }
    debug_mode_ = node_->get_parameter("debug").as_bool();
    
    // Create publishers (publish TwistStamped to /cmd_vel for Gazebo bridge)
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    lookahead_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(
        "/motion/lookahead", 10
    );
    
    if (debug_mode_) {
        fov_cells_pub_ = node_->create_publisher<nav_msgs::msg::GridCells>(
            "/motion/fov_cells", 10
        );
        close_wall_cells_pub_ = node_->create_publisher<nav_msgs::msg::GridCells>(
            "/motion/close_wall_cells", 10
        );
    }
    
    RCLCPP_INFO(node_->get_logger(), "Motion Controller initialized");
}

void MotionController::setPath(const nav_msgs::msg::Path& path) {
    current_path_ = path;
    has_path_ = !path.poses.empty();
    
    if (has_path_) {
        auto goal = path.poses.back().pose.position;
        RCLCPP_INFO(node_->get_logger(), "Path set with %zu waypoints, goal at (%.2f, %.2f)", 
                   path.poses.size(), goal.x, goal.y);
    }
}

void MotionController::clearPath() {
    current_path_.poses.clear();
    has_path_ = false;
}

bool MotionController::hasPath() const {
    return has_path_ && !current_path_.poses.empty();
}

void MotionController::setEnabled(bool enabled) {
    enabled_ = enabled;
}

bool MotionController::isEnabled() const {
    return enabled_;
}

double MotionController::distance(double x0, double y0, double x1, double y1) {
    return std::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
}

double MotionController::getDistanceToWaypoint(const geometry_msgs::msg::Pose& pose, int index) const {
    if (index < 0 || index >= static_cast<int>(current_path_.poses.size())) {
        return -1.0;
    }
    
    const auto& waypoint = current_path_.poses[index].pose.position;
    return distance(pose.position.x, pose.position.y, waypoint.x, waypoint.y);
}

int MotionController::findNearestWaypointIndex(const geometry_msgs::msg::Pose& pose) const {
    int nearest_index = -1;
    double closest_dist = std::numeric_limits<double>::infinity();
    
    for (size_t i = 0; i < current_path_.poses.size(); i++) {
        double dist = getDistanceToWaypoint(pose, i);
        if (dist >= 0 && dist < closest_dist) {
            closest_dist = dist;
            nearest_index = i;
        }
    }
    
    return nearest_index;
}

geometry_msgs::msg::Point MotionController::findLookahead(
    const geometry_msgs::msg::Pose& pose, int nearest_waypoint_index) const {
    
    int i = nearest_waypoint_index;
    while (i < static_cast<int>(current_path_.poses.size()) &&
           getDistanceToWaypoint(pose, i) < LOOKAHEAD_DISTANCE) {
        i++;
    }
    
    if (i > 0 && i <= static_cast<int>(current_path_.poses.size())) {
        return current_path_.poses[i - 1].pose.position;
    }
    
    return geometry_msgs::msg::Point();
}

geometry_msgs::msg::Point MotionController::getGoal() const {
    if (current_path_.poses.empty()) {
        return geometry_msgs::msg::Point();
    }
    return current_path_.poses.back().pose.position;
}

bool MotionController::isAtGoal() const {
    return !has_path_;
}

double MotionController::calculateSteeringAdjustment(
    const geometry_msgs::msg::Pose& pose,
    const nav_msgs::msg::OccupancyGrid& map) {
    
    // Get robot yaw
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double yaw_deg = yaw * 180.0 / M_PI;
    
    // Get robot grid cell
    GridCell robot_cell = PathPlanner::worldToGrid(map, pose.position);
    
    double weighted_sum_of_angles = 0.0;
    double total_weight = 0.0;
    closest_distance_ = std::numeric_limits<double>::infinity();
    
    std::vector<GridCell> fov_cells, wall_cells;
    int wall_cell_count = 0;
    
    // Scan area around robot
    for (int dx = -FOV_DISTANCE; dx <= FOV_DISTANCE; dx++) {
        for (int dy = -FOV_DISTANCE; dy <= FOV_DISTANCE; dy++) {
            GridCell cell = {robot_cell.first + dx, robot_cell.second + dy};
            double dist = PathPlanner::euclideanDistance(robot_cell, cell);
            
            if (!PathPlanner::isCellInBounds(map, cell)) {
                continue;
            }
            
            bool is_wall = !PathPlanner::isCellWalkable(map, cell);
            if (is_wall && dist < closest_distance_) {
                closest_distance_ = dist;
            }
            
            // Calculate angle relative to robot
            double angle = std::atan2(dy, dx) * 180.0 / M_PI - yaw_deg;
            if (reversed_) {
                angle += 180.0;
            }
            
            // Normalize angle to [-180, 180]
            while (angle < -180.0) angle += 360.0;
            while (angle > 180.0) angle -= 360.0;
            
            // Check if in FOV
            bool is_in_fov = (dist <= FOV_DISTANCE &&
                             angle >= -FOV / 2.0 && angle <= FOV / 2.0 &&
                             std::abs(angle) >= FOV_DEADZONE / 2.0);
            bool is_in_small_fov = (dist <= SMALL_FOV_DISTANCE &&
                                   angle >= -SMALL_FOV / 2.0 && angle <= SMALL_FOV / 2.0);
            
            if (!is_in_fov && !is_in_small_fov) {
                continue;
            }
            
            if (debug_mode_) {
                fov_cells.push_back(cell);
            }
            
            if (!is_wall) {
                continue;
            }
            
            double weight = (dist != 0) ? 1.0 / (dist * dist) : 0.0;
            weighted_sum_of_angles += weight * angle;
            total_weight += weight;
            wall_cell_count++;
            
            if (debug_mode_) {
                wall_cells.push_back(cell);
            }
        }
    }
    
    // Publish debug visualization
    if (debug_mode_) {
        if (fov_cells_pub_) {
            auto grid_cells = PathPlanner::getGridCells(map, fov_cells);
            grid_cells.header.stamp = node_->now();
            fov_cells_pub_->publish(grid_cells);
        }
        if (close_wall_cells_pub_) {
            auto grid_cells = PathPlanner::getGridCells(map, wall_cells);
            grid_cells.header.stamp = node_->now();
            close_wall_cells_pub_->publish(grid_cells);
        }
    }
    
    if (total_weight == 0.0 || wall_cell_count == 0) {
        return 0.0;
    }
    
    double average_angle = weighted_sum_of_angles / total_weight;
    return -OBSTACLE_AVOIDANCE_GAIN * average_angle / wall_cell_count;
}

geometry_msgs::msg::Twist MotionController::computeVelocityCommand(
    const geometry_msgs::msg::Pose& current_pose,
    const nav_msgs::msg::OccupancyGrid& map) {
    
    geometry_msgs::msg::Twist cmd_vel;
    
    if (!enabled_ || !hasPath()) {
        // Publish zero velocity as TwistStamped
        geometry_msgs::msg::TwistStamped cmd_vel_stamped;
        cmd_vel_stamped.header.stamp = node_->now();
        cmd_vel_stamped.header.frame_id = "base_footprint";
        cmd_vel_stamped.twist = cmd_vel;
        cmd_vel_pub_->publish(cmd_vel_stamped);
        return cmd_vel;
    }
    
    // Find nearest waypoint and lookahead point
    int nearest_idx = findNearestWaypointIndex(current_pose);
    if (nearest_idx < 0) {
        // Publish zero velocity as TwistStamped
        geometry_msgs::msg::TwistStamped cmd_vel_stamped;
        cmd_vel_stamped.header.stamp = node_->now();
        cmd_vel_stamped.header.frame_id = "base_footprint";
        cmd_vel_stamped.twist = cmd_vel;
        cmd_vel_pub_->publish(cmd_vel_stamped);
        return cmd_vel;
    }
    
    geometry_msgs::msg::Point lookahead = findLookahead(current_pose, nearest_idx);
    geometry_msgs::msg::Point goal = getGoal();
    
    // If lookahead is empty (all waypoints too close), use the goal directly
    if (lookahead.x == 0.0 && lookahead.y == 0.0 && lookahead.z == 0.0) {
        lookahead = goal;
        RCLCPP_DEBUG(node_->get_logger(), "Using goal as lookahead (all waypoints within lookahead distance)");
    }
    
    // Publish lookahead for visualization
    if (lookahead_pub_) {
        geometry_msgs::msg::PointStamped lookahead_msg;
        lookahead_msg.header.frame_id = "map";
        lookahead_msg.header.stamp = node_->now();
        lookahead_msg.point = lookahead;
        lookahead_pub_->publish(lookahead_msg);
    }
    
    // Calculate alpha (angle to lookahead)
    tf2::Quaternion q(
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    double dx = lookahead.x - current_pose.position.x;
    double dy = lookahead.y - current_pose.position.y;
    alpha_ = std::atan2(dy, dx) - yaw;
    
    // Normalize alpha
    while (alpha_ > M_PI) alpha_ -= 2.0 * M_PI;
    while (alpha_ < -M_PI) alpha_ += 2.0 * M_PI;
    
    // Check if reversed
    reversed_ = std::abs(alpha_) > M_PI / 2.0;
    
    // Calculate drive speed
    double lookahead_distance = distance(current_pose.position.x, current_pose.position.y,
                                        lookahead.x, lookahead.y);
    
    // Prevent division by zero or very small values
    double sin_alpha = std::sin(alpha_);
    if (std::abs(sin_alpha) < 0.01) {
        sin_alpha = (sin_alpha >= 0) ? 0.01 : -0.01;
    }
    
    double radius_of_curvature = lookahead_distance / (2.0 * sin_alpha);
    
    double drive_speed = (reversed_ ? -1.0 : 1.0) * max_drive_speed_;
    
    // Check if at goal
    double distance_to_goal = distance(current_pose.position.x, current_pose.position.y,
                                      goal.x, goal.y);
    if (distance_to_goal < DISTANCE_TOLERANCE) {
        RCLCPP_INFO(node_->get_logger(), "Reached goal! Distance: %.3fm", distance_to_goal);
        clearPath();
        // Publish zero velocity as TwistStamped
        geometry_msgs::msg::TwistStamped cmd_vel_stamped;
        cmd_vel_stamped.header.stamp = node_->now();
        cmd_vel_stamped.header.frame_id = "base_footprint";
        cmd_vel_stamped.twist = cmd_vel;
        cmd_vel_pub_->publish(cmd_vel_stamped);
        return cmd_vel;
    }
    
    // Calculate turn speed
    double turn_speed = TURN_SPEED_KP * drive_speed / radius_of_curvature;
    
    // Add obstacle avoidance
    turn_speed += calculateSteeringAdjustment(current_pose, map);
    
    // Clamp turn speed
    turn_speed = std::clamp(turn_speed, -max_turn_speed_, max_turn_speed_);
    
    // Slow down if close to obstacle
    if (closest_distance_ < OBSTACLE_AVOIDANCE_MAX_SLOW_DOWN_DISTANCE) {
        double slow_factor = std::clamp(
            (closest_distance_ - OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_DISTANCE) /
            (OBSTACLE_AVOIDANCE_MAX_SLOW_DOWN_DISTANCE - OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_DISTANCE),
            OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_FACTOR, 1.0
        );
        drive_speed *= slow_factor;
    }
    
    cmd_vel.linear.x = drive_speed;
    cmd_vel.angular.z = turn_speed;
    
    // Publish the velocity command as TwistStamped
    geometry_msgs::msg::TwistStamped cmd_vel_stamped;
    cmd_vel_stamped.header.stamp = node_->now();
    cmd_vel_stamped.header.frame_id = "base_footprint";
    cmd_vel_stamped.twist = cmd_vel;
    cmd_vel_pub_->publish(cmd_vel_stamped);
    
    return cmd_vel;
}

// ============================================================================
// Speed Configuration
// ============================================================================

void MotionController::setMaxSpeeds(double linear_speed, double angular_speed) {
    max_drive_speed_ = linear_speed;
    max_turn_speed_ = angular_speed;
    RCLCPP_INFO(node_->get_logger(), 
               "Motion speeds set: linear=%.3f m/s, angular=%.3f rad/s", 
               linear_speed, angular_speed);
}

void MotionController::setInspectionSpeeds() {
    setMaxSpeeds(INSPECTION_LINEAR_SPEED, INSPECTION_ANGULAR_SPEED);
    RCLCPP_INFO(node_->get_logger(), "Using INSPECTION speed profile (slow for AprilTag detection)");
}

void MotionController::setDeliverySpeeds() {
    setMaxSpeeds(DELIVERY_LINEAR_SPEED, DELIVERY_ANGULAR_SPEED);
    RCLCPP_INFO(node_->get_logger(), "Using DELIVERY speed profile (fast)");
}

void MotionController::setExplorationSpeeds() {
    setMaxSpeeds(EXPLORATION_LINEAR_SPEED, EXPLORATION_ANGULAR_SPEED);
    RCLCPP_INFO(node_->get_logger(), "Using EXPLORATION speed profile (fast)");
}
