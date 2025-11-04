// ============================================================================
// MTRX3760 Project 2 - 
// File: WarehouseRobot.cpp
// Description: Implementation of WarehouseRobot base class. Provides common
//              functionality for delivery and inspection robots including
//              TSP optimization, docking, relocalization, and navigation utilities.
// Author(s): Dylan George
// Last Edited: 2025-11-02
// ============================================================================

#include "Robot/WarehouseRobot.hpp"
#include <cmath>
#include <random>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

WarehouseRobot::WarehouseRobot(rclcpp::Node::SharedPtr node)
    : node_(node),
      in_docking_mode_(false),
      in_target_docking_mode_(false),
      initial_yaw_(0.0),
      has_relocalized_(false),
      relocalization_start_time_(node->now()),
      use_tsp_optimization_(false),
      path_completed_(false),
      total_distance_(0.0),
      battery_level_(100.0f),
      battery_low_threshold_(-1.0),
      battery_monitoring_enabled_(false),
      low_battery_return_triggered_(false)
{
    // Initialize SLAM components
    slam_controller_ = std::make_unique<SlamController>(node);
    motion_controller_ = std::make_unique<MotionController>(node);
    
    // Create status publisher
    status_pub_ = node_->create_publisher<std_msgs::msg::String>("/robot_status", 10);
    
    RCLCPP_INFO(node_->get_logger(), "WarehouseRobot base class initialized");
}

bool WarehouseRobot::hasValidMap() const {
    return slam_controller_ && slam_controller_->hasValidMap();
}

// ============================================================================
// TSP Route Optimization
// ============================================================================

std::vector<std::vector<double>> WarehouseRobot::buildDistanceMatrix(
    const geometry_msgs::msg::Point& start,
    const std::vector<geometry_msgs::msg::Point>& points) {
    
    size_t n = points.size();
    std::vector<std::vector<double>> matrix(n, std::vector<double>(n, 0.0));
    
    auto current_map = slam_controller_->getCurrentMap();
    if (!current_map) {
        RCLCPP_ERROR(node_->get_logger(), "No map available for distance matrix calculation!");
        return matrix;
    }
    
    // Precompute C-space and cost map once
    auto [cspace, cspace_cells] = PathPlanner::calcCSpace(*current_map, false);
    cv::Mat cost_map = PathPlanner::calcCostMap(*current_map);
    
    // Calculate distance between every pair of points using A*
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            GridCell start_cell = PathPlanner::worldToGrid(*current_map, points[i]);
            GridCell goal_cell = PathPlanner::worldToGrid(*current_map, points[j]);
            
            auto [path, cost, actual_start, actual_goal] = 
                PathPlanner::aStar(cspace, cost_map, start_cell, goal_cell);
            
            if (!path.empty()) {
                // Use actual path cost from A*
                matrix[i][j] = cost;
                matrix[j][i] = cost;  // Symmetric
            } else {
                // If no path found, use Euclidean distance as fallback (with penalty)
                double dx = points[j].x - points[i].x;
                double dy = points[j].y - points[i].y;
                double euclidean = std::sqrt(dx*dx + dy*dy);
                matrix[i][j] = euclidean * 10.0;  // Heavy penalty for unreachable
                matrix[j][i] = euclidean * 10.0;
                
                RCLCPP_WARN(node_->get_logger(), 
                           "No A* path found between points %zu and %zu, using penalized Euclidean distance",
                           i, j);
            }
        }
    }
    
    return matrix;
}

std::vector<int> WarehouseRobot::simulatedAnnealing(
    const std::vector<std::vector<double>>& distance_matrix,
    int start_idx,
    double initial_temp,
    double cooling_rate,
    int max_iterations) {
    
    int n = distance_matrix.size();
    if (n <= 1) {
        return {start_idx};
    }
    
    // Initialize with greedy nearest neighbor tour
    std::vector<int> current_tour;
    std::vector<bool> visited(n, false);
    current_tour.push_back(start_idx);
    visited[start_idx] = true;
    
    int current_city = start_idx;
    for (int i = 1; i < n; ++i) {
        double min_dist = std::numeric_limits<double>::max();
        int nearest = -1;
        
        for (int j = 0; j < n; ++j) {
            if (!visited[j] && distance_matrix[current_city][j] < min_dist) {
                min_dist = distance_matrix[current_city][j];
                nearest = j;
            }
        }
        
        if (nearest != -1) {
            current_tour.push_back(nearest);
            visited[nearest] = true;
            current_city = nearest;
        }
    }
    
    std::vector<int> best_tour = current_tour;
    double current_cost = calculateTourCost(current_tour, distance_matrix);
    double best_cost = current_cost;
    
    double temperature = initial_temp;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    
    int improvements = 0;
    
    // Simulated Annealing main loop
    for (int iter = 0; iter < max_iterations; ++iter) {
        // Generate neighbor solution using 2-opt swap
        std::vector<int> new_tour = current_tour;
        
        // Select two random positions (not including start)
        std::uniform_int_distribution<> pos_dis(1, n - 1);
        int i = pos_dis(gen);
        int j = pos_dis(gen);
        
        if (i > j) std::swap(i, j);
        if (i == j) continue;
        
        // Reverse segment between i and j (2-opt move)
        std::reverse(new_tour.begin() + i, new_tour.begin() + j + 1);
        
        double new_cost = calculateTourCost(new_tour, distance_matrix);
        double delta = new_cost - current_cost;
        
        // Accept or reject new solution
        if (delta < 0 || dis(gen) < std::exp(-delta / temperature)) {
            current_tour = new_tour;
            current_cost = new_cost;
            
            if (current_cost < best_cost) {
                best_tour = current_tour;
                best_cost = current_cost;
                improvements++;
            }
        }
        
        // Cool down
        temperature *= cooling_rate;
        
        // Log progress periodically
        if (iter % 1000 == 0 && iter > 0) {
            RCLCPP_DEBUG(node_->get_logger(), 
                        "SA iteration %d: best_cost=%.2f, current_cost=%.2f, temp=%.2f",
                        iter, best_cost, current_cost, temperature);
        }
    }
    
    RCLCPP_INFO(node_->get_logger(), 
               "Simulated Annealing complete: %d improvements, final cost: %.2fm",
               improvements, best_cost);
    
    return best_tour;
}

double WarehouseRobot::calculateTourCost(
    const std::vector<int>& tour,
    const std::vector<std::vector<double>>& distance_matrix) {
    
    if (tour.size() <= 1) {
        return 0.0;
    }
    
    double total_cost = 0.0;
    for (size_t i = 0; i < tour.size() - 1; ++i) {
        total_cost += distance_matrix[tour[i]][tour[i + 1]];
    }
    
    // Add return to start (home)
    total_cost += distance_matrix[tour.back()][tour[0]];
    
    return total_cost;
}

// ============================================================================
// Navigation and Docking
// ============================================================================

bool WarehouseRobot::performRelocalization() {
    auto current_pose = slam_controller_->getCurrentPose();
    auto current_map = slam_controller_->getCurrentMap();
    
    double elapsed = (node_->now() - relocalization_start_time_).seconds();
    
    // Store initial orientation on first call (when elapsed is very small)
    if (elapsed < 0.1) {
        tf2::Quaternion q(
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, initial_yaw_);
        
        // Check if we're too close to walls before starting relocalization
        if (current_map) {
            double min_distance_to_wall = checkMinDistanceToWalls(current_pose.position, *current_map);
            
            if (min_distance_to_wall < 0.5) {  // Less than 50cm from wall
                RCLCPP_WARN(node_->get_logger(), 
                           "Too close to wall (%.2fm) for safe relocalization spin - skipping", 
                           min_distance_to_wall);
                has_relocalized_ = true;
                return true;  // Skip relocalization
            }
        }
        
        RCLCPP_INFO(node_->get_logger(), "Starting relocalization spin (360°)...");
        relocalization_start_time_ = node_->now();
        elapsed = 0.0;
    }
    
    if (elapsed < RELOCALIZATION_DURATION) {
        // Spin in place for relocalization
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = node_->now();
        cmd_vel.header.frame_id = "base_footprint";
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = RELOCALIZATION_SPEED;
        
        auto cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        cmd_vel_pub->publish(cmd_vel);
        
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "Relocalizing... %.1fs remaining", RELOCALIZATION_DURATION - elapsed);
        return false;  // Still spinning
    } else {
        // Stop spinning
        stopMotion();
        
        has_relocalized_ = true;
        RCLCPP_INFO(node_->get_logger(), "✓ Relocalization complete!");
        return true;  // Complete
    }
}

void WarehouseRobot::returnToHome() {
    if (!slam_controller_->hasValidMap() || !slam_controller_->hasValidPose()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                            "Waiting for valid map and pose to return home...");
        return;
    }
    
    auto current_map = slam_controller_->getCurrentMap();
    auto current_pose = slam_controller_->getCurrentPose();
    
    geometry_msgs::msg::Point home_position;
    home_position.x = 0.0;
    home_position.y = 0.0;
    home_position.z = 0.0;
    
    // Check distance to home
    double dx = home_position.x - current_pose.position.x;
    double dy = home_position.y - current_pose.position.y;
    double distance_to_home = std::sqrt(dx * dx + dy * dy);
    
    // Check if reached home with tight tolerance (5cm)
    if (distance_to_home < HOME_TOLERANCE) {
        // Get current orientation
        tf2::Quaternion q(
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, current_yaw;
        m.getRPY(roll, pitch, current_yaw);
        
        // Calculate angle difference from initial orientation
        double angle_diff = initial_yaw_ - current_yaw;
        while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
        
        // If not aligned with initial orientation, rotate to match
        if (std::abs(angle_diff) > 0.05) {  // ~3 degrees tolerance
            geometry_msgs::msg::TwistStamped cmd_vel;
            cmd_vel.header.stamp = node_->now();
            cmd_vel.header.frame_id = "base_footprint";
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.angular.z = (angle_diff > 0) ? 0.2 : -0.2;
            
            auto cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
            cmd_vel_pub->publish(cmd_vel);
            
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                                "Aligning to initial orientation (%.1f° remaining)", 
                                std::abs(angle_diff) * 180.0 / M_PI);
            return;
        }
        
        // Stop all motion completely
        stopMotion();
        
        // Give time for stop command to take effect
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        
        RCLCPP_INFO(node_->get_logger(), "✓ At home position (%.3fm, aligned).", distance_to_home);
        return;
    }
    
    // Enter precise docking mode when close to home (within 50cm)
    if (distance_to_home < DOCKING_DISTANCE) {
        // Check if there's a clear path to home
        bool has_clear_path = hasLineOfSight(current_pose.position, home_position, *current_map);
        
        if (has_clear_path) {
            if (!in_docking_mode_) {
                RCLCPP_INFO(node_->get_logger(), "Entering precise docking mode (%.3fm from home, clear path)", 
                           distance_to_home);
                in_docking_mode_ = true;
                motion_controller_->clearPath();
            }
            
            preciseDocking(current_pose, distance_to_home);
            return;
        } else {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                "Close to home (%.2fm) but obstacle detected - continuing with path planner", 
                                distance_to_home);
            in_docking_mode_ = false;
        }
    }
    
    // Reset docking mode if we're far from home
    if (in_docking_mode_ && distance_to_home >= DOCKING_DISTANCE) {
        in_docking_mode_ = false;
    }
    
    // Execute motion if we have a path
    if (motion_controller_->hasPath()) {
        motion_controller_->computeVelocityCommand(current_pose, *current_map);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                            "Returning home: %.2fm remaining", distance_to_home);
        return;
    }
    
    // Need to plan path home
    RCLCPP_INFO(node_->get_logger(), "Planning path home (%.2fm)...", distance_to_home);
    
    GridCell start = PathPlanner::worldToGrid(*current_map, current_pose.position);
    GridCell goal = PathPlanner::worldToGrid(*current_map, home_position);
    
    auto [cspace, cspace_cells] = PathPlanner::calcCSpace(*current_map, false);
    cv::Mat cost_map = PathPlanner::calcCostMap(*current_map);
    
    auto [path, cost, actual_start, actual_goal] = 
        PathPlanner::aStar(cspace, cost_map, start, goal);
    
    if (path.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot find path home!");
        return;
    }
    
    auto path_msg = PathPlanner::pathToMessage(*current_map, path);
    motion_controller_->setPath(path_msg);
    RCLCPP_INFO(node_->get_logger(), "Path to home planned with %zu waypoints", path_msg.poses.size());
    publishStatus("Returning home");
}

void WarehouseRobot::preciseDocking(
    const geometry_msgs::msg::Pose& current_pose, 
    double distance_to_home) {
    
    geometry_msgs::msg::Point home_position;
    home_position.x = 0.0;
    home_position.y = 0.0;
    
    // Calculate direction to home
    double dx = home_position.x - current_pose.position.x;
    double dy = home_position.y - current_pose.position.y;
    double angle_to_home = std::atan2(dy, dx);
    
    // Get current robot orientation
    tf2::Quaternion q(
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // Calculate angle difference
    double angle_diff = angle_to_home - yaw;
    // Normalize to [-pi, pi]
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
    
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = node_->now();
    cmd_vel.header.frame_id = "base_footprint";
    
    // If angle is off by more than 10 degrees, rotate first
    if (std::abs(angle_diff) > 0.175) {  // ~10 degrees
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = (angle_diff > 0) ? DOCKING_ANGULAR_SPEED : -DOCKING_ANGULAR_SPEED;
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "Docking: Aligning (angle error: %.1f°)", angle_diff * 180.0 / M_PI);
    } else {
        // Move forward slowly toward home
        cmd_vel.twist.linear.x = DOCKING_LINEAR_SPEED;
        cmd_vel.twist.angular.z = angle_diff * 0.5;  // Gentle correction
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "Docking: Moving forward (%.3fm remaining)", distance_to_home);
    }
    
    // Publish directly (bypass motion controller during docking)
    auto cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    cmd_vel_pub->publish(cmd_vel);
}

void WarehouseRobot::preciseTargetDocking(
    const geometry_msgs::msg::Pose& current_pose,
    const geometry_msgs::msg::Point& target_position,
    double distance_to_target,
    const std::string& target_name) {
    
    // Calculate direction to target
    double dx = target_position.x - current_pose.position.x;
    double dy = target_position.y - current_pose.position.y;
    double angle_to_target = std::atan2(dy, dx);
    
    // Get current robot orientation
    double yaw = getYawFromQuaternion(current_pose.orientation);
    
    // Calculate angle difference
    double angle_diff = angle_to_target - yaw;
    // Normalize to [-pi, pi]
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
    
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = node_->now();
    cmd_vel.header.frame_id = "base_footprint";
    
    // If angle is off by more than 10 degrees, rotate first
    if (std::abs(angle_diff) > 0.175) {  // ~10 degrees
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = (angle_diff > 0) ? DOCKING_ANGULAR_SPEED : -DOCKING_ANGULAR_SPEED;
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "Target Docking: Aligning to %s (angle error: %.1f°)", 
                            target_name.c_str(), angle_diff * 180.0 / M_PI);
    } else {
        // Move forward slowly toward target
        cmd_vel.twist.linear.x = DOCKING_LINEAR_SPEED;
        cmd_vel.twist.angular.z = angle_diff * 0.5;  // Gentle correction
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "Target Docking: Moving to %s (%.3fm remaining)", 
                            target_name.c_str(), distance_to_target);
    }
    
    // Publish directly (bypass motion controller during docking)
    auto cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    cmd_vel_pub->publish(cmd_vel);
}

bool WarehouseRobot::hasLineOfSight(
    const geometry_msgs::msg::Point& from, 
    const geometry_msgs::msg::Point& to,
    const nav_msgs::msg::OccupancyGrid& map) {
    
    // Use Bresenham's line algorithm to check if path is clear
    GridCell start = PathPlanner::worldToGrid(map, from);
    GridCell goal = PathPlanner::worldToGrid(map, to);
    
    int x0 = start.first;
    int y0 = start.second;
    int x1 = goal.first;
    int y1 = goal.second;
    
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    // Robot radius in grid cells (TurtleBot3 is ~0.178m diameter, so ~0.09m radius)
    // With 0.05m resolution, that's ~2 cells radius
    const int robot_radius_cells = 2;
    
    while (true) {
        // Check if current cell and surrounding cells are walkable (robot footprint)
        for (int dx_check = -robot_radius_cells; dx_check <= robot_radius_cells; ++dx_check) {
            for (int dy_check = -robot_radius_cells; dy_check <= robot_radius_cells; ++dy_check) {
                // Only check cells within robot's circular footprint
                if (dx_check * dx_check + dy_check * dy_check <= robot_radius_cells * robot_radius_cells) {
                    GridCell check_cell = {x0 + dx_check, y0 + dy_check};
                    if (!PathPlanner::isCellInBounds(map, check_cell) || 
                        !PathPlanner::isCellWalkable(map, check_cell)) {
                        return false;  // Obstacle would hit robot body
                    }
                }
            }
        }
        
        if (x0 == x1 && y0 == y1) {
            break;  // Reached goal
        }
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
    
    return true;  // Clear path with robot footprint considered
}

double WarehouseRobot::checkMinDistanceToWalls(
    const geometry_msgs::msg::Point& position,
    const nav_msgs::msg::OccupancyGrid& map) {
    
    GridCell center = PathPlanner::worldToGrid(map, position);
    
    double min_distance = std::numeric_limits<double>::max();
    
    // Check in 8 directions around the robot
    std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},  // Cardinal
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}  // Diagonal
    };
    
    for (const auto& [dx, dy] : directions) {
        // Cast ray in this direction until we hit an obstacle
        for (int step = 1; step < 100; ++step) {  // Max 100 cells (~5m at 0.05m resolution)
            GridCell check_cell = {center.first + dx * step, center.second + dy * step};
            
            if (!PathPlanner::isCellInBounds(map, check_cell)) {
                break;  // Hit map boundary
            }
            
            if (!PathPlanner::isCellWalkable(map, check_cell)) {
                // Found obstacle - calculate distance
                geometry_msgs::msg::Point obstacle_pos = PathPlanner::gridToWorld(map, check_cell);
                double dist_x = obstacle_pos.x - position.x;
                double dist_y = obstacle_pos.y - position.y;
                double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);
                
                min_distance = std::min(min_distance, distance);
                break;
            }
        }
    }
    
    return min_distance;
}

// ============================================================================
// Battery Monitoring
// ============================================================================

void WarehouseRobot::initializeBatteryMonitoring(double threshold) {
    battery_low_threshold_ = threshold;
    battery_monitoring_enabled_ = (threshold >= 0.0);
    low_battery_return_triggered_ = false;
    
    if (battery_monitoring_enabled_) {
        RCLCPP_INFO(node_->get_logger(), "🔋 Battery monitoring ENABLED - Low threshold: %.1f%%", threshold);
        
        // Subscribe to battery state
        battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
            "/battery_state", 10,
            std::bind(&WarehouseRobot::onBatteryState, this, std::placeholders::_1));
    } else {
        RCLCPP_INFO(node_->get_logger(), "🔋 Battery monitoring DISABLED");
    }
}

void WarehouseRobot::onBatteryState(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    // Calculate percentage from voltage if not provided
    if (std::isnan(msg->percentage)) {
        // For 3S LiPo: 12.6V = 100%, 10.5V = 0%
        battery_level_ = ((msg->voltage - 10.5) / (12.6 - 10.5)) * 100.0f;
        battery_level_ = std::max(0.0f, std::min(100.0f, battery_level_));
    } else {
        battery_level_ = msg->percentage;
    }
    
    // Log battery level periodically
    static auto last_log_time = node_->now();
    if ((node_->now() - last_log_time).seconds() > 30.0) {
        RCLCPP_INFO(node_->get_logger(), "🔋 Battery: %.1f%%", battery_level_);
        last_log_time = node_->now();
    }
}

bool WarehouseRobot::checkLowBattery() {
    if (!battery_monitoring_enabled_ || low_battery_return_triggered_) {
        return false;
    }
    
    if (battery_level_ < battery_low_threshold_) {
        RCLCPP_WARN(node_->get_logger(), 
                   "⚠️ LOW BATTERY DETECTED! (%.1f%% < %.1f%%)", 
                   battery_level_, battery_low_threshold_);
        RCLCPP_WARN(node_->get_logger(), 
                   "🚨 EMERGENCY RETURN TO HOME INITIATED!");
        
        low_battery_return_triggered_ = true;
        emergencyReturnHome();
        return true;
    }
    
    return false;
}

void WarehouseRobot::emergencyReturnHome() {
    // Default implementation - derived classes can override
    RCLCPP_WARN(node_->get_logger(), "Emergency return home triggered - returning to base");
    publishStatus("EMERGENCY: Low battery - returning home");
    returnToHome();
}

// ============================================================================
// Utility Methods
// ============================================================================

void WarehouseRobot::publishStatus(const std::string& status) {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
}

std::string WarehouseRobot::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

double WarehouseRobot::calculateDistance(
    const geometry_msgs::msg::Point& p1,
    const geometry_msgs::msg::Point& p2) const {
    
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

double WarehouseRobot::getYawFromQuaternion(
    const geometry_msgs::msg::Quaternion& orientation) const {
    
    tf2::Quaternion q(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void WarehouseRobot::stopMotion() {
    geometry_msgs::msg::TwistStamped stop_cmd;
    stop_cmd.header.stamp = node_->now();
    stop_cmd.header.frame_id = "base_footprint";
    stop_cmd.twist.linear.x = 0.0;
    stop_cmd.twist.linear.y = 0.0;
    stop_cmd.twist.linear.z = 0.0;
    stop_cmd.twist.angular.x = 0.0;
    stop_cmd.twist.angular.y = 0.0;
    stop_cmd.twist.angular.z = 0.0;
    
    auto cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    cmd_vel_pub->publish(stop_cmd);
}