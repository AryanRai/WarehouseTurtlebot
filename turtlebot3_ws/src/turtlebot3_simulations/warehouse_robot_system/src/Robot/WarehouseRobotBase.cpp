// ============================================================================
// MTRX3760 Project 2 - 
// File: WarehouseRobotBase.cpp
// Description: Implementation of Robot base class. Provides common
//              functionality for delivery and inspection robots including
//              TSP optimization, docking, and navigation utilities.
//              Inherits from rclcpp::Node for ROS2 integration.
// Author(s): Dylan George
// Last Edited: 2025-11-02
// ============================================================================

#include "Robot/WarehouseRobotBase.hpp"
#include <cmath>
#include <random>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

Robot::Robot(const std::string& node_name)
    : Node(node_name),
      in_docking_mode_(false),
      initial_yaw_(0.0),
      has_relocalized_(false),
      use_tsp_optimization_(true)
{
    RCLCPP_INFO(this->get_logger(), "Robot base class created: %s", node_name.c_str());
}

void Robot::initialize() {
    // Initialize SLAM controller (now we can use shared_from_this())
    slam_controller_ = std::make_unique<SlamController>(this->shared_from_this());
    
    // Initialize motion controller
    motion_controller_ = std::make_unique<MotionController>(this->shared_from_this());
    
    // Create status publisher
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_status", 10);
    
    RCLCPP_INFO(this->get_logger(), "Robot base class initialized");
}

bool Robot::hasValidMap() const {
    return slam_controller_ && slam_controller_->hasValidMap();
}

// ============================================================================
// TSP Route Optimization
// ============================================================================

std::vector<std::vector<double>> Robot::buildDistanceMatrix(
    const geometry_msgs::msg::Point& start,
    const std::vector<geometry_msgs::msg::Point>& points) {
    
    size_t n = points.size();
    std::vector<std::vector<double>> matrix(n, std::vector<double>(n, 0.0));
    
    auto current_map = slam_controller_->getCurrentMap();
    if (!current_map) {
        RCLCPP_ERROR(this->get_logger(), "No map available for distance matrix calculation!");
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
                
                RCLCPP_WARN(this->get_logger(), 
                           "No A* path found between points %zu and %zu, using penalized Euclidean distance",
                           i, j);
            }
        }
    }
    
    return matrix;
}

std::vector<int> Robot::simulatedAnnealing(
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
            RCLCPP_DEBUG(this->get_logger(), 
                        "SA iteration %d: best_cost=%.2f, current_cost=%.2f, temp=%.2f",
                        iter, best_cost, current_cost, temperature);
        }
    }
    
    RCLCPP_INFO(this->get_logger(), 
               "Simulated Annealing complete: %d improvements, final cost: %.2fm",
               improvements, best_cost);
    
    return best_tour;
}

double Robot::calculateTourCost(
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

void Robot::returnToHome() {
    if (!slam_controller_->hasValidMap() || !slam_controller_->hasValidPose()) {
        RCLCPP_WARN(this->get_logger(), "Cannot return home - no valid map or pose");
        return;
    }
    
    geometry_msgs::msg::Point home_position;
    home_position.x = 0.0;
    home_position.y = 0.0;
    home_position.z = 0.0;
    
    auto current_pose = slam_controller_->getCurrentPose();
    double distance = calculateDistance(current_pose.position, home_position);
    
    if (distance < HOME_TOLERANCE) {
        RCLCPP_INFO(this->get_logger(), "Already at home position");
        motion_controller_->stop();
        return;
    }
    
    // Use motion controller for navigation
    motion_controller_->navigateToGoal(home_position);
}

void Robot::preciseDocking(
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
    double yaw = getYawFromQuaternion(current_pose.orientation);
    
    // Calculate angle difference
    double angle_diff = angle_to_home - yaw;
    // Normalize to [-pi, pi]
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
    
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = this->now();
    cmd_vel.header.frame_id = "base_footprint";
    
    // If angle is off by more than 10 degrees, rotate first
    if (std::abs(angle_diff) > 0.175) {  // ~10 degrees
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = (angle_diff > 0) ? DOCKING_ANGULAR_SPEED : -DOCKING_ANGULAR_SPEED;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Docking: Aligning (angle error: %.1f°)", angle_diff * 180.0 / M_PI);
    } else {
        // Move forward slowly toward home
        cmd_vel.twist.linear.x = DOCKING_LINEAR_SPEED;
        cmd_vel.twist.angular.z = angle_diff * 0.5;  // Gentle correction
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Docking: Moving forward (%.3fm remaining)", distance_to_home);
    }
    
    // Publish directly (bypass motion controller during docking)
    auto cmd_vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    cmd_vel_pub->publish(cmd_vel);
}

bool Robot::hasLineOfSight(
    const geometry_msgs::msg::Point& from,
    const geometry_msgs::msg::Point& to,
    const nav_msgs::msg::OccupancyGrid& map) {
    
    // Use Bresenham's line algorithm to check if path is clear
    // Also check cells around the line to account for robot width
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

double Robot::checkMinDistanceToWalls(
    const geometry_msgs::msg::Point& position,
    const nav_msgs::msg::OccupancyGrid& map) {
    
    GridCell center = PathPlanner::worldToGrid(map, position);
    
    double min_distance = std::numeric_limits<double>::max();
    
    // Check in 8 directions around the robot
    std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},  // Cardinal
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}  // Diagonal
    };
    
    for (const auto& dir : directions) {
        int distance_cells = 0;
        GridCell current = center;
        
        // March in this direction until we hit a wall or map boundary
        while (PathPlanner::isCellInBounds(map, current) && 
               PathPlanner::isCellWalkable(map, current)) {
            current.first += dir.first;
            current.second += dir.second;
            distance_cells++;
            
            // Limit search distance to avoid excessive computation
            if (distance_cells > 100) break;
        }
        
        // Convert cells to meters
        double distance_meters = distance_cells * map.info.resolution;
        min_distance = std::min(min_distance, distance_meters);
    }
    
    return min_distance;
}

// ============================================================================
// Utility Methods
// ============================================================================

void Robot::publishStatus(const std::string& status) {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Status: %s", status.c_str());
}

std::string Robot::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

double Robot::calculateDistance(
    const geometry_msgs::msg::Point& p1,
    const geometry_msgs::msg::Point& p2) const {
    
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

double Robot::getYawFromQuaternion(
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
