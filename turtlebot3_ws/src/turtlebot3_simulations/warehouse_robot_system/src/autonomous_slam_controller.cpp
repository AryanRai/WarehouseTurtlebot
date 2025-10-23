// MTRX3760 2025 Project 2: Autonomous SLAM Controller Implementation
// File: autonomous_slam_controller.cpp
// Author(s): Aryan Rai
//
// Implementation of autonomous SLAM system with frontier exploration

#include "autonomous_slam_controller.hpp"
#include <cmath>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace slam {

AutonomousSlamController::AutonomousSlamController()
    : Node("autonomous_slam_controller"),
      current_state_(SlamState::INITIALIZING),
      mapping_state_(MappingState::SEARCHING_FRONTIERS),
      consecutive_no_frontiers_(0),
      consecutive_no_paths_(0),
      last_distance_to_goal_(std::numeric_limits<double>::max()),
      total_frontiers_explored_(0),
      total_distance_traveled_(0.0)
{
    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize SLAM components
    frontier_searcher_ = std::make_unique<FrontierSearch>();
    path_planner_ = std::make_unique<PathPlanner>();

    // Set origin point (0, 0)
    origin_point_.x = 0.0;
    origin_point_.y = 0.0;
    origin_point_.z = 0.0;

    // Create publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/slam/planned_path", 10);
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/slam/current_goal", 10);

    // Create subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&AutonomousSlamController::mapCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&AutonomousSlamController::odomCallback, this, std::placeholders::_1));

    // Create control timer
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / config_.frontier_search_rate_hz)),
        std::bind(&AutonomousSlamController::executeStateMachine, this));

    // Initialize timestamps
    state_start_time_ = this->now();
    last_progress_time_ = this->now();
    last_movement_time_ = this->now();
    exploration_start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Autonomous SLAM Controller initialized");
    RCLCPP_INFO(this->get_logger(), "Starting in INITIALIZING state");
}

AutonomousSlamController::~AutonomousSlamController() {
    stopRobot();
    RCLCPP_INFO(this->get_logger(), "Autonomous SLAM Controller shutdown");
}

void AutonomousSlamController::run() {
    RCLCPP_INFO(this->get_logger(), "Starting autonomous SLAM exploration...");
    rclcpp::spin(shared_from_this());
}

void AutonomousSlamController::executeStateMachine() {
    switch (current_state_) {
        case SlamState::INITIALIZING:
            handleInitializingState();
            break;
        case SlamState::MAPPING:
            handleMappingState();
            break;
        case SlamState::RETURNING_HOME:
            handleReturningHomeState();
            break;
        case SlamState::OPERATIONAL:
            handleOperationalState();
            break;
        case SlamState::ERROR:
            handleErrorState();
            break;
    }
}

void AutonomousSlamController::handleInitializingState() {
    // Wait for map data, but don't require pose initially (map frame may not exist yet)
    if (!current_map_) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Waiting for map data...");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Map data received, starting exploration");
    RCLCPP_INFO(this->get_logger(), "Note: Pose will be available once SLAM creates map frame");
    transitionToState(SlamState::MAPPING);
}

void AutonomousSlamController::handleMappingState() {
    // Check for timeout
    if (hasExplorationTimedOut()) {
        RCLCPP_WARN(this->get_logger(), "Exploration timeout reached, returning home");
        transitionToState(SlamState::RETURNING_HOME);
        return;
    }

    // Execute mapping sub-state machine
    switch (mapping_state_) {
        case MappingState::SEARCHING_FRONTIERS:
            handleSearchingFrontiers();
            break;
        case MappingState::PLANNING_PATH:
            handlePlanningPath();
            break;
        case MappingState::NAVIGATING:
            handleNavigating();
            break;
        case MappingState::EXPLORING_AREA:
            handleExploringArea();
            break;
        case MappingState::STUCK_RECOVERY:
            handleStuckRecovery();
            break;
    }
}

void AutonomousSlamController::handleSearchingFrontiers() {
    // Check if we have pose data - if not, wait for SLAM to initialize
    if (!updateCurrentPose()) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Waiting for map frame to be created by SLAM...");
        return;
    }
    
    if (!searchForFrontiers()) {
        consecutive_no_frontiers_++;
        RCLCPP_DEBUG(this->get_logger(), "No frontiers found (count: %d)", consecutive_no_frontiers_);
        
        if (consecutive_no_frontiers_ >= config_.max_no_frontier_count) {
            RCLCPP_INFO(this->get_logger(), "No more frontiers found, exploration complete!");
            printExplorationStatistics();
            transitionToState(SlamState::RETURNING_HOME);
            return;
        }
        
        // Try local exploration
        transitionToMappingState(MappingState::EXPLORING_AREA);
        return;
    }

    consecutive_no_frontiers_ = 0;
    transitionToMappingState(MappingState::PLANNING_PATH);
}

void AutonomousSlamController::handlePlanningPath() {
    if (detected_frontiers_.empty()) {
        transitionToMappingState(MappingState::SEARCHING_FRONTIERS);
        return;
    }

    // Select best frontier
    Frontier best_frontier = selectBestFrontier(detected_frontiers_);
    current_goal_.x = best_frontier.centroid.x;
    current_goal_.y = best_frontier.centroid.y;
    current_goal_.z = 0.0;

    // Plan path to frontier
    if (!planPathToGoal(current_goal_)) {
        consecutive_no_paths_++;
        RCLCPP_WARN(this->get_logger(), "Failed to plan path to frontier (count: %d)", consecutive_no_paths_);
        
        if (consecutive_no_paths_ >= config_.max_no_path_count) {
            RCLCPP_WARN(this->get_logger(), "Too many path planning failures, trying recovery");
            transitionToMappingState(MappingState::STUCK_RECOVERY);
            return;
        }
        
        transitionToMappingState(MappingState::SEARCHING_FRONTIERS);
        return;
    }

    consecutive_no_paths_ = 0;
    last_distance_to_goal_ = calculateDistance(poseToPoint(current_pose_), current_goal_);
    
    // Publish goal for visualization
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = this->now();
    goal_pose.pose.position = current_goal_;
    goal_pose.pose.orientation.w = 1.0;
    goal_pub_->publish(goal_pose);

    RCLCPP_INFO(this->get_logger(), "Planned path to frontier at (%.2f, %.2f), size: %d", 
                current_goal_.x, current_goal_.y, best_frontier.size);
    
    transitionToMappingState(MappingState::NAVIGATING);
}

void AutonomousSlamController::handleNavigating() {
    if (!updateCurrentPose()) {
        RCLCPP_WARN(this->get_logger(), "Lost pose during navigation");
        stopRobot();
        return;
    }

    // Check if goal reached
    if (isGoalReached(current_goal_, config_.goal_tolerance)) {
        RCLCPP_INFO(this->get_logger(), "Reached frontier goal");
        total_frontiers_explored_++;
        stopRobot();
        transitionToMappingState(MappingState::EXPLORING_AREA);
        return;
    }

    // Check if stuck
    if (isRobotStuck()) {
        RCLCPP_WARN(this->get_logger(), "Robot appears stuck during navigation");
        transitionToMappingState(MappingState::STUCK_RECOVERY);
        return;
    }

    // Follow path
    if (!followCurrentPath()) {
        RCLCPP_WARN(this->get_logger(), "Path following failed, replanning");
        transitionToMappingState(MappingState::PLANNING_PATH);
        return;
    }

    updateProgressTracking();
}

void AutonomousSlamController::handleExploringArea() {
    // Simple local exploration: rotate in place to scan area
    static int rotation_count = 0;
    static rclcpp::Time rotation_start = this->now();
    
    const double rotation_duration = 3.0; // seconds
    const double rotation_speed = 0.5; // rad/s
    
    if ((this->now() - rotation_start).seconds() < rotation_duration) {
        publishVelocityCommand(0.0, rotation_speed);
        return;
    }
    
    stopRobot();
    rotation_count++;
    
    if (rotation_count >= 2) { // Two full rotations
        rotation_count = 0;
        RCLCPP_INFO(this->get_logger(), "Local exploration complete, searching for new frontiers");
        transitionToMappingState(MappingState::SEARCHING_FRONTIERS);
    } else {
        rotation_start = this->now();
    }
}

void AutonomousSlamController::handleStuckRecovery() {
    static rclcpp::Time recovery_start = this->now();
    const double recovery_duration = 5.0; // seconds
    
    // Simple recovery: back up and turn
    if ((this->now() - recovery_start).seconds() < recovery_duration / 2) {
        publishVelocityCommand(-0.1, 0.0); // Back up
    } else if ((this->now() - recovery_start).seconds() < recovery_duration) {
        publishVelocityCommand(0.0, 0.8); // Turn
    } else {
        stopRobot();
        RCLCPP_INFO(this->get_logger(), "Recovery complete, resuming exploration");
        recovery_start = this->now();
        transitionToMappingState(MappingState::SEARCHING_FRONTIERS);
    }
}

void AutonomousSlamController::handleReturningHomeState() {
    if (!updateCurrentPose()) {
        RCLCPP_WARN(this->get_logger(), "Lost pose while returning home");
        return;
    }

    // Check if at origin
    if (isGoalReached(origin_point_, config_.origin_tolerance)) {
        RCLCPP_INFO(this->get_logger(), "Successfully returned to origin");
        stopRobot();
        printExplorationStatistics();
        transitionToState(SlamState::OPERATIONAL);
        return;
    }

    // Plan path to origin if needed
    if (current_path_.poses.empty() || 
        !isGoalReached(current_goal_, config_.goal_tolerance)) {
        
        current_goal_ = origin_point_;
        if (!planPathToGoal(current_goal_)) {
            RCLCPP_WARN(this->get_logger(), "Failed to plan path home, trying again...");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Planned path home to origin (0, 0)");
    }

    // Follow path home
    if (!followCurrentPath()) {
        RCLCPP_WARN(this->get_logger(), "Path following failed while returning home");
        current_path_.poses.clear(); // Force replanning
    }
}

void AutonomousSlamController::handleOperationalState() {
    // Robot is at origin and ready for warehouse operations
    stopRobot();
    
    // Log status periodically
    static rclcpp::Time last_status_log = this->now();
    if ((this->now() - last_status_log).seconds() > 10.0) {
        RCLCPP_INFO(this->get_logger(), "SLAM complete - Robot operational at origin");
        last_status_log = this->now();
    }
}

void AutonomousSlamController::handleErrorState() {
    stopRobot();
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "SLAM system in error state");
}

bool AutonomousSlamController::searchForFrontiers() {
    if (!current_map_ || !updateCurrentPose()) {
        return false;
    }

    // Convert current pose to grid coordinates
    GridCell start_cell = PathPlanner::worldToGrid(*current_map_, poseToPoint(current_pose_));
    
    // Search for frontiers
    auto [frontier_list, frontier_cells] = frontier_searcher_->search(*current_map_, start_cell, false);
    
    detected_frontiers_.clear();
    for (const auto& frontier : frontier_list.frontiers) {
        if (frontier.size >= config_.frontier_min_size) {
            detected_frontiers_.push_back(frontier);
        }
    }

    RCLCPP_DEBUG(this->get_logger(), "Found %zu valid frontiers", detected_frontiers_.size());
    return !detected_frontiers_.empty();
}

Frontier AutonomousSlamController::selectBestFrontier(const std::vector<Frontier>& frontiers) {
    if (frontiers.empty()) {
        return Frontier{};
    }

    // Sort frontiers by size (larger is better)
    std::vector<Frontier> sorted_frontiers = frontiers;
    std::sort(sorted_frontiers.begin(), sorted_frontiers.end(),
              [](const Frontier& a, const Frontier& b) {
                  return a.size > b.size;
              });

    // Consider only top frontiers
    int max_to_check = std::min(static_cast<int>(sorted_frontiers.size()), 
                               config_.max_frontiers_to_check);
    
    double best_cost = std::numeric_limits<double>::max();
    Frontier best_frontier = sorted_frontiers[0];
    
    geometry_msgs::msg::Point current_pos = poseToPoint(current_pose_);
    
    for (int i = 0; i < max_to_check; ++i) {
        const auto& frontier = sorted_frontiers[i];
        
        // Calculate cost: distance + inverse size bonus
        double distance = calculateDistance(current_pos, frontier.centroid);
        double cost = config_.a_star_cost_weight * distance + 
                     config_.frontier_size_weight / frontier.size;
        
        if (cost < best_cost) {
            best_cost = cost;
            best_frontier = frontier;
        }
    }
    
    return best_frontier;
}

bool AutonomousSlamController::planPathToGoal(const geometry_msgs::msg::Point& goal) {
    if (!current_map_ || !updateCurrentPose()) {
        return false;
    }

    GridCell start = PathPlanner::worldToGrid(*current_map_, poseToPoint(current_pose_));
    GridCell goal_cell = PathPlanner::worldToGrid(*current_map_, goal);

    // Calculate C-space and cost map
    auto [cspace, cspace_cells] = PathPlanner::calcCspace(*current_map_, false);
    
    // Plan path using A*
    auto [path, cost, actual_start, actual_goal] = 
        PathPlanner::aStar(*current_map_, cv::Mat(), start, goal_cell);

    if (!path.has_value() || path->empty()) {
        return false;
    }

    // Convert path to ROS message and publish
    publishPath(path.value());
    return true;
}

bool AutonomousSlamController::followCurrentPath() {
    if (current_path_.poses.empty()) {
        return false;
    }

    geometry_msgs::msg::Point lookahead = calculateLookaheadPoint();
    calculatePurePursuitControl(lookahead);
    return true;
}

geometry_msgs::msg::Point AutonomousSlamController::calculateLookaheadPoint() {
    const double lookahead_distance = 0.3; // meters
    
    if (current_path_.poses.empty()) {
        return current_goal_;
    }

    geometry_msgs::msg::Point current_pos = poseToPoint(current_pose_);
    
    // Find closest point on path
    double min_distance = std::numeric_limits<double>::max();
    size_t closest_index = 0;
    
    for (size_t i = 0; i < current_path_.poses.size(); ++i) {
        double dist = calculateDistance(current_pos, current_path_.poses[i].pose.position);
        if (dist < min_distance) {
            min_distance = dist;
            closest_index = i;
        }
    }

    // Find lookahead point
    for (size_t i = closest_index; i < current_path_.poses.size(); ++i) {
        double dist = calculateDistance(current_pos, current_path_.poses[i].pose.position);
        if (dist >= lookahead_distance) {
            return current_path_.poses[i].pose.position;
        }
    }

    // Return last point if no lookahead found
    return current_path_.poses.back().pose.position;
}

void AutonomousSlamController::calculatePurePursuitControl(const geometry_msgs::msg::Point& lookahead) {
    geometry_msgs::msg::Point current_pos = poseToPoint(current_pose_);
    double current_yaw = calculateYawFromPose(current_pose_);

    // Calculate angle to lookahead point
    double dx = lookahead.x - current_pos.x;
    double dy = lookahead.y - current_pos.y;
    double target_yaw = std::atan2(dy, dx);
    
    // Calculate angular error
    double yaw_error = target_yaw - current_yaw;
    
    // Normalize angle
    while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

    // Calculate velocities
    double linear_vel = config_.max_linear_velocity;
    double angular_vel = std::clamp(yaw_error * 2.0, 
                                   -config_.max_angular_velocity, 
                                   config_.max_angular_velocity);

    // Reduce linear velocity when turning
    if (std::abs(angular_vel) > 0.3) {
        linear_vel *= 0.5;
    }

    publishVelocityCommand(linear_vel, angular_vel);
}

void AutonomousSlamController::publishPath(const std::vector<GridCell>& path) {
    if (!current_map_) return;

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();

    for (const auto& cell : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        
        geometry_msgs::msg::Point world_point = PathPlanner::gridToWorld(*current_map_, cell);
        pose.pose.position = world_point;
        pose.pose.orientation.w = 1.0;
        
        path_msg.poses.push_back(pose);
    }

    current_path_ = path_msg;
    path_pub_->publish(path_msg);
}

// Callback implementations
void AutonomousSlamController::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = msg;
}

void AutonomousSlamController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Odometry callback - pose is updated via TF
}

// Utility function implementations
bool AutonomousSlamController::updateCurrentPose() {
    try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            "map", "base_footprint", tf2::TimePointZero);
        
        current_pose_.header.frame_id = "map";
        current_pose_.header.stamp = this->now();
        current_pose_.pose.position.x = transform.transform.translation.x;
        current_pose_.pose.position.y = transform.transform.translation.y;
        current_pose_.pose.position.z = transform.transform.translation.z;
        current_pose_.pose.orientation = transform.transform.rotation;
        
        return true;
    } catch (tf2::TransformException& ex) {
        RCLCPP_DEBUG(this->get_logger(), "Could not get transform: %s", ex.what());
        return false;
    }
}

void AutonomousSlamController::publishVelocityCommand(double linear, double angular) {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_footprint";
    cmd.twist.linear.x = linear;
    cmd.twist.angular.z = angular;
    cmd_vel_pub_->publish(cmd);
    
    RCLCPP_DEBUG(this->get_logger(), "Published velocity: linear=%.3f, angular=%.3f", linear, angular);
}

void AutonomousSlamController::stopRobot() {
    publishVelocityCommand(0.0, 0.0);
}

bool AutonomousSlamController::isGoalReached(const geometry_msgs::msg::Point& goal, double tolerance) {
    if (!updateCurrentPose()) return false;
    
    double distance = calculateDistance(poseToPoint(current_pose_), goal);
    return distance < tolerance;
}

bool AutonomousSlamController::isRobotStuck() {
    const double stuck_distance_threshold = 0.05; // meters
    
    double current_distance = calculateDistance(poseToPoint(current_pose_), current_goal_);
    
    if (std::abs(current_distance - last_distance_to_goal_) < stuck_distance_threshold) {
        if ((this->now() - last_movement_time_).seconds() > config_.stuck_timeout_s) {
            return true;
        }
    } else {
        last_movement_time_ = this->now();
        last_distance_to_goal_ = current_distance;
    }
    
    return false;
}

bool AutonomousSlamController::hasExplorationTimedOut() {
    return (this->now() - exploration_start_time_).seconds() > config_.exploration_timeout_s;
}

void AutonomousSlamController::updateProgressTracking() {
    static geometry_msgs::msg::Point last_position;
    static bool first_update = true;
    
    geometry_msgs::msg::Point current_pos = poseToPoint(current_pose_);
    
    if (!first_update) {
        total_distance_traveled_ += calculateDistance(last_position, current_pos);
    }
    
    last_position = current_pos;
    first_update = false;
}

// State transition functions
void AutonomousSlamController::transitionToState(SlamState new_state) {
    SlamState old_state = current_state_;
    current_state_ = new_state;
    state_start_time_ = this->now();
    
    logStateTransition(old_state, new_state);
}

void AutonomousSlamController::transitionToMappingState(MappingState new_mapping_state) {
    MappingState old_state = mapping_state_;
    mapping_state_ = new_mapping_state;
    
    logMappingStateTransition(old_state, new_mapping_state);
}

void AutonomousSlamController::forceOperationalMode() {
    RCLCPP_INFO(this->get_logger(), "Forcing transition to operational mode");
    transitionToState(SlamState::OPERATIONAL);
}

// Utility calculations
double AutonomousSlamController::calculateDistance(const geometry_msgs::msg::Point& p1, 
                                                  const geometry_msgs::msg::Point& p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

double AutonomousSlamController::calculateYawFromPose(const geometry_msgs::msg::PoseStamped& pose) {
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

geometry_msgs::msg::Point AutonomousSlamController::poseToPoint(const geometry_msgs::msg::PoseStamped& pose) {
    return pose.pose.position;
}

// Logging functions
void AutonomousSlamController::logStateTransition(SlamState old_state, SlamState new_state) {
    RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s", 
                stateToString(old_state).c_str(), stateToString(new_state).c_str());
}

void AutonomousSlamController::logMappingStateTransition(MappingState old_state, MappingState new_state) {
    RCLCPP_DEBUG(this->get_logger(), "Mapping state transition: %s -> %s",
                 mappingStateToString(old_state).c_str(), mappingStateToString(new_state).c_str());
}

void AutonomousSlamController::printExplorationStatistics() {
    double exploration_time = (this->now() - exploration_start_time_).seconds();
    
    RCLCPP_INFO(this->get_logger(), "=== EXPLORATION STATISTICS ===");
    RCLCPP_INFO(this->get_logger(), "Total exploration time: %.1f seconds", exploration_time);
    RCLCPP_INFO(this->get_logger(), "Total frontiers explored: %d", total_frontiers_explored_);
    RCLCPP_INFO(this->get_logger(), "Total distance traveled: %.2f meters", total_distance_traveled_);
    RCLCPP_INFO(this->get_logger(), "Average speed: %.2f m/s", total_distance_traveled_ / exploration_time);
    RCLCPP_INFO(this->get_logger(), "===============================");
}

std::string AutonomousSlamController::stateToString(SlamState state) {
    switch (state) {
        case SlamState::INITIALIZING: return "INITIALIZING";
        case SlamState::MAPPING: return "MAPPING";
        case SlamState::RETURNING_HOME: return "RETURNING_HOME";
        case SlamState::OPERATIONAL: return "OPERATIONAL";
        case SlamState::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

std::string AutonomousSlamController::mappingStateToString(MappingState state) {
    switch (state) {
        case MappingState::SEARCHING_FRONTIERS: return "SEARCHING_FRONTIERS";
        case MappingState::PLANNING_PATH: return "PLANNING_PATH";
        case MappingState::NAVIGATING: return "NAVIGATING";
        case MappingState::EXPLORING_AREA: return "EXPLORING_AREA";
        case MappingState::STUCK_RECOVERY: return "STUCK_RECOVERY";
        default: return "UNKNOWN";
    }
}

} // namespace slam