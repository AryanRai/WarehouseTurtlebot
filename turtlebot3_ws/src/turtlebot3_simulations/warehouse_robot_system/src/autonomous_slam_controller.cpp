// MTRX3760 2025 Project 2: Autonomous SLAM Controller Implementation
// File: autonomous_slam_controller.cpp
// Author(s): Aryan Rai
//
// Implementation of autonomous SLAM system with frontier exploration

#include "autonomous_slam_controller.hpp"
#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace slam
{

AutonomousSlamController::AutonomousSlamController()
    : Node("autonomous_slam_controller"),
      current_state_(SlamState::INITIALIZING),
      mapping_state_(MappingState::SEARCHING_FRONTIERS),
      consecutive_no_frontiers_(0), consecutive_no_paths_(0),
      last_distance_to_goal_(std::numeric_limits<double>::max()),
      total_frontiers_explored_(0), total_distance_traveled_(0.0)
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
    path_pub_ =
        this->create_publisher<nav_msgs::msg::Path>("/slam/planned_path", 10);
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/slam/current_goal", 10);

    // Create subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&AutonomousSlamController::mapCallback, this,
                  std::placeholders::_1));
    odom_sub_ = this->cr

                    // Initialize timestamps
                    state_start_time_ = this->now();
    last_progress_time_ = this->now();
    last_movement_time_ = this->now();
    exploration_start_time_ = this->now();

    // Initialize tracking variables
    last_distance_to_goal_ = std::numeric_limits<double>::max();

    RCLCPP_INFO(this->get_logger(), "Autonomous SLAM Controller initialized");
    RCLCPP_INFO(this->get_logger(), "Starting in INITIALIZING state");
}

AutonomousSlamController::~AutonomousSlamController()
{
    stopRobot();
    RCLCPP_INFO(this->get_logger(), "Autonomous SLAM Controller shutdown");
}

void AutonomousSlamController::run()
{
    RCLCPP_INFO(this->get_logger(), "Starting autonomous SLAM exploration...");
    rclcpp::spin(shared_from_this());
}

void AutonomousSlamController::executeStateMachine()
{
    switch (current_state_)
    {
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

void AutonomousSlamController::handleInitializingState()
{
    // Wait for map data, but don't require pose initially (map frame may not
    // exist yet)
    if (!current_map_)
    {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Waiting for map data...");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Map data received, starting exploration");
    RCLCPP_INFO(this->get_logger(),
                "Note: Pose will be available once SLAM creates map frame");
    transitionToState(SlamState::MAPPING);
}

void AutonomousSlamController::handleMappingState()
{
    // Check for timeout
    if (hasExplorationTimedOut())
    {
        RCLCPP_WARN(this->get_logger(),
                    "Exploration timeout reached, returning home");
        transitionToState(SlamState::RETURNING_HOME);
        return;
    }

    // Execute mapping sub-state machine
    switch (mapping_state_)
    {
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

void AutonomousSlamController::handleSearchingFrontiers()
{
    // Check if we have pose data - if not, wait for SLAM to initialize
    if (!updateCurrentPose())
    {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Waiting for map frame to be created by SLAM...");
        return;
    }

    if (!searchForFrontiers())
    {
        consecutive_no_frontiers_++;
        RCLCPP_DEBUG(this->get_logger(), "No frontiers found (count: %d)",
                     consecutive_no_frontiers_);

        // Before giving up, do a final sweep to check for hidden frontiers
        if (consecutive_no_frontiers_ >= config_.max_no_frontier_count / 2 &&
            consecutive_no_frontiers_ < config_.max_no_frontier_count)
        {
            RCLCPP_INFO(this->get_logger(),
                        "No frontiers visible, performing final sweep to check "
                        "for hidden areas...");
            transitionToMappingState(MappingState::EXPLORING_AREA);
            return;
        }

        if (consecutive_no_frontiers_ >= config_.max_no_frontier_count)
        {
            RCLCPP_INFO(this->get_logger(),
                        "No more frontiers found after final sweep, "
                        "exploration complete!");
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

void AutonomousSlamController::handlePlanningPath()
{
    if (detected_frontiers_.empty())
    {
        transitionToMappingState(MappingState::SEARCHING_FRONTIERS);
        return;
    }

    // Select best frontier
    FrontierSearch::Frontier best_frontier =
        selectBestFrontier(detected_frontiers_);
    current_goal_.x = best_frontier.centroid.x;
    current_goal_.y = best_frontier.centroid.y;
    current_goal_.z = 0.0;

    // Plan path to frontier
    if (!planPathToGoal(current_goal_))
    {
        consecutive_no_paths_++;
        RCLCPP_WARN(this->get_logger(),
                    "Failed to plan path to frontier (count: %d)",
                    consecutive_no_paths_);

        if (consecutive_no_paths_ >= config_.max_no_path_count)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Too many path planning failures, trying recovery");
            transitionToMappingState(MappingState::STUCK_RECOVERY);
            return;
        }

        transitionToMappingState(MappingState::SEARCHING_FRONTIERS);
        return;
    }

    consecutive_no_paths_ = 0;
    last_distance_to_goal_ =
        calculateDistance(poseToPoint(current_pose_), current_goal_);
    last_movement_time_ = this->now(); // Reset movement timer for new path

    // Publish goal for visualization
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = this->now();
    goal_pose.pose.position = current_goal_;
    goal_pose.pose.orientation.w = 1.0;
    goal_pub_->publish(goal_pose);

    RCLCPP_INFO(this->get_logger(),
                "Planned path to frontier at (%.2f, %.2f), size: %d",
                current_goal_.x, current_goal_.y, best_frontier.size);

    transitionToMappingState(MappingState::NAVIGATING);
}

void AutonomousSlamController::handleNavigating()
{
    if (!updateCurrentPose())
    {
        RCLCPP_WARN(this->get_logger(), "Lost pose during navigation");
        stopRobot();
        return;
    }

    // Check if path is empty (shouldn't happen, but safety check)
    if (current_path_.poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Path became empty during navigation");
        transitionToMappingState(MappingState::SEARCHING_FRONTIERS);
        return;
    }

    // Check if goal reached
    if (isGoalReached(current_goal_, config_.goal_tolerance))
    {
        RCLCPP_INFO(this->get_logger(), "Reached frontier goal at (%.2f, %.2f)",
                    current_goal_.x, current_goal_.y);
        total_frontiers_explored_++;

        // Mark this location as visited
        visited_frontiers_.push_back(current_goal_);

        stopRobot();
        transitionToMappingState(MappingState::EXPLORING_AREA);
        return;
    }

    // Check if stuck
    if (isRobotStuck())
    {
        RCLCPP_WARN(this->get_logger(),
                    "Robot appears stuck during navigation");
        stopRobot();
        transitionToMappingState(MappingState::STUCK_RECOVERY);
        return;
    }

    // Follow path
    if (!followCurrentPath())
    {
        RCLCPP_WARN(this->get_logger(), "Path following failed, replanning");
        stopRobot();
        transitionToMappingState(MappingState::PLANNING_PATH);
        return;
    }

    updateProgressTracking();
}

void AutonomousSlamController::handleExploringArea()
{
    // Simple but safe exploration: just rotate in place
    static rclcpp::Time rotation_start = this->now();
    static int scan_count = 0;

    const double rotation_duration = 2.0; // seconds - full 360° rotation
    const double rotation_speed = 0.5;    // rad/s - slower for better scanning

    if ((this->now() - rotation_start).seconds() < rotation_duration)
    {
        publishVelocityCommand(0.0, rotation_speed);
        return;
    }

    stopRobot();
    rotation_start = this->now();
    scan_count++;

    RCLCPP_INFO(this->get_logger(),
                "Exploration scan complete, searching for new frontiers");

    // If we've scanned multiple times without finding frontiers, increment
    // counter
    if (scan_count > 1)
    {
        scan_count = 0;
        consecutive_no_frontiers_++; // Increment to help trigger completion
    }

    transitionToMappingState(MappingState::SEARCHING_FRONTIERS);
}

void AutonomousSlamController::handleStuckRecovery()
{
    static rclcpp::Time recovery_start = this->now();
    static int recovery_attempt = 0;
    const double recovery_duration = 3.0; // seconds - reduced from 5.0

    double elapsed = (this->now() - recovery_start).seconds();

    // More aggressive recovery: back up faster and turn more
    if (elapsed < recovery_duration * 0.4)
    {
        publishVelocityCommand(-0.15, 0.0); // Back up faster
    }
    else if (elapsed < recovery_duration * 0.8)
    {
        publishVelocityCommand(0.0, 1.0); // Turn faster
    }
    else if (elapsed < recovery_duration)
    {
        publishVelocityCommand(0.1, 0.0); // Move forward briefly
    }
    else
    {
        stopRobot();
        recovery_attempt++;
        RCLCPP_INFO(this->get_logger(),
                    "Recovery complete (attempt %d), resuming exploration",
                    recovery_attempt);
        recovery_start = this->now();

        // Clear current path to force replanning
        current_path_.poses.clear();

        transitionToMappingState(MappingState::SEARCHING_FRONTIERS);
    }
}

void AutonomousSlamController::handleReturningHomeState()
{
    if (!updateCurrentPose())
    {
        RCLCPP_WARN(this->get_logger(), "Lost pose while returning home");
        return;
    }

    // Check if at origin
    if (isGoalReached(origin_point_, config_.origin_tolerance))
    {
        RCLCPP_INFO(this->get_logger(), "Successfully returned to origin");
        stopRobot();
        printExplorationStatistics();
        transitionToState(SlamState::OPERATIONAL);
        return;
    }

    // Plan path to origin if needed
    if (current_path_.poses.empty() ||
        !isGoalReached(current_goal_, config_.goal_tolerance))
    {

        current_goal_ = origin_point_;
        if (!planPathToGoal(current_goal_))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to plan path home, trying again...");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Planned path home to origin (0, 0)");
    }

    // Follow path home
    if (!followCurrentPath())
    {
        RCLCPP_WARN(this->get_logger(),
                    "Path following failed while returning home");
        current_path_.poses.clear(); // Force replanning
    }
}

void AutonomousSlamController::handleOperationalState()
{
    // Robot is at origin and ready for warehouse operations
    stopRobot();

    // Log status periodically
    static rclcpp::Time last_status_log = this->now();
    if ((this->now() - last_status_log).seconds() > 10.0)
    {
        RCLCPP_INFO(this->get_logger(),
                    "SLAM complete - Robot operational at origin");
        last_status_log = this->now();
    }
}

void AutonomousSlamController::handleErrorState()
{
    stopRobot();
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "SLAM system in error state");
}

bool AutonomousSlamController::searchForFrontiers()
{
    if (!current_map_ || !updateCurrentPose())
    {
        return false;
    }

    // Convert current pose to grid coordinates
    GridCell start_cell =
        PathPlanner::worldToGrid(*current_map_, poseToPoint(current_pose_));

    // Search for frontiers
    auto [frontier_list, frontier_cells] =
        frontier_searcher_->search(*current_map_, start_cell, false);

    detected_frontiers_.clear();
    for (const auto &frontier : frontier_list.frontiers)
    {
        if (frontier.size >= config_.frontier_min_size)
        {
            detected_frontiers_.push_back(frontier);
        }
    }

    RCLCPP_DEBUG(this->get_logger(), "Found %zu valid frontiers",
                 detected_frontiers_.size());
    return !detected_frontiers_.empty();
}

FrontierSearch::Frontier AutonomousSlamController::selectBestFrontier(
    const std::vector<FrontierSearch::Frontier> &frontiers)
{
    if (frontiers.empty())
    {
        return FrontierSearch::Frontier{};
    }

    // Filter out recently visited frontiers
    std::vector<FrontierSearch::Frontier> unvisited_frontiers;
    for (const auto &frontier : frontiers)
    {
        bool is_visited = false;
        for (const auto &visited : visited_frontiers_)
        {
            double dist = calculateDistance(frontier.centroid, visited);
            if (dist < config_.visited_frontier_radius)
            {
                is_visited = true;
                break;
            }
        }
        if (!is_visited)
        {
            unvisited_frontiers.push_back(frontier);
        }
    }

    // If all frontiers visited, clear history and use all
    if (unvisited_frontiers.empty())
    {
        RCLCPP_INFO(this->get_logger(),
                    "All frontiers visited, clearing history");
        visited_frontiers_.clear();
        unvisited_frontiers = frontiers;
    }

    // Sort frontiers by size (larger is better)
    std::vector<FrontierSearch::Frontier> sorted_frontiers =
        unvisited_frontiers;
    std::sort(
        sorted_frontiers.begin(), sorted_frontiers.end(),
        [](const FrontierSearch::Frontier &a, const FrontierSearch::Frontier &b)
        { return a.size > b.size; });

    // Consider only top frontiers
    int max_to_check = std::min(static_cast<int>(sorted_frontiers.size()),
                                config_.max_frontiers_to_check);

    double best_cost = std::numeric_limits<double>::max();
    Frontier best_frontier = sorted_frontiers[0];

    geometry_msgs::msg::Point current_pos = poseToPoint(current_pose_);

    for (int i = 0; i < max_to_check; ++i)
    {
        const auto &frontier = sorted_frontiers[i];

        // Calculate cost: distance + inverse size bonus
        double distance = calculateDistance(current_pos, frontier.centroid);
        double cost = config_.a_star_cost_weight * distance +
                      config_.frontier_size_weight / frontier.size;

        if (cost < best_cost)
        {
            best_cost = cost;
            best_frontier = frontier;
        }
    }

    return best_frontier;
}

bool AutonomousSlamController::planPathToGoal(
    const geometry_msgs::msg::Point &goal)
{
    if (!current_map_ || !updateCurrentPose())
    {
        return false;
    }

    GridCell start =
        PathPlanner::worldToGrid(*current_map_, poseToPoint(current_pose_));
    GridCell goal_cell = PathPlanner::worldToGrid(*current_map_, goal);

    // Calculate C-space (inflated obstacles for safe navigation)
    auto [cspace, cspace_cells] = PathPlanner::calcCspace(*current_map_, false);

    // Calculate cost map (prefers middle of hallways, avoids walls)
    cv::Mat cost_map = PathPlanner::calcCostMap(*current_map_);

    // Plan path using A* with C-space AND cost map
    auto [path, cost, actual_start, actual_goal] =
        PathPlanner::aStar(cspace, cost_map, start, goal_cell);

    if (!path.has_value() || path->empty())
    {
        return false;
    }

    // Convert path to ROS message and publish (use original map for world
    // coordinates)
    publishPath(path.value());
    return true;
}

bool AutonomousSlamController::followCurrentPath()
{
    if (current_path_.poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No path to follow");
        return false;
    }

    geometry_msgs::msg::Point current_pos = poseToPoint(current_pose_);
    geometry_msgs::msg::Point lookahead = calculateLookaheadPoint();
    double dist_to_goal = calculateDistance(current_pos, current_goal_);

    calculatePurePursuitControl(lookahead);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Following path: %zu poses, lookahead=(%.2f, %.2f), goal_dist=%.2f",
        current_path_.poses.size(), lookahead.x, lookahead.y, dist_to_goal);
    return true;
}

geometry_msgs::msg::Point AutonomousSlamController::calculateLookaheadPoint()
{
    const double lookahead_distance =
        0.30; // meters - increased to prevent cutting corners

    if (current_path_.poses.empty())
    {
        return current_goal_;
    }

    geometry_msgs::msg::Point current_pos = poseToPoint(current_pose_);

    // Find closest point on path (nearest waypoint)
    double min_distance = std::numeric_limits<double>::max();
    size_t nearest_index = 0;

    for (size_t i = 0; i < current_path_.poses.size(); ++i)
    {
        double dist = calculateDistance(current_pos,
                                        current_path_.poses[i].pose.position);
        if (dist < min_distance)
        {
            min_distance = dist;
            nearest_index = i;
        }
    }

    // Find lookahead point: first point beyond lookahead_distance from current
    // position
    size_t lookahead_index = nearest_index;
    for (size_t i = nearest_index; i < current_path_.poses.size(); ++i)
    {
        double dist = calculateDistance(current_pos,
                                        current_path_.poses[i].pose.position);
        if (dist >= lookahead_distance)
        {
            lookahead_index = i;
            break;
        }
        lookahead_index = i; // Keep updating to get furthest point
    }

    // Return the lookahead point (or last point if we're near the end)
    return current_path_.poses[lookahead_index].pose.position;
}

void AutonomousSlamController::calculatePurePursuitControl(
    const geometry_msgs::msg::Point &lookahead)
{
    geometry_msgs::msg::Point current_pos = poseToPoint(current_pose_);
    double current_yaw = calculateYawFromPose(current_pose_);

    // Calculate vector to lookahead point
    double dx = lookahead.x - current_pos.x;
    double dy = lookahead.y - current_pos.y;
    double lookahead_dist = std::sqrt(dx * dx + dy * dy);

    // Avoid division by zero
    if (lookahead_dist < 0.01)
    {
        stopRobot();
        return;
    }

    // Calculate target angle and alpha (angle error in robot frame)
    double target_yaw = std::atan2(dy, dx);
    double alpha = target_yaw - current_yaw;

    // Normalize alpha to [-pi, pi]
    while (alpha > M_PI)
        alpha -= 2.0 * M_PI;
    while (alpha < -M_PI)
        alpha += 2.0 * M_PI;

    // NEVER reverse - always drive forward
    // Reversal causes slow backward movement and is not needed for frontier
    // exploration
    double linear_vel = config_.max_linear_velocity;

    // Pure pursuit formula: angular_vel = linear_vel * 2 * sin(alpha) / L
    double sin_alpha = std::sin(alpha);

    // Avoid singularity when alpha is very small
    if (std::abs(sin_alpha) < 0.01)
    {
        sin_alpha = (sin_alpha >= 0) ? 0.01 : -0.01;
    }

    // Calculate angular velocity using pure pursuit
    double angular_vel = (2.0 * linear_vel * sin_alpha) / lookahead_dist;

    // Clamp angular velocity
    angular_vel = std::clamp(angular_vel, -config_.max_angular_velocity,
                             config_.max_angular_velocity);

    // Adaptive speed reduction for sharp turns (more conservative)
    double abs_alpha = std::abs(alpha);
    if (abs_alpha > 1.2)
    {                       // Very sharp turn (>68°)
        linear_vel *= 0.25; // Slow way down
    }
    else if (abs_alpha > 0.8)
    { // Sharp turn (>45°)
        linear_vel *= 0.4;
    }
    else if (abs_alpha > 0.5)
    { // Moderate turn (>28°)
        linear_vel *= 0.6;
    }
    else if (abs_alpha > 0.3)
    { // Gentle turn (>17°)
        linear_vel *= 0.8;
    }
    // else: straight ahead, full speed

    publishVelocityCommand(linear_vel, angular_vel);

    RCLCPP_DEBUG(this->get_logger(),
                 "Pure pursuit: alpha=%.3f (%.1f°), dist=%.3f, linear=%.3f, "
                 "angular=%.3f",
                 alpha, alpha * 180.0 / M_PI, lookahead_dist, linear_vel,
                 angular_vel);
}

void AutonomousSlamController::publishPath(const std::vector<GridCell> &path)
{
    if (!current_map_)
        return;

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();

    for (const auto &cell : path)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;

        geometry_msgs::msg::Point world_point =
            PathPlanner::gridToWorld(*current_map_, cell);
        pose.pose.position = world_point;
        pose.pose.orientation.w = 1.0;

        path_msg.poses.push_back(pose);
    }

    current_path_ = path_msg;
    path_pub_->publish(path_msg);
}

// Callback implementations
void AutonomousSlamController::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    current_map_ = msg;
}

void AutonomousSlamController::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Odometry callback - pose is updated via TF
}

// Utility function implementations
bool AutonomousSlamController::updateCurrentPose()
{
    try
    {
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform("map", "base_footprint",
                                        tf2::TimePointZero);

        current_pose_.header.frame_id = "map";
        current_pose_.header.stamp = this->now();
        current_pose_.pose.position.x = transform.transform.translation.x;
        current_pose_.pose.position.y = transform.transform.translation.y;
        current_pose_.pose.position.z = transform.transform.translation.z;
        current_pose_.pose.orientation = transform.transform.rotation;

        return true;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_DEBUG(this->get_logger(), "Could not get transform: %s",
                     ex.what());
        return false;
    }
}

void AutonomousSlamController::publishVelocityCommand(double linear,
                                                      double angular)
{
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_footprint";
    cmd.twist.linear.x = linear;
    cmd.twist.angular.z = angular;
    cmd_vel_pub_->publish(cmd);

    RCLCPP_DEBUG(this->get_logger(),
                 "Published velocity: linear=%.3f, angular=%.3f", linear,
                 angular);
}

void AutonomousSlamController::stopRobot() { publishVelocityCommand(0.0, 0.0); }

bool AutonomousSlamController::isGoalReached(
    const geometry_msgs::msg::Point &goal, double tolerance)
{
    if (!updateCurrentPose())
        return false;

    double distance = calculateDistance(poseToPoint(current_pose_), goal);
    return distance < tolerance;
}

bool AutonomousSlamController::isRobotStuck()
{
    const double stuck_distance_threshold =
        0.03;                         // meters - reduced for better detection
    const double stuck_timeout = 4.0; // seconds - reduced for faster recovery

    double current_distance =
        calculateDistance(poseToPoint(current_pose_), current_goal_);

    // Check if robot has made progress toward goal
    if (std::abs(current_distance - last_distance_to_goal_) >
        stuck_distance_threshold)
    {
        last_movement_time_ = this->now();
        last_distance_to_goal_ = current_distance;
        return false;
    }

    // Robot hasn't made progress - check if stuck for too long
    if ((this->now() - last_movement_time_).seconds() > stuck_timeout)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Stuck detected: no progress for %.1f seconds (dist: %.3f)",
                    stuck_timeout, current_distance);
        return true;
    }

    return false;
}

bool AutonomousSlamController::hasExplorationTimedOut()
{
    return (this->now() - exploration_start_time_).seconds() >
           config_.exploration_timeout_s;
}

void AutonomousSlamController::updateProgressTracking()
{
    static geometry_msgs::msg::Point last_position;
    static bool first_update = true;

    geometry_msgs::msg::Point current_pos = poseToPoint(current_pose_);

    if (!first_update)
    {
        total_distance_traveled_ +=
            calculateDistance(last_position, current_pos);
    }

    last_position = current_pos;
    first_update = false;
}

// State transition functions
void AutonomousSlamController::transitionToState(SlamState new_state)
{
    SlamState old_state = current_state_;
    current_state_ = new_state;
    state_start_time_ = this->now();

    logStateTransition(old_state, new_state);
}

void AutonomousSlamController::transitionToMappingState(
    MappingState new_mapping_state)
{
    MappingState old_state = mapping_state_;
    mapping_state_ = new_mapping_state;

    logMappingStateTransition(old_state, new_mapping_state);
}

void AutonomousSlamController::forceOperationalMode()
{
    RCLCPP_INFO(this->get_logger(), "Forcing transition to operational mode");
    transitionToState(SlamState::OPERATIONAL);
}

// Utility calculations
double
AutonomousSlamController::calculateDistance(const geometry_msgs::msg::Point &p1,
                                            const geometry_msgs::msg::Point &p2)
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

double AutonomousSlamController::calculateYawFromPose(
    const geometry_msgs::msg::PoseStamped &pose)
{
    tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y,
                      pose.pose.orientation.z, pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

geometry_msgs::msg::Point AutonomousSlamController::poseToPoint(
    const geometry_msgs::msg::PoseStamped &pose)
{
    return pose.pose.position;
}

// Logging functions
void AutonomousSlamController::logStateTransition(SlamState old_state,
                                                  SlamState new_state)
{
    RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s",
                stateToString(old_state).c_str(),
                stateToString(new_state).c_str());
}

void AutonomousSlamController::logMappingStateTransition(MappingState old_state,
                                                         MappingState new_state)
{
    RCLCPP_DEBUG(this->get_logger(), "Mapping state transition: %s -> %s",
                 mappingStateToString(old_state).c_str(),
                 mappingStateToString(new_state).c_str());
}

void AutonomousSlamController::printExplorationStatistics()
{
    double exploration_time = (this->now() - exploration_start_time_).seconds();

    RCLCPP_INFO(this->get_logger(), "=== EXPLORATION STATISTICS ===");
    RCLCPP_INFO(this->get_logger(), "Total exploration time: %.1f seconds",
                exploration_time);
    RCLCPP_INFO(this->get_logger(), "Total frontiers explored: %d",
                total_frontiers_explored_);
    RCLCPP_INFO(this->get_logger(), "Total distance traveled: %.2f meters",
                total_distance_traveled_);
    RCLCPP_INFO(this->get_logger(), "Average speed: %.2f m/s",
                total_distance_traveled_ / exploration_time);
    RCLCPP_INFO(this->get_logger(), "===============================");
}

std::string AutonomousSlamController::stateToString(SlamState state)
{
    switch (state)
    {
    case SlamState::INITIALIZING:
        return "INITIALIZING";
    case SlamState::MAPPING:
        return "MAPPING";
    case SlamState::RETURNING_HOME:
        return "RETURNING_HOME";
    case SlamState::OPERATIONAL:
        return "OPERATIONAL";
    case SlamState::ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

std::string AutonomousSlamController::mappingStateToString(MappingState state)
{
    switch (state)
    {
    case MappingState::SEARCHING_FRONTIERS:
        return "SEARCHING_FRONTIERS";
    case MappingState::PLANNING_PATH:
        return "PLANNING_PATH";
    case MappingState::NAVIGATING:
        return "NAVIGATING";
    case MappingState::EXPLORING_AREA:
        return "EXPLORING_AREA";
    case MappingState::STUCK_RECOVERY:
        return "STUCK_RECOVERY";
    default:
        return "UNKNOWN";
    }
}

} // namespace slam