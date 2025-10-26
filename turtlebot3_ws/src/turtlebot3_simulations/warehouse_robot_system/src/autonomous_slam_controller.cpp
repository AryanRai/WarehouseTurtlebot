// MTRX3760 2025 Project 2: Autonomous SLAM Controller (Refactored)
// File: autonomous_slam_controller.cpp
// Author(s): Aryan Rai
//
// Refactored implementation using separate manager classes

#include "autonomous_slam_controller.hpp"

namespace slam {

AutonomousSlamController::AutonomousSlamController()
    : Node("autonomous_slam_controller")
{
    // Initialize manager components
    state_manager_ = std::make_unique<SlamStateManager>(this);
    
    NavigationController::Config nav_config;
    navigation_controller_ = std::make_unique<NavigationController>(this, nav_config);
    
    exp_config_ = ExplorationManager::Config{};  // Store as member variable
    exploration_manager_ = std::make_unique<ExplorationManager>(this, exp_config_);

    // Set origin point (0, 0)
    origin_point_.x = 0.0;
    origin_point_.y = 0.0;
    origin_point_.z = 0.0;

    // Create publishers
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/slam/planned_path", 10);
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/slam/current_goal", 10);

    // Create subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&AutonomousSlamController::mapCallback, 
        this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&AutonomousSlamController::odomCallback, 
        this, std::placeholders::_1));

    // Create control timer
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / config_.control_rate_hz)),
        std::bind(&AutonomousSlamController::executeStateMachine, this));

    RCLCPP_INFO(this->get_logger(), "Autonomous SLAM Controller initialized (Refactored)");
    RCLCPP_INFO(this->get_logger(), "Starting in INITIALIZING state");
}

AutonomousSlamController::~AutonomousSlamController() {
    navigation_controller_->stopRobot();
    RCLCPP_INFO(this->get_logger(), "Autonomous SLAM Controller shutdown");
}

void AutonomousSlamController::run() {
    RCLCPP_INFO(this->get_logger(), "Starting autonomous SLAM exploration...");
    rclcpp::spin(shared_from_this());
}

void AutonomousSlamController::executeStateMachine() {
    switch (state_manager_->getCurrentState()) {
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
    if (!current_map_) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Waiting for map data...");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Map data received, starting exploration");
    state_manager_->transitionToState(SlamState::MAPPING);
}

void AutonomousSlamController::handleMappingState() {
    // Check for timeout
    if (state_manager_->hasExplorationTimedOut(config_.exploration_timeout_s)) {
        RCLCPP_WARN(this->get_logger(), "Exploration timeout reached, returning home");
        state_manager_->transitionToState(SlamState::RETURNING_HOME);
        return;
    }

    // Execute mapping sub-state machine
    switch (state_manager_->getMappingState()) {
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
    // Check if we have pose data
    if (!navigation_controller_->updateCurrentPose()) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Waiting for map frame to be created by SLAM...");
        return;
    }

    auto current_pose = navigation_controller_->getCurrentPose();
    
    if (!exploration_manager_->searchForFrontiers(*current_map_, current_pose)) {
        exploration_manager_->incrementNoFrontierCount();
        RCLCPP_DEBUG(this->get_logger(), "No frontiers found (count: %d)",
                     exploration_manager_->getNoFrontierCount());
        
        // Before giving up, do a final sweep
        if (exploration_manager_->getNoFrontierCount() >= 
            exp_config_.max_no_frontier_count / 2 && 
            exploration_manager_->getNoFrontierCount() < 
            exp_config_.max_no_frontier_count) {
            RCLCPP_INFO(this->get_logger(), 
                       "No frontiers visible, performing final sweep...");
            state_manager_->transitionToMappingState(MappingState::EXPLORING_AREA);
            return;
        }
        
        if (exploration_manager_->getNoFrontierCount() >= 
            exp_config_.max_no_frontier_count) {
            RCLCPP_INFO(this->get_logger(), 
                       "No more frontiers found after final sweep, exploration complete!");
            exploration_manager_->printStatistics();
            state_manager_->transitionToState(SlamState::RETURNING_HOME);
            return;
        }
        
        // Try local exploration
        state_manager_->transitionToMappingState(MappingState::EXPLORING_AREA);
        return;
    }

    exploration_manager_->resetNoFrontierCount();
    state_manager_->transitionToMappingState(MappingState::PLANNING_PATH);
}

void AutonomousSlamController::handlePlanningPath() {
    if (!exploration_manager_->hasFrontiers()) {
        state_manager_->transitionToMappingState(MappingState::SEARCHING_FRONTIERS);
        return;
    }

    auto current_pose = navigation_controller_->getCurrentPose();
    Frontier best_frontier = exploration_manager_->selectBestFrontier(current_pose);
    
    geometry_msgs::msg::Point goal;
    goal.x = best_frontier.centroid.x;
    goal.y = best_frontier.centroid.y;
    goal.z = 0.0;

    nav_msgs::msg::Path planned_path;
    if (!exploration_manager_->planPathToGoal(*current_map_, current_pose, goal, planned_path)) {
        exploration_manager_->incrementNoPathCount();
        RCLCPP_WARN(this->get_logger(), "Failed to plan path to frontier (count: %d)",
                    exploration_manager_->getNoPathCount());
        
        if (exploration_manager_->hasExceededNoPathLimit()) {
            RCLCPP_WARN(this->get_logger(), 
                       "Too many path planning failures, trying recovery");
            state_manager_->transitionToMappingState(MappingState::STUCK_RECOVERY);
            return;
        }
        
        state_manager_->transitionToMappingState(MappingState::SEARCHING_FRONTIERS);
        return;
    }

    exploration_manager_->resetNoPathCount();
    
    // Set navigation goal and path
    navigation_controller_->setCurrentGoal(goal);
    navigation_controller_->setCurrentPath(planned_path);
    navigation_controller_->resetStuckDetection();
    
    // Publish for visualization
    path_pub_->publish(planned_path);
    publishGoalVisualization(goal);

    RCLCPP_INFO(this->get_logger(), "Planned path to frontier at (%.2f, %.2f), size: %d",
                goal.x, goal.y, best_frontier.size);
    
    state_manager_->transitionToMappingState(MappingState::NAVIGATING);
}

void AutonomousSlamController::handleNavigating() {
    if (!navigation_controller_->updateCurrentPose()) {
        RCLCPP_WARN(this->get_logger(), "Lost pose during navigation");
        navigation_controller_->stopRobot();
        return;
    }

    // Check if path is empty
    if (!navigation_controller_->hasPath()) {
        RCLCPP_WARN(this->get_logger(), "Path became empty during navigation");
        state_manager_->transitionToMappingState(MappingState::SEARCHING_FRONTIERS);
        return;
    }

    // Check if goal reached
    if (navigation_controller_->isCurrentGoalReached()) {
        auto goal = navigation_controller_->getCurrentGoal();
        RCLCPP_INFO(this->get_logger(), "Reached frontier goal at (%.2f, %.2f)",
                    goal.x, goal.y);
        exploration_manager_->incrementFrontiersExplored();
        exploration_manager_->markFrontierVisited(goal);
        
        navigation_controller_->stopRobot();
        state_manager_->transitionToMappingState(MappingState::EXPLORING_AREA);
        return;
    }

    // Check if stuck
    if (navigation_controller_->isRobotStuck()) {
        RCLCPP_WARN(this->get_logger(), "Robot appears stuck during navigation");
        navigation_controller_->stopRobot();
        state_manager_->transitionToMappingState(MappingState::STUCK_RECOVERY);
        return;
    }

    // Follow path
    if (!navigation_controller_->followPath()) {
        RCLCPP_WARN(this->get_logger(), "Path following failed, replanning");
        navigation_controller_->stopRobot();
        state_manager_->transitionToMappingState(MappingState::PLANNING_PATH);
        return;
    }

    updateTravelDistance();
}

void AutonomousSlamController::handleExploringArea() {
    // Simple exploration: rotate in place to scan area
    static rclcpp::Time rotation_start = this->now();
    static int scan_count = 0;
    
    const double rotation_duration = 2.0;
    const double rotation_speed = 0.5;
    
    if ((this->now() - rotation_start).seconds() < rotation_duration) {
        navigation_controller_->publishVelocityCommand(0.0, rotation_speed);
        return;
    }
    
    navigation_controller_->stopRobot();
    rotation_start = this->now();
    scan_count++;
    
    RCLCPP_INFO(this->get_logger(), "Exploration scan complete, searching for new frontiers");
    
    if (scan_count > 1) {
        scan_count = 0;
        exploration_manager_->incrementNoFrontierCount();
    }
    
    state_manager_->transitionToMappingState(MappingState::SEARCHING_FRONTIERS);
}

void AutonomousSlamController::handleStuckRecovery() {
    navigation_controller_->executeRecoveryBehavior();
    
    // Check if recovery is complete (simplified - should track internally)
    static int recovery_count = 0;
    recovery_count++;
    
    if (recovery_count > 15) {  // ~3 seconds at 2 Hz
        recovery_count = 0;
        RCLCPP_INFO(this->get_logger(), "Recovery complete, resuming exploration");
        navigation_controller_->clearPath();
        state_manager_->transitionToMappingState(MappingState::SEARCHING_FRONTIERS);
    }
}

void AutonomousSlamController::handleReturningHomeState() {
    if (!navigation_controller_->updateCurrentPose()) {
        RCLCPP_WARN(this->get_logger(), "Lost pose while returning home");
        return;
    }

    // Check if at origin
    if (navigation_controller_->isGoalReached(origin_point_, 0.2)) {
        RCLCPP_INFO(this->get_logger(), "Successfully returned to origin");
        navigation_controller_->stopRobot();
        exploration_manager_->printStatistics();
        state_manager_->transitionToState(SlamState::OPERATIONAL);
        return;
    }

    // Plan path to origin if needed
    if (!navigation_controller_->hasPath() ||
        !navigation_controller_->isGoalReached(
            navigation_controller_->getCurrentGoal(), 0.5)) {
        
        auto current_pose = navigation_controller_->getCurrentPose();
        nav_msgs::msg::Path path_home;
        
        if (!exploration_manager_->planPathToGoal(*current_map_, current_pose, 
                                                  origin_point_, path_home)) {
            RCLCPP_WARN(this->get_logger(), "Failed to plan path home, trying again...");
            return;
        }
        
        navigation_controller_->setCurrentGoal(origin_point_);
        navigation_controller_->setCurrentPath(path_home);
        path_pub_->publish(path_home);
        RCLCPP_INFO(this->get_logger(), "Planned path home to origin (0, 0)");
    }

    // Follow path home
    if (!navigation_controller_->followPath()) {
        RCLCPP_WARN(this->get_logger(), "Path following failed while returning home");
        navigation_controller_->clearPath();
    }
}

void AutonomousSlamController::handleOperationalState() {
    navigation_controller_->stopRobot();
    
    static rclcpp::Time last_status_log = this->now();
    if ((this->now() - last_status_log).seconds() > 10.0) {
        RCLCPP_INFO(this->get_logger(), "SLAM complete - Robot operational at origin");
        last_status_log = this->now();
    }
}

void AutonomousSlamController::handleErrorState() {
    navigation_controller_->stopRobot();
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "SLAM system in error state");
}

// Callback implementations
void AutonomousSlamController::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = msg;
}

void AutonomousSlamController::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Odometry callback - pose is updated via TF in navigation controller
}

// Utility function implementations
void AutonomousSlamController::publishGoalVisualization(
    const geometry_msgs::msg::Point& goal) {
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = this->now();
    goal_pose.pose.position = goal;
    goal_pose.pose.orientation.w = 1.0;
    goal_pub_->publish(goal_pose);
}

void AutonomousSlamController::updateTravelDistance() {
    static geometry_msgs::msg::Point last_position;
    static bool first_update = true;
    
    auto current_pos = navigation_controller_->getCurrentPosition();
    
    if (!first_update) {
        double dx = current_pos.x - last_position.x;
        double dy = current_pos.y - last_position.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        exploration_manager_->updateDistanceTraveled(distance);
    }
    
    last_position = current_pos;
    first_update = false;
}

} // namespace slam