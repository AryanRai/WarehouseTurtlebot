#include "AutonomousExplorationRobot.hpp"

AutonomousExplorationRobot::AutonomousExplorationRobot(rclcpp::Node::SharedPtr node)
    : node_(node),
      is_exploring_(false),
      is_paused_(false),
      last_update_time_(node->now()),
      last_replan_time_(node->now()),
      consecutive_no_path_count_(0),
      recovery_start_time_(node->now()),
      in_recovery_(false),
      recovery_attempt_(0),
      returning_home_(false) {
    
    // Initialize SLAM components
    slam_controller_ = std::make_unique<SlamController>(node);
    exploration_planner_ = std::make_unique<ExplorationPlanner>(node);
    motion_controller_ = std::make_unique<MotionController>(node);
    
    // Create publishers (MotionController already publishes cmd_vel)
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/exploration/path", 10);
    recovery_cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    
    // Create subscribers for obstacle detection
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
            current_scan_ = msg;
        });
    
    // Initialize home position (0, 0)
    home_position_.x = 0.0;
    home_position_.y = 0.0;
    home_position_.z = 0.0;
    
    // Initialize last goal position
    last_goal_position_.x = 0.0;
    last_goal_position_.y = 0.0;
    last_goal_position_.z = 0.0;
    
    RCLCPP_INFO(node_->get_logger(), "Autonomous Exploration Robot initialized");
    RCLCPP_INFO(node_->get_logger(), "Home position set to (%.2f, %.2f)", 
               home_position_.x, home_position_.y);
}

bool AutonomousExplorationRobot::isObstacleAhead(double min_distance) {
    if (!current_scan_ || current_scan_->ranges.empty()) {
        return false;  // No scan data, assume clear
    }
    
    // Check front 60 degrees (30 degrees on each side)
    int num_ranges = current_scan_->ranges.size();
    int front_range = num_ranges / 6;  // 60 degrees out of 360
    
    // Check front center ranges
    for (int i = -front_range/2; i < front_range/2; i++) {
        int idx = (num_ranges / 2 + i + num_ranges) % num_ranges;
        float range = current_scan_->ranges[idx];
        
        // Check if valid range and too close
        if (std::isfinite(range) && range > 0.0 && range < min_distance) {
            return true;
        }
    }
    
    return false;
}

void AutonomousExplorationRobot::performRecovery() {
    // Recovery behavior: forward, backward, then forward+rotate pattern
    // This helps discover new frontiers and break oscillation loops
    // Now with obstacle avoidance!
    
    if (!in_recovery_) {
        int pattern = recovery_attempt_ % 3;
        const char* action = (pattern == 0) ? "forward" : 
                            (pattern == 1) ? "backward" : 
                            "forward + rotate";
        RCLCPP_WARN(node_->get_logger(), 
                   "Starting recovery behavior (attempt %d) - %s to find new frontiers",
                   recovery_attempt_ + 1, action);
        in_recovery_ = true;
        recovery_start_time_ = node_->now();
        motion_controller_->clearPath();  // Clear any existing path
    }
    
    double elapsed = (node_->now() - recovery_start_time_).seconds();
    
    if (elapsed < RECOVERY_DURATION) {
        int pattern = recovery_attempt_ % 3;
        double linear_speed = 0.0;
        double angular_speed = 0.0;
        
        if (pattern == 0) {
            // Forward - but stop if obstacle ahead
            if (!isObstacleAhead(0.3)) {
                linear_speed = 0.1;
                angular_speed = 0.0;
            } else {
                // Obstacle ahead, just rotate in place
                linear_speed = 0.0;
                angular_speed = 0.5;
                RCLCPP_DEBUG(node_->get_logger(), "Obstacle detected during forward recovery, rotating instead");
            }
        } else if (pattern == 1) {
            // Backward - always safe
            linear_speed = -0.1;
            angular_speed = 0.0;
        } else {
            // Forward + rotate (to break oscillation)
            if (!isObstacleAhead(0.3)) {
                linear_speed = 0.1;
                angular_speed = 0.5;
            } else {
                // Obstacle ahead, just rotate in place
                linear_speed = 0.0;
                angular_speed = 0.5;
                RCLCPP_DEBUG(node_->get_logger(), "Obstacle detected during forward+rotate recovery, rotating only");
            }
        }
        
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = node_->now();
        cmd_vel.header.frame_id = "base_footprint";
        cmd_vel.twist.linear.x = linear_speed;
        cmd_vel.twist.angular.z = angular_speed;
        recovery_cmd_vel_pub_->publish(cmd_vel);
        
        const char* action = (pattern == 0) ? "forward" : 
                            (pattern == 1) ? "backward" : 
                            "forward + rotating";
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Recovery: %s (%.1fs / %.1fs)", action, elapsed, RECOVERY_DURATION);
    } else {
        // Stop movement
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = node_->now();
        cmd_vel.header.frame_id = "base_footprint";
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        recovery_cmd_vel_pub_->publish(cmd_vel);
        
        RCLCPP_INFO(node_->get_logger(), "Recovery complete - resuming exploration");
        in_recovery_ = false;
        consecutive_no_path_count_ = 0;
        recovery_attempt_++;  // Increment for next time
        last_replan_time_ = node_->now() - rclcpp::Duration::from_seconds(MIN_REPLAN_INTERVAL);  // Allow immediate replan
        
        // Check if we've tried too many recoveries - exploration might be complete
        if (recovery_attempt_ >= MAX_RECOVERY_ATTEMPTS) {
            RCLCPP_WARN(node_->get_logger(), 
                       "Max recovery attempts reached - exploration appears complete");
            RCLCPP_INFO(node_->get_logger(), "Initiating return to home");
            returning_home_ = true;
            recovery_attempt_ = 0;  // Reset for future use
        }
    }
}

void AutonomousExplorationRobot::returnToHome() {
    if (!slam_controller_->hasValidMap() || !slam_controller_->hasValidPose()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                            "Waiting for valid map and pose to return home...");
        return;
    }
    
    auto current_map = slam_controller_->getCurrentMap();
    auto current_pose = slam_controller_->getCurrentPose();
    
    // Check if already at home
    double dx = home_position_.x - current_pose.position.x;
    double dy = home_position_.y - current_pose.position.y;
    double distance_to_home = std::sqrt(dx * dx + dy * dy);
    
    if (distance_to_home < 0.3) {  // Within 30cm of home
        RCLCPP_INFO(node_->get_logger(), "Successfully returned to home position!");
        RCLCPP_INFO(node_->get_logger(), "Exploration complete - saving final map");
        saveMap("warehouse_map_final");
        stopExploration();
        return;
    }
    
    // Plan path to home if we don't have one or if goal doesn't match home
    if (!motion_controller_->hasPath() || 
        std::abs(last_goal_position_.x - home_position_.x) > 0.1 ||
        std::abs(last_goal_position_.y - home_position_.y) > 0.1) {
        
        last_goal_position_ = home_position_;
        
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                            "Planning path to home (%.2f, %.2f), distance: %.2fm",
                            home_position_.x, home_position_.y, distance_to_home);
        
        // Use the path planner to plan a path to home
        GridCell start = PathPlanner::worldToGrid(*current_map, current_pose.position);
        GridCell goal = PathPlanner::worldToGrid(*current_map, home_position_);
        
        auto [cspace, cspace_cells] = PathPlanner::calcCSpace(*current_map, false);
        cv::Mat cost_map = PathPlanner::calcCostMap(*current_map);
        
        auto [path, cost, actual_start, actual_goal] = 
            PathPlanner::aStar(cspace, cost_map, start, goal);
        
        if (!path.empty()) {
            auto path_msg = PathPlanner::pathToMessage(*current_map, path);
            motion_controller_->setPath(path_msg);
            path_pub_->publish(path_msg);
            RCLCPP_INFO(node_->get_logger(), "Path to home planned with %zu waypoints", 
                       path_msg.poses.size());
        } else {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                                "Failed to plan path home, trying again...");
            return;
        }
    }
    
    // Follow path home
    if (motion_controller_->hasPath()) {
        auto cmd_vel = motion_controller_->computeVelocityCommand(current_pose, *current_map);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                            "Returning home: %.2fm remaining", distance_to_home);
        
        // If path following fails (returns zero velocity unexpectedly), clear path to force replan
        if (cmd_vel.linear.x == 0.0 && cmd_vel.angular.z == 0.0 && distance_to_home > 0.5) {
            RCLCPP_WARN(node_->get_logger(), "Path following failed while returning home, replanning...");
            motion_controller_->clearPath();
        }
    }
}

void AutonomousExplorationRobot::startExploration() {
    if (!is_exploring_) {
        is_exploring_ = true;
        is_paused_ = false;
        motion_controller_->setEnabled(true);
        RCLCPP_INFO(node_->get_logger(), "Starting autonomous exploration");
    }
}

void AutonomousExplorationRobot::stopExploration() {
    if (is_exploring_) {
        is_exploring_ = false;
        is_paused_ = false;
        motion_controller_->setEnabled(false);
        motion_controller_->clearPath();
        
        RCLCPP_INFO(node_->get_logger(), "Stopped autonomous exploration");
    }
}

void AutonomousExplorationRobot::pauseExploration() {
    if (is_exploring_ && !is_paused_) {
        is_paused_ = true;
        motion_controller_->setEnabled(false);
        
        RCLCPP_INFO(node_->get_logger(), "Paused autonomous exploration");
    }
}

void AutonomousExplorationRobot::resumeExploration() {
    if (is_exploring_ && is_paused_) {
        is_paused_ = false;
        motion_controller_->setEnabled(true);
        RCLCPP_INFO(node_->get_logger(), "Resumed autonomous exploration");
    }
}

bool AutonomousExplorationRobot::isExplorationComplete() const {
    return exploration_planner_->isExplorationComplete() || 
           slam_controller_->isExplorationComplete();
}

bool AutonomousExplorationRobot::isExploring() const {
    return is_exploring_ && !is_paused_;
}

void AutonomousExplorationRobot::saveMap(const std::string& map_name) {
    slam_controller_->saveMap(map_name);
}

void AutonomousExplorationRobot::update() {
    // Check if we should update based on rate
    auto current_time = node_->now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 1.0 / UPDATE_RATE) {
        return;
    }
    last_update_time_ = current_time;
    
    // If not exploring or paused, do nothing
    if (!is_exploring_ || is_paused_) {
        return;
    }
    
    // Check if exploration is complete
    if (isExplorationComplete()) {
        RCLCPP_INFO(node_->get_logger(), "Exploration complete! Saving map...");
        saveMap("warehouse_map");
        slam_controller_->setExplorationComplete(true);
        stopExploration();
        return;
    }
    
    // Check if we have valid map and pose
    if (!slam_controller_->hasValidMap() || !slam_controller_->hasValidPose()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                            "Waiting for valid map and pose from SLAM...");
        return;
    }
    
    // Get current state
    auto current_map = slam_controller_->getCurrentMap();
    auto current_pose = slam_controller_->getCurrentPose();
    
    // Check if returning home
    if (returning_home_) {
        returnToHome();
        return;  // Skip normal exploration when returning home
    }
    
    // Check if in recovery mode
    if (in_recovery_) {
        performRecovery();
        return;  // Skip normal exploration during recovery
    }
    
    // If no path or reached goal, plan new path (with minimum replan interval)
    bool should_replan = !motion_controller_->hasPath() || motion_controller_->isAtGoal();
    double time_since_last_replan = (current_time - last_replan_time_).seconds();
    
    if (should_replan && time_since_last_replan >= MIN_REPLAN_INTERVAL) {
        RCLCPP_INFO(node_->get_logger(), "Planning new exploration path...");
        
        auto new_path = exploration_planner_->planExplorationPath(*current_map, current_pose);
        
        if (!new_path.poses.empty()) {
            // Store the new goal position
            last_goal_position_ = new_path.poses.back().pose.position;
            last_replan_time_ = current_time;
            consecutive_no_path_count_ = 0;  // Reset counter on success
            
            motion_controller_->setPath(new_path);
            path_pub_->publish(new_path);
            RCLCPP_INFO(node_->get_logger(), "New path planned with %zu waypoints", 
                       new_path.poses.size());
        } else {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                "No valid exploration path found (%d/%d attempts)", 
                                consecutive_no_path_count_ + 1, MAX_NO_PATH_BEFORE_RECOVERY);
            consecutive_no_path_count_++;
            
            // If we've failed too many times, enter recovery mode
            if (consecutive_no_path_count_ >= MAX_NO_PATH_BEFORE_RECOVERY) {
                performRecovery();
                return;
            }
        }
    } else if (should_replan) {
        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Waiting %.1fs before replanning (%.1fs elapsed)", 
                             MIN_REPLAN_INTERVAL, time_since_last_replan);
    }
    
    // Compute and publish velocity command (MotionController publishes directly)
    if (motion_controller_->hasPath()) {
        auto cmd_vel = motion_controller_->computeVelocityCommand(current_pose, *current_map);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "Motion control: linear=%.3f, angular=%.3f", 
                            cmd_vel.linear.x, cmd_vel.angular.z);
    } else {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, 
                            "No path available for motion control");
    }
}
