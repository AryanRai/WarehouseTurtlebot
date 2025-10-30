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
      returning_home_(false),
      at_home_(false),
      consecutive_no_frontiers_count_(0),
      return_home_failures_(0),
      last_return_home_progress_(node->now()),
      last_distance_to_home_(std::numeric_limits<double>::max()) {
    
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
    }
}

void AutonomousExplorationRobot::returnToHome() {
    // If already at home, do nothing
    if (at_home_) {
        return;
    }
    
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
    
    if (distance_to_home < 0.10) {  // Within 10cm of home
        at_home_ = true;
        motion_controller_->clearPath();
        
        // Send zero velocity to stop the robot
        geometry_msgs::msg::TwistStamped stop_cmd;
        stop_cmd.header.stamp = node_->now();
        stop_cmd.header.frame_id = "base_footprint";
        stop_cmd.twist.linear.x = 0.0;
        stop_cmd.twist.angular.z = 0.0;
        recovery_cmd_vel_pub_->publish(stop_cmd);
        
        RCLCPP_INFO(node_->get_logger(), "Successfully returned to home position! (distance: %.3fm)", distance_to_home);
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
        GridCell goal_cell = PathPlanner::worldToGrid(*current_map, home_position_);
        
        auto [cspace, cspace_cells] = PathPlanner::calcCSpace(*current_map, false);
        cv::Mat cost_map = PathPlanner::calcCostMap(*current_map);
        
        // First try: plan to exact home position
        auto [path, cost, actual_start, actual_goal] = 
            PathPlanner::aStar(cspace, cost_map, start, goal_cell);
        
        // If that fails, try to find nearest walkable cell to home
        if (path.empty()) {
            static rclcpp::Time last_search_warn = node_->now();
            if ((node_->now() - last_search_warn).seconds() > 5.0) {
                RCLCPP_WARN(node_->get_logger(), "Cannot reach exact home position, searching for nearest accessible point...");
                last_search_warn = node_->now();
            }
            
            // Search in expanding radius around home (check only 8 directions per radius for speed)
            bool found_goal = false;
            std::vector<std::pair<int,int>> directions = {
                {1,0}, {-1,0}, {0,1}, {0,-1},  // Cardinal
                {1,1}, {1,-1}, {-1,1}, {-1,-1}  // Diagonal
            };
            
            for (int radius = 1; radius <= 30 && !found_goal; radius++) {
                for (const auto& [dx_unit, dy_unit] : directions) {
                    GridCell candidate = {
                        goal_cell.first + dx_unit * radius, 
                        goal_cell.second + dy_unit * radius
                    };
                    
                    // Check if this cell is walkable
                    if (PathPlanner::isCellInBounds(*current_map, candidate) &&
                        PathPlanner::isCellWalkable(*current_map, candidate)) {
                        
                        // Try to plan to this cell
                        auto [alt_path, alt_cost, alt_start, alt_goal] = 
                            PathPlanner::aStar(cspace, cost_map, start, candidate);
                        
                        if (!alt_path.empty()) {
                            path = alt_path;
                            found_goal = true;
                            geometry_msgs::msg::Point alt_home = PathPlanner::gridToWorld(*current_map, candidate);
                            RCLCPP_INFO(node_->get_logger(), 
                                       "Found accessible point near home at (%.2f, %.2f), %.2fm from origin",
                                       alt_home.x, alt_home.y, 
                                       std::sqrt(alt_home.x*alt_home.x + alt_home.y*alt_home.y));
                            break;
                        }
                    }
                }
            }
        }
        
        if (!path.empty()) {
            auto path_msg = PathPlanner::pathToMessage(*current_map, path);
            motion_controller_->setPath(path_msg);
            path_pub_->publish(path_msg);
            RCLCPP_INFO(node_->get_logger(), "Path to home planned with %zu waypoints", 
                       path_msg.poses.size());
        } else {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "Cannot find any path home! Robot may be trapped.");
            return;
        }
    }
    
    // Check if making progress toward home
    if (std::abs(distance_to_home - last_distance_to_home_) > 0.05) {
        // Made progress
        last_return_home_progress_ = node_->now();
        last_distance_to_home_ = distance_to_home;
        return_home_failures_ = 0;
    } else if ((node_->now() - last_return_home_progress_).seconds() > 5.0) {
        // Stuck for 5 seconds
        return_home_failures_++;
        last_return_home_progress_ = node_->now();
        
        if (return_home_failures_ >= 3) {
            RCLCPP_WARN(node_->get_logger(), "Stuck while returning home, performing recovery...");
            motion_controller_->clearPath();
            
            // Do a simple recovery: back up and rotate
            geometry_msgs::msg::TwistStamped cmd_vel;
            cmd_vel.header.stamp = node_->now();
            cmd_vel.header.frame_id = "base_footprint";
            cmd_vel.twist.linear.x = -0.1;
            cmd_vel.twist.angular.z = 0.5;
            recovery_cmd_vel_pub_->publish(cmd_vel);
            
            // Wait a bit
            rclcpp::sleep_for(std::chrono::seconds(2));
            
            // Stop
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.angular.z = 0.0;
            recovery_cmd_vel_pub_->publish(cmd_vel);
            
            return_home_failures_ = 0;
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
            RCLCPP_DEBUG(node_->get_logger(), "Path following returned zero velocity, clearing path");
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
            consecutive_no_frontiers_count_ = 0;  // Reset no-frontiers counter
            
            motion_controller_->setPath(new_path);
            path_pub_->publish(new_path);
            RCLCPP_INFO(node_->get_logger(), "New path planned with %zu waypoints", 
                       new_path.poses.size());
        } else {
            // Track if this is due to no frontiers
            if (exploration_planner_->getNoFrontiersCounter() > 0) {
                consecutive_no_frontiers_count_++;
            }
            
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                "No valid exploration path found (%d/%d attempts, %d no-frontiers)", 
                                consecutive_no_path_count_ + 1, MAX_NO_PATH_BEFORE_RECOVERY,
                                consecutive_no_frontiers_count_);
            consecutive_no_path_count_++;
            
            // If we've had many "no frontiers" results, reduce recovery attempts
            int max_recovery = MAX_RECOVERY_ATTEMPTS;
            if (consecutive_no_frontiers_count_ >= 10) {
                max_recovery = 5;  // Reduce to 5 attempts when map seems complete
                RCLCPP_DEBUG(node_->get_logger(), "Map appears complete, reducing recovery attempts to %d", max_recovery);
            }
            
            // If we've failed too many times, enter recovery mode
            if (consecutive_no_path_count_ >= MAX_NO_PATH_BEFORE_RECOVERY) {
                // Check if we should give up
                if (recovery_attempt_ >= max_recovery) {
                    RCLCPP_WARN(node_->get_logger(), 
                               "Max recovery attempts reached - exploration appears complete");
                    RCLCPP_INFO(node_->get_logger(), "Initiating return to home");
                    returning_home_ = true;
                    recovery_attempt_ = 0;
                    return;
                }
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
