#include "AutonomousExplorationRobot.hpp"

AutonomousExplorationRobot::AutonomousExplorationRobot(rclcpp::Node::SharedPtr node)
    : node_(node),
      is_exploring_(false),
      is_paused_(false),
      last_update_time_(node->now()),
      last_replan_time_(node->now()) {
    
    // Initialize SLAM components
    slam_controller_ = std::make_unique<SlamController>(node);
    exploration_planner_ = std::make_unique<ExplorationPlanner>(node);
    motion_controller_ = std::make_unique<MotionController>(node);
    
    // Create publishers (MotionController already publishes cmd_vel)
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/exploration/path", 10);
    
    // Initialize last goal position
    last_goal_position_.x = 0.0;
    last_goal_position_.y = 0.0;
    last_goal_position_.z = 0.0;
    
    RCLCPP_INFO(node_->get_logger(), "Autonomous Exploration Robot initialized");
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
            
            motion_controller_->setPath(new_path);
            path_pub_->publish(new_path);
            RCLCPP_INFO(node_->get_logger(), "New path planned with %zu waypoints", 
                       new_path.poses.size());
        } else {
            RCLCPP_WARN(node_->get_logger(), "No valid exploration path found");
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
