// MTRX3760 2025 Project 2: Warehouse Robot
// File: ExplorationPlanner.cpp
// Author(s): Aryan Rai
//
// Exploration Planner - Selects exploration goals using frontier detection

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "ExplorationPlanner.hpp"
#include "FrontierSearch.hpp"
#include "PathPlanner.hpp"
#include "slam_types.hpp"

namespace slam {

ExplorationPlanner::ExplorationPlanner() : Node("exploration_planner"), 
                        state_(ExplorationState::WAITING_FOR_DATA),
                        consecutive_no_frontiers_(0),
                        total_frontiers_explored_(0) 
{
    RCLCPP_INFO(this->get_logger(), "Exploration Planner starting...");
    
    // Subscribers - receive map and pose from SlamController
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&ExplorationPlanner::mapCallback, this, std::placeholders::_1));
    
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot_pose", 10,
        std::bind(&ExplorationPlanner::poseCallback, this, std::placeholders::_1));
    
    // Subscriber - receive motion status from MotionController
    motion_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/motion_status", 10,
        std::bind(&ExplorationPlanner::motionStatusCallback, this, std::placeholders::_1));
    
    // Publisher - send exploration goals to PathPlanner
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/exploration_goal", 10);
    
    // Timer for exploration planning
    planning_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ExplorationPlanner::planningLoop, this));
    
    // Initialize frontier searcher
    frontier_searcher_ = std::make_unique<FrontierSearch>();
    
    // Configuration
    config_.frontier_min_size = 8;
    config_.max_no_frontier_count = 8;
    config_.visited_frontier_radius = 0.5;
    config_.exploration_timeout_s = 300.0;
    
    origin_point_.x = 0.0;
    origin_point_.y = 0.0;
    origin_point_.z = 0.0;
    
    exploration_start_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Exploration Planner initialized");
}

void ExplorationPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
{
    current_map_ = msg;
    if (state_ == ExplorationState::WAITING_FOR_DATA && current_pose_) {
        state_ = ExplorationState::SEARCHING_FRONTIERS;
        RCLCPP_INFO(this->get_logger(), "Received map, starting exploration");
    }
}

void ExplorationPlanner::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{
    current_pose_ = msg;
    if (state_ == ExplorationState::WAITING_FOR_DATA && current_map_) {
        state_ = ExplorationState::SEARCHING_FRONTIERS;
        RCLCPP_INFO(this->get_logger(), "Received pose, starting exploration");
    }
}

void ExplorationPlanner::motionStatusCallback(const std_msgs::msg::String::SharedPtr msg) 
{
    // When robot reaches goal, search for new frontiers
    if (msg->data == "goal_reached" && state_ == ExplorationState::GOAL_PUBLISHED) {
        total_frontiers_explored_++;
        RCLCPP_INFO(this->get_logger(), "Goal reached, searching for new frontier");
        state_ = ExplorationState::SEARCHING_FRONTIERS;
    }
}

void ExplorationPlanner::planningLoop() 
{
    switch (state_) {
        case ExplorationState::WAITING_FOR_DATA:
            // Wait for map and pose
            break;
            
        case ExplorationState::SEARCHING_FRONTIERS:
            searchAndPublishFrontier();
            break;
            
        case ExplorationState::GOAL_PUBLISHED:
            // Wait for robot to reach goal
            break;
            
        case ExplorationState::EXPLORATION_COMPLETE:
            returnHome();
            break;
            
        case ExplorationState::RETURNING_HOME:
            // Wait for robot to reach home
            break;
    }
    
    // Check for timeout
    if ((this->now() - exploration_start_time_).seconds() > config_.exploration_timeout_s) {
        if (state_ != ExplorationState::EXPLORATION_COMPLETE && 
            state_ != ExplorationState::RETURNING_HOME) {
            RCLCPP_WARN(this->get_logger(), "Exploration timeout, returning home");
            state_ = ExplorationState::EXPLORATION_COMPLETE;
        }
    }
}

void ExplorationPlanner::searchAndPublishFrontier() 
{
    if (!current_map_ || !current_pose_) {
        return;
    }
    
    // Search for frontiers
    GridCell start_cell = PathPlanner::worldToGrid(*current_map_, current_pose_->pose.position);
    auto [frontier_list, frontier_cells] = frontier_searcher_->search(*current_map_, start_cell, false);
    
    // Filter frontiers by size
    std::vector<Frontier> valid_frontiers;
    for (const auto& frontier : frontier_list.frontiers) {
        if (frontier.size >= config_.frontier_min_size) {
            // Check if not recently visited
            bool is_visited = false;
            for (const auto& visited : visited_frontiers_) {
                double dist = calculateDistance(frontier.centroid, visited);
                if (dist < config_.visited_frontier_radius) {
                    is_visited = true;
                    break;
                }
            }
            if (!is_visited) {
                valid_frontiers.push_back(frontier);
            }
        }
    }
    
    if (valid_frontiers.empty()) {
        consecutive_no_frontiers_++;
        RCLCPP_DEBUG(this->get_logger(), "No valid frontiers found (count: %d)", 
                    consecutive_no_frontiers_);
        
        if (consecutive_no_frontiers_ >= config_.max_no_frontier_count) {
            RCLCPP_INFO(this->get_logger(), "Exploration complete! Explored %d frontiers", 
                        total_frontiers_explored_);
            state_ = ExplorationState::EXPLORATION_COMPLETE;
        }
        return;
    }
    
    // Select best frontier
    consecutive_no_frontiers_ = 0;
    Frontier best_frontier = selectBestFrontier(valid_frontiers);
    
    // Mark as visited
    visited_frontiers_.push_back(best_frontier.centroid);
    
    // Publish goal
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    goal.pose.position = best_frontier.centroid;
    goal.pose.orientation.w = 1.0;
    
    goal_pub_->publish(goal);
    state_ = ExplorationState::GOAL_PUBLISHED;
    
    RCLCPP_INFO(this->get_logger(), "Published exploration goal at (%.2f, %.2f), size: %d",
                best_frontier.centroid.x, best_frontier.centroid.y, best_frontier.size);
}

Frontier ExplorationPlanner::selectBestFrontier(const std::vector<Frontier>& frontiers) 
{
    if (frontiers.empty()) {
        return Frontier{};
    }
    
    // Sort by size (larger is better)
    std::vector<Frontier> sorted = frontiers;
    std::sort(sorted.begin(), sorted.end(),
                [](const Frontier& a, const Frontier& b) { return a.size > b.size; });
    
    // Consider distance and size
    double best_cost = std::numeric_limits<double>::max();
    Frontier best_frontier = sorted[0];
    
    int max_to_check = std::min(static_cast<int>(sorted.size()), 8);
    for (int i = 0; i < max_to_check; ++i) {
        const auto& frontier = sorted[i];
        double distance = calculateDistance(current_pose_->pose.position, frontier.centroid);
        double cost = 10.0 * distance + 1.0 / frontier.size;
        
        if (cost < best_cost) {
            best_cost = cost;
            best_frontier = frontier;
        }
    }
    
    return best_frontier;
}

void ExplorationPlanner::returnHome() 
{
    if (state_ != ExplorationState::EXPLORATION_COMPLETE) {
        return;
    }
    
    // Publish goal to return to origin
    geometry_msgs::msg::PoseStamped home_goal;
    home_goal.header.frame_id = "map";
    home_goal.header.stamp = this->now();
    home_goal.pose.position = origin_point_;
    home_goal.pose.orientation.w = 1.0;
    
    goal_pub_->publish(home_goal);
    state_ = ExplorationState::RETURNING_HOME;
    
    RCLCPP_INFO(this->get_logger(), "Returning to home position (0, 0)");
}

double ExplorationPlanner::calculateDistance(const geometry_msgs::msg::Point& p1, 
                        const geometry_msgs::msg::Point& p2) 
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}


} // namespace slam
