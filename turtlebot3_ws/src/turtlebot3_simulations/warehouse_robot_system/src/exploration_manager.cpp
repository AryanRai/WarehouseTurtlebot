// MTRX3760 2025 Project 2: Exploration Manager
// File: exploration_manager.cpp
// Author(s): Aryan Rai
//
// Implementation of frontier-based exploration

#include "exploration_manager.hpp"
#include <algorithm>
#include <cmath>

namespace slam {

ExplorationManager::ExplorationManager(rclcpp::Node* node, const Config& config)
    : node_(node),
      config_(config),
      consecutive_no_frontiers_(0),
      consecutive_no_paths_(0)
{
    frontier_searcher_ = std::make_unique<FrontierSearch>();
    path_planner_ = std::make_unique<PathPlanner>();
    stats_.start_time = node_->now();
}

bool ExplorationManager::searchForFrontiers(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::PoseStamped& current_pose) {
    
    // Convert current pose to grid coordinates
    geometry_msgs::msg::Point pos = current_pose.pose.position;
    GridCell start_cell = PathPlanner::worldToGrid(map, pos);
    
    // Search for frontiers
    auto [frontier_list, frontier_cells] = frontier_searcher_->search(map, start_cell, false);
    
    detected_frontiers_.clear();
    for (const auto& frontier : frontier_list.frontiers) {
        if (frontier.size >= config_.frontier_min_size) {
            detected_frontiers_.push_back(frontier);
        }
    }

    RCLCPP_DEBUG(node_->get_logger(), "Found %zu valid frontiers", 
                 detected_frontiers_.size());
    return !detected_frontiers_.empty();
}

Frontier ExplorationManager::selectBestFrontier(
    const geometry_msgs::msg::PoseStamped& current_pose) {
    
    if (detected_frontiers_.empty()) {
        return Frontier{};
    }

    // Filter out recently visited frontiers
    std::vector<Frontier> unvisited_frontiers;
    for (const auto& frontier : detected_frontiers_) {
        if (!isFrontierVisited(frontier)) {
            unvisited_frontiers.push_back(frontier);
        }
    }
    
    // If all frontiers visited, clear history and use all
    if (unvisited_frontiers.empty()) {
        RCLCPP_INFO(node_->get_logger(), "All frontiers visited, clearing history");
        visited_frontiers_.clear();
        unvisited_frontiers = detected_frontiers_;
    }

    // Sort frontiers by size (larger is better)
    std::vector<Frontier> sorted_frontiers = unvisited_frontiers;
    std::sort(sorted_frontiers.begin(), sorted_frontiers.end(),
              [](const Frontier& a, const Frontier& b) {
                  return a.size > b.size;
              });

    // Consider only top frontiers
    int max_to_check = std::min(static_cast<int>(sorted_frontiers.size()),
                               config_.max_frontiers_to_check);
    
    double best_cost = std::numeric_limits<double>::max();
    Frontier best_frontier = sorted_frontiers[0];
    
    geometry_msgs::msg::Point current_pos = current_pose.pose.position;
    
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

bool ExplorationManager::planPathToGoal(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::PoseStamped& current_pose,
    const geometry_msgs::msg::Point& goal,
    nav_msgs::msg::Path& path_out) {
    
    GridCell start = PathPlanner::worldToGrid(map, current_pose.pose.position);
    GridCell goal_cell = PathPlanner::worldToGrid(map, goal);

    // Calculate C-space (inflated obstacles for safe navigation)
    auto [cspace, cspace_cells] = PathPlanner::calcCspace(map, false);
    
    // Calculate cost map (prefers middle of hallways, avoids walls)
    cv::Mat cost_map = PathPlanner::calcCostMap(map);
    
    // Plan path using A* with C-space AND cost map
    auto [path, cost, actual_start, actual_goal] =
        PathPlanner::aStar(cspace, cost_map, start, goal_cell);

    if (!path.has_value() || path->empty()) {
        return false;
    }

    // Convert path to ROS message
    path_out.header.frame_id = "map";
    path_out.header.stamp = node_->now();
    path_out.poses.clear();

    for (const auto& cell : path.value()) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_out.header;
        
        geometry_msgs::msg::Point world_point = PathPlanner::gridToWorld(map, cell);
        pose.pose.position = world_point;
        pose.pose.orientation.w = 1.0;
        
        path_out.poses.push_back(pose);
    }

    return true;
}

void ExplorationManager::markFrontierVisited(const geometry_msgs::msg::Point& frontier_location) {
    visited_frontiers_.push_back(frontier_location);
}

void ExplorationManager::clearVisitedFrontiers() {
    visited_frontiers_.clear();
}

bool ExplorationManager::hasExceededNoPathLimit() const {
    return consecutive_no_paths_ >= config_.max_no_path_count;
}

void ExplorationManager::printStatistics() {
    double exploration_time = (node_->now() - stats_.start_time).seconds();
    
    RCLCPP_INFO(node_->get_logger(), "=== EXPLORATION STATISTICS ===");
    RCLCPP_INFO(node_->get_logger(), "Total exploration time: %.1f seconds", exploration_time);
    RCLCPP_INFO(node_->get_logger(), "Total frontiers explored: %d", 
                stats_.total_frontiers_explored);
    RCLCPP_INFO(node_->get_logger(), "Total distance traveled: %.2f meters",
                stats_.total_distance_traveled);
    if (exploration_time > 0) {
        RCLCPP_INFO(node_->get_logger(), "Average speed: %.2f m/s",
                    stats_.total_distance_traveled / exploration_time);
    }
    RCLCPP_INFO(node_->get_logger(), "===============================");
}

bool ExplorationManager::isFrontierVisited(const Frontier& frontier) const {
    for (const auto& visited : visited_frontiers_) {
        double dist = calculateDistance(frontier.centroid, visited);
        if (dist < config_.visited_frontier_radius) {
            return true;
        }
    }
    return false;
}

double ExplorationManager::calculateDistance(
    const geometry_msgs::msg::Point& p1,
    const geometry_msgs::msg::Point& p2) const {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace slam