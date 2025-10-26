// MTRX3760 2025 Project 2: Exploration Manager
// File: exploration_manager.hpp
// Author(s): Aryan Rai
//
// Manages frontier-based exploration strategy

#ifndef EXPLORATION_MANAGER_HPP
#define EXPLORATION_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vector>
#include <memory>

#include "slam_types.hpp"
#include "frontier_search.hpp"
#include "path_planner.hpp"

namespace slam {

/**
 * @class ExplorationManager
 * @brief Manages frontier detection, selection, and path planning for exploration
 */
class ExplorationManager {
public:
    struct Config {
        double frontier_min_size;
        double a_star_cost_weight;
        double frontier_size_weight;
        int max_frontiers_to_check;
        double visited_frontier_radius;
        int max_no_frontier_count;
        int max_no_path_count;
        
        Config()
            : frontier_min_size(8),
              a_star_cost_weight(10.0),
              frontier_size_weight(1.0),
              max_frontiers_to_check(8),
              visited_frontier_radius(0.5),
              max_no_frontier_count(8),
              max_no_path_count(15) {}
    };

    struct Statistics {
        int total_frontiers_explored;
        double total_distance_traveled;
        rclcpp::Time start_time;
        
        Statistics() 
            : total_frontiers_explored(0),
              total_distance_traveled(0.0) {}
    };

    ExplorationManager(rclcpp::Node* node, const Config& config = Config());

    // Frontier management
    bool searchForFrontiers(const nav_msgs::msg::OccupancyGrid& map,
                           const geometry_msgs::msg::PoseStamped& current_pose);
    Frontier selectBestFrontier(const geometry_msgs::msg::PoseStamped& current_pose);
    std::vector<Frontier> getDetectedFrontiers() const { return detected_frontiers_; }
    bool hasFrontiers() const { return !detected_frontiers_.empty(); }
    
    // Path planning
    bool planPathToGoal(const nav_msgs::msg::OccupancyGrid& map,
                       const geometry_msgs::msg::PoseStamped& current_pose,
                       const geometry_msgs::msg::Point& goal,
                       nav_msgs::msg::Path& path_out);
    
    // Frontier tracking
    void markFrontierVisited(const geometry_msgs::msg::Point& frontier_location);
    void clearVisitedFrontiers();
    
    // Progress tracking
    void incrementNoFrontierCount() { consecutive_no_frontiers_++; }
    void resetNoFrontierCount() { consecutive_no_frontiers_ = 0; }
    int getNoFrontierCount() const { return consecutive_no_frontiers_; }
    
    void incrementNoPathCount() { consecutive_no_paths_++; }
    void resetNoPathCount() { consecutive_no_paths_ = 0; }
    int getNoPathCount() const { return consecutive_no_paths_; }
    bool hasExceededNoPathLimit() const;
    
    // Statistics
    void incrementFrontiersExplored() { stats_.total_frontiers_explored++; }
    void updateDistanceTraveled(double distance) { stats_.total_distance_traveled += distance; }
    Statistics getStatistics() const { return stats_; }
    void printStatistics();

private:
    rclcpp::Node* node_;
    Config config_;
    Statistics stats_;
    
    // Frontier exploration components
    std::unique_ptr<FrontierSearch> frontier_searcher_;
    std::unique_ptr<PathPlanner> path_planner_;
    
    // Frontier data
    std::vector<Frontier> detected_frontiers_;
    std::vector<geometry_msgs::msg::Point> visited_frontiers_;
    
    // Progress tracking
    int consecutive_no_frontiers_;
    int consecutive_no_paths_;
    
    // Helper functions
    bool isFrontierVisited(const Frontier& frontier) const;
    double calculateDistance(const geometry_msgs::msg::Point& p1,
                           const geometry_msgs::msg::Point& p2) const;
};

} // namespace slam

#endif // EXPLORATION_MANAGER_HPP