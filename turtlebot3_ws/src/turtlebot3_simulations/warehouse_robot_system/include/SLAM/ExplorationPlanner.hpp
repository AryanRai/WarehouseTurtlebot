// ============================================================================
// MTRX3760 Project 2 - 
// File: ExplorationPlanner.hpp
// Description: Header for ExplorationPlanner class. Defines frontier-based
//              exploration system for autonomous mapping and systematic
//              discovery of unknown warehouse environments.
// Author(s): Dylan George
// Last Edited: 2025-11-02
// ============================================================================

#ifndef EXPLORATION_PLANNER_HPP
#define EXPLORATION_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include "SLAM/FrontierSearch.hpp"
#include "SLAM/PathPlanner.hpp"
#include <memory>

class ExplorationPlanner {
public:
    ExplorationPlanner(rclcpp::Node::SharedPtr node);
    ~ExplorationPlanner() = default;
    
    // Main exploration function
    nav_msgs::msg::Path planExplorationPath(
        const nav_msgs::msg::OccupancyGrid& map,
        const geometry_msgs::msg::Pose& current_pose
    );
    
    // Status
    bool isExplorationComplete() const;
    int getNoFrontiersCounter() const;
    int getNoPathCounter() const;
    bool isReducingLookahead() const;  // Check if actively reducing distance for same goal
    
private:
    rclcpp::Node::SharedPtr node_;
    
    // Publishers for visualization
    rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr frontier_cells_pub_;
    rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr start_pub_;
    rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr goal_pub_;
    rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr cspace_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_map_pub_;
    
    // Configuration
    static constexpr double A_STAR_COST_WEIGHT = 10.0;
    static constexpr double FRONTIER_SIZE_COST_WEIGHT = 1.0;
    static constexpr int MAX_NUM_FRONTIERS_TO_CHECK = 8;
    static constexpr int NUM_EXPLORE_FAILS_BEFORE_FINISH = 100;  // Increased to allow more time for SLAM to build map
    
    // State
    int no_frontiers_found_counter_;
    int no_path_found_counter_;
    bool is_finished_exploring_;
    bool debug_mode_;
    
    // Dynamic lookahead distance
    geometry_msgs::msg::Point last_rejected_goal_;
    int consecutive_rejections_;
    double current_min_distance_;
    static constexpr double INITIAL_MIN_DISTANCE = 0.20;  // 20cm
    static constexpr double MINIMUM_MIN_DISTANCE = 0.05;  // 5cm absolute minimum
    static constexpr double DISTANCE_REDUCTION_STEP = 0.03;  // Reduce by 3cm each time
    
    // Helper functions
    std::vector<FrontierSearch::Frontier> getTopFrontiers(const std::vector<FrontierSearch::Frontier>& frontiers, int n);
    void publishCostMap(const nav_msgs::msg::OccupancyGrid& mapdata, const cv::Mat& cost_map);
    void updateDynamicLookahead(const geometry_msgs::msg::Point& rejected_goal);
    void resetDynamicLookahead();
};

#endif // EXPLORATION_PLANNER_HPP
