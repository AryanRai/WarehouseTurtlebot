// Exploration Planner - Frontier-based exploration
// Adapted from SLAM_Reference.md frontier_exploration.py

#ifndef EXPLORATION_PLANNER_HPP
#define EXPLORATION_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include "FrontierSearch.hpp"
#include "PathPlanner.hpp"
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
    
    // Helper functions
    std::vector<Frontier> getTopFrontiers(const std::vector<Frontier>& frontiers, int n);
    void publishCostMap(const nav_msgs::msg::OccupancyGrid& mapdata, const cv::Mat& cost_map);
};

#endif // EXPLORATION_PLANNER_HPP
