// MTRX3760 2025 Project 2: Warehouse Robot
// File: ExplorationPlanner.cpp
// Author(s): Aryan Rai
//
// Exploration Planner - Selects exploration goals using frontier detection

#ifndef EXPLORATION_PLANNER_HPP
#define EXPLORATION_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "FrontierSearch.hpp"
#include "PathPlanner.hpp"
#include "slam_types.hpp"

namespace slam {

class ExplorationPlanner : public rclcpp::Node {
public:
    ExplorationPlanner();

    enum ExplorationState {
        WAITING_FOR_DATA,
        SEARCHING_FRONTIERS,
        GOAL_PUBLISHED,
        EXPLORATION_COMPLETE,
        RETURNING_HOME
    };

private:
    struct Config {
        double frontier_min_size;
        int max_no_frontier_count;
        double visited_frontier_radius;
        double exploration_timeout_s;
    } config_;
    
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    void motionStatusCallback(const std_msgs::msg::String::SharedPtr msg);
    
    void planningLoop();
    
    void searchAndPublishFrontier();
    
    Frontier selectBestFrontier(const std::vector<Frontier>& frontiers);
    
    void returnHome();
    
    double calculateDistance(const geometry_msgs::msg::Point& p1, 
                           const geometry_msgs::msg::Point& p2);
    
    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::TimerBase::SharedPtr planning_timer_;
    
    // State
    ExplorationState state_;
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
    
    // Exploration data
    std::unique_ptr<FrontierSearch> frontier_searcher_;
    std::vector<geometry_msgs::msg::Point> visited_frontiers_;
    geometry_msgs::msg::Point origin_point_;
    int consecutive_no_frontiers_;
    int total_frontiers_explored_;
    rclcpp::Time exploration_start_time_;
};

} // namespace slam

#endif // EXPLORATION_PLANNER_HPP