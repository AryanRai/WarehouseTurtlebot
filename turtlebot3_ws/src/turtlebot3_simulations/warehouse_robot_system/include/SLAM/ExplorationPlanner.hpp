// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: ExplorationPlanner.hpp
// Author(s): Inez Dumas, Tony Bechara, Aryan Rai, Filip Gusavac
//
// Description: exploration planner for autonomous robot mapping.

#ifndef EXPLORATION_PLANNER_HPP
#define EXPLORATION_PLANNER_HPP

#include "SLAM/FrontierSearch.hpp"
#include "SLAM/PathPlanner.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/grid_cells.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class ExplorationPlanner : public rclcpp::Node
{
    public:
        ExplorationPlanner();
        ~ExplorationPlanner() = default;

        // Public methods for backwards compatibility
        nav_msgs::msg::Path
        planExplorationPath(const nav_msgs::msg::OccupancyGrid &map,
                            const geometry_msgs::msg::Pose &current_pose);
        bool isExplorationComplete() const { return is_finished_exploring_; }
        int getNoFrontiersCounter() const
        {
            return no_frontiers_found_counter_;
        }
        int getNoPathCounter() const { return no_path_found_counter_; }
        bool isReducingLookahead() const { return consecutive_rejections_ > 0; }

    private:
        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
            pose_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
            exploration_status_sub_;

        // Publishers
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr exploration_path_pub_;
        rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr
            frontier_cells_pub_;
        rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr cspace_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
            cost_map_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
            exploration_complete_pub_;

        // Timers
        rclcpp::TimerBase::SharedPtr exploration_timer_;

        // State
        nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
        nav_msgs::msg::Path last_path_;
        nav_msgs::msg::OccupancyGrid map_;
        geometry_msgs::msg::Pose current_pose_;
        bool has_valid_map_;
        bool has_valid_pose_;
        bool is_finished_exploring_;
        bool is_planning_;

        // Counters
        int no_frontiers_found_counter_;
        int no_path_found_counter_;

        // Dynamic lookahead distance
        geometry_msgs::msg::Point last_rejected_goal_;
        int consecutive_rejections_;
        double current_min_distance_;

        // Parameters
        static constexpr double A_STAR_COST_WEIGHT = 10.0;
        static constexpr double FRONTIER_SIZE_COST_WEIGHT = 1.0;
        static constexpr int MAX_NUM_FRONTIERS_TO_CHECK = 8;
        static constexpr int NUM_EXPLORE_FAILS_BEFORE_FINISH = 100;
        static constexpr double INITIAL_MIN_DISTANCE = 0.20;
        static constexpr double MINIMUM_MIN_DISTANCE = 0.05;
        static constexpr double DISTANCE_REDUCTION_STEP = 0.03;
        static constexpr double EXPLORATION_RATE = 2.0;

        bool debug_mode_;

        // Callbacks
        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void
        explorationStatusCallback(const std_msgs::msg::Bool::SharedPtr msg);
        void explorationTimerCallback();

        // Helper functions
        void planExplorationPathInternal();
        std::vector<FrontierSearch::Frontier>
        getTopFrontiers(const std::vector<FrontierSearch::Frontier> &frontiers,
                        int n);
        void publishCostMap(const nav_msgs::msg::OccupancyGrid &mapdata,
                            const cv::Mat &cost_map);
        void
        updateDynamicLookahead(const geometry_msgs::msg::Point &rejected_goal);
        void resetDynamicLookahead();
        void declareParameters();
};

#endif // EXPLORATION_PLANNER_HPP