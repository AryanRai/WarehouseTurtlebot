// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: ExplorationPlanner.cpp
// Author(s): Inez Dumas, Dylan George, Aryan Rai
//
// Description: Implementation of exploration planner for autonomous robot
// mapping.

#include "SLAM/ExplorationPlanner.hpp"
#include <algorithm>

ExplorationPlanner::ExplorationPlanner()
    : Node("exploration_planner"), has_valid_map_(false),
      has_valid_pose_(false), is_finished_exploring_(false),
      is_planning_(false), no_frontiers_found_counter_(0),
      no_path_found_counter_(0), consecutive_rejections_(0),
      current_min_distance_(INITIAL_MIN_DISTANCE)
{

    last_rejected_goal_.x = 0.0;
    last_rejected_goal_.y = 0.0;
    last_rejected_goal_.z = 0.0;

    // Declare parameters
    declareParameters();

    debug_mode_ = this->get_parameter("debug").as_bool();

    // Subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/slam/map", 10,
        std::bind(&ExplorationPlanner::mapCallback, this,
                  std::placeholders::_1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/slam/pose", 10,
        std::bind(&ExplorationPlanner::poseCallback, this,
                  std::placeholders::_1));

    exploration_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/slam/exploration_complete", 10,
        std::bind(&ExplorationPlanner::explorationStatusCallback, this,
                  std::placeholders::_1));

    // Publishers
    exploration_path_pub_ =
        this->create_publisher<nav_msgs::msg::Path>("/motion/path", 10);
    exploration_complete_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/exploration/complete", 10);

    if (debug_mode_)
    {
        frontier_cells_pub_ = this->create_publisher<nav_msgs::msg::GridCells>(
            "/exploration/frontiers", 10);
        cspace_pub_ = this->create_publisher<nav_msgs::msg::GridCells>(
            "/exploration/cspace", 10);
        cost_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/exploration/cost_map", 10);
    }

    // Path planner reference (will be set externally if needed for direct
    // calls) Or use topic-based communication

    // Services - none needed in simplified version

    // Timer for periodic exploration
    exploration_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / EXPLORATION_RATE),
        std::bind(&ExplorationPlanner::explorationTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "ExplorationPlanner node initialized");
    RCLCPP_INFO(this->get_logger(), "  Exploration rate: %.1f Hz",
                EXPLORATION_RATE);
}

void ExplorationPlanner::declareParameters()
{
    this->declare_parameter("debug", false);
}

void ExplorationPlanner::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    current_map_ = msg;
    has_valid_map_ = true;
}

void ExplorationPlanner::poseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    current_pose_ = msg->pose;
    has_valid_pose_ = true;
}

void ExplorationPlanner::explorationStatusCallback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data && !is_finished_exploring_)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Exploration marked complete by SLAM controller");
        is_finished_exploring_ = true;
    }
}

void ExplorationPlanner::explorationTimerCallback()
{
    if (!is_finished_exploring_ && !is_planning_)
    {
        planExplorationPathInternal();
    }
}

void ExplorationPlanner::planExplorationPathInternal()
{
    if (!has_valid_map_ || !has_valid_pose_)
    {
        RCLCPP_DEBUG(this->get_logger(), "Waiting for map and pose data");
        return;
    }

    if (is_finished_exploring_)
    {
        return;
    }

    is_planning_ = true;

    // Get current position in grid coordinates
    GridCell start =
        PathPlanner::worldToGrid(*current_map_, current_pose_.position);

    // Search for frontiers
    auto [frontier_list, frontier_cells] =
        FrontierSearch::search(*current_map_, start, debug_mode_);

    // Publish frontier cells for visualization
    if (debug_mode_ && frontier_cells_pub_)
    {
        auto grid_cells =
            PathPlanner::getGridCells(*current_map_, frontier_cells);
        grid_cells.header.stamp = this->now();
        frontier_cells_pub_->publish(grid_cells);
    }

    // Check if frontiers were found
    if (frontier_list.frontiers.empty())
    {
        RCLCPP_INFO(this->get_logger(), "No frontiers found");
        no_frontiers_found_counter_++;

        if (no_frontiers_found_counter_ >= NUM_EXPLORE_FAILS_BEFORE_FINISH)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Exploration complete - no more frontiers");
            is_finished_exploring_ = true;

            std_msgs::msg::Bool complete_msg;
            complete_msg.data = true;
            exploration_complete_pub_->publish(complete_msg);
        }

        is_planning_ = false;
        return;
    }
    else
    {
        no_frontiers_found_counter_ = 0;
    }

    // Get top frontiers by size
    auto top_frontiers =
        getTopFrontiers(frontier_list.frontiers, MAX_NUM_FRONTIERS_TO_CHECK);

    RCLCPP_INFO(this->get_logger(), "Evaluating %zu frontiers",
                top_frontiers.size());

    // Find best frontier
    double lowest_cost = std::numeric_limits<double>::infinity();
    geometry_msgs::msg::PoseStamped best_goal;
    nav_msgs::msg::Path best_path;
    bool found_valid_path = false;

    const double MIN_FRONTIER_DISTANCE = current_min_distance_;
    const double GOAL_TOLERANCE = 0.10; // 10cm

    int frontiers_checked = 0;
    int frontiers_too_close = 0;

    for (const auto &frontier : top_frontiers)
    {
        // Check distance to frontier
        double dx = frontier.centroid.x - current_pose_.position.x;
        double dy = frontier.centroid.y - current_pose_.position.y;
        double distance_to_frontier = std::sqrt(dx * dx + dy * dy);

        // Skip if within goal tolerance
        if (distance_to_frontier < GOAL_TOLERANCE)
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "Skipping frontier - within goal tolerance (%.3fm)",
                         distance_to_frontier);
            continue;
        }

        // Skip if too close for current threshold
        if (distance_to_frontier < MIN_FRONTIER_DISTANCE)
        {
            frontiers_too_close++;
            RCLCPP_DEBUG(this->get_logger(),
                         "Skipping frontier - too close (%.3fm < %.2fm)",
                         distance_to_frontier, MIN_FRONTIER_DISTANCE);
            continue;
        }

        frontiers_checked++;

        // Use PathPlanner static methods directly (no service needed)
        auto [cspace, cspace_cells] =
            PathPlanner::calcCSpace(*current_map_, false);
        cv::Mat cost_map = PathPlanner::calcCostMap(*current_map_);

        GridCell start =
            PathPlanner::worldToGrid(*current_map_, current_pose_.position);
        GridCell goal =
            PathPlanner::worldToGrid(*current_map_, frontier.centroid);

        // Execute A*
        auto [path_cells, a_star_cost, actual_start, actual_goal] =
            PathPlanner::aStar(cspace, cost_map, start, goal);

        if (!path_cells.empty() && a_star_cost > 0.0)
        {
            // Calculate cost
            double cost = (A_STAR_COST_WEIGHT * distance_to_frontier) +
                          (FRONTIER_SIZE_COST_WEIGHT / frontier.size);

            if (cost < lowest_cost)
            {
                lowest_cost = cost;
                best_path =
                    PathPlanner::pathToMessage(*current_map_, path_cells);
                found_valid_path = true;
            }
        }
    }

    // Log status
    if (frontiers_checked == 0)
    {
        if (frontiers_too_close > 0)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "All %d frontiers too close (< %.2fm) - waiting for map update",
                frontiers_too_close, MIN_FRONTIER_DISTANCE);
        }
    }

    // Publish results
    if (found_valid_path)
    {
        // Verify goal distance
        auto goal_pos = best_path.poses.back().pose.position;
        double dx = goal_pos.x - current_pose_.position.x;
        double dy = goal_pos.y - current_pose_.position.y;
        double goal_distance = std::sqrt(dx * dx + dy * dy);

        if (goal_distance < GOAL_TOLERANCE)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Best path goal within tolerance (%.3fm) - robot needs to move",
                goal_distance);
            resetDynamicLookahead();
        }
        else if (goal_distance < MIN_FRONTIER_DISTANCE)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Best path goal too close (%.3fm < %.3fm)",
                        goal_distance, MIN_FRONTIER_DISTANCE);
            updateDynamicLookahead(goal_pos);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Publishing path with cost %.2f",
                        lowest_cost);
            best_path.header.stamp = this->now();
            exploration_path_pub_->publish(best_path);
            no_path_found_counter_ = 0;

            // Reset lookahead if path is long enough
            if (best_path.poses.size() >= 5)
            {
                resetDynamicLookahead();
            }
        }
    }
    else
    {
        // Don't count "all too close" as failure
        if (frontiers_checked == 0 && frontiers_too_close > 0)
        {
            RCLCPP_DEBUG(this->get_logger(), "Waiting for new frontiers");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No valid paths found");
            no_path_found_counter_++;

            int completion_threshold = NUM_EXPLORE_FAILS_BEFORE_FINISH;
            if (top_frontiers.size() <= 2)
            {
                completion_threshold = 15;
            }

            if (no_path_found_counter_ >= completion_threshold)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Exploration complete - no valid paths");
                is_finished_exploring_ = true;

                std_msgs::msg::Bool complete_msg;
                complete_msg.data = true;
                exploration_complete_pub_->publish(complete_msg);
            }
        }
    }

    is_planning_ = false;
}

std::vector<FrontierSearch::Frontier> ExplorationPlanner::getTopFrontiers(
    const std::vector<FrontierSearch::Frontier> &frontiers, int n)
{

    std::vector<FrontierSearch::Frontier> sorted_frontiers = frontiers;
    std::sort(
        sorted_frontiers.begin(), sorted_frontiers.end(),
        [](const FrontierSearch::Frontier &a, const FrontierSearch::Frontier &b)
        { return a.size > b.size; });

    if (sorted_frontiers.size() > static_cast<size_t>(n))
    {
        sorted_frontiers.resize(n);
    }

    return sorted_frontiers;
}

void ExplorationPlanner::publishCostMap(
    const nav_msgs::msg::OccupancyGrid &mapdata, const cv::Mat &cost_map)
{
    if (!debug_mode_ || !cost_map_pub_)
        return;

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = this->now();
    grid.header.frame_id = "map";
    grid.info = mapdata.info;

    // Normalize cost map to [0, 100]
    double max_val;
    cv::minMaxLoc(cost_map, nullptr, &max_val);

    grid.data.resize(cost_map.rows * cost_map.cols);
    for (int y = 0; y < cost_map.rows; y++)
    {
        for (int x = 0; x < cost_map.cols; x++)
        {
            int idx = y * cost_map.cols + x;
            double normalized = (cost_map.at<uint8_t>(y, x) / max_val) * 100.0;
            grid.data[idx] = static_cast<int8_t>(normalized);
        }
    }

    cost_map_pub_->publish(grid);
}

void ExplorationPlanner::updateDynamicLookahead(
    const geometry_msgs::msg::Point &rejected_goal)
{
    // Check if same goal as last time
    double dx = rejected_goal.x - last_rejected_goal_.x;
    double dy = rejected_goal.y - last_rejected_goal_.y;
    double distance_to_last = std::sqrt(dx * dx + dy * dy);

    if (distance_to_last < 0.05)
    { // Same goal (within 5cm)
        consecutive_rejections_++;

        // Reduce minimum distance every 2 rejections
        if (consecutive_rejections_ % 2 == 0)
        {
            double new_min_distance =
                current_min_distance_ - DISTANCE_REDUCTION_STEP;
            current_min_distance_ =
                std::max(new_min_distance, MINIMUM_MIN_DISTANCE);

            RCLCPP_INFO(
                this->get_logger(),
                "Reducing minimum frontier distance to %.3fm (rejection #%d)",
                current_min_distance_, consecutive_rejections_);
        }
    }
    else
    {
        consecutive_rejections_ = 1;
    }

    last_rejected_goal_ = rejected_goal;
}

void ExplorationPlanner::resetDynamicLookahead()
{
    consecutive_rejections_ = 0;
    current_min_distance_ = INITIAL_MIN_DISTANCE;
    last_rejected_goal_.x = 0.0;
    last_rejected_goal_.y = 0.0;
    last_rejected_goal_.z = 0.0;
}

nav_msgs::msg::Path ExplorationPlanner::planExplorationPath(
    const nav_msgs::msg::OccupancyGrid &map,
    const geometry_msgs::msg::Pose &current_pose)
{
    // cache inputs for the internal routine
    map_ = map;
    current_pose_ = current_pose;

    // run your existing pipeline
    planExplorationPathInternal();

    // return whatever the internal routine computed
    return last_path_;
}