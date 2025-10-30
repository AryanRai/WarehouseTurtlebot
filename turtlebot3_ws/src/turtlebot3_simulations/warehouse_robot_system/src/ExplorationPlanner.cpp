#include "ExplorationPlanner.hpp"
#include <algorithm>

ExplorationPlanner::ExplorationPlanner(rclcpp::Node::SharedPtr node)
    : node_(node),
      no_frontiers_found_counter_(0),
      no_path_found_counter_(0),
      is_finished_exploring_(false),
      debug_mode_(false) {
    
    // Check if debug mode is enabled
    if (!node_->has_parameter("debug")) {
        node_->declare_parameter("debug", false);
    }
    debug_mode_ = node_->get_parameter("debug").as_bool();
    
    // Create publishers for visualization
    if (debug_mode_) {
        frontier_cells_pub_ = node_->create_publisher<nav_msgs::msg::GridCells>(
            "/exploration/frontier_cells", 10
        );
        start_pub_ = node_->create_publisher<nav_msgs::msg::GridCells>(
            "/exploration/start", 10
        );
        goal_pub_ = node_->create_publisher<nav_msgs::msg::GridCells>(
            "/exploration/goal", 10
        );
        cspace_pub_ = node_->create_publisher<nav_msgs::msg::GridCells>(
            "/exploration/cspace", 10
        );
        cost_map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/exploration/cost_map", 10
        );
    }
    
    RCLCPP_INFO(node_->get_logger(), "Exploration Planner initialized (debug=%s)", 
                debug_mode_ ? "true" : "false");
}

std::vector<Frontier> ExplorationPlanner::getTopFrontiers(
    const std::vector<Frontier>& frontiers, int n) {
    
    std::vector<Frontier> sorted_frontiers = frontiers;
    std::sort(sorted_frontiers.begin(), sorted_frontiers.end(),
              [](const Frontier& a, const Frontier& b) {
                  return a.size > b.size;
              });
    
    if (sorted_frontiers.size() > static_cast<size_t>(n)) {
        sorted_frontiers.resize(n);
    }
    
    return sorted_frontiers;
}

void ExplorationPlanner::publishCostMap(const nav_msgs::msg::OccupancyGrid& mapdata, 
                                        const cv::Mat& cost_map) {
    if (!debug_mode_ || !cost_map_pub_) return;
    
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = node_->now();
    grid.header.frame_id = "map";
    grid.info = mapdata.info;
    
    // Normalize cost map to [0, 100]
    double max_val;
    cv::minMaxLoc(cost_map, nullptr, &max_val);
    
    grid.data.resize(cost_map.rows * cost_map.cols);
    for (int y = 0; y < cost_map.rows; y++) {
        for (int x = 0; x < cost_map.cols; x++) {
            int idx = y * cost_map.cols + x;
            double normalized = (cost_map.at<uint8_t>(y, x) / max_val) * 100.0;
            grid.data[idx] = static_cast<int8_t>(normalized);
        }
    }
    
    cost_map_pub_->publish(grid);
}

nav_msgs::msg::Path ExplorationPlanner::planExplorationPath(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Pose& current_pose) {
    
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = "map";
    
    if (is_finished_exploring_) {
        return empty_path;
    }
    
    // Get current position in grid coordinates
    GridCell start = PathPlanner::worldToGrid(map, current_pose.position);
    
    // Search for frontiers
    auto [frontier_list, frontier_cells] = FrontierSearch::search(map, start, debug_mode_);
    
    // Publish frontier cells for visualization
    if (debug_mode_ && frontier_cells_pub_) {
        auto grid_cells = PathPlanner::getGridCells(map, frontier_cells);
        grid_cells.header.stamp = node_->now();
        frontier_cells_pub_->publish(grid_cells);
    }
    
    // Check if frontiers were found
    if (frontier_list.frontiers.empty()) {
        RCLCPP_INFO(node_->get_logger(), "No frontiers found");
        no_frontiers_found_counter_++;
        
        if (no_frontiers_found_counter_ >= NUM_EXPLORE_FAILS_BEFORE_FINISH) {
            RCLCPP_INFO(node_->get_logger(), "Exploration complete - no more frontiers");
            is_finished_exploring_ = true;
        }
        
        return empty_path;
    } else {
        no_frontiers_found_counter_ = 0;
    }
    
    // Calculate C-space
    auto [cspace, cspace_cells] = PathPlanner::calcCSpace(map, debug_mode_);
    if (debug_mode_ && cspace_pub_) {
        cspace_cells.header.stamp = node_->now();
        cspace_pub_->publish(cspace_cells);
    }
    
    // Calculate cost map
    cv::Mat cost_map = PathPlanner::calcCostMap(map);
    if (debug_mode_) {
        publishCostMap(map, cost_map);
    }
    
    // Get top frontiers by size
    auto top_frontiers = getTopFrontiers(frontier_list.frontiers, MAX_NUM_FRONTIERS_TO_CHECK);
    
    RCLCPP_INFO(node_->get_logger(), "Exploring %zu frontiers", top_frontiers.size());
    
    // Find best path among all frontiers
    double lowest_cost = std::numeric_limits<double>::infinity();
    std::vector<GridCell> best_path;
    std::vector<GridCell> starts, goals;
    
    // Minimum distance to frontier (in meters) to avoid planning to current location
    const double MIN_FRONTIER_DISTANCE = 0.20;  // 20cm minimum - increased from 15cm
    
    int frontiers_checked = 0;
    int frontiers_too_close = 0;
    
    for (const auto& frontier : top_frontiers) {
        GridCell goal = PathPlanner::worldToGrid(map, frontier.centroid);
        
        // Check if frontier is too close to current position
        double dx = frontier.centroid.x - current_pose.position.x;
        double dy = frontier.centroid.y - current_pose.position.y;
        double distance_to_frontier = std::sqrt(dx * dx + dy * dy);
        
        if (distance_to_frontier < MIN_FRONTIER_DISTANCE) {
            frontiers_too_close++;
            RCLCPP_INFO(node_->get_logger(), 
                        "Skipping frontier at (%.2f, %.2f) - too close (%.3fm < %.2fm)", 
                        frontier.centroid.x, frontier.centroid.y, distance_to_frontier, MIN_FRONTIER_DISTANCE);
            continue;
        }
        
        RCLCPP_DEBUG(node_->get_logger(), 
                    "Checking frontier at (%.2f, %.2f) - distance: %.3fm", 
                    frontier.centroid.x, frontier.centroid.y, distance_to_frontier);
        
        frontiers_checked++;
        
        // Execute A*
        auto [path, a_star_cost, actual_start, actual_goal] = 
            PathPlanner::aStar(cspace, cost_map, start, goal);
        
        if (debug_mode_) {
            starts.push_back(actual_start);
            goals.push_back(actual_goal);
        }
        
        if (path.empty() || a_star_cost == 0.0) {
            continue;
        }
        
        // Calculate total cost
        double cost = (A_STAR_COST_WEIGHT * a_star_cost) + 
                     (FRONTIER_SIZE_COST_WEIGHT / frontier.size);
        
        if (cost < lowest_cost) {
            lowest_cost = cost;
            best_path = path;
        }
    }
    
    // Log if all frontiers were too close
    if (frontiers_too_close > 0 && frontiers_checked == 0) {
        RCLCPP_INFO(node_->get_logger(), 
                   "All %d frontiers were too close (< %.2fm) - waiting for map to update", 
                   frontiers_too_close, MIN_FRONTIER_DISTANCE);
    }
    
    // Publish start and goal for visualization
    if (debug_mode_ && start_pub_ && goal_pub_) {
        auto start_cells = PathPlanner::getGridCells(map, starts);
        start_cells.header.stamp = node_->now();
        start_pub_->publish(start_cells);
        
        auto goal_cells = PathPlanner::getGridCells(map, goals);
        goal_cells.header.stamp = node_->now();
        goal_pub_->publish(goal_cells);
    }
    
    // Return best path
    if (!best_path.empty()) {
        // Final check: verify the goal is far enough away
        auto path_msg = PathPlanner::pathToMessage(map, best_path);
        if (!path_msg.poses.empty()) {
            auto goal_pos = path_msg.poses.back().pose.position;
            double dx = goal_pos.x - current_pose.position.x;
            double dy = goal_pos.y - current_pose.position.y;
            double goal_distance = std::sqrt(dx * dx + dy * dy);
            
            if (goal_distance < MIN_FRONTIER_DISTANCE) {
                RCLCPP_WARN(node_->get_logger(), 
                           "Best path goal at (%.2f, %.2f) is too close (%.3fm) - rejecting", 
                           goal_pos.x, goal_pos.y, goal_distance);
                // Don't count as failure - just wait for better frontiers
                return empty_path;
            }
        }
        
        RCLCPP_INFO(node_->get_logger(), "Found best path with cost %.2f", lowest_cost);
        no_path_found_counter_ = 0;
        return path_msg;
    } else {
        // Don't count "all frontiers too close" as a failure - just wait for map to update
        if (frontiers_checked == 0 && frontiers_too_close > 0) {
            RCLCPP_DEBUG(node_->get_logger(), "Waiting for new frontiers (all current ones too close)");
            // Don't increment no_path_found_counter_ - this is temporary
        } else {
            RCLCPP_INFO(node_->get_logger(), "No paths found");
            no_path_found_counter_++;
            
            if (no_path_found_counter_ >= NUM_EXPLORE_FAILS_BEFORE_FINISH) {
                RCLCPP_INFO(node_->get_logger(), "Exploration complete - no valid paths");
                is_finished_exploring_ = true;
            }
        }
        
        return empty_path;
    }
}

bool ExplorationPlanner::isExplorationComplete() const {
    return is_finished_exploring_;
}

int ExplorationPlanner::getNoFrontiersCounter() const {
    return no_frontiers_found_counter_;
}

int ExplorationPlanner::getNoPathCounter() const {
    return no_path_found_counter_;
}
