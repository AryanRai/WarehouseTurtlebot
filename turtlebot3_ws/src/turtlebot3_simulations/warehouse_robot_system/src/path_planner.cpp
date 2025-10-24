// MTRX3760 2025 Project 2: Warehouse Robot 
// File: path_planner.cpp
// Author(s): Aryan Rai
//
// PathPlanner implementation converted from Python reference

#include "path_planner.hpp"
#include "priority_queue.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <map>
#include <set>

namespace slam {

// Grid utility functions
int PathPlanner::gridToIndex(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p) {
    return p.second * mapdata.info.width + p.first;
}

int PathPlanner::getCellValue(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p) {
    if (!isCellInBounds(mapdata, p)) {
        return -1; // Unknown/out of bounds
    }
    return mapdata.data[gridToIndex(mapdata, p)];
}

double PathPlanner::euclideanDistance(const GridCell& p1, const GridCell& p2) {
    return std::sqrt(std::pow(p2.first - p1.first, 2) + std::pow(p2.second - p1.second, 2));
}

double PathPlanner::euclideanDistance(const std::pair<double, double>& p1, const std::pair<double, double>& p2) {
    return std::sqrt(std::pow(p2.first - p1.first, 2) + std::pow(p2.second - p1.second, 2));
}

// Coordinate transformations
geometry_msgs::msg::Point PathPlanner::gridToWorld(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p) {
    geometry_msgs::msg::Point point;
    point.x = (p.first + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.x;
    point.y = (p.second + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.y;
    point.z = 0.0;
    return point;
}

GridCell PathPlanner::worldToGrid(const nav_msgs::msg::OccupancyGrid& mapdata, const geometry_msgs::msg::Point& wp) {
    int x = static_cast<int>((wp.x - mapdata.info.origin.position.x) / mapdata.info.resolution);
    int y = static_cast<int>((wp.y - mapdata.info.origin.position.y) / mapdata.info.resolution);
    return {x, y};
}

// Grid validation
bool PathPlanner::isCellInBounds(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p) {
    return p.first >= 0 && p.first < static_cast<int>(mapdata.info.width) &&
           p.second >= 0 && p.second < static_cast<int>(mapdata.info.height);
}

bool PathPlanner::isCellWalkable(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p) {
    if (!isCellInBounds(mapdata, p)) {
        return false;
    }
    int cell_value = getCellValue(mapdata, p);
    return cell_value >= 0 && cell_value < WALKABLE_THRESHOLD;
}

// Neighbor finding
std::vector<GridCell> PathPlanner::neighbors(
    const nav_msgs::msg::OccupancyGrid& mapdata,
    const GridCell& p,
    const std::vector<GridCell>& directions,
    bool must_be_walkable) {
    
    std::vector<GridCell> result;
    for (const auto& direction : directions) {
        GridCell candidate = {p.first + direction.first, p.second + direction.second};
        
        if (must_be_walkable) {
            if (isCellWalkable(mapdata, candidate)) {
                result.push_back(candidate);
            }
        } else {
            if (isCellInBounds(mapdata, candidate)) {
                result.push_back(candidate);
            }
        }
    }
    return result;
}

std::vector<GridCell> PathPlanner::neighborsOf4(
    const nav_msgs::msg::OccupancyGrid& mapdata,
    const GridCell& p,
    bool must_be_walkable) {
    return neighbors(mapdata, p, DIRECTIONS_OF_4, must_be_walkable);
}

std::vector<GridCell> PathPlanner::neighborsOf8(
    const nav_msgs::msg::OccupancyGrid& mapdata,
    const GridCell& p,
    bool must_be_walkable) {
    return neighbors(mapdata, p, DIRECTIONS_OF_8, must_be_walkable);
}

// Neighbors with distances
std::vector<std::pair<GridCell, double>> PathPlanner::neighborsAndDistances(
    const nav_msgs::msg::OccupancyGrid& mapdata,
    const GridCell& p,
    const std::vector<GridCell>& directions,
    bool must_be_walkable) {
    
    std::vector<std::pair<GridCell, double>> result;
    for (const auto& direction : directions) {
        GridCell candidate = {p.first + direction.first, p.second + direction.second};
        
        bool valid = must_be_walkable ? isCellWalkable(mapdata, candidate) : isCellInBounds(mapdata, candidate);
        if (valid) {
            double distance = euclideanDistance({0, 0}, direction);
            result.emplace_back(candidate, distance);
        }
    }
    return result;
}

std::vector<std::pair<GridCell, double>> PathPlanner::neighborsAndDistancesOf4(
    const nav_msgs::msg::OccupancyGrid& mapdata,
    const GridCell& p,
    bool must_be_walkable) {
    return neighborsAndDistances(mapdata, p, DIRECTIONS_OF_4, must_be_walkable);
}

std::vector<std::pair<GridCell, double>> PathPlanner::neighborsAndDistancesOf8(
    const nav_msgs::msg::OccupancyGrid& mapdata,
    const GridCell& p,
    bool must_be_walkable) {
    return neighborsAndDistances(mapdata, p, DIRECTIONS_OF_8, must_be_walkable);
}

// Path conversion (minimal implementation)
std::vector<geometry_msgs::msg::PoseStamped> PathPlanner::pathToPoses(
    const nav_msgs::msg::OccupancyGrid& mapdata, 
    const std::vector<GridCell>& path) {
    
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    // TODO: Implement full pose conversion with orientations
    return poses;
}

nav_msgs::msg::Path PathPlanner::pathToMessage(
    const nav_msgs::msg::OccupancyGrid& mapdata, 
    const std::vector<GridCell>& path) {
    
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    // TODO: Implement full path message conversion
    return path_msg;
}

// C-space calculation (OpenCV-based implementation)
std::pair<nav_msgs::msg::OccupancyGrid, std::vector<GridCell>> PathPlanner::calcCspace(
    const nav_msgs::msg::OccupancyGrid& mapdata,
    bool include_cells) {
    
    const int PADDING = 2; // Number of cells around obstacles (conservative for tight spaces)
    
    int width = mapdata.info.width;
    int height = mapdata.info.height;
    
    // For now, just return the original map as C-space
    // The padding was causing too many path failures
    // TODO: Implement proper C-space with dynamic padding based on environment
    nav_msgs::msg::OccupancyGrid cspace = mapdata;
    std::vector<GridCell> cspace_cells;
    
    std::cout << "C-space calculation complete. Using original map (padding disabled for stability)." << std::endl;
    
    return {cspace, cspace_cells};
}

// Cost map calculation (iterative dilation implementation)
cv::Mat PathPlanner::calcCostMap(const nav_msgs::msg::OccupancyGrid& mapdata) {
    std::cout << "Calculating cost map..." << std::endl;
    
    int width = mapdata.info.width;
    int height = mapdata.info.height;
    
    // Create OpenCV Mat from mapdata
    cv::Mat map(height, width, CV_8UC1);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            int8_t value = mapdata.data[idx];
            // Convert -1 (unknown) to 100, keep others as is
            map.at<uint8_t>(y, x) = (value == -1) ? 100 : static_cast<uint8_t>(std::max(0, static_cast<int>(value)));
        }
    }
    
    // Iteratively dilate walls until no changes are made
    cv::Mat cost_map = cv::Mat::zeros(height, width, CV_8UC1);
    cv::Mat dilated_map = map.clone();
    int iterations = 0;
    
    // Cross-shaped kernel for 4-connectivity
    cv::Mat kernel = (cv::Mat_<uint8_t>(3, 3) << 0, 1, 0, 1, 1, 1, 0, 1, 0);
    
    while (cv::countNonZero(dilated_map == 0) > 0 && iterations < 50) { // Limit iterations
        iterations++;
        
        // Dilate the map
        cv::Mat next_dilated_map;
        cv::dilate(dilated_map, next_dilated_map, kernel, cv::Point(-1, -1), 1);
        
        // Get difference to find outline
        cv::Mat difference;
        cv::subtract(next_dilated_map, dilated_map, difference);
        
        // Assign cost to outline cells
        cv::Mat cost_layer = cv::Mat::zeros(height, width, CV_8UC1);
        cost_layer.setTo(iterations, difference > 0);
        
        // Add to cost map
        cv::bitwise_or(cost_map, cost_layer, cost_map);
        
        // Update dilated map
        dilated_map = next_dilated_map;
    }
    
    // Create hallway mask (simplified version)
    cv::Mat hallway_mask = createHallwayMask(mapdata, cost_map, iterations / 4);
    
    // Iteratively dilate hallway mask
    cv::Mat final_cost_map = hallway_mask.clone();
    dilated_map = hallway_mask.clone();
    int cost = 1;
    
    for (int i = 0; i < iterations && i < 20; i++) { // Limit iterations
        cost++;
        
        cv::Mat next_dilated_map;
        cv::dilate(dilated_map, next_dilated_map, kernel, cv::Point(-1, -1), 1);
        
        cv::Mat difference;
        cv::subtract(next_dilated_map, dilated_map, difference);
        
        cv::Mat cost_layer = cv::Mat::zeros(height, width, CV_8UC1);
        cost_layer.setTo(cost, difference > 0);
        
        cv::bitwise_or(final_cost_map, cost_layer, final_cost_map);
        dilated_map = next_dilated_map;
    }
    
    // Subtract 1 from all non-zero values
    cv::Mat mask = final_cost_map > 0;
    final_cost_map.setTo(final_cost_map - 1, mask);
    
    std::cout << "Cost map calculation complete after " << iterations << " iterations." << std::endl;
    
    return final_cost_map;
}

int PathPlanner::getCostMapValue(const cv::Mat& cost_map, const GridCell& p) {
    if (p.second >= 0 && p.second < cost_map.rows && p.first >= 0 && p.first < cost_map.cols) {
        return cost_map.at<uint8_t>(p.second, p.first);
    }
    return 0;
}

// Hallway detection implementation
cv::Mat PathPlanner::createHallwayMask(
    const nav_msgs::msg::OccupancyGrid& mapdata,
    const cv::Mat& cost_map,
    int threshold) {
    
    cv::Mat mask = cv::Mat::zeros(cost_map.size(), CV_8UC1);
    
    // Find non-zero cells in cost map
    for (int y = 0; y < cost_map.rows; y++) {
        for (int x = 0; x < cost_map.cols; x++) {
            if (cost_map.at<uint8_t>(y, x) > 0) {
                if (isHallwayCell(mapdata, cost_map, {x, y}, threshold)) {
                    mask.at<uint8_t>(y, x) = 1;
                }
            }
        }
    }
    
    return mask;
}

bool PathPlanner::isHallwayCell(
    const nav_msgs::msg::OccupancyGrid& mapdata,
    const cv::Mat& cost_map,
    const GridCell& p,
    int threshold) {
    
    int cost_map_value = getCostMapValue(cost_map, p);
    
    // Check all 8-connected neighbors
    auto neighbors = neighborsOf8(mapdata, p, false);
    for (const auto& neighbor : neighbors) {
        int neighbor_cost = getCostMapValue(cost_map, neighbor);
        if (neighbor_cost < threshold || neighbor_cost > cost_map_value) {
            return false;
        }
    }
    
    return true;
}

// A* pathfinding (minimal implementation)
GridCell PathPlanner::getFirstWalkableNeighbor(
    const nav_msgs::msg::OccupancyGrid& mapdata,
    const GridCell& start) {
    
    std::queue<GridCell> queue;
    std::map<GridCell, bool> visited;
    
    queue.push(start);
    
    while (!queue.empty()) {
        GridCell current = queue.front();
        queue.pop();
        
        if (isCellWalkable(mapdata, current)) {
            return current;
        }
        
        auto neighbors = neighborsOf4(mapdata, current, false);
        for (const auto& neighbor : neighbors) {
            if (visited.find(neighbor) == visited.end()) {
                visited[neighbor] = true;
                queue.push(neighbor);
            }
        }
    }
    
    return start; // Return original if nothing found
}

std::tuple<std::optional<std::vector<GridCell>>, std::optional<double>, GridCell, GridCell> 
PathPlanner::aStar(
    const nav_msgs::msg::OccupancyGrid& mapdata,
    const cv::Mat& cost_map,
    const GridCell& start,
    const GridCell& goal) {
    
    std::cout << "A* pathfinding from (" << start.first << "," << start.second 
              << ") to (" << goal.first << "," << goal.second << ")" << std::endl;
    
    // Check if start and goal are valid
    if (!isCellInBounds(mapdata, start) || !isCellInBounds(mapdata, goal)) {
        std::cout << "Start or goal out of bounds" << std::endl;
        return {std::nullopt, std::nullopt, start, goal};
    }
    
    // Get walkable neighbors if start/goal not walkable
    GridCell actual_start = start;
    GridCell actual_goal = goal;
    
    if (!isCellWalkable(mapdata, start)) {
        actual_start = getFirstWalkableNeighbor(mapdata, start);
        std::cout << "Start not walkable, using (" << actual_start.first << "," << actual_start.second << ")" << std::endl;
    }
    
    if (!isCellWalkable(mapdata, goal)) {
        actual_goal = getFirstWalkableNeighbor(mapdata, goal);
        std::cout << "Goal not walkable, using (" << actual_goal.first << "," << actual_goal.second << ")" << std::endl;
    }
    
    // A* algorithm implementation
    PriorityQueue open_set;
    std::map<GridCell, double> g_score;
    std::map<GridCell, double> distance_cost;
    std::map<GridCell, GridCell> came_from;
    std::set<GridCell> closed_set;
    
    const double COST_MAP_WEIGHT = 1000.0;
    
    // Initialize scores
    g_score[actual_start] = 0.0;
    distance_cost[actual_start] = 0.0;
    double h_score = euclideanDistance(actual_start, actual_goal);
    open_set.put(actual_start, h_score);
    
    while (!open_set.empty()) {
        GridCell current = open_set.get();
        
        if (current == actual_goal) {
            // Reconstruct path
            std::vector<GridCell> path;
            GridCell node = actual_goal;
            double total_distance = distance_cost[actual_goal];
            
            while (came_from.find(node) != came_from.end()) {
                path.push_back(node);
                node = came_from[node];
            }
            path.push_back(actual_start);
            
            std::reverse(path.begin(), path.end());
            
            // Check minimum path length (reduced for nearby frontiers)
            const int MIN_PATH_LENGTH = 3;  // Reduced from 12 to allow nearby frontiers
            if (path.size() < MIN_PATH_LENGTH) {
                std::cout << "Path too short (" << path.size() << " < " << MIN_PATH_LENGTH << ")" << std::endl;
                return {std::nullopt, std::nullopt, actual_start, actual_goal};
            }
            
            // Truncate last few poses (but not if path is already short)
            const int POSES_TO_TRUNCATE = 5;  // Reduced from 8
            if (path.size() > POSES_TO_TRUNCATE + 3) {  // Only truncate if we have enough poses
                path.erase(path.end() - POSES_TO_TRUNCATE, path.end());
            }
            
            std::cout << "A* found path with " << path.size() << " nodes, distance: " << total_distance << std::endl;
            return {path, total_distance, actual_start, actual_goal};
        }
        
        closed_set.insert(current);
        
        // Check all neighbors with distances
        auto neighbors_with_dist = neighborsAndDistancesOf8(mapdata, current, true);
        for (const auto& [neighbor, distance] : neighbors_with_dist) {
            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }
            
            // Calculate cost including distance and cost map
            double added_cost = distance;
            if (!cost_map.empty() && neighbor.second < cost_map.rows && neighbor.first < cost_map.cols) {
                added_cost += COST_MAP_WEIGHT * getCostMapValue(cost_map, neighbor);
            }
            
            double tentative_g = g_score[current] + added_cost;
            double tentative_distance = distance_cost[current] + distance;
            
            if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                distance_cost[neighbor] = tentative_distance;
                
                double priority = tentative_g + euclideanDistance(neighbor, actual_goal);
                open_set.put(neighbor, priority);
            }
        }
    }
    
    std::cout << "A* failed to find path" << std::endl;
    return {std::nullopt, std::nullopt, actual_start, actual_goal};
}

// Helper functions
void PathPlanner::showMap(const std::string& name, const cv::Mat& map) {
    // TODO: Implement OpenCV visualization
    std::cout << "Showing map: " << name << " (visualization not implemented)" << std::endl;
}

} // namespace slam