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
    
    const int PADDING = 5; // Number of cells around obstacles
    
    int width = mapdata.info.width;
    int height = mapdata.info.height;
    
    // Create OpenCV Mat from mapdata
    cv::Mat map(height, width, CV_8UC1);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            int8_t value = mapdata.data[idx];
            // Convert -1 (unknown) to 255, keep others as is
            map.at<uint8_t>(y, x) = (value == -1) ? 255 : static_cast<uint8_t>(value);
        }
    }
    
    // Get mask of unknown areas (-1 becomes 255)
    cv::Mat unknown_area_mask;
    cv::inRange(map, 255, 255, unknown_area_mask);
    
    // Erode unknown areas
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(PADDING, PADDING));
    cv::erode(unknown_area_mask, unknown_area_mask, kernel, cv::Point(-1, -1), 1);
    
    // Change unknown areas to free space for processing
    map.setTo(0, map == 255);
    
    // Inflate obstacles
    cv::Mat obstacle_mask;
    cv::dilate(map, obstacle_mask, kernel, cv::Point(-1, -1), 1);
    
    // Combine obstacle mask with unknown areas
    cv::Mat cspace_mat;
    cv::bitwise_or(obstacle_mask, unknown_area_mask, cspace_mat);
    
    // Create output occupancy grid
    nav_msgs::msg::OccupancyGrid cspace = mapdata; // Copy header and info
    cspace.data.resize(width * height);
    
    // Convert back to occupancy grid format
    std::vector<GridCell> cspace_cells;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            uint8_t value = cspace_mat.at<uint8_t>(y, x);
            
            // Convert back: 255 -> -1 (unknown), others stay as is
            cspace.data[idx] = (value == 255) ? -1 : static_cast<int8_t>(value);
            
            // Collect cells that were added for debugging
            if (include_cells && obstacle_mask.at<uint8_t>(y, x) > 0) {
                cspace_cells.push_back({x, y});
            }
        }
    }
    
    std::cout << "C-space calculation complete. Added " << cspace_cells.size() << " inflated cells." << std::endl;
    
    return {cspace, cspace_cells};
}

// Cost map calculation (minimal implementation)
cv::Mat PathPlanner::calcCostMap(const nav_msgs::msg::OccupancyGrid& mapdata) {
    int width = mapdata.info.width;
    int height = mapdata.info.height;
    
    cv::Mat cost_map = cv::Mat::zeros(height, width, CV_8UC1);
    
    // TODO: Implement full cost map calculation
    std::cout << "Cost map calculation - TODO: Implement iterative dilation" << std::endl;
    
    return cost_map;
}

int PathPlanner::getCostMapValue(const cv::Mat& cost_map, const GridCell& p) {
    if (p.second >= 0 && p.second < cost_map.rows && p.first >= 0 && p.first < cost_map.cols) {
        return cost_map.at<uint8_t>(p.second, p.first);
    }
    return 0;
}

// Hallway detection (minimal implementation)
cv::Mat PathPlanner::createHallwayMask(
    const nav_msgs::msg::OccupancyGrid& mapdata,
    const cv::Mat& cost_map,
    int threshold) {
    
    cv::Mat mask = cv::Mat::zeros(cost_map.size(), CV_8UC1);
    // TODO: Implement hallway detection
    return mask;
}

bool PathPlanner::isHallwayCell(
    const nav_msgs::msg::OccupancyGrid& mapdata,
    const cv::Mat& cost_map,
    const GridCell& p,
    int threshold) {
    
    // TODO: Implement hallway cell detection
    return false;
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
    
    if (!isCellWalkable(mapdata, start) || !isCellWalkable(mapdata, goal)) {
        std::cout << "Start or goal not walkable" << std::endl;
        return {std::nullopt, std::nullopt, start, goal};
    }
    
    // A* algorithm implementation
    PriorityQueue open_set;
    std::map<GridCell, double> g_score;
    std::map<GridCell, double> f_score;
    std::map<GridCell, GridCell> came_from;
    std::set<GridCell> closed_set;
    
    // Initialize scores
    g_score[start] = 0.0;
    f_score[start] = euclideanDistance(start, goal);
    open_set.put(start, f_score[start]);
    
    while (!open_set.empty()) {
        GridCell current = open_set.get();
        
        if (current == goal) {
            // Reconstruct path
            std::vector<GridCell> path;
            GridCell node = goal;
            double total_cost = g_score[goal];
            
            while (came_from.find(node) != came_from.end()) {
                path.push_back(node);
                node = came_from[node];
            }
            path.push_back(start);
            
            std::reverse(path.begin(), path.end());
            std::cout << "A* found path with " << path.size() << " nodes, cost: " << total_cost << std::endl;
            return {path, total_cost, start, goal};
        }
        
        closed_set.insert(current);
        
        // Check all neighbors
        auto neighbors = neighborsOf8(mapdata, current, true);
        for (const auto& neighbor : neighbors) {
            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }
            
            double tentative_g = g_score[current] + euclideanDistance(current, neighbor);
            
            // Add cost map penalty if available
            if (!cost_map.empty() && neighbor.second < cost_map.rows && neighbor.first < cost_map.cols) {
                tentative_g += cost_map.at<uint8_t>(neighbor.second, neighbor.first) * 0.01;
            }
            
            if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                f_score[neighbor] = tentative_g + euclideanDistance(neighbor, goal);
                
                open_set.put(neighbor, f_score[neighbor]);
            }
        }
    }
    
    std::cout << "A* failed to find path" << std::endl;
    return {std::nullopt, std::nullopt, start, goal};
}

// Helper functions
void PathPlanner::showMap(const std::string& name, const cv::Mat& map) {
    // TODO: Implement OpenCV visualization
    std::cout << "Showing map: " << name << " (visualization not implemented)" << std::endl;
}

} // namespace slam