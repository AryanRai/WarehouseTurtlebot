// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: path_planner.cpp
// Author(s): Dylan George
//
// PathPlanner implementation converted from Python reference

#include "path_planner.hpp"
#include "priority_queue.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <map>

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

// C-space calculation (minimal implementation)
std::pair<nav_msgs::msg::OccupancyGrid, std::vector<GridCell>> PathPlanner::calcCspace(
    const nav_msgs::msg::OccupancyGrid& mapdata,
    bool include_cells) {
    
    nav_msgs::msg::OccupancyGrid cspace = mapdata; // Copy original
    std::vector<GridCell> cspace_cells;
    
    // TODO: Implement C-space inflation using OpenCV
    std::cout << "C-space calculation - TODO: Implement OpenCV-based inflation" << std::endl;
    
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
    
    // TODO: Implement full A* algorithm
    // For now, return empty result
    return {std::nullopt, std::nullopt, start, goal};
}

// Helper functions
void PathPlanner::showMap(const std::string& name, const cv::Mat& map) {
    // TODO: Implement OpenCV visualization
    std::cout << "Showing map: " << name << " (visualization not implemented)" << std::endl;
}

} // namespace slam