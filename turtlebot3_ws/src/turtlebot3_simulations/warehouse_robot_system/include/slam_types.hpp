// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: slam_types.hpp
// Author(s): Dylan George
//
// SLAM data structures and types converted from Python reference

#ifndef SLAM_TYPES_HPP
#define SLAM_TYPES_HPP

#include <vector>
#include <utility>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace slam {

// Grid cell coordinate type
using GridCell = std::pair<int, int>;
using GridCellList = std::vector<GridCell>;

// Frontier structure (converted from Python Frontier.msg)
struct Frontier {
    uint32_t size;
    geometry_msgs::msg::Point centroid;
    
    Frontier() : size(0) {}
    Frontier(uint32_t s, const geometry_msgs::msg::Point& c) : size(s), centroid(c) {}
};

// Frontier list structure (converted from Python FrontierList.msg)
struct FrontierList {
    std::vector<Frontier> frontiers;
    
    void clear() { frontiers.clear(); }
    size_t size() const { return frontiers.size(); }
    bool empty() const { return frontiers.empty(); }
};

// Direction vectors for 4-connectivity and 8-connectivity
const std::vector<GridCell> DIRECTIONS_OF_4 = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1}
};

const std::vector<GridCell> DIRECTIONS_OF_8 = {
    {-1, -1}, {-1, 0}, {-1, 1}, 
    {0, -1},           {0, 1}, 
    {1, -1},  {1, 0},  {1, 1}
};

// Constants
const int WALKABLE_THRESHOLD = 50;
const int MIN_FRONTIER_SIZE = 8;
const double COST_MAP_WEIGHT = 1000.0;
const int PADDING = 5; // C-space padding

} // namespace slam

#endif // SLAM_TYPES_HPP