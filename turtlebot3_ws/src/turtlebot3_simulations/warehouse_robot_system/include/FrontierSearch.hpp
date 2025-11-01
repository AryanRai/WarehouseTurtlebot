// Frontier Search - Expanding Wavefront Frontier Detection
// Adapted from SLAM_Reference.md frontier_search.py

#ifndef FRONTIER_SEARCH_HPP
#define FRONTIER_SEARCH_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include "PathPlanner.hpp"

struct Frontier {
    uint32_t size;
    geometry_msgs::msg::Point centroid;
    
    Frontier() : size(0) {}
    Frontier(uint32_t s, const geometry_msgs::msg::Point& c) : size(s), centroid(c) {}
};

struct FrontierList {
    std::vector<Frontier> frontiers;
};

class FrontierSearch {
public:
    static std::pair<FrontierList, std::vector<GridCell>> search(
        const nav_msgs::msg::OccupancyGrid& mapdata,
        const GridCell& start,
        bool include_frontier_cells = false
    );

private:
    static constexpr uint32_t MIN_FRONTIER_SIZE = 8;
    static constexpr int WALKABLE_THRESHOLD = 50;
    
    static bool isNewFrontierCell(
        const nav_msgs::msg::OccupancyGrid& mapdata,
        const GridCell& cell,
        const std::unordered_map<GridCell, bool, std::hash<std::pair<int, int>>>& is_frontier
    );
    
    static std::pair<Frontier, std::vector<GridCell>> buildNewFrontier(
        const nav_msgs::msg::OccupancyGrid& mapdata,
        const GridCell& initial_cell,
        std::unordered_map<GridCell, bool, std::hash<std::pair<int, int>>>& is_frontier,
        bool include_frontier_cells = false
    );
};

// Hash specialization for GridCell
namespace std {
    template<>
    struct hash<std::pair<int, int>> {
        size_t operator()(const std::pair<int, int>& p) const {
            return hash<int>()(p.first) ^ (hash<int>()(p.second) << 1);
        }
    };
}

#endif // FRONTIER_SEARCH_HPP
