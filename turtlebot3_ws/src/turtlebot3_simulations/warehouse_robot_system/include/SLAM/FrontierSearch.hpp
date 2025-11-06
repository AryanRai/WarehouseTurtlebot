// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: FrontierSearch.hpp
// Author(s): Inez Dumas, Tony Bechara, Aryan Rai, Filip Gusavac
//
// Description: frontier search utility for autonomous exploration.

#ifndef FRONTIER_SEARCH_HPP
#define FRONTIER_SEARCH_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include "SLAM/PathPlanner.hpp"

class FrontierSearch {
public:
    // Nested struct for a single frontier
    struct Frontier {
        uint32_t size;
        geometry_msgs::msg::Point centroid;
        
        Frontier() : size(0) {}
        Frontier(uint32_t s, const geometry_msgs::msg::Point& c) : size(s), centroid(c) {}
    };

    // Nested struct for a list of frontiers
    struct FrontierList {
        std::vector<Frontier> frontiers;
    };

    // Hash functor for GridCell (std::pair<int, int>) to use in unordered_map
    struct GridCellHash {
        size_t operator()(const GridCell& cell) const {
            return std::hash<int>()(cell.first) ^ (std::hash<int>()(cell.second) << 1);
        }
    };

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
        const std::unordered_map<GridCell, bool, GridCellHash>& is_frontier
    );
    
    static std::pair<Frontier, std::vector<GridCell>> buildNewFrontier(
        const nav_msgs::msg::OccupancyGrid& mapdata,
        const GridCell& initial_cell,
        std::unordered_map<GridCell, bool, GridCellHash>& is_frontier,
        bool include_frontier_cells = false
    );
};

#endif // FRONTIER_SEARCH_HPP
