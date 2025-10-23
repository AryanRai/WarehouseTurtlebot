// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: frontier_search.hpp
// Author(s): Dylan George
//
// FrontierSearch class converted from Python reference for frontier detection

#ifndef FRONTIER_SEARCH_HPP
#define FRONTIER_SEARCH_HPP

#include "slam_types.hpp"
#include "path_planner.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <map>
#include <queue>

namespace slam {

class FrontierSearch {
public:
    // Main frontier search function using Expanding Wavefront Frontier Detection
    static std::pair<FrontierList, std::vector<GridCell>> search(
        const nav_msgs::msg::OccupancyGrid& mapdata,
        const GridCell& start,
        bool include_frontier_cells = false);
    
    // Build a new frontier from an initial frontier cell
    static std::pair<Frontier, std::vector<GridCell>> buildNewFrontier(
        const nav_msgs::msg::OccupancyGrid& mapdata,
        const GridCell& initial_cell,
        std::map<GridCell, bool>& is_frontier,
        bool include_frontier_cells = false);
    
    // Check if a cell is a new frontier cell
    static bool isNewFrontierCell(
        const nav_msgs::msg::OccupancyGrid& mapdata,
        const GridCell& cell,
        const std::map<GridCell, bool>& is_frontier);

private:
    // Helper function to compare grid cells for map operations
    struct GridCellComparator {
        bool operator()(const GridCell& a, const GridCell& b) const {
            if (a.first != b.first) return a.first < b.first;
            return a.second < b.second;
        }
    };
};

} // namespace slam

#endif // FRONTIER_SEARCH_HPP