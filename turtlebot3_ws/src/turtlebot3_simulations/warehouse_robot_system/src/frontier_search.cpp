// MTRX3760 2025 Project 2: Warehouse Robot 
// File: frontier_search.cpp
// Author(s): Aryan Rai
//
// FrontierSearch implementation converted from Python reference

#include "frontier_search.hpp"
#include <queue>
#include <iostream>

namespace slam {

std::pair<FrontierList, std::vector<GridCell>> FrontierSearch::search(
    const nav_msgs::msg::OccupancyGrid& mapdata,
    const GridCell& start,
    bool include_frontier_cells) {
    
    std::cout << "Frontier search starting from (" << start.first << "," << start.second << ")" << std::endl;
    
    // Create queue for breadth-first search
    std::queue<GridCell> queue;
    queue.push(start);
    
    // Initialize dictionaries for keeping track of visited and frontier cells
    std::map<GridCell, bool> visited;
    std::map<GridCell, bool> is_frontier;
    visited[start] = true;
    
    // Initialize list of frontiers
    FrontierList frontier_list;
    std::vector<GridCell> frontier_cells;
    
    while (!queue.empty()) {
        GridCell current = queue.front();
        queue.pop();
        
        auto neighbors = PathPlanner::neighborsOf4(mapdata, current, false);
        for (const auto& neighbor : neighbors) {
            int neighbor_value = PathPlanner::getCellValue(mapdata, neighbor);
            
            if (neighbor_value >= 0 && visited.find(neighbor) == visited.end()) {
                visited[neighbor] = true;
                queue.push(neighbor);
            } else if (isNewFrontierCell(mapdata, neighbor, is_frontier)) {
                // Mark as frontier
                is_frontier[neighbor] = true;
                
                // Build new frontier
                auto [new_frontier, new_frontier_cells] = buildNewFrontier(
                    mapdata, neighbor, is_frontier, include_frontier_cells);
                
                if (new_frontier.size >= MIN_FRONTIER_SIZE) {
                    frontier_list.frontiers.push_back(new_frontier);
                    if (include_frontier_cells) {
                        frontier_cells.insert(frontier_cells.end(), 
                                            new_frontier_cells.begin(), 
                                            new_frontier_cells.end());
                    }
                }
            }
        }
    }
    
    std::cout << "Found " << frontier_list.frontiers.size() << " frontiers" << std::endl;
    return {frontier_list, frontier_cells};
}

std::pair<Frontier, std::vector<GridCell>> FrontierSearch::buildNewFrontier(
    const nav_msgs::msg::OccupancyGrid& mapdata,
    const GridCell& initial_cell,
    std::map<GridCell, bool>& is_frontier,
    bool include_frontier_cells) {
    
    // Initialize frontier fields
    uint32_t size = 1;
    double centroid_x = initial_cell.first;
    double centroid_y = initial_cell.second;
    
    // Create queue for breadth-first search
    std::queue<GridCell> queue;
    queue.push(initial_cell);
    
    // Initialize list of frontier cells
    std::vector<GridCell> frontier_cells;
    
    // Breadth-first search for frontier cells
    while (!queue.empty()) {
        GridCell current = queue.front();
        queue.pop();
        
        if (include_frontier_cells) {
            frontier_cells.push_back(current);
        }
        
        auto neighbors = PathPlanner::neighborsOf8(mapdata, current, false);
        for (const auto& neighbor : neighbors) {
            if (isNewFrontierCell(mapdata, neighbor, is_frontier)) {
                // Mark as frontier
                is_frontier[neighbor] = true;
                
                // Update size and centroid
                size++;
                centroid_x += neighbor.first;
                centroid_y += neighbor.second;
                queue.push(neighbor);
            }
        }
    }
    
    // Calculate centroid by taking the average
    centroid_x /= size;
    centroid_y /= size;
    
    // Make and return new frontier
    geometry_msgs::msg::Point centroid = PathPlanner::gridToWorld(
        mapdata, {static_cast<int>(centroid_x), static_cast<int>(centroid_y)});
    
    Frontier frontier(size, centroid);
    return {frontier, frontier_cells};
}

bool FrontierSearch::isNewFrontierCell(
    const nav_msgs::msg::OccupancyGrid& mapdata,
    const GridCell& cell,
    const std::map<GridCell, bool>& is_frontier) {
    
    // Cell must be unknown and not already a frontier
    int cell_value = PathPlanner::getCellValue(mapdata, cell);
    if (cell_value != -1 || is_frontier.find(cell) != is_frontier.end()) {
        return false;
    }
    
    // Cell should have at least one connected cell that is free
    auto neighbors = PathPlanner::neighborsOf4(mapdata, cell, false);
    for (const auto& neighbor : neighbors) {
        int neighbor_value = PathPlanner::getCellValue(mapdata, neighbor);
        if (neighbor_value >= 0 && neighbor_value < WALKABLE_THRESHOLD) {
            return true;
        }
    }
    
    return false;
}

} // namespace slam