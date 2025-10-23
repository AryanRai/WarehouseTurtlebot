// MTRX3760 2025 Project 2: Warehouse Robot 
// File: slam_test.cpp
// Author(s): Aryan Rai
//
// Simple test for SLAM module functionality

#include "path_planner.hpp"
#include "frontier_search.hpp"
#include <iostream>
#include <nav_msgs/msg/occupancy_grid.hpp>

int main() {
    std::cout << "=== SLAM Module Test ===" << std::endl;
    
    // Create a simple test occupancy grid
    nav_msgs::msg::OccupancyGrid test_map;
    test_map.info.width = 10;
    test_map.info.height = 10;
    test_map.info.resolution = 0.1;
    test_map.info.origin.position.x = 0.0;
    test_map.info.origin.position.y = 0.0;
    
    // Fill with free space (0) and some obstacles (100)
    test_map.data.resize(100, 0); // All free space initially
    test_map.data[44] = 100; // Add an obstacle at (4,4)
    test_map.data[45] = 100; // Add an obstacle at (5,4)
    test_map.data[54] = 100; // Add an obstacle at (4,5)
    test_map.data[55] = 100; // Add an obstacle at (5,5)
    
    // Test PathPlanner functions
    std::cout << "\n--- Testing PathPlanner ---" << std::endl;
    
    slam::GridCell test_cell = {2, 3};
    std::cout << "Testing cell (" << test_cell.first << "," << test_cell.second << ")" << std::endl;
    
    bool in_bounds = slam::PathPlanner::isCellInBounds(test_map, test_cell);
    std::cout << "Cell in bounds: " << (in_bounds ? "Yes" : "No") << std::endl;
    
    bool walkable = slam::PathPlanner::isCellWalkable(test_map, test_cell);
    std::cout << "Cell walkable: " << (walkable ? "Yes" : "No") << std::endl;
    
    auto neighbors = slam::PathPlanner::neighborsOf4(test_map, test_cell);
    std::cout << "Found " << neighbors.size() << " walkable neighbors" << std::endl;
    
    // Test coordinate conversion
    auto world_point = slam::PathPlanner::gridToWorld(test_map, test_cell);
    std::cout << "Grid (" << test_cell.first << "," << test_cell.second 
              << ") -> World (" << world_point.x << "," << world_point.y << ")" << std::endl;
    
    auto grid_cell = slam::PathPlanner::worldToGrid(test_map, world_point);
    std::cout << "World (" << world_point.x << "," << world_point.y 
              << ") -> Grid (" << grid_cell.first << "," << grid_cell.second << ")" << std::endl;
    
    // Test FrontierSearch
    std::cout << "\n--- Testing FrontierSearch ---" << std::endl;
    
    slam::GridCell start_cell = {1, 1};
    auto [frontier_list, frontier_cells] = slam::FrontierSearch::search(test_map, start_cell, true);
    
    std::cout << "Frontier search completed" << std::endl;
    std::cout << "Found " << frontier_list.frontiers.size() << " frontiers" << std::endl;
    std::cout << "Total frontier cells: " << frontier_cells.size() << std::endl;
    
    for (size_t i = 0; i < frontier_list.frontiers.size(); ++i) {
        const auto& frontier = frontier_list.frontiers[i];
        std::cout << "Frontier " << i << ": size=" << frontier.size 
                  << ", centroid=(" << frontier.centroid.x << "," << frontier.centroid.y << ")" << std::endl;
    }
    
    std::cout << "\n=== SLAM Module Test Complete ===" << std::endl;
    return 0;
}