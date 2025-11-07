// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: FrontierSearch.cpp
// Author(s): Aryan Rai
//
// Description: Implementation of frontier search algorithm for autonomous
// exploration.

#include "SLAM/FrontierSearch.hpp"
#include <queue>
#include <unordered_map>

std::pair<FrontierSearch::FrontierList, std::vector<GridCell>>
FrontierSearch::search(const nav_msgs::msg::OccupancyGrid &mapdata,
                       const GridCell &start, bool include_frontier_cells)
{

    // Create queue for breadth-first search
    std::vector<GridCell> queue;
    queue.push_back(start);

    // Initialize dictionaries for visited and frontier cells
    std::unordered_map<GridCell, bool, FrontierSearch::GridCellHash> visited;
    std::unordered_map<GridCell, bool, FrontierSearch::GridCellHash>
        is_frontier;
    visited[start] = true;

    // Initialize list of frontiers
    std::vector<FrontierSearch::Frontier> frontiers;
    std::vector<GridCell> frontier_cells;

    while (!queue.empty())
    {
        GridCell current = queue.front();
        queue.erase(queue.begin());

        // Check ALL neighbors (including unknown ones) - must_be_walkable =
        // false
        for (const auto &neighbor :
             PathPlanner::neighborsOf4(mapdata, current, false))
        {
            int8_t neighbor_value =
                PathPlanner::getCellValue(mapdata, neighbor);

            // If neighbor is free space and not visited, add to queue
            if (neighbor_value >= 0 && visited.find(neighbor) == visited.end())
            {
                visited[neighbor] = true;
                queue.push_back(neighbor);
            }
            // If neighbor is unknown, check if it's a frontier cell
            else if (isNewFrontierCell(mapdata, neighbor, is_frontier))
            {
                // Mark as frontier
                is_frontier[neighbor] = true;

                // Build new frontier
                auto [new_frontier, new_frontier_cells] = buildNewFrontier(
                    mapdata, neighbor, is_frontier, include_frontier_cells);

                if (new_frontier.size >= MIN_FRONTIER_SIZE)
                {
                    frontiers.push_back(new_frontier);
                    if (include_frontier_cells)
                    {
                        frontier_cells.insert(frontier_cells.end(),
                                              new_frontier_cells.begin(),
                                              new_frontier_cells.end());
                    }
                }
            }
        }
    }

    return {FrontierSearch::FrontierList{frontiers}, frontier_cells};
}

bool FrontierSearch::isNewFrontierCell(
    const nav_msgs::msg::OccupancyGrid &mapdata, const GridCell &cell,
    const std::unordered_map<GridCell, bool, FrontierSearch::GridCellHash>
        &is_frontier)
{

    // Cell must be unknown and not already a frontier
    if (PathPlanner::getCellValue(mapdata, cell) != -1 ||
        is_frontier.find(cell) != is_frontier.end())
    {
        return false;
    }

    // Cell should have at least one connected cell that is free
    for (const auto &neighbor : PathPlanner::neighborsOf4(mapdata, cell))
    {
        int8_t neighbor_value = PathPlanner::getCellValue(mapdata, neighbor);
        if (neighbor_value >= 0 && neighbor_value < WALKABLE_THRESHOLD)
        {
            return true;
        }
    }

    return false;
}

std::pair<FrontierSearch::Frontier, std::vector<GridCell>>
FrontierSearch::buildNewFrontier(
    const nav_msgs::msg::OccupancyGrid &mapdata, const GridCell &initial_cell,
    std::unordered_map<GridCell, bool, FrontierSearch::GridCellHash>
        &is_frontier,
    bool include_frontier_cells)
{

    // Initialize frontier fields
    uint32_t size = 1;
    double centroid_x = initial_cell.first;
    double centroid_y = initial_cell.second;

    // Create queue for breadth-first search
    std::vector<GridCell> queue;
    queue.push_back(initial_cell);

    // Initialize list of frontier cells
    std::vector<GridCell> frontier_cells;

    // Breadth-first search for frontier cells
    while (!queue.empty())
    {
        GridCell current = queue.front();
        queue.erase(queue.begin());

        if (include_frontier_cells)
        {
            frontier_cells.push_back(current);
        }

        // Check ALL neighbors (including unknown ones) for frontier expansion
        for (const auto &neighbor :
             PathPlanner::neighborsOf8(mapdata, current, false))
        {
            if (isNewFrontierCell(mapdata, neighbor, is_frontier))
            {
                // Mark as frontier
                is_frontier[neighbor] = true;

                // Update size and centroid
                size++;
                centroid_x += neighbor.first;
                centroid_y += neighbor.second;
                queue.push_back(neighbor);
            }
        }
    }

    // Calculate centroid by taking the average
    centroid_x /= size;
    centroid_y /= size;

    // Make and return new frontier
    geometry_msgs::msg::Point centroid = PathPlanner::gridToWorld(
        mapdata, {static_cast<int>(centroid_x), static_cast<int>(centroid_y)});

    return {FrontierSearch::Frontier(size, centroid), frontier_cells};
}
