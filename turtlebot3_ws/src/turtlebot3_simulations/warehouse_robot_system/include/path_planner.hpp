// MTRX3760 2025 Project 2: Warehouse Robot 
// File: path_planner.hpp
// Author(s): Aryan Rai
//
// PathPlanner class converted from Python reference for A* pathfinding

#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include "slam_types.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <tuple>
#include <optional>

namespace slam {

class PathPlanner {
public:
    // Grid utility functions
    static int gridToIndex(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p);
    static int getCellValue(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p);
    static double euclideanDistance(const GridCell& p1, const GridCell& p2);
    static double euclideanDistance(const std::pair<double, double>& p1, const std::pair<double, double>& p2);
    
    // Coordinate transformations
    static geometry_msgs::msg::Point gridToWorld(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p);
    static GridCell worldToGrid(const nav_msgs::msg::OccupancyGrid& mapdata, const geometry_msgs::msg::Point& wp);
    
    // Path conversion
    static std::vector<geometry_msgs::msg::PoseStamped> pathToPoses(
        const nav_msgs::msg::OccupancyGrid& mapdata, 
        const std::vector<GridCell>& path);
    static nav_msgs::msg::Path pathToMessage(
        const nav_msgs::msg::OccupancyGrid& mapdata, 
        const std::vector<GridCell>& path);
    
    // Grid validation
    static bool isCellInBounds(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p);
    static bool isCellWalkable(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p);
    
    // Neighbor finding
    static std::vector<GridCell> neighbors(
        const nav_msgs::msg::OccupancyGrid& mapdata,
        const GridCell& p,
        const std::vector<GridCell>& directions,
        bool must_be_walkable = true);
    
    static std::vector<GridCell> neighborsOf4(
        const nav_msgs::msg::OccupancyGrid& mapdata,
        const GridCell& p,
        bool must_be_walkable = true);
    
    static std::vector<GridCell> neighborsOf8(
        const nav_msgs::msg::OccupancyGrid& mapdata,
        const GridCell& p,
        bool must_be_walkable = true);
    
    // Neighbors with distances
    static std::vector<std::pair<GridCell, double>> neighborsAndDistances(
        const nav_msgs::msg::OccupancyGrid& mapdata,
        const GridCell& p,
        const std::vector<GridCell>& directions,
        bool must_be_walkable = true);
    
    static std::vector<std::pair<GridCell, double>> neighborsAndDistancesOf4(
        const nav_msgs::msg::OccupancyGrid& mapdata,
        const GridCell& p,
        bool must_be_walkable = true);
    
    static std::vector<std::pair<GridCell, double>> neighborsAndDistancesOf8(
        const nav_msgs::msg::OccupancyGrid& mapdata,
        const GridCell& p,
        bool must_be_walkable = true);
    
    // C-space calculation
    static std::pair<nav_msgs::msg::OccupancyGrid, std::vector<GridCell>> calcCspace(
        const nav_msgs::msg::OccupancyGrid& mapdata,
        bool include_cells = false);
    
    // Cost map calculation
    static cv::Mat calcCostMap(const nav_msgs::msg::OccupancyGrid& mapdata);
    static int getCostMapValue(const cv::Mat& cost_map, const GridCell& p);
    
    // Hallway detection
    static cv::Mat createHallwayMask(
        const nav_msgs::msg::OccupancyGrid& mapdata,
        const cv::Mat& cost_map,
        int threshold);
    
    static bool isHallwayCell(
        const nav_msgs::msg::OccupancyGrid& mapdata,
        const cv::Mat& cost_map,
        const GridCell& p,
        int threshold);
    
    // A* pathfinding
    static GridCell getFirstWalkableNeighbor(
        const nav_msgs::msg::OccupancyGrid& mapdata,
        const GridCell& start);
    
    static std::tuple<std::optional<std::vector<GridCell>>, std::optional<double>, GridCell, GridCell> 
    aStar(
        const nav_msgs::msg::OccupancyGrid& mapdata,
        const cv::Mat& cost_map,
        const GridCell& start,
        const GridCell& goal);

private:
    // Helper functions
    static void showMap(const std::string& name, const cv::Mat& map);

};

} // namespace slam

#endif // PATH_PLANNER_HPP