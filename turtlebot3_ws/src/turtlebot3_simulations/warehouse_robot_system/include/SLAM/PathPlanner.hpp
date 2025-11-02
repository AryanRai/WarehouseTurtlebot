// PathPlanner - A* pathfinding with cost map support
// Adapted from SLAM_Reference.md path_planner.py

#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <SLAM/priority_queue.hpp>
#include <vector>
#include <tuple>
#include <cmath>
#include <opencv2/opencv.hpp>

using GridCell = std::pair<int, int>;

class PathPlanner {
public:
    // Grid/World conversions
    static int gridToIndex(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p);
    static int8_t getCellValue(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p);
    static geometry_msgs::msg::Point gridToWorld(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p);
    static GridCell worldToGrid(const nav_msgs::msg::OccupancyGrid& mapdata, const geometry_msgs::msg::Point& wp);
    
    // Distance calculations
    static double euclideanDistance(const GridCell& p1, const GridCell& p2);
    static double euclideanDistance(const std::pair<double, double>& p1, const std::pair<double, double>& p2);
    
    // Cell validation
    static bool isCellInBounds(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p);
    static bool isCellWalkable(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p);
    
    // Neighbor finding
    static std::vector<GridCell> neighborsOf4(const nav_msgs::msg::OccupancyGrid& mapdata, 
                                               const GridCell& p, bool must_be_walkable = true);
    static std::vector<GridCell> neighborsOf8(const nav_msgs::msg::OccupancyGrid& mapdata, 
                                               const GridCell& p, bool must_be_walkable = true);
    static std::vector<std::pair<GridCell, double>> neighborsAndDistancesOf8(
        const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p, bool must_be_walkable = true);
    
    // C-Space calculation
    static std::tuple<nav_msgs::msg::OccupancyGrid, nav_msgs::msg::GridCells> 
        calcCSpace(const nav_msgs::msg::OccupancyGrid& mapdata, bool include_cells = false);
    
    // Cost map calculation
    static cv::Mat calcCostMap(const nav_msgs::msg::OccupancyGrid& mapdata);
    
    // A* pathfinding
    static std::tuple<std::vector<GridCell>, double, GridCell, GridCell> 
        aStar(const nav_msgs::msg::OccupancyGrid& mapdata, const cv::Mat& cost_map,
              const GridCell& start, const GridCell& goal);
    
    // Path conversion
    static nav_msgs::msg::Path pathToMessage(const nav_msgs::msg::OccupancyGrid& mapdata, 
                                              const std::vector<GridCell>& path);
    static std::vector<geometry_msgs::msg::PoseStamped> pathToPoses(
        const nav_msgs::msg::OccupancyGrid& mapdata, const std::vector<GridCell>& path);
    
    // Grid cells message
    static nav_msgs::msg::GridCells getGridCells(const nav_msgs::msg::OccupancyGrid& mapdata, 
                                                  const std::vector<GridCell>& cells);

private:
    static constexpr int WALKABLE_THRESHOLD = 50;
    static constexpr int CSPACE_PADDING = 3;  // Reduced from 5 to 3 (15cm vs 25cm inflation)
    static constexpr double COST_MAP_WEIGHT = 1000.0;
    static constexpr int MIN_PATH_LENGTH = 12;
    static constexpr int POSES_TO_TRUNCATE = 8;
    
    static GridCell getFirstWalkableNeighbor(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& start);
    static double getCostMapValue(const cv::Mat& cost_map, const GridCell& p);
    static cv::Mat createHallwayMask(const nav_msgs::msg::OccupancyGrid& mapdata, 
                                     const cv::Mat& cost_map, int threshold);
    static bool isHallwayCell(const nav_msgs::msg::OccupancyGrid& mapdata, 
                              const cv::Mat& cost_map, const GridCell& p, int threshold);
};

#endif // PATH_PLANNER_HPP
