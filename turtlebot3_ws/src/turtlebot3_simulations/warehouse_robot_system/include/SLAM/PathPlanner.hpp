// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: PathPlanner.hpp
// Author(s): Inez Dumas, Tony Bechara, Aryan Rai, Filip Gusavac
//
// Description: path planner for A* path planning with cost map support.

#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <SLAM/priority_queue.hpp>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <vector>

using GridCell = std::pair<int, int>;

// Simple path request/response structures (no nav2_msgs dependency)
namespace path_planning
{
struct PathRequest
{
    geometry_msgs::msg::PoseStamped start;
    geometry_msgs::msg::PoseStamped goal;
};

struct PathResponse
{
    nav_msgs::msg::Path path;
    bool success;
};
} // namespace path_planning

class PathPlanner : public rclcpp::Node
{
    public:
        PathPlanner();
        ~PathPlanner() = default;

        // Public method for direct path planning (backwards compatible)
        nav_msgs::msg::Path
        planPath(const geometry_msgs::msg::PoseStamped &start,
                 const geometry_msgs::msg::PoseStamped &goal);

        // Static utility functions (kept as static for use by other nodes)
        static int gridToIndex(const nav_msgs::msg::OccupancyGrid &mapdata,
                               const GridCell &p);
        static int8_t getCellValue(const nav_msgs::msg::OccupancyGrid &mapdata,
                                   const GridCell &p);
        static geometry_msgs::msg::Point
        gridToWorld(const nav_msgs::msg::OccupancyGrid &mapdata,
                    const GridCell &p);
        static GridCell worldToGrid(const nav_msgs::msg::OccupancyGrid &mapdata,
                                    const geometry_msgs::msg::Point &wp);

        static double euclideanDistance(const GridCell &p1, const GridCell &p2);
        static double euclideanDistance(const std::pair<double, double> &p1,
                                        const std::pair<double, double> &p2);

        static bool isCellInBounds(const nav_msgs::msg::OccupancyGrid &mapdata,
                                   const GridCell &p);
        static bool isCellWalkable(const nav_msgs::msg::OccupancyGrid &mapdata,
                                   const GridCell &p);

        static std::vector<GridCell>
        neighborsOf4(const nav_msgs::msg::OccupancyGrid &mapdata,
                     const GridCell &p, bool must_be_walkable = true);
        static std::vector<GridCell>
        neighborsOf8(const nav_msgs::msg::OccupancyGrid &mapdata,
                     const GridCell &p, bool must_be_walkable = true);
        static std::vector<std::pair<GridCell, double>>
        neighborsAndDistancesOf8(const nav_msgs::msg::OccupancyGrid &mapdata,
                                 const GridCell &p,
                                 bool must_be_walkable = true);

        static std::tuple<nav_msgs::msg::OccupancyGrid,
                          nav_msgs::msg::GridCells>
        calcCSpace(const nav_msgs::msg::OccupancyGrid &mapdata,
                   bool include_cells = false);

        static cv::Mat calcCostMap(const nav_msgs::msg::OccupancyGrid &mapdata);

        static std::tuple<std::vector<GridCell>, double, GridCell, GridCell>
        aStar(const nav_msgs::msg::OccupancyGrid &mapdata,
              const cv::Mat &cost_map, const GridCell &start,
              const GridCell &goal);

        static nav_msgs::msg::Path
        pathToMessage(const nav_msgs::msg::OccupancyGrid &mapdata,
                      const std::vector<GridCell> &path);
        static std::vector<geometry_msgs::msg::PoseStamped>
        pathToPoses(const nav_msgs::msg::OccupancyGrid &mapdata,
                    const std::vector<GridCell> &path);

        static nav_msgs::msg::GridCells
        getGridCells(const nav_msgs::msg::OccupancyGrid &mapdata,
                     const std::vector<GridCell> &cells);

    private:
        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

        // Publishers
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cspace_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
            cost_map_pub_;
        rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr path_cells_pub_;

        // State
        nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
        cv::Mat current_cost_map_;
        bool has_valid_map_;

        // Parameters
        static constexpr int WALKABLE_THRESHOLD = 50;
        static constexpr int CSPACE_PADDING = 3;
        static constexpr double COST_MAP_WEIGHT = 1000.0;
        static constexpr int MIN_PATH_LENGTH = 12;
        static constexpr int POSES_TO_TRUNCATE = 8;

        // Callbacks
        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        // Helper functions
        static GridCell
        getFirstWalkableNeighbor(const nav_msgs::msg::OccupancyGrid &mapdata,
                                 const GridCell &start);
        static double getCostMapValue(const cv::Mat &cost_map,
                                      const GridCell &p);
        static cv::Mat
        createHallwayMask(const nav_msgs::msg::OccupancyGrid &mapdata,
                          const cv::Mat &cost_map, int threshold);
        static bool isHallwayCell(const nav_msgs::msg::OccupancyGrid &mapdata,
                                  const cv::Mat &cost_map, const GridCell &p,
                                  int threshold);

        void updateCostMap();
        void publishVisualization(const std::vector<GridCell> &path);
};

#endif // PATH_PLANNER_HPP