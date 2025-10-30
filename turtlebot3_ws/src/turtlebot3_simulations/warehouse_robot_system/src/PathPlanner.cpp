#include "PathPlanner.hpp"
#include "priority_queue.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_map>
#include <algorithm>

// Directions for neighbor search
static const std::vector<GridCell> DIRECTIONS_OF_4 = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
static const std::vector<GridCell> DIRECTIONS_OF_8 = {
    {-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}
};

// Hash function for GridCell to use in unordered_map
struct GridCellHash {
    std::size_t operator()(const GridCell& cell) const {
        return std::hash<int>()(cell.first) ^ (std::hash<int>()(cell.second) << 1);
    }
};

int PathPlanner::gridToIndex(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p) {
    return p.second * mapdata.info.width + p.first;
}

int8_t PathPlanner::getCellValue(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p) {
    return mapdata.data[gridToIndex(mapdata, p)];
}

geometry_msgs::msg::Point PathPlanner::gridToWorld(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p) {
    geometry_msgs::msg::Point point;
    point.x = (p.first + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.x;
    point.y = (p.second + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.y;
    point.z = 0.0;
    return point;
}

GridCell PathPlanner::worldToGrid(const nav_msgs::msg::OccupancyGrid& mapdata, const geometry_msgs::msg::Point& wp) {
    int x = static_cast<int>((wp.x - mapdata.info.origin.position.x) / mapdata.info.resolution);
    int y = static_cast<int>((wp.y - mapdata.info.origin.position.y) / mapdata.info.resolution);
    return {x, y};
}

double PathPlanner::euclideanDistance(const GridCell& p1, const GridCell& p2) {
    return std::sqrt(std::pow(p2.first - p1.first, 2) + std::pow(p2.second - p1.second, 2));
}

double PathPlanner::euclideanDistance(const std::pair<double, double>& p1, const std::pair<double, double>& p2) {
    return std::sqrt(std::pow(p2.first - p1.first, 2) + std::pow(p2.second - p1.second, 2));
}

bool PathPlanner::isCellInBounds(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p) {
    return p.first >= 0 && p.first < static_cast<int>(mapdata.info.width) &&
           p.second >= 0 && p.second < static_cast<int>(mapdata.info.height);
}

bool PathPlanner::isCellWalkable(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p) {
    if (!isCellInBounds(mapdata, p)) return false;
    return getCellValue(mapdata, p) >= 0 && getCellValue(mapdata, p) < WALKABLE_THRESHOLD;
}

std::vector<GridCell> PathPlanner::neighborsOf4(const nav_msgs::msg::OccupancyGrid& mapdata, 
                                                 const GridCell& p, bool must_be_walkable) {
    std::vector<GridCell> neighbors;
    for (const auto& dir : DIRECTIONS_OF_4) {
        GridCell candidate = {p.first + dir.first, p.second + dir.second};
        if ((must_be_walkable && isCellWalkable(mapdata, candidate)) ||
            (!must_be_walkable && isCellInBounds(mapdata, candidate))) {
            neighbors.push_back(candidate);
        }
    }
    return neighbors;
}

std::vector<GridCell> PathPlanner::neighborsOf8(const nav_msgs::msg::OccupancyGrid& mapdata, 
                                                 const GridCell& p, bool must_be_walkable) {
    std::vector<GridCell> neighbors;
    for (const auto& dir : DIRECTIONS_OF_8) {
        GridCell candidate = {p.first + dir.first, p.second + dir.second};
        if ((must_be_walkable && isCellWalkable(mapdata, candidate)) ||
            (!must_be_walkable && isCellInBounds(mapdata, candidate))) {
            neighbors.push_back(candidate);
        }
    }
    return neighbors;
}

std::vector<std::pair<GridCell, double>> PathPlanner::neighborsAndDistancesOf8(
    const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& p, bool must_be_walkable) {
    std::vector<std::pair<GridCell, double>> neighbors;
    for (const auto& dir : DIRECTIONS_OF_8) {
        GridCell candidate = {p.first + dir.first, p.second + dir.second};
        if ((must_be_walkable && isCellWalkable(mapdata, candidate)) ||
            (!must_be_walkable && isCellInBounds(mapdata, candidate))) {
            double distance = euclideanDistance({0, 0}, dir);
            neighbors.push_back({candidate, distance});
        }
    }
    return neighbors;
}

std::tuple<nav_msgs::msg::OccupancyGrid, nav_msgs::msg::GridCells> 
PathPlanner::calcCSpace(const nav_msgs::msg::OccupancyGrid& mapdata, bool include_cells) {
    int width = mapdata.info.width;
    int height = mapdata.info.height;
    
    // Create OpenCV matrix from map data
    cv::Mat map(height, width, CV_8UC1);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int8_t value = mapdata.data[y * width + x];
            map.at<uint8_t>(y, x) = (value == -1) ? 255 : static_cast<uint8_t>(value);
        }
    }
    
    // Get mask of unknown areas
    cv::Mat unknown_mask;
    cv::inRange(map, 255, 255, unknown_mask);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(CSPACE_PADDING, CSPACE_PADDING));
    cv::erode(unknown_mask, unknown_mask, kernel);
    
    // Change unknown areas to free space
    map.setTo(0, map == 255);
    
    // Inflate obstacles
    cv::Mat obstacle_mask;
    cv::dilate(map, obstacle_mask, kernel);
    cv::Mat cspace_mat;
    cv::bitwise_or(obstacle_mask, unknown_mask, cspace_mat);
    
    // Create C-space occupancy grid
    nav_msgs::msg::OccupancyGrid cspace = mapdata;
    cspace.data.resize(width * height);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            cspace.data[y * width + x] = cspace_mat.at<uint8_t>(y, x);
        }
    }
    
    // Create grid cells if requested
    nav_msgs::msg::GridCells cspace_cells;
    if (include_cells) {
        cspace_cells.header.frame_id = "map";
        cspace_cells.cell_width = mapdata.info.resolution;
        cspace_cells.cell_height = mapdata.info.resolution;
        
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (obstacle_mask.at<uint8_t>(y, x) > 0) {
                    cspace_cells.cells.push_back(gridToWorld(mapdata, {x, y}));
                }
            }
        }
    }
    
    return {cspace, cspace_cells};
}

double PathPlanner::getCostMapValue(const cv::Mat& cost_map, const GridCell& p) {
    return cost_map.at<uint8_t>(p.second, p.first);
}

bool PathPlanner::isHallwayCell(const nav_msgs::msg::OccupancyGrid& mapdata, 
                                const cv::Mat& cost_map, const GridCell& p, int threshold) {
    double cost_value = getCostMapValue(cost_map, p);
    for (const auto& neighbor : neighborsOf8(mapdata, p, false)) {
        double neighbor_cost = getCostMapValue(cost_map, neighbor);
        if (neighbor_cost < threshold || neighbor_cost > cost_value) {
            return false;
        }
    }
    return true;
}

cv::Mat PathPlanner::createHallwayMask(const nav_msgs::msg::OccupancyGrid& mapdata, 
                                       const cv::Mat& cost_map, int threshold) {
    cv::Mat mask = cv::Mat::zeros(cost_map.size(), CV_8UC1);
    
    for (int y = 0; y < cost_map.rows; y++) {
        for (int x = 0; x < cost_map.cols; x++) {
            if (cost_map.at<uint8_t>(y, x) > 0) {
                if (isHallwayCell(mapdata, cost_map, {x, y}, threshold)) {
                    mask.at<uint8_t>(y, x) = 1;
                }
            }
        }
    }
    
    return mask;
}

cv::Mat PathPlanner::calcCostMap(const nav_msgs::msg::OccupancyGrid& mapdata) {
    int width = mapdata.info.width;
    int height = mapdata.info.height;
    
    // Create map matrix
    cv::Mat map(height, width, CV_8UC1);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int8_t value = mapdata.data[y * width + x];
            map.at<uint8_t>(y, x) = (value == -1) ? 100 : static_cast<uint8_t>(value);
        }
    }
    
    // Iteratively dilate walls
    cv::Mat cost_map = cv::Mat::zeros(map.size(), CV_8UC1);
    cv::Mat dilated_map = map.clone();
    int iterations = 0;
    cv::Mat kernel = (cv::Mat_<uint8_t>(3, 3) << 0, 1, 0, 1, 1, 1, 0, 1, 0);
    
    while (cv::countNonZero(dilated_map == 0) > 0 && iterations < 100) {
        iterations++;
        cv::Mat next_dilated;
        cv::dilate(dilated_map, next_dilated, kernel);
        
        cv::Mat difference = next_dilated - dilated_map;
        cv::Mat mask = difference > 0;
        difference.setTo(cv::Scalar(iterations), mask);
        
        cv::bitwise_or(cost_map, difference, cost_map);
        dilated_map = next_dilated;
    }
    
    // Create hallway mask
    cost_map = createHallwayMask(mapdata, cost_map, iterations / 4);
    
    // Dilate hallway mask
    dilated_map = cost_map.clone();
    int cost = 1;
    for (int i = 0; i < iterations; i++) {
        cost++;
        cv::Mat next_dilated;
        cv::dilate(dilated_map, next_dilated, kernel);
        
        cv::Mat difference = next_dilated - dilated_map;
        cv::Mat mask = difference > 0;
        difference.setTo(cv::Scalar(cost), mask);
        
        cv::bitwise_or(cost_map, difference, cost_map);
        dilated_map = next_dilated;
    }
    
    // Subtract 1 from all non-zero values
    cv::Mat non_zero_mask = cost_map > 0;
    for (int y = 0; y < cost_map.rows; y++) {
        for (int x = 0; x < cost_map.cols; x++) {
            if (non_zero_mask.at<uint8_t>(y, x)) {
                cost_map.at<uint8_t>(y, x) -= 1;
            }
        }
    }
    
    return cost_map;
}

GridCell PathPlanner::getFirstWalkableNeighbor(const nav_msgs::msg::OccupancyGrid& mapdata, const GridCell& start) {
    std::vector<GridCell> queue = {start};
    std::unordered_map<GridCell, bool, GridCellHash> visited;
    
    while (!queue.empty()) {
        GridCell current = queue.front();
        queue.erase(queue.begin());
        
        if (isCellWalkable(mapdata, current)) {
            return current;
        }
        
        for (const auto& neighbor : neighborsOf4(mapdata, current, false)) {
            if (visited.find(neighbor) == visited.end()) {
                visited[neighbor] = true;
                queue.push_back(neighbor);
            }
        }
    }
    
    return start;
}

std::tuple<std::vector<GridCell>, double, GridCell, GridCell> 
PathPlanner::aStar(const nav_msgs::msg::OccupancyGrid& mapdata, const cv::Mat& cost_map,
                   const GridCell& start_in, const GridCell& goal_in) {
    // Get walkable start and goal
    GridCell start = isCellWalkable(mapdata, start_in) ? start_in : getFirstWalkableNeighbor(mapdata, start_in);
    GridCell goal = isCellWalkable(mapdata, goal_in) ? goal_in : getFirstWalkableNeighbor(mapdata, goal_in);
    
    PriorityQueue<GridCell> pq;
    pq.put(start, 0.0);
    
    std::unordered_map<GridCell, double, GridCellHash> cost_so_far;
    std::unordered_map<GridCell, double, GridCellHash> distance_cost_so_far;
    std::unordered_map<GridCell, GridCell, GridCellHash> came_from;
    
    cost_so_far[start] = 0.0;
    distance_cost_so_far[start] = 0.0;
    came_from[start] = {-1, -1};
    
    while (!pq.empty()) {
        GridCell current = pq.get();
        
        if (current == goal) break;
        
        for (const auto& [neighbor, distance] : neighborsAndDistancesOf8(mapdata, current)) {
            double added_cost = distance;
            if (!cost_map.empty()) {
                added_cost += COST_MAP_WEIGHT * getCostMapValue(cost_map, neighbor);
            }
            double new_cost = cost_so_far[current] + added_cost;
            
            if (cost_so_far.find(neighbor) == cost_so_far.end() || new_cost < cost_so_far[neighbor]) {
                cost_so_far[neighbor] = new_cost;
                distance_cost_so_far[neighbor] = distance_cost_so_far[current] + distance;
                double priority = new_cost + euclideanDistance(neighbor, goal);
                pq.put(neighbor, priority);
                came_from[neighbor] = current;
            }
        }
    }
    
    // Reconstruct path
    std::vector<GridCell> path;
    GridCell cell = goal;
    
    while (cell.first != -1 && cell.second != -1) {
        path.insert(path.begin(), cell);
        if (came_from.find(cell) != came_from.end()) {
            cell = came_from[cell];
        } else {
            return {{}, 0.0, start, goal};
        }
    }
    
    // Check minimum path length
    if (path.size() < MIN_PATH_LENGTH) {
        return {{}, 0.0, start, goal};
    }
    
    // Truncate last poses
    if (path.size() > POSES_TO_TRUNCATE) {
        path.erase(path.end() - POSES_TO_TRUNCATE, path.end());
    }
    
    return {path, distance_cost_so_far[goal], start, goal};
}

std::vector<geometry_msgs::msg::PoseStamped> PathPlanner::pathToPoses(
    const nav_msgs::msg::OccupancyGrid& mapdata, const std::vector<GridCell>& path) {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    
    for (size_t i = 0; i < path.size() - 1; i++) {
        const auto& cell = path[i];
        const auto& next_cell = path[i + 1];
        
        double angle = std::atan2(next_cell.second - cell.second, next_cell.first - cell.first);
        
        tf2::Quaternion q;
        q.setRPY(0, 0, angle);
        
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position = gridToWorld(mapdata, cell);
        pose.pose.orientation = tf2::toMsg(q);
        
        poses.push_back(pose);
    }
    
    return poses;
}

nav_msgs::msg::Path PathPlanner::pathToMessage(const nav_msgs::msg::OccupancyGrid& mapdata, 
                                                const std::vector<GridCell>& path) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.poses = pathToPoses(mapdata, path);
    return path_msg;
}

nav_msgs::msg::GridCells PathPlanner::getGridCells(const nav_msgs::msg::OccupancyGrid& mapdata, 
                                                    const std::vector<GridCell>& cells) {
    nav_msgs::msg::GridCells grid_cells;
    grid_cells.header.frame_id = "map";
    grid_cells.cell_width = mapdata.info.resolution;
    grid_cells.cell_height = mapdata.info.resolution;
    
    for (const auto& cell : cells) {
        grid_cells.cells.push_back(gridToWorld(mapdata, cell));
    }
    
    return grid_cells;
}
