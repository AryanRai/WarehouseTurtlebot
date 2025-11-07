// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: InspectionRobot.cpp
// Author(s): Aryan Rai
//
// Description: Implementation of CInspectionRobot. Performs damage site
// inspection
//              operations including AprilTag detection, 360° scanning,
//              exploration mode, and Tier 1 Safety features.

#include "Robot/InspectionRobot.hpp"
#include <algorithm>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <limits>
#include <random>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <set>
#include <sstream>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

InspectionRobot::InspectionRobot(rclcpp::Node::SharedPtr node)
    : WarehouseRobot(node), is_inspecting_(false), current_inspection_index_(0),
      is_scanning_(false), scan_start_yaw_(0.0), scan_start_time_(node->now()),
      exploration_mode_(false), current_patrol_index_(0), tf_health_ok_(true),
      obstacle_detected_(false),
      min_obstacle_distance_(std::numeric_limits<double>::max())
{

    // Set file paths
    sites_file_ = "damage_sites.yaml";
    inspection_log_file_ = "inspection_log.csv";

    // Create CSV header if file doesn't exist
    std::ifstream check(inspection_log_file_);
    if (!check.good())
    {
        std::ofstream log(inspection_log_file_);
        log << "Timestamp,RequestID,Site,TagsDetected,Distance(m),Time(s),"
               "Status,Notes\n";
        log.close();
    }

    // Initialize TF2 for health monitoring
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    last_tf_check_time_ = node_->now();

    // Subscribe to AprilTag detections
    apriltag_sub_ =
        node_->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "/apriltag/detections", 10,
            std::bind(&InspectionRobot::onAprilTagDetection, this,
                      std::placeholders::_1));

    // Subscribe to laser scan for obstacle avoidance (Tier 1 Safety)
    laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar/scan", 10,
        std::bind(&InspectionRobot::onLaserScan, this, std::placeholders::_1));

    // Create marker publisher for RViz visualization
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/inspection_markers", 10);

    // NOTE: Clicked point subscription removed - use site_marker_node instead

    // Create services
    start_inspection_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        "/start_inspections",
        std::bind(&InspectionRobot::onStartInspectionService, this,
                  std::placeholders::_1, std::placeholders::_2));

    save_sites_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        "/save_damage_sites",
        std::bind(&InspectionRobot::onSaveSitesService, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Try to load existing sites
    try
    {
        loadSitesFromFile(sites_file_);
        RCLCPP_INFO(node_->get_logger(), "Loaded %zu damage sites",
                    damage_sites_.size());
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "No existing sites file, starting fresh");
    }

    RCLCPP_INFO(node_->get_logger(), "Inspection Robot initialized");
    RCLCPP_INFO(
        node_->get_logger(),
        "Tier 1 Safety features ENABLED (TF health + obstacle avoidance)");
    RCLCPP_INFO(node_->get_logger(), "Using %zu pre-defined damage sites",
                damage_sites_.size());
}

void InspectionRobot::loadSitesFromFile(const std::string &filename)
{
    YAML::Node config = YAML::LoadFile(filename);
    damage_sites_.clear();

    if (config["damage_sites"])
    {
        for (const auto &site_node : config["damage_sites"])
        {
            InspectionData::DamageSite site;
            site.name = site_node["name"].as<std::string>();
            site.position.x = site_node["x"].as<double>();
            site.position.y = site_node["y"].as<double>();
            site.position.z = 0.0;
            site.description = site_node["description"].as<std::string>("");
            damage_sites_.push_back(site);
        }
    }
}

void InspectionRobot::saveSitesToFile(const std::string &filename)
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "damage_sites";
    out << YAML::Value << YAML::BeginSeq;

    for (const auto &site : damage_sites_)
    {
        out << YAML::BeginMap;
        out << YAML::Key << "name" << YAML::Value << site.name;
        out << YAML::Key << "x" << YAML::Value << site.position.x;
        out << YAML::Key << "y" << YAML::Value << site.position.y;
        out << YAML::Key << "description" << YAML::Value << site.description;
        out << YAML::EndMap;
    }

    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream fout(filename);
    fout << out.c_str();
    fout.close();

    RCLCPP_INFO(node_->get_logger(), "Saved %zu sites to %s",
                damage_sites_.size(), filename.c_str());
}

void InspectionRobot::addSite(const InspectionData::DamageSite &site)
{
    damage_sites_.push_back(site);
}

void InspectionRobot::clearSites() { damage_sites_.clear(); }

void InspectionRobot::addInspectionRequest(
    const InspectionData::InspectionRequest &request)
{
    inspection_queue_.push_back(request);
    RCLCPP_INFO(node_->get_logger(), "Added inspection: %s (Priority: %d)",
                request.site_name.c_str(), request.priority);
}

void InspectionRobot::clearInspectionQueue() { inspection_queue_.clear(); }

void InspectionRobot::startInspections()
{
    if (inspection_queue_.empty() && !exploration_mode_)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "No inspections to execute and exploration mode disabled");
        return;
    }

    is_inspecting_ = true;
    current_inspection_index_ = 0;
    total_distance_ = 0.0;

    if (exploration_mode_)
    {
        RCLCPP_INFO(node_->get_logger(), "Starting in EXPLORATION MODE");
        auto current_map = slam_controller_->getCurrentMap();
        if (current_map)
        {
            patrol_points_ = generatePatrolPoints(*current_map);
            RCLCPP_INFO(node_->get_logger(), "Generated %zu patrol points",
                        patrol_points_.size());
        }
        current_patrol_index_ = 0;
    }
    else
    {
        // Get current position
        auto current_pose = slam_controller_->getCurrentPose();

        // Optimize route based on mode
        if (use_tsp_optimization_)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "Using TSP optimization (A* + Simulated Annealing)");
            optimized_route_ =
                optimizeRouteTSP(current_pose.position, inspection_queue_);
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "Using ordered inspection sequence");
            optimized_route_ =
                optimizeRoute(current_pose.position, inspection_queue_);
        }

        RCLCPP_INFO(node_->get_logger(),
                    "Starting inspections with %s route of %zu stops",
                    use_tsp_optimization_ ? "TSP-optimized" : "ordered",
                    optimized_route_.size());
    }

    publishStatus("Inspections started");
}

void InspectionRobot::stopInspections()
{
    is_inspecting_ = false;
    is_scanning_ = false;
    motion_controller_->clearPath();
    RCLCPP_INFO(node_->get_logger(), "Inspections stopped");
    publishStatus("Inspections stopped");
}

std::vector<InspectionData::DamageSite> InspectionRobot::optimizeRoute(
    const geometry_msgs::msg::Point &start,
    const std::vector<InspectionData::InspectionRequest> &requests)
{

    // Ordered mode: Follow requests in sequential order
    std::vector<InspectionData::DamageSite> route;

    for (const auto &request : requests)
    {
        auto *site = findSite(request.site_name);
        if (site)
        {
            route.push_back(*site);
        }
    }

    return route;
}

std::vector<InspectionData::DamageSite> InspectionRobot::optimizeRouteTSP(
    const geometry_msgs::msg::Point &start,
    const std::vector<InspectionData::InspectionRequest> &requests)
{

    if (requests.empty())
    {
        return {};
    }

    // Build list of all unique points (start + all sites)
    std::vector<geometry_msgs::msg::Point> points;
    std::vector<std::string> point_names;

    // Add start position
    points.push_back(start);
    point_names.push_back("Start");

    // Add all unique sites from requests
    std::set<std::string> added_sites;
    for (const auto &req : requests)
    {
        if (added_sites.find(req.site_name) == added_sites.end())
        {
            auto *site = findSite(req.site_name);
            if (site)
            {
                points.push_back(site->position);
                point_names.push_back(site->name);
                added_sites.insert(req.site_name);
            }
        }
    }

    RCLCPP_INFO(node_->get_logger(),
                "Building distance matrix for %zu points using A*...",
                points.size());

    // Build distance matrix using A* pathfinding (from base class)
    auto distance_matrix = buildDistanceMatrix(start, points);

    // Run Simulated Annealing to find optimal tour (from base class)
    RCLCPP_INFO(node_->get_logger(),
                "Running Simulated Annealing for TSP optimization...");
    auto optimal_tour =
        simulatedAnnealing(distance_matrix, 0); // Start from index 0

    // Build optimized route from tour
    std::vector<InspectionData::DamageSite> route;
    for (size_t i = 1; i < optimal_tour.size(); ++i)
    { // Skip index 0 (start)
        int point_idx = optimal_tour[i];
        if (point_idx > 0 && point_idx < static_cast<int>(points.size()))
        {
            std::string site_name = point_names[point_idx];
            auto *site = findSite(site_name);
            if (site)
            {
                route.push_back(*site);
            }
        }
    }

    // Calculate total tour cost (from base class)
    double total_cost = calculateTourCost(optimal_tour, distance_matrix);
    RCLCPP_INFO(node_->get_logger(),
                "TSP optimization complete! Total path cost: %.2fm",
                total_cost);

    // Log the optimized route
    std::stringstream route_str;
    route_str << "Optimized route: Start";
    for (const auto &site : route)
    {
        route_str << " → " << site.name;
    }
    route_str << " → Home";
    RCLCPP_INFO(node_->get_logger(), "%s", route_str.str().c_str());

    return route;
}

void InspectionRobot::update()
{
    // Tier 1 Safety: Check TF health periodically
    if ((node_->now() - last_tf_check_time_).seconds() > TF_CHECK_INTERVAL)
    {
        tf_health_ok_ = checkTFHealth();
        last_tf_check_time_ = node_->now();

        if (!tf_health_ok_)
        {
            emergencyStop("TF transform health check failed");
            return;
        }
    }

    // Tier 1 Safety: Check for obstacles
    if (!isSafeToMove())
    {
        emergencyStop("Obstacle detected in path");
        return;
    }

    if (!is_inspecting_)
    {
        return;
    }

    // Check if we have valid map and pose
    if (!slam_controller_->hasValidMap() || !slam_controller_->hasValidPose())
    {
        return;
    }

    // Perform relocalization spin at the start (base class method)
    if (!has_relocalized_)
    {
        if (performRelocalization())
        {
            RCLCPP_INFO(node_->get_logger(), "Starting inspections...");
        }
        return;
    }

    // Handle exploration mode
    if (exploration_mode_)
    {
        executeExploration();
        return;
    }

    // Check if we've completed all inspections
    if (current_inspection_index_ >= optimized_route_.size())
    {
        // Use base class return to home
        returnToHome();

        // Check if we're actually at home
        auto current_pose = slam_controller_->getCurrentPose();
        double distance_to_home = calculateDistance(
            current_pose.position, geometry_msgs::msg::Point());

        if (distance_to_home < HOME_TOLERANCE)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "✓ All inspections completed! Robot at home.");

            // Create completion marker file
            std::ofstream marker("/tmp/inspection_complete.marker");
            marker << "Inspection completed at " << getCurrentTimestamp()
                   << std::endl;
            marker.close();

            stopInspections();
        }
        return;
    }

    auto &current_site = optimized_route_[current_inspection_index_];
    auto current_pose = slam_controller_->getCurrentPose();
    auto current_map = slam_controller_->getCurrentMap();

    // Calculate distance to current site
    double distance_to_site =
        calculateDistance(current_pose.position, current_site.position);

    // Check if we're at the site and need to scan
    if (distance_to_site < TARGET_TOLERANCE)
    {
        if (!is_scanning_)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "✓ Reached site: %s - Starting 360° scan",
                        current_site.name.c_str());
            is_scanning_ = true;
            scan_start_time_ = node_->now();

            // Store start yaw
            scan_start_yaw_ = getYawFromQuaternion(current_pose.orientation);

            // Clear detected tags for this site
            detected_tags_.clear();
        }

        // Perform scanning
        if (performSiteScan())
        {
            // Scan complete — log straight to CSV
            std::ofstream log(inspection_log_file_, std::ios::app);
            const std::string timestamp = getCurrentTimestamp();
            const std::string request_id = "-";
            const std::string site = current_site.name;
            const size_t tags_count = detected_tags_.size();
            const double distance_m = total_distance_;
            const double time_s = 0.0; // fill if you track it
            const std::string status = "OK";

            std::stringstream notes;
            notes << "Inspection completed with " << tags_count
                  << " tags detected";
            if (!detected_tags_.empty())
            {
                notes << " (IDs:";
                for (size_t i = 0; i < detected_tags_.size(); ++i)
                    notes << (i ? " " : " ") << detected_tags_[i];
                notes << ")";
            }

            log << timestamp << "," << request_id << "," << site << ","
                << tags_count << "," << std::fixed << std::setprecision(3)
                << distance_m << "," << std::fixed << std::setprecision(2)
                << time_s << "," << status << "," << notes.str() << "\n";

            // advance to next site
            is_scanning_ = false;
            in_target_docking_mode_ = false;
            path_completed_ = false;
            current_inspection_index_++;
            publishStatus("Completed: " + current_site.name);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }

        return;
    }

    // Enter precise docking mode when close to site (within 1m)
    if (distance_to_site < 1.0)
    {
        // Check if there's a clear path to site (base class method)
        bool has_clear_path = hasLineOfSight(
            current_pose.position, current_site.position, *current_map);

        if (has_clear_path)
        {
            if (!in_target_docking_mode_)
            {
                RCLCPP_INFO(node_->get_logger(),
                            "Entering precise site docking mode (%.3fm from "
                            "%s, clear path)",
                            distance_to_site, current_site.name.c_str());
                in_target_docking_mode_ = true;
                motion_controller_->clearPath();
            }

            // Use precise docking control (base class method)
            preciseTargetDocking(current_pose, current_site.position,
                                 distance_to_site, current_site.name);
            return;
        }
        else
        {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "Close to site (%.2fm) but obstacle detected "
                                 "- continuing with path planner",
                                 distance_to_site);
            in_target_docking_mode_ = false;
        }
    }

    // Reset docking mode if we're far from site
    if (in_target_docking_mode_ && distance_to_site >= 1.2)
    {
        in_target_docking_mode_ = false;
    }

    // Execute motion if we have a path
    if (motion_controller_->hasPath())
    {
        motion_controller_->computeVelocityCommand(current_pose, *current_map);

        // Check if we've reached the path goal
        if (motion_controller_->isAtGoal() && !path_completed_)
        {
            path_completed_ = true;
            RCLCPP_INFO(node_->get_logger(),
                        "Path complete to %s (%.3fm from exact position)",
                        current_site.name.c_str(), distance_to_site);
        }

        return;
    }

    // No path - need to plan a path to site
    if (!motion_controller_->hasPath())
    {
        RCLCPP_INFO(node_->get_logger(), "Planning path to site: %s",
                    current_site.name.c_str());

        GridCell start =
            PathPlanner::worldToGrid(*current_map, current_pose.position);
        GridCell goal =
            PathPlanner::worldToGrid(*current_map, current_site.position);

        auto [cspace, cspace_cells] =
            PathPlanner::calcCSpace(*current_map, false);
        cv::Mat cost_map = PathPlanner::calcCostMap(*current_map);

        auto [path, cost, actual_start, actual_goal] =
            PathPlanner::aStar(cspace, cost_map, start, goal);

        if (path.empty())
        {
            std::ofstream log(inspection_log_file_, std::ios::app);
            const std::string timestamp = getCurrentTimestamp();
            log << timestamp << ","
                << "-" << "," << current_site.name << "," << 0 << ","
                << std::fixed << std::setprecision(3) << total_distance_ << ","
                << std::fixed << std::setprecision(2) << 0.0 << ","
                << "FAIL" << ","
                << "Site unreachable - no valid path found"
                << "\n";

            current_inspection_index_++;
            publishStatus("Skipped unreachable: " + current_site.name);
            return;
        }

        auto path_msg = PathPlanner::pathToMessage(*current_map, path);
        motion_controller_->setPath(path_msg);

        RCLCPP_INFO(node_->get_logger(), "Navigating to %s",
                    current_site.name.c_str());
        publishStatus("Navigating to: " + current_site.name);
        path_completed_ = false;
    }
}

bool InspectionRobot::performSiteScan()
{
    auto current_pose = slam_controller_->getCurrentPose();
    double elapsed = (node_->now() - scan_start_time_).seconds();

    if (elapsed < SCAN_DURATION)
    {
        // Continue rotating for 360° scan
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = node_->now();
        cmd_vel.header.frame_id = "base_footprint";
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = SCAN_SPEED;

        auto cmd_vel_pub =
            node_->create_publisher<geometry_msgs::msg::TwistStamped>(
                "/cmd_vel", 10);
        cmd_vel_pub->publish(cmd_vel);

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Scanning... %.1fs remaining, %zu tags detected",
                             SCAN_DURATION - elapsed, detected_tags_.size());
        return false; // Still scanning
    }
    else
    {
        // Stop spinning (base class method)
        stopMotion();
        return true; // Scan complete
    }
}

bool InspectionRobot::isAtSite(const InspectionData::DamageSite &site)
{
    auto current_pose = slam_controller_->getCurrentPose();
    double distance = calculateDistance(current_pose.position, site.position);
    return distance < TARGET_REACHED_THRESHOLD;
}

void InspectionRobot::onAprilTagDetection(
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{

    if (!is_scanning_)
        return;

    for (const auto &d : msg->detections)
    {
        // Standard apriltag_ros message has a single int32 'id'
        const int tag_id = d.id;
        if (!isTagAlreadyDetected(tag_id))
        {
            detected_tags_.push_back(tag_id);
            RCLCPP_INFO(node_->get_logger(), "️  Detected AprilTag ID: %d",
                        tag_id);
        }
    }
}

bool InspectionRobot::isTagAlreadyDetected(int tag_id) const
{
    return std::find(detected_tags_.begin(), detected_tags_.end(), tag_id) !=
           detected_tags_.end();
}

bool InspectionRobot::checkTFHealth()
{
    try
    {
        // Check if we can get the transform from map to base_footprint
        auto transform = tf_buffer_->lookupTransform("map", "base_footprint",
                                                     tf2::TimePointZero,
                                                     tf2::durationFromSec(0.1));

        // Check if transform is recent (not stale)
        auto transform_time = tf2::timeFromSec(
            transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9);
        auto now = tf2::timeFromSec(node_->now().seconds());
        auto age = tf2::durationToSec(now - transform_time);

        if (age > TF_TIMEOUT)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "TF transform is stale (%.2fs old)", age);
            return false;
        }

        return true;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN(node_->get_logger(), "TF health check failed: %s",
                    ex.what());
        return false;
    }
}

void InspectionRobot::onLaserScan(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Find minimum distance in front of robot (60° cone)
    size_t range_size = msg->ranges.size();
    size_t front_start = range_size * 5 / 12; // -30°
    size_t front_end = range_size * 7 / 12;   // +30°

    min_obstacle_distance_ = std::numeric_limits<double>::max();

    for (size_t i = front_start; i < front_end; ++i)
    {
        if (std::isfinite(msg->ranges[i]))
        {
            min_obstacle_distance_ = std::min(
                min_obstacle_distance_, static_cast<double>(msg->ranges[i]));
        }
    }

    // Check if obstacle is too close
    if (min_obstacle_distance_ < OBSTACLE_STOP_DISTANCE)
    {
        obstacle_detected_ = true;
    }
    else
    {
        obstacle_detected_ = false;
    }
}

bool InspectionRobot::isSafeToMove()
{
    // During scanning, we don't check obstacles (robot is stationary and
    // rotating)
    if (is_scanning_)
    {
        return true;
    }

    // Check if obstacle is detected
    if (obstacle_detected_)
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Obstacle detected at %.2fm - STOPPING",
                             min_obstacle_distance_);
        return false;
    }

    return true;
}

void InspectionRobot::emergencyStop(const std::string &reason)
{
    RCLCPP_ERROR(node_->get_logger(), " EMERGENCY STOP: %s", reason.c_str());

    // Stop all motion (base class method)
    stopMotion();

    // Clear path
    motion_controller_->clearPath();

    // Publish status
    publishStatus("EMERGENCY STOP: " + reason);

    // Note: We don't stop the inspection completely, just pause
    // Once the issue is resolved, the update loop will resume
}

std::vector<geometry_msgs::msg::Point>
InspectionRobot::generatePatrolPoints(const nav_msgs::msg::OccupancyGrid &map)
{

    std::vector<geometry_msgs::msg::Point> patrol_points;

    // Grid-based patrol point generation
    // Divide map into grid and place patrol points in open areas

    double grid_spacing = 2.0; // 2 meters between patrol points

    double min_x = map.info.origin.position.x;
    double min_y = map.info.origin.position.y;
    double max_x = min_x + map.info.width * map.info.resolution;
    double max_y = min_y + map.info.height * map.info.resolution;

    for (double x = min_x + grid_spacing; x < max_x; x += grid_spacing)
    {
        for (double y = min_y + grid_spacing; y < max_y; y += grid_spacing)
        {
            geometry_msgs::msg::Point point;
            point.x = x;
            point.y = y;
            point.z = 0.0;

            // Check if this is a valid patrol point (walkable, far from walls)
            if (isValidPatrolPoint(point, map))
            {
                patrol_points.push_back(point);
            }
        }
    }

    // Shuffle patrol points for variety
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(patrol_points.begin(), patrol_points.end(), g);

    // Limit to reasonable number
    if (patrol_points.size() > 20)
    {
        patrol_points.resize(20);
    }

    return patrol_points;
}

bool InspectionRobot::isValidPatrolPoint(
    const geometry_msgs::msg::Point &point,
    const nav_msgs::msg::OccupancyGrid &map)
{

    // Check if cell is walkable
    GridCell cell = PathPlanner::worldToGrid(map, point);
    if (!PathPlanner::isCellInBounds(map, cell) ||
        !PathPlanner::isCellWalkable(map, cell))
    {
        return false;
    }

    // Check if far enough from walls (base class method)
    double min_dist = checkMinDistanceToWalls(point, map);
    if (min_dist < 0.8)
    { // At least 80cm from walls
        return false;
    }

    return true;
}

void InspectionRobot::executeExploration()
{
    if (patrol_points_.empty())
    {
        RCLCPP_WARN(node_->get_logger(),
                    "No patrol points available for exploration");
        stopInspections();
        return;
    }

    if (current_patrol_index_ >= patrol_points_.size())
    {
        RCLCPP_INFO(node_->get_logger(),
                    "✓ Exploration complete - visited all patrol points");
        stopInspections();
        return;
    }

    auto current_pose = slam_controller_->getCurrentPose();
    auto current_map = slam_controller_->getCurrentMap();
    auto &current_patrol = patrol_points_[current_patrol_index_];

    // Check if reached patrol point
    double distance = calculateDistance(current_pose.position, current_patrol);

    if (distance < TARGET_TOLERANCE)
    {
        RCLCPP_INFO(node_->get_logger(), "✓ Reached patrol point %zu/%zu",
                    current_patrol_index_ + 1, patrol_points_.size());

        // Brief pause at patrol point
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        current_patrol_index_++;
        motion_controller_->clearPath();
        return;
    }

    // Navigate to patrol point
    if (motion_controller_->hasPath())
    {
        motion_controller_->computeVelocityCommand(current_pose, *current_map);
        return;
    }

    // Plan path to patrol point
    GridCell start =
        PathPlanner::worldToGrid(*current_map, current_pose.position);
    GridCell goal = PathPlanner::worldToGrid(*current_map, current_patrol);

    auto [cspace, cspace_cells] = PathPlanner::calcCSpace(*current_map, false);
    cv::Mat cost_map = PathPlanner::calcCostMap(*current_map);

    auto [path, cost, actual_start, actual_goal] =
        PathPlanner::aStar(cspace, cost_map, start, goal);

    if (path.empty())
    {
        RCLCPP_WARN(node_->get_logger(),
                    "Cannot reach patrol point %zu, skipping",
                    current_patrol_index_);
        current_patrol_index_++;
        return;
    }

    auto path_msg = PathPlanner::pathToMessage(*current_map, path);
    motion_controller_->setPath(path_msg);

    RCLCPP_INFO(node_->get_logger(), "Exploring: patrol point %zu/%zu",
                current_patrol_index_ + 1, patrol_points_.size());
}

void InspectionRobot::publishSiteMarkers()
{
    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    for (const auto &site : damage_sites_)
    {
        marker_array.markers.push_back(createSiteMarker(site, id++));
    }

    marker_pub_->publish(marker_array);
}

visualization_msgs::msg::Marker
InspectionRobot::createSiteMarker(const InspectionData::DamageSite &site,
                                  int id)
{

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node_->now();
    marker.ns = "damage_sites";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position = site.position;
    marker.pose.position.z = 0.5; // Raise marker for visibility
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 1.0;

    // All markers yellow (no severity field)
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.7;

    marker.lifetime = rclcpp::Duration(0, 0); // Forever

    return marker;
}

InspectionData::DamageSite *
InspectionRobot::findSite(const std::string &site_name)
{
    for (auto &site : damage_sites_)
    {
        if (site.name == site_name)
        {
            return &site;
        }
    }
    return nullptr;
}

void InspectionRobot::onPointClicked(
    const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    // Add new site with auto-generated name
    InspectionData::DamageSite site;
    site.name = "Site_" + std::to_string(damage_sites_.size() + 1);
    site.position = msg->point;
    site.description = "Damage site added via RViz";

    addSite(site);

    RCLCPP_INFO(node_->get_logger(), "Added damage site: %s at (%.2f, %.2f)",
                site.name.c_str(), site.position.x, site.position.y);

    publishStatus("Site added: " + site.name);
    publishSiteMarkers();
}

void InspectionRobot::onStartInspectionService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{

    if (inspection_queue_.empty() && !exploration_mode_)
    {
        response->success = false;
        response->message =
            "No inspection requests in queue and exploration mode disabled";
        return;
    }

    startInspections();

    if (exploration_mode_)
    {
        response->success = true;
        response->message = "Started exploration mode with " +
                            std::to_string(patrol_points_.size()) +
                            " patrol points";
    }
    else
    {
        response->success = true;
        response->message = "Started inspections for " +
                            std::to_string(inspection_queue_.size()) +
                            " requests";
    }
}

void InspectionRobot::onSaveSitesService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{

    try
    {
        saveSitesToFile(sites_file_);
        response->success = true;
        response->message = "Saved " + std::to_string(damage_sites_.size()) +
                            " sites to " + sites_file_;
    }
    catch (const std::exception &e)
    {
        response->success = false;
        response->message = std::string("Failed to save sites: ") + e.what();
    }
}