#include "DeliveryRobot.hpp"
#include <yaml-cpp/yaml.h>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

DeliveryRobot::DeliveryRobot(rclcpp::Node::SharedPtr node)
    : node_(node),
      is_delivering_(false),
      current_delivery_index_(0),
      total_distance_(0.0),
      in_docking_mode_(false) {
    
    // Initialize SLAM components
    slam_controller_ = std::make_unique<SlamController>(node);
    motion_controller_ = std::make_unique<MotionController>(node);
    
    // Set file paths
    zones_file_ = "delivery_zones.yaml";
    delivery_log_file_ = "delivery_log.csv";
    
    // Create CSV header if file doesn't exist
    std::ifstream check(delivery_log_file_);
    if (!check.good()) {
        std::ofstream log(delivery_log_file_);
        log << "Timestamp,RequestID,FromZone,ToZone,Distance(m),Time(s),Status,Notes\n";
        log.close();
    }
    
    // Subscribe to RViz clicked points
    clicked_point_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10,
        std::bind(&DeliveryRobot::onPointClicked, this, std::placeholders::_1));
    
    // Create services
    start_delivery_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        "/start_deliveries",
        std::bind(&DeliveryRobot::onStartDeliveryService, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    save_zones_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        "/save_delivery_zones",
        std::bind(&DeliveryRobot::onSaveZonesService, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    // Status publisher
    status_pub_ = node_->create_publisher<std_msgs::msg::String>("/delivery/status", 10);
    
    // Try to load existing zones
    try {
        loadZonesFromFile(zones_file_);
        RCLCPP_INFO(node_->get_logger(), "Loaded %zu delivery zones", zones_.size());
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "No existing zones file, starting fresh");
    }
    
    RCLCPP_INFO(node_->get_logger(), "Delivery Robot initialized");
    RCLCPP_INFO(node_->get_logger(), "Click points in RViz to add delivery zones");
    RCLCPP_INFO(node_->get_logger(), "Call /save_delivery_zones service to save zones");
    RCLCPP_INFO(node_->get_logger(), "Call /start_deliveries service to begin deliveries");
}

void DeliveryRobot::onPointClicked(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    // Add new zone with auto-generated name
    DeliveryZone zone;
    zone.name = "Zone_" + std::to_string(zones_.size() + 1);
    zone.position = msg->point;
    zone.description = "Delivery zone added via RViz";
    
    addZone(zone);
    
    RCLCPP_INFO(node_->get_logger(), "Added delivery zone: %s at (%.2f, %.2f)",
                zone.name.c_str(), zone.position.x, zone.position.y);
    
    publishStatus("Zone added: " + zone.name);
}

void DeliveryRobot::onStartDeliveryService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    if (delivery_queue_.empty()) {
        response->success = false;
        response->message = "No delivery requests in queue";
        return;
    }
    
    startDeliveries();
    response->success = true;
    response->message = "Started deliveries for " + std::to_string(delivery_queue_.size()) + " requests";
}

void DeliveryRobot::onSaveZonesService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    try {
        saveZonesToFile(zones_file_);
        response->success = true;
        response->message = "Saved " + std::to_string(zones_.size()) + " zones to " + zones_file_;
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to save zones: ") + e.what();
    }
}

void DeliveryRobot::loadZonesFromFile(const std::string& filename) {
    YAML::Node config = YAML::LoadFile(filename);
    zones_.clear();
    
    if (config["delivery_zones"]) {
        for (const auto& zone_node : config["delivery_zones"]) {
            DeliveryZone zone;
            zone.name = zone_node["name"].as<std::string>();
            zone.position.x = zone_node["x"].as<double>();
            zone.position.y = zone_node["y"].as<double>();
            zone.position.z = 0.0;
            zone.description = zone_node["description"].as<std::string>("");
            zones_.push_back(zone);
        }
    }
}

void DeliveryRobot::saveZonesToFile(const std::string& filename) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "delivery_zones";
    out << YAML::Value << YAML::BeginSeq;
    
    for (const auto& zone : zones_) {
        out << YAML::BeginMap;
        out << YAML::Key << "name" << YAML::Value << zone.name;
        out << YAML::Key << "x" << YAML::Value << zone.position.x;
        out << YAML::Key << "y" << YAML::Value << zone.position.y;
        out << YAML::Key << "description" << YAML::Value << zone.description;
        out << YAML::EndMap;
    }
    
    out << YAML::EndSeq;
    out << YAML::EndMap;
    
    std::ofstream fout(filename);
    fout << out.c_str();
    fout.close();
    
    RCLCPP_INFO(node_->get_logger(), "Saved %zu zones to %s", zones_.size(), filename.c_str());
}

void DeliveryRobot::addZone(const DeliveryZone& zone) {
    zones_.push_back(zone);
}

void DeliveryRobot::clearZones() {
    zones_.clear();
}

void DeliveryRobot::addDeliveryRequest(const DeliveryRequest& request) {
    delivery_queue_.push_back(request);
    RCLCPP_INFO(node_->get_logger(), "Added delivery: %s -> %s (Priority: %d)",
                request.from_zone.c_str(), request.to_zone.c_str(), request.priority);
}

void DeliveryRobot::clearDeliveryQueue() {
    delivery_queue_.clear();
}

void DeliveryRobot::startDeliveries() {
    if (delivery_queue_.empty()) {
        RCLCPP_WARN(node_->get_logger(), "No deliveries to execute");
        return;
    }
    
    is_delivering_ = true;
    current_delivery_index_ = 0;
    total_distance_ = 0.0;
    
    // Get current position
    auto current_pose = slam_controller_->getCurrentPose();
    
    // Optimize route
    optimized_route_ = optimizeRoute(current_pose.position, delivery_queue_);
    
    RCLCPP_INFO(node_->get_logger(), "Starting deliveries with optimized route of %zu stops",
                optimized_route_.size());
    
    publishStatus("Deliveries started");
}

void DeliveryRobot::stopDeliveries() {
    is_delivering_ = false;
    motion_controller_->clearPath();
    RCLCPP_INFO(node_->get_logger(), "Deliveries stopped");
    publishStatus("Deliveries stopped");
}

std::vector<DeliveryZone> DeliveryRobot::optimizeRoute(
    const geometry_msgs::msg::Point& start,
    const std::vector<DeliveryRequest>& requests) {
    
    // Simple nearest neighbor TSP
    std::vector<DeliveryZone> route;
    std::vector<DeliveryRequest> remaining = requests;
    auto current_pos = start;
    
    while (!remaining.empty()) {
        double min_dist = std::numeric_limits<double>::max();
        size_t nearest_idx = 0;
        
        // Find nearest unvisited zone
        for (size_t i = 0; i < remaining.size(); ++i) {
            auto* from_zone = findZone(remaining[i].from_zone);
            if (!from_zone) continue;
            
            double dx = from_zone->position.x - current_pos.x;
            double dy = from_zone->position.y - current_pos.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }
        
        // Add from and to zones to route
        auto* from_zone = findZone(remaining[nearest_idx].from_zone);
        auto* to_zone = findZone(remaining[nearest_idx].to_zone);
        
        if (from_zone && to_zone) {
            route.push_back(*from_zone);
            route.push_back(*to_zone);
            current_pos = to_zone->position;
        }
        
        remaining.erase(remaining.begin() + nearest_idx);
    }
    
    return route;
}

DeliveryZone* DeliveryRobot::findZone(const std::string& zone_name) {
    for (auto& zone : zones_) {
        if (zone.name == zone_name) {
            return &zone;
        }
    }
    return nullptr;
}

bool DeliveryRobot::isAtZone(const DeliveryZone& zone) {
    auto current_pose = slam_controller_->getCurrentPose();
    double dx = zone.position.x - current_pose.position.x;
    double dy = zone.position.y - current_pose.position.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    return distance < ZONE_REACHED_THRESHOLD;
}

void DeliveryRobot::update() {
    if (!is_delivering_) {
        return;
    }
    
    // Check if we have valid map and pose
    if (!slam_controller_->hasValidMap() || !slam_controller_->hasValidPose()) {
        return;
    }
    
    // Check if we've completed all deliveries
    if (current_delivery_index_ >= optimized_route_.size()) {
        returnToHome();
        return;
    }
    
    auto& current_zone = optimized_route_[current_delivery_index_];
    
    // Execute motion if we have a path
    if (motion_controller_->hasPath()) {
        auto current_map = slam_controller_->getCurrentMap();
        auto current_pose = slam_controller_->getCurrentPose();
        motion_controller_->computeVelocityCommand(current_pose, *current_map);
        return;  // Continue following path
    }
    
    // No path - check if we just finished navigating to this zone
    auto current_pose = slam_controller_->getCurrentPose();
    double dx = current_zone.position.x - current_pose.position.x;
    double dy = current_zone.position.y - current_pose.position.y;
    double distance_to_zone = std::sqrt(dx*dx + dy*dy);
    
    // If we're close to the zone and have no path, we must have just arrived
    if (distance_to_zone < ZONE_REACHED_THRESHOLD * 2.0) {  // Use 2x threshold for adjusted positions
        RCLCPP_INFO(node_->get_logger(), "Reached zone: %s (%.2fm)", 
                   current_zone.name.c_str(), distance_to_zone);
        
        // Save delivery record
        DeliveryRecord record;
        record.timestamp = getCurrentTimestamp();
        record.from_zone = current_delivery_index_ > 0 ? 
            optimized_route_[current_delivery_index_ - 1].name : "Start";
        record.to_zone = current_zone.name;
        record.distance_traveled = total_distance_;
        record.success = true;
        record.notes = "Delivery completed successfully";
        
        saveDeliveryRecord(record);
        
        // Move to next zone
        current_delivery_index_++;
        
        publishStatus("Reached: " + current_zone.name);
        
        return;
    }
    
    // No path and not at zone - need to plan a path
    if (!motion_controller_->hasPath()) {
        auto current_map = slam_controller_->getCurrentMap();
        auto current_pose = slam_controller_->getCurrentPose();
        
        // Plan path using A*
        GridCell start = PathPlanner::worldToGrid(*current_map, current_pose.position);
        GridCell goal = PathPlanner::worldToGrid(*current_map, current_zone.position);
        
        auto [cspace, cspace_cells] = PathPlanner::calcCSpace(*current_map, false);
        cv::Mat cost_map = PathPlanner::calcCostMap(*current_map);
        
        auto [path, cost, actual_start, actual_goal] = 
            PathPlanner::aStar(cspace, cost_map, start, goal);
        
        // If path planning fails, try to find nearest accessible point
        if (path.empty()) {
            RCLCPP_WARN(node_->get_logger(),
                       "Cannot reach exact zone position, searching for nearest accessible point...");
            
            // Search in expanding radius around goal
            bool found_goal = false;
            std::vector<std::pair<int,int>> directions = {
                {1,0}, {-1,0}, {0,1}, {0,-1},  // Cardinal
                {1,1}, {1,-1}, {-1,1}, {-1,-1}  // Diagonal
            };
            
            for (int radius = 1; radius <= 30 && !found_goal; radius++) {
                for (const auto& [dx_unit, dy_unit] : directions) {
                    GridCell candidate = {
                        goal.first + dx_unit * radius, 
                        goal.second + dy_unit * radius
                    };
                    
                    // Check if this cell is walkable
                    if (PathPlanner::isCellInBounds(*current_map, candidate) &&
                        PathPlanner::isCellWalkable(*current_map, candidate)) {
                        
                        // Try to plan to this cell
                        auto [alt_path, alt_cost, alt_start, alt_goal] = 
                            PathPlanner::aStar(cspace, cost_map, start, candidate);
                        
                        if (!alt_path.empty()) {
                            path = alt_path;
                            found_goal = true;
                            geometry_msgs::msg::Point alt_pos = PathPlanner::gridToWorld(*current_map, candidate);
                            RCLCPP_INFO(node_->get_logger(), 
                                       "Found accessible point near %s at (%.2f, %.2f)",
                                       current_zone.name.c_str(), alt_pos.x, alt_pos.y);
                            break;
                        }
                    }
                }
            }
            
            // If we still can't find a path, skip this zone
            if (!found_goal) {
                RCLCPP_ERROR(node_->get_logger(),
                            "Failed to plan path to %s - zone unreachable, skipping...", 
                            current_zone.name.c_str());
                
                // Save failed delivery record
                DeliveryRecord record;
                record.timestamp = getCurrentTimestamp();
                record.from_zone = current_delivery_index_ > 0 ? 
                    optimized_route_[current_delivery_index_ - 1].name : "Start";
                record.to_zone = current_zone.name;
                record.distance_traveled = total_distance_;
                record.success = false;
                record.notes = "Zone unreachable - no valid path found";
                saveDeliveryRecord(record);
                
                // Move to next zone
                current_delivery_index_++;
                publishStatus("Skipped unreachable: " + current_zone.name);
                return;
            }
        }
        
        if (!path.empty()) {
            auto path_msg = PathPlanner::pathToMessage(*current_map, path);
            motion_controller_->setPath(path_msg);
            
            RCLCPP_INFO(node_->get_logger(), "Navigating to %s", current_zone.name.c_str());
            publishStatus("Navigating to: " + current_zone.name);
        }
    }
}

void DeliveryRobot::saveDeliveryRecord(const DeliveryRecord& record) {
    std::ofstream log(delivery_log_file_, std::ios::app);
    log << record.toLogString() << "\n";
    log.close();
    
    delivery_history_.push_back(record);
    
    RCLCPP_INFO(node_->get_logger(), "Delivery record saved: %s -> %s",
                record.from_zone.c_str(), record.to_zone.c_str());
}

void DeliveryRobot::publishStatus(const std::string& status) {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
}

std::string DeliveryRobot::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

bool DeliveryRobot::hasValidMap() const {
    return slam_controller_ && slam_controller_->hasValidMap();
}

void DeliveryRobot::returnToHome() {
    if (!slam_controller_->hasValidMap() || !slam_controller_->hasValidPose()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                            "Waiting for valid map and pose to return home...");
        return;
    }
    
    auto current_map = slam_controller_->getCurrentMap();
    auto current_pose = slam_controller_->getCurrentPose();
    
    geometry_msgs::msg::Point home_position;
    home_position.x = 0.0;
    home_position.y = 0.0;
    home_position.z = 0.0;
    
    // Check distance to home
    double dx = home_position.x - current_pose.position.x;
    double dy = home_position.y - current_pose.position.y;
    double distance_to_home = std::sqrt(dx * dx + dy * dy);
    
    // Check if reached home with tight tolerance (5cm)
    if (distance_to_home < HOME_TOLERANCE) {
        RCLCPP_INFO(node_->get_logger(), "✓ All deliveries completed! Robot at home (%.3fm).", distance_to_home);
        
        // Create completion marker file for script
        std::ofstream marker("/tmp/delivery_complete.marker");
        marker << "Delivery completed at " << getCurrentTimestamp() << std::endl;
        marker.close();
        
        stopDeliveries();
        return;
    }
    
    // Enter precise docking mode when close to home (within 50cm)
    if (distance_to_home < DOCKING_DISTANCE) {
        if (!in_docking_mode_) {
            RCLCPP_INFO(node_->get_logger(), "Entering precise docking mode (%.3fm from home)", distance_to_home);
            in_docking_mode_ = true;
            motion_controller_->clearPath();  // Stop using path planner
        }
        
        // Use precise docking control
        preciseDocking(current_pose, distance_to_home);
        return;
    }
    
    // Reset docking mode if we're far from home
    if (in_docking_mode_ && distance_to_home >= DOCKING_DISTANCE) {
        in_docking_mode_ = false;
    }
    
    // Execute motion if we have a path
    if (motion_controller_->hasPath()) {
        motion_controller_->computeVelocityCommand(current_pose, *current_map);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                            "Returning home: %.2fm remaining", distance_to_home);
        return;
    }
    
    // No path - check if we're close enough to switch to docking
    // This handles the case where path planner can't get us closer (home in obstacle)
    if (distance_to_home < DOCKING_DISTANCE * 1.5) {  // Within 75cm
        RCLCPP_INFO(node_->get_logger(), 
                   "Path complete, %.2fm from home - switching to docking mode", 
                   distance_to_home);
        in_docking_mode_ = true;
        motion_controller_->clearPath();
        preciseDocking(current_pose, distance_to_home);
        return;
    }
    
    // Need to plan path home
    RCLCPP_INFO(node_->get_logger(), "All deliveries completed! Returning home (%.2fm)...", distance_to_home);
    
    GridCell start = PathPlanner::worldToGrid(*current_map, current_pose.position);
    GridCell goal = PathPlanner::worldToGrid(*current_map, home_position);
    
    auto [cspace, cspace_cells] = PathPlanner::calcCSpace(*current_map, false);
    cv::Mat cost_map = PathPlanner::calcCostMap(*current_map);
    
    auto [path, cost, actual_start, actual_goal] = 
        PathPlanner::aStar(cspace, cost_map, start, goal);
    
    // If direct path fails, search for nearest accessible point
    if (path.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Cannot reach exact home position with path planner, will use docking mode when close");
        
        bool found_goal = false;
        std::vector<std::pair<int,int>> directions = {
            {1,0}, {-1,0}, {0,1}, {0,-1},  // Cardinal
            {1,1}, {1,-1}, {-1,1}, {-1,-1}  // Diagonal
        };
        
        for (int radius = 1; radius <= 30 && !found_goal; radius++) {
            for (const auto& [dx_unit, dy_unit] : directions) {
                GridCell candidate = {
                    goal.first + dx_unit * radius, 
                    goal.second + dy_unit * radius
                };
                
                if (PathPlanner::isCellInBounds(*current_map, candidate) &&
                    PathPlanner::isCellWalkable(*current_map, candidate)) {
                    
                    auto [alt_path, alt_cost, alt_start, alt_goal] = 
                        PathPlanner::aStar(cspace, cost_map, start, candidate);
                    
                    if (!alt_path.empty()) {
                        path = alt_path;
                        found_goal = true;
                        geometry_msgs::msg::Point alt_home = PathPlanner::gridToWorld(*current_map, candidate);
                        RCLCPP_INFO(node_->get_logger(), 
                                   "Found accessible point near home at (%.2f, %.2f), will dock from there",
                                   alt_home.x, alt_home.y);
                        break;
                    }
                }
            }
        }
    }
    
    if (!path.empty()) {
        auto path_msg = PathPlanner::pathToMessage(*current_map, path);
        motion_controller_->setPath(path_msg);
        RCLCPP_INFO(node_->get_logger(), "Path to home planned with %zu waypoints", path_msg.poses.size());
        publishStatus("Returning home");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Cannot find any path home! Stopping deliveries.");
        stopDeliveries();
    }
}

void DeliveryRobot::preciseDocking(const geometry_msgs::msg::Pose& current_pose, double distance_to_home) {
    geometry_msgs::msg::Point home_position;
    home_position.x = 0.0;
    home_position.y = 0.0;
    
    // Calculate direction to home
    double dx = home_position.x - current_pose.position.x;
    double dy = home_position.y - current_pose.position.y;
    double angle_to_home = std::atan2(dy, dx);
    
    // Get current robot orientation
    tf2::Quaternion q(
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // Calculate angle difference
    double angle_diff = angle_to_home - yaw;
    // Normalize to [-pi, pi]
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
    
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = node_->now();
    cmd_vel.header.frame_id = "base_footprint";
    
    // If angle is off by more than 10 degrees, rotate first
    if (std::abs(angle_diff) > 0.175) {  // ~10 degrees
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = (angle_diff > 0) ? DOCKING_ANGULAR_SPEED : -DOCKING_ANGULAR_SPEED;
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "Docking: Aligning (angle error: %.1f°)", angle_diff * 180.0 / M_PI);
    } else {
        // Move forward slowly toward home
        cmd_vel.twist.linear.x = DOCKING_LINEAR_SPEED;
        cmd_vel.twist.angular.z = angle_diff * 0.5;  // Gentle correction
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "Docking: Moving forward (%.3fm remaining)", distance_to_home);
    }
    
    // Publish directly (bypass motion controller during docking)
    auto cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    cmd_vel_pub->publish(cmd_vel);
}
