// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: Camera_Node.cpp
// Author(s): Inez Dumas, Aryan Rai
//
// Description: Implementation of delivery robot for multi-point delivery
//              with route optimization via TSP.

#include "Robot/DeliveryRobot.hpp"
#include <yaml-cpp/yaml.h>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <set>
#include <limits>

DeliveryRobot::DeliveryRobot(rclcpp::Node::SharedPtr node)
    : WarehouseRobot(node),
      is_delivering_(false),
      current_delivery_index_(0) {
    
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
    
    // NOTE: Clicked point subscription removed - use zone_marker_node instead
    // This prevents duplicate zone creation when both nodes are running
    
    // Create services
    start_delivery_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        "/start_deliveries",
        std::bind(&DeliveryRobot::onStartDeliveryService, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    save_zones_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        "/save_delivery_zones",
        std::bind(&DeliveryRobot::onSaveZonesService, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    // Try to load existing zones
    try {
        loadZonesFromFile(zones_file_);
        RCLCPP_INFO(node_->get_logger(), "Loaded %zu delivery zones", zones_.size());
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "No existing zones file, starting fresh");
    }
    
    RCLCPP_INFO(node_->get_logger(), "Delivery Robot initialized");
    RCLCPP_INFO(node_->get_logger(), "Using %zu pre-defined delivery zones", zones_.size());
    RCLCPP_INFO(node_->get_logger(), "Use 'Define Delivery Zones' mode to add/edit zones");
}

// ============================================================================
// Zone Management
// ============================================================================

void DeliveryRobot::loadZonesFromFile(const std::string& filename) {
    YAML::Node config = YAML::LoadFile(filename);
    zones_.clear();
    
    if (config["delivery_zones"]) {
        for (const auto& zone_node : config["delivery_zones"]) {
            DeliveryData::DeliveryZone zone;
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

void DeliveryRobot::addZone(const DeliveryData::DeliveryZone& zone) {
    zones_.push_back(zone);
}

void DeliveryRobot::clearZones() {
    zones_.clear();
}

// ============================================================================
// Delivery Management
// ============================================================================

void DeliveryRobot::addDeliveryRequest(const DeliveryData::DeliveryRequest& request) {
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
    
    // Optimize route based on mode
    if (use_tsp_optimization_) {
        RCLCPP_INFO(node_->get_logger(), "Using TSP optimization (A* + Simulated Annealing)");
        optimized_route_ = optimizeRouteTSP(current_pose.position, delivery_queue_);
    } else {
        RCLCPP_INFO(node_->get_logger(), "Using ordered delivery sequence");
        optimized_route_ = optimizeRoute(current_pose.position, delivery_queue_);
    }
    
    RCLCPP_INFO(node_->get_logger(), "Starting deliveries with %s route of %zu stops",
                use_tsp_optimization_ ? "TSP-optimized" : "ordered",
                optimized_route_.size());
    
    publishStatus("Deliveries started");
}

void DeliveryRobot::stopDeliveries() {
    is_delivering_ = false;
    motion_controller_->clearPath();
    RCLCPP_INFO(node_->get_logger(), "Deliveries stopped");
    publishStatus("Deliveries stopped");
}

// ============================================================================
// Route Optimization
// ============================================================================

std::vector<DeliveryData::DeliveryZone> DeliveryRobot::optimizeRoute(
    const geometry_msgs::msg::Point& start,
    const std::vector<DeliveryData::DeliveryRequest>& requests) {
    
    // Ordered mode: Follow requests in sequential order (no optimization)
    std::vector<DeliveryData::DeliveryZone> route;
    
    for (const auto& request : requests) {
        // Add from and to zones in order
        auto* from_zone = findZone(request.from_zone);
        auto* to_zone = findZone(request.to_zone);
        
        if (from_zone && to_zone) {
            route.push_back(*from_zone);
            route.push_back(*to_zone);
        }
    }
    
    return route;
}

std::vector<DeliveryData::DeliveryZone> DeliveryRobot::optimizeRouteTSP(
    const geometry_msgs::msg::Point& start,
    const std::vector<DeliveryData::DeliveryRequest>& requests) {
    
    if (requests.empty()) {
        return {};
    }
    
    // Build list of all unique points (start + all from/to zones)
    std::vector<geometry_msgs::msg::Point> points;
    std::vector<std::string> point_names;
    
    // Add start position
    points.push_back(start);
    point_names.push_back("Start");
    
    // Add all unique zones from requests
    std::set<std::string> added_zones;
    for (const auto& req : requests) {
        // Add from zone
        if (added_zones.find(req.from_zone) == added_zones.end()) {
            auto* zone = findZone(req.from_zone);
            if (zone) {
                points.push_back(zone->position);
                point_names.push_back(zone->name);
                added_zones.insert(req.from_zone);
            }
        }
        
        // Add to zone
        if (added_zones.find(req.to_zone) == added_zones.end()) {
            auto* zone = findZone(req.to_zone);
            if (zone) {
                points.push_back(zone->position);
                point_names.push_back(zone->name);
                added_zones.insert(req.to_zone);
            }
        }
    }
    
    RCLCPP_INFO(node_->get_logger(), "Building distance matrix for %zu points using A*...", points.size());
    
    // Build distance matrix using A* pathfinding (from base class)
    auto distance_matrix = buildDistanceMatrix(start, points);
    
    // Run Simulated Annealing to find optimal tour (from base class)
    RCLCPP_INFO(node_->get_logger(), "Running Simulated Annealing for TSP optimization...");
    auto optimal_tour = simulatedAnnealing(distance_matrix, 0);  // Start from index 0 (start position)
    
    // Build optimized route from tour
    std::vector<DeliveryData::DeliveryZone> route;
    for (size_t i = 1; i < optimal_tour.size(); ++i) {  // Skip index 0 (start)
        int point_idx = optimal_tour[i];
        if (point_idx > 0 && point_idx < static_cast<int>(points.size())) {
            std::string zone_name = point_names[point_idx];
            auto* zone = findZone(zone_name);
            if (zone) {
                route.push_back(*zone);
            }
        }
    }
    
    // Calculate total tour cost (from base class)
    double total_cost = calculateTourCost(optimal_tour, distance_matrix);
    RCLCPP_INFO(node_->get_logger(), "TSP optimization complete! Total path cost: %.2fm", total_cost);
    
    // Log the optimized route
    std::stringstream route_str;
    route_str << "Optimized route: Start";
    for (const auto& zone : route) {
        route_str << " → " << zone.name;
    }
    route_str << " → Home";
    RCLCPP_INFO(node_->get_logger(), "%s", route_str.str().c_str());
    
    return route;
}

// ============================================================================
// Main Update Loop
// ============================================================================

void DeliveryRobot::update() {
    if (!is_delivering_) {
        return;
    }
    
    // Check if we have valid map and pose
    if (!slam_controller_->hasValidMap() || !slam_controller_->hasValidPose()) {
        return;
    }
    
    // Perform relocalization spin at the start (base class method)
    if (!has_relocalized_) {
        if (performRelocalization()) {
            RCLCPP_INFO(node_->get_logger(), "Starting deliveries...");
        }
        return;
    }
    
    // Check if we've completed all deliveries
    if (current_delivery_index_ >= optimized_route_.size()) {
        // Use base class return to home
        returnToHome();
        
        // Check if we're actually at home
        auto current_pose = slam_controller_->getCurrentPose();
        double distance_to_home = calculateDistance(current_pose.position, geometry_msgs::msg::Point());
        
        if (distance_to_home < HOME_TOLERANCE) {
            RCLCPP_INFO(node_->get_logger(), "✓ All deliveries completed! Robot at home.");
            
            // Create completion marker file for script
            std::ofstream marker("/tmp/delivery_complete.marker");
            marker << "Delivery completed at " << getCurrentTimestamp() << std::endl;
            marker.close();
            
            stopDeliveries();
        }
        return;
    }
    
    auto& current_zone = optimized_route_[current_delivery_index_];
    auto current_pose = slam_controller_->getCurrentPose();
    auto current_map = slam_controller_->getCurrentMap();
    
    // Calculate distance to current zone
    double dx = current_zone.position.x - current_pose.position.x;
    double dy = current_zone.position.y - current_pose.position.y;
    double distance_to_zone = std::sqrt(dx*dx + dy*dy);
    
    // Check if we've reached the zone with tight tolerance OR if path is complete and we're close
    bool zone_reached = (distance_to_zone < TARGET_TOLERANCE) || 
                       (path_completed_ && distance_to_zone < TARGET_REACHED_THRESHOLD);
    
    if (zone_reached) {
        RCLCPP_INFO(node_->get_logger(), "✓ Reached zone: %s (%.3fm)", 
                   current_zone.name.c_str(), distance_to_zone);
        
        // Stop all motion (base class method)
        stopMotion();
        
        // Save delivery record
        DeliveryData::DeliveryRecord record;
        record.timestamp = getCurrentTimestamp();
        record.from_zone = current_delivery_index_ > 0 ? 
            optimized_route_[current_delivery_index_ - 1].name : "Start";
        record.to_zone = current_zone.name;
        record.distance_traveled = total_distance_;
        record.success = true;
        record.notes = "Delivery completed successfully";
        
        saveDeliveryRecord(record);
        
        // Reset docking mode and flags, move to next zone
        in_target_docking_mode_ = false;
        path_completed_ = false;
        current_delivery_index_++;
        
        publishStatus("Reached: " + current_zone.name);
        
        // Small pause at delivery zone (simulate unloading)
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        
        return;
    }
    
    // Enter precise docking mode when reasonably close to zone (within 1m)
    // BUT only if there's a clear line of sight to zone
    if (distance_to_zone < 1.0) {
        // Check if there's a clear path to zone (base class method)
        bool has_clear_path = hasLineOfSight(current_pose.position, current_zone.position, *current_map);
        
        if (has_clear_path) {
            if (!in_target_docking_mode_) {
                RCLCPP_INFO(node_->get_logger(), "Entering precise zone docking mode (%.3fm from %s, clear path)", 
                           distance_to_zone, current_zone.name.c_str());
                in_target_docking_mode_ = true;
                motion_controller_->clearPath();
            }
            
            // Use precise docking control (base class method)
            preciseTargetDocking(current_pose, current_zone.position, distance_to_zone, current_zone.name);
            return;
        } else {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                "Close to zone (%.2fm) but obstacle detected - continuing with path planner", 
                                distance_to_zone);
            in_target_docking_mode_ = false;
        }
    }
    
    // Reset docking mode if we're far from zone
    if (in_target_docking_mode_ && distance_to_zone >= 1.2) {
        in_target_docking_mode_ = false;
    }
    
    // Execute motion if we have a path
    if (motion_controller_->hasPath()) {
        motion_controller_->computeVelocityCommand(current_pose, *current_map);
        
        // Check if we've reached the path goal
        if (motion_controller_->isAtGoal() && !path_completed_) {
            path_completed_ = true;
            RCLCPP_INFO(node_->get_logger(), 
                       "Path complete to %s (%.3fm from exact position)", 
                       current_zone.name.c_str(), distance_to_zone);
        }
        
        return;  // Continue following path
    }
    
    // No path - check if we've already completed a path to this zone
    if (path_completed_) {
        // We've reached as close as we can get via path planning
        // Always try docking if there's line of sight, regardless of distance
        bool has_clear_path = hasLineOfSight(current_pose.position, current_zone.position, *current_map);
        
        if (has_clear_path && distance_to_zone < 1.0) {
            RCLCPP_INFO(node_->get_logger(), 
                       "Path complete, %.2fm from zone with clear line of sight - switching to docking mode", 
                       distance_to_zone);
            in_target_docking_mode_ = true;
            motion_controller_->clearPath();
            preciseTargetDocking(current_pose, current_zone.position, distance_to_zone, current_zone.name);
            return;
        } else if (!has_clear_path) {
            // Obstacle in way - accept this as "reached"
            RCLCPP_INFO(node_->get_logger(),
                       "Path complete, %.2fm from zone but obstacle detected - accepting position", 
                       distance_to_zone);
            
            // Mark as reached even though not at exact position
            path_completed_ = false;
            
            // Save delivery record
            DeliveryData::DeliveryRecord record;
            record.timestamp = getCurrentTimestamp();
            record.from_zone = current_delivery_index_ > 0 ? 
                optimized_route_[current_delivery_index_ - 1].name : "Start";
            record.to_zone = current_zone.name;
            record.distance_traveled = total_distance_;
            record.success = true;
            record.notes = "Reached nearest accessible point (obstacle at exact location)";
            
            saveDeliveryRecord(record);
            
            current_delivery_index_++;
            publishStatus("Reached: " + current_zone.name + " (nearest point)");
            return;
        } else {
            // Path complete but too far from zone (>1m) - accept position
            RCLCPP_WARN(node_->get_logger(),
                       "Path complete but %.2fm from zone (too far for docking) - accepting position", 
                       distance_to_zone);
            
            path_completed_ = false;
            
            DeliveryData::DeliveryRecord record;
            record.timestamp = getCurrentTimestamp();
            record.from_zone = current_delivery_index_ > 0 ? 
                optimized_route_[current_delivery_index_ - 1].name : "Start";
            record.to_zone = current_zone.name;
            record.distance_traveled = total_distance_;
            record.success = true;
            record.notes = "Reached nearest accessible point";
            
            saveDeliveryRecord(record);
            
            current_delivery_index_++;
            publishStatus("Reached: " + current_zone.name + " (nearest point)");
            return;
        }
    }
    
    // No path and not at zone - need to plan a path (first time or replanning)
    if (!motion_controller_->hasPath()) {
        RCLCPP_INFO(node_->get_logger(), "Planning path to zone: %s", current_zone.name.c_str());
        
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
                DeliveryData::DeliveryRecord record;
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
            path_completed_ = false;
        }
    }
}

// ============================================================================
// Helper Methods
// ============================================================================

DeliveryData::DeliveryZone* DeliveryRobot::findZone(const std::string& zone_name) {
    for (auto& zone : zones_) {
        if (zone.name == zone_name) {
            return &zone;
        }
    }
    return nullptr;
}

bool DeliveryRobot::isAtZone(const DeliveryData::DeliveryZone& zone) {
    auto current_pose = slam_controller_->getCurrentPose();
    double distance = calculateDistance(current_pose.position, zone.position);
    return distance < TARGET_REACHED_THRESHOLD;
}

void DeliveryRobot::saveDeliveryRecord(const DeliveryData::DeliveryRecord& record) {
    std::ofstream log(delivery_log_file_, std::ios::app);
    log << record.toLogString() << "\n";
    log.close();
    
    delivery_history_.push_back(record);
    
    RCLCPP_INFO(node_->get_logger(), "Delivery record saved: %s -> %s",
                record.from_zone.c_str(), record.to_zone.c_str());
}

// ============================================================================
// Service Callbacks
// ============================================================================

void DeliveryRobot::onPointClicked(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    // Add new zone with auto-generated name
    DeliveryData::DeliveryZone zone;
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