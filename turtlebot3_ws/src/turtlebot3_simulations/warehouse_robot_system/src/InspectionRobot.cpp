// ============================================================================
// File: InspectionRobot.cpp
// Description: Implementation of inspection robot for warehouse damage
//              detection using camera and AprilTag markers.
// Author(s): Warehouse Robot System Team
// Last Edited: 2025-11-01
// ============================================================================

#include "InspectionRobot.hpp"
#include <yaml-cpp/yaml.h>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <random>
#include <set>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

InspectionRobot::InspectionRobot(rclcpp::Node::SharedPtr node)
    : node_(node),
      is_inspecting_(false),
      current_inspection_index_(0),
      total_distance_(0.0),
      site_path_completed_(false),
      in_docking_mode_(false),
      in_site_docking_mode_(false),
      initial_yaw_(0.0),
      has_relocalized_(false),
      relocalization_start_time_(node->now()),
      tag_detected_at_site_(false),
      last_detected_tag_id_(-1),
      last_tag_detection_time_(node->now()),
      is_reading_tag_(false),
      tag_reading_start_time_(node->now()),
      use_tsp_optimization_(false),
      exploration_mode_(false),
      current_patrol_index_(0) {
    
    // Initialize SLAM components
    slam_controller_ = std::make_unique<SlamController>(node);
    motion_controller_ = std::make_unique<MotionController>(node);
    
    // Set file paths
    sites_file_ = "damage_sites.yaml";
    inspection_log_file_ = "inspection_log.csv";
    
    // Create CSV header if file doesn't exist
    std::ifstream check(inspection_log_file_);
    if (!check.good()) {
        std::ofstream log(inspection_log_file_);
        log << "Timestamp,SiteName,AprilTagID,Position_X,Position_Y,Distance(m),Time(s),Status,Notes\n";
        log.close();
    }
    
    // Subscribe to clicked points for manual site marking
    clicked_point_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10,
        std::bind(&InspectionRobot::onPointClicked, this, std::placeholders::_1));
    
    // Subscribe to AprilTag detections
    apriltag_sub_ = node_->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/apriltag_detections", 10,
        std::bind(&InspectionRobot::onAprilTagDetection, this, std::placeholders::_1));
    
    // Create services
    start_inspection_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        "/start_inspections",
        std::bind(&InspectionRobot::onStartInspectionService, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    save_sites_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        "/save_damage_sites",
        std::bind(&InspectionRobot::onSaveSitesService, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    // Status publisher
    status_pub_ = node_->create_publisher<std_msgs::msg::String>("/inspection/status", 10);
    
    // Try to load existing sites
    try {
        loadSitesFromFile(sites_file_);
        RCLCPP_INFO(node_->get_logger(), "Loaded %zu damage sites", damage_sites_.size());
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "No existing sites file, starting fresh");
    }
    
    RCLCPP_INFO(node_->get_logger(), "Inspection Robot initialized");
    RCLCPP_INFO(node_->get_logger(), "Camera and AprilTag detection ready");
}

void InspectionRobot::onPointClicked(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    // Add new damage site with auto-generated name
    DamageSite site;
    site.name = "Damage_" + std::to_string(damage_sites_.size() + 1);
    site.position = msg->point;
    site.apriltag_id = -1;  // Will be detected later
    site.description = "Damage site added via RViz";
    
    addSite(site);
    
    RCLCPP_INFO(node_->get_logger(), "Added damage site: %s at (%.2f, %.2f)",
                site.name.c_str(), site.position.x, site.position.y);
    
    publishStatus("Site added: " + site.name);
}

void InspectionRobot::onAprilTagDetection(
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
    
    if (msg->detections.empty()) {
        return;
    }
    
    // Store the most recent detection
    last_detected_tag_id_ = msg->detections[0].id;
    last_tag_detection_time_ = node_->now();
    
    // If we're in exploration mode, automatically save discovered sites
    if (exploration_mode_ && is_inspecting_) {
        auto current_pose = slam_controller_->getCurrentPose();
        
        // Check if we've already discovered this tag at this location
        bool already_discovered = false;
        for (const auto& site : damage_sites_) {
            if (site.apriltag_id == last_detected_tag_id_) {
                double dx = site.position.x - current_pose.position.x;
                double dy = site.position.y - current_pose.position.y;
                double distance = std::sqrt(dx*dx + dy*dy);
                
                if (distance < 1.0) {  // Within 1m of existing site
                    already_discovered = true;
                    break;
                }
            }
        }
        
        if (!already_discovered) {
            RCLCPP_INFO(node_->get_logger(), "🎯 NEW AprilTag discovered! ID: %d at (%.2f, %.2f)",
                       last_detected_tag_id_, current_pose.position.x, current_pose.position.y);
            saveDiscoveredSite(last_detected_tag_id_, current_pose.position);
        }
        
        return;
    }
    
    // If we're at a site and reading tags, mark as detected
    if (is_reading_tag_) {
        tag_detected_at_site_ = true;
        RCLCPP_INFO(node_->get_logger(), "Detected AprilTag ID: %d at current site",
                    last_detected_tag_id_);
    }
}

void InspectionRobot::onStartInspectionService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    if (inspection_queue_.empty()) {
        response->success = false;
        response->message = "No inspection requests in queue";
        return;
    }
    
    startInspections();
    response->success = true;
    response->message = "Started inspections for " + std::to_string(inspection_queue_.size()) + " requests";
}

void InspectionRobot::onSaveSitesService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    try {
        saveSitesToFile(sites_file_);
        response->success = true;
        response->message = "Saved " + std::to_string(damage_sites_.size()) + " sites to " + sites_file_;
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to save sites: ") + e.what();
    }
}

void InspectionRobot::loadSitesFromFile(const std::string& filename) {
    YAML::Node config = YAML::LoadFile(filename);
    damage_sites_.clear();
    
    if (config["damage_sites"]) {
        for (const auto& site_node : config["damage_sites"]) {
            DamageSite site;
            site.name = site_node["name"].as<std::string>();
            site.position.x = site_node["x"].as<double>();
            site.position.y = site_node["y"].as<double>();
            site.position.z = 0.0;
            site.apriltag_id = site_node["apriltag_id"].as<int>(-1);
            site.description = site_node["description"].as<std::string>("");
            damage_sites_.push_back(site);
        }
    }
}

void InspectionRobot::saveSitesToFile(const std::string& filename) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "damage_sites";
    out << YAML::Value << YAML::BeginSeq;
    
    for (const auto& site : damage_sites_) {
        out << YAML::BeginMap;
        out << YAML::Key << "name" << YAML::Value << site.name;
        out << YAML::Key << "x" << YAML::Value << site.position.x;
        out << YAML::Key << "y" << YAML::Value << site.position.y;
        out << YAML::Key << "apriltag_id" << YAML::Value << site.apriltag_id;
        out << YAML::Key << "description" << YAML::Value << site.description;
        out << YAML::EndMap;
    }
    
    out << YAML::EndSeq;
    out << YAML::EndMap;
    
    std::ofstream fout(filename);
    fout << out.c_str();
}

void InspectionRobot::addSite(const DamageSite& site) {
    damage_sites_.push_back(site);
    saveSitesToFile(sites_file_);
}

void InspectionRobot::clearSites() {
    damage_sites_.clear();
    saveSitesToFile(sites_file_);
}

void InspectionRobot::addInspectionRequest(const InspectionRequest& request) {
    inspection_queue_.push_back(request);
}

void InspectionRobot::clearInspectionQueue() {
    inspection_queue_.clear();
}

bool InspectionRobot::hasValidMap() const {
    return slam_controller_->hasValidMap();
}

void InspectionRobot::startInspections() {
    if (inspection_queue_.empty()) {
        RCLCPP_WARN(node_->get_logger(), "No inspection requests to process");
        return;
    }
    
    if (!hasValidMap()) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot start inspections without valid map");
        return;
    }
    
    // Get current position
    auto current_pose = slam_controller_->getCurrentPose();
    geometry_msgs::msg::Point start_pos = current_pose.position;
    
    // Optimize route based on mode
    if (use_tsp_optimization_) {
        RCLCPP_INFO(node_->get_logger(), "Optimizing inspection route using TSP...");
        optimized_route_ = optimizeRouteTSP(start_pos, inspection_queue_);
    } else {
        RCLCPP_INFO(node_->get_logger(), "Using ordered inspection sequence...");
        optimized_route_ = optimizeRoute(start_pos, inspection_queue_);
    }
    
    is_inspecting_ = true;
    current_inspection_index_ = 0;
    total_distance_ = 0.0;
    site_path_completed_ = false;
    
    RCLCPP_INFO(node_->get_logger(), "Starting inspections for %zu sites", optimized_route_.size());
    publishStatus("Inspection started");
}

void InspectionRobot::stopInspections() {
    is_inspecting_ = false;
    motion_controller_->clearPath();
    publishStatus("Inspection stopped");
}


void InspectionRobot::update() {
    if (!is_inspecting_) {
        return;
    }
    
    // Check if we have valid map and pose
    if (!slam_controller_->hasValidMap() || !slam_controller_->hasValidPose()) {
        return;
    }
    
    // Handle exploration mode
    if (exploration_mode_) {
        auto current_map = slam_controller_->getCurrentMap();
        if (!current_map) {
            return;
        }
        
        // Generate patrol points if not done yet
        if (patrol_points_.empty()) {
            RCLCPP_INFO(node_->get_logger(), "Generating patrol points for systematic exploration...");
            generatePatrolPoints(*current_map);
            
            if (patrol_points_.empty()) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to generate patrol points!");
                return;
            }
            
            RCLCPP_INFO(node_->get_logger(), "Generated %zu patrol points", patrol_points_.size());
            publishStatus("Starting systematic patrol");
        }
        
        // Process any AprilTag detections
        processAprilTagDetections();
        
        // Check if we recently detected an AprilTag and should dock to it for better inspection
        if (last_detected_tag_id_ >= 0) {
            double time_since_detection = (node_->now() - last_tag_detection_time_).seconds();
            
            // If tag was detected recently (within 2 seconds) and we haven't saved it yet
            if (time_since_detection < 2.0) {
                auto current_pose = slam_controller_->getCurrentPose();
                
                // Check if this tag is already discovered
                bool already_discovered = false;
                for (const auto& site : damage_sites_) {
                    if (site.apriltag_id == last_detected_tag_id_) {
                        double dx = site.position.x - current_pose.position.x;
                        double dy = site.position.y - current_pose.position.y;
                        double distance = std::sqrt(dx*dx + dy*dy);
                        
                        if (distance < 1.0) {
                            already_discovered = true;
                            break;
                        }
                    }
                }
                
                if (!already_discovered) {
                    RCLCPP_INFO(node_->get_logger(), "🎯 AprilTag ID %d detected! Pausing for better inspection...",
                               last_detected_tag_id_);
                    
                    // Pause briefly for better detection
                    rclcpp::sleep_for(std::chrono::milliseconds(1000));
                    
                    // The onAprilTagDetection callback will save it
                }
            }
        }
        
        // Check if we've completed all patrol points
        if (current_patrol_index_ >= patrol_points_.size()) {
            auto current_pose = slam_controller_->getCurrentPose();
            double dx = 0.0 - current_pose.position.x;
            double dy = 0.0 - current_pose.position.y;
            double distance_to_home = std::sqrt(dx*dx + dy*dy);
            
            // Check if we're already home (5cm tolerance)
            if (distance_to_home < 0.05) {
                // Check alignment
                tf2::Quaternion q(
                    current_pose.orientation.x,
                    current_pose.orientation.y,
                    current_pose.orientation.z,
                    current_pose.orientation.w
                );
                tf2::Matrix3x3 m(q);
                double roll, pitch, current_yaw;
                m.getRPY(roll, pitch, current_yaw);
                
                double angle_diff = initial_yaw_ - current_yaw;
                while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
                while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
                
                if (std::abs(angle_diff) > 0.05) {
                    // Align to initial orientation
                    geometry_msgs::msg::TwistStamped cmd_vel;
                    cmd_vel.header.stamp = node_->now();
                    cmd_vel.header.frame_id = "base_footprint";
                    cmd_vel.twist.linear.x = 0.0;
                    cmd_vel.twist.angular.z = (angle_diff > 0) ? 0.2 : -0.2;
                    
                    auto cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
                    cmd_vel_pub->publish(cmd_vel);
                    
                    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                                        "Aligning to initial orientation (%.1f° remaining)", 
                                        std::abs(angle_diff) * 180.0 / M_PI);
                    return;
                }
                
                RCLCPP_INFO(node_->get_logger(), "✅ Patrol complete and robot at home (aligned)!");
                
                // Stop all motion
                geometry_msgs::msg::TwistStamped stop_cmd;
                stop_cmd.header.stamp = node_->now();
                stop_cmd.header.frame_id = "base_footprint";
                auto cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
                cmd_vel_pub->publish(stop_cmd);
                
                // Create completion marker
                std::ofstream marker("/tmp/inspection_exploration_complete.marker");
                marker << "complete";
                marker.close();
                
                publishStatus("Patrol complete - at home");
                is_inspecting_ = false;
                return;
            }
            
            // Return home with precise docking
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                "Patrol complete! Returning home (%.2fm away)...", distance_to_home);
            
            geometry_msgs::msg::Point home_position;
            home_position.x = 0.0;
            home_position.y = 0.0;
            home_position.z = 0.0;
            
            // Enter precise docking mode when close to home (within 50cm)
            if (distance_to_home < DOCKING_DISTANCE) {
                bool has_clear_path = hasLineOfSight(current_pose.position, home_position, *current_map);
                
                if (has_clear_path) {
                    if (!in_docking_mode_) {
                        RCLCPP_INFO(node_->get_logger(), "Entering home docking mode (%.3fm, clear path)", 
                                   distance_to_home);
                        in_docking_mode_ = true;
                        motion_controller_->clearPath();
                    }
                    
                    preciseDocking(current_pose, distance_to_home);
                    return;
                } else {
                    in_docking_mode_ = false;
                }
            }
            
            // Reset docking if far from home
            if (in_docking_mode_ && distance_to_home >= DOCKING_DISTANCE) {
                in_docking_mode_ = false;
            }
            
            // Execute motion if we have a path
            if (motion_controller_->hasPath()) {
                motion_controller_->computeVelocityCommand(current_pose, *current_map);
                return;
            }
            
            // No path - plan one to home
            returnToHome();
            publishStatus("Returning home");
            return;
        }
        
        // Navigate to current patrol point
        auto& current_point = patrol_points_[current_patrol_index_];
        auto current_pose = slam_controller_->getCurrentPose();
        
        double dx = current_point.x - current_pose.position.x;
        double dy = current_point.y - current_pose.position.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                            "Patrol point %zu/%zu: target=(%.2f, %.2f), distance=%.2fm",
                            current_patrol_index_ + 1, patrol_points_.size(),
                            current_point.x, current_point.y, distance);
        
        // Execute motion if we have a path
        if (motion_controller_->hasPath()) {
            motion_controller_->computeVelocityCommand(current_pose, *current_map);
            
            // Check if we've reached the path goal
            if (motion_controller_->isAtGoal()) {
                RCLCPP_INFO(node_->get_logger(), "✓ Reached patrol point %zu/%zu (%.2fm from target)", 
                           current_patrol_index_ + 1, patrol_points_.size(), distance);
                
                // Clear the path
                motion_controller_->clearPath();
                
                // Pause briefly to allow AprilTag detection
                rclcpp::sleep_for(std::chrono::milliseconds(500));
                
                // Move to next patrol point
                current_patrol_index_++;
                publishStatus("Moving to next patrol point");
            }
            
            return;  // Continue following path
        }
        
        // No path - need to plan one
        if (!navigateToPatrolPoint(current_point)) {
            RCLCPP_WARN(node_->get_logger(), "Failed to navigate to patrol point %zu, skipping...", 
                       current_patrol_index_);
            current_patrol_index_++;
        }
        
        return;
    }
    
    // Perform relocalization spin at the start
    if (!has_relocalized_) {
        auto current_pose = slam_controller_->getCurrentPose();
        auto current_map = slam_controller_->getCurrentMap();
        
        double elapsed = (node_->now() - relocalization_start_time_).seconds();
        
        if (elapsed < 0.1) {
            tf2::Quaternion q(
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w
            );
            tf2::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, initial_yaw_);
            
            // Check if we're too close to walls
            if (current_map) {
                double min_distance_to_wall = checkMinDistanceToWalls(current_pose.position, *current_map);
                
                if (min_distance_to_wall < 0.5) {
                    RCLCPP_WARN(node_->get_logger(), 
                               "Too close to wall (%.2fm) for safe relocalization spin - skipping", 
                               min_distance_to_wall);
                    has_relocalized_ = true;
                    return;
                }
            }
            
            RCLCPP_INFO(node_->get_logger(), "Starting relocalization spin (360°)...");
            relocalization_start_time_ = node_->now();
            elapsed = 0.0;
        }
        
        if (elapsed < RELOCALIZATION_DURATION) {
            geometry_msgs::msg::TwistStamped cmd_vel;
            cmd_vel.header.stamp = node_->now();
            cmd_vel.header.frame_id = "base_footprint";
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.angular.z = RELOCALIZATION_SPEED;
            
            auto cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
            cmd_vel_pub->publish(cmd_vel);
            
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                "Relocalizing... %.1fs remaining", RELOCALIZATION_DURATION - elapsed);
            return;
        } else {
            geometry_msgs::msg::TwistStamped stop_cmd;
            stop_cmd.header.stamp = node_->now();
            stop_cmd.header.frame_id = "base_footprint";
            stop_cmd.twist.linear.x = 0.0;
            stop_cmd.twist.angular.z = 0.0;
            
            auto cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
            cmd_vel_pub->publish(stop_cmd);
            
            has_relocalized_ = true;
            RCLCPP_INFO(node_->get_logger(), "✓ Relocalization complete! Starting inspections...");
            return;
        }
    }
    
    // Check if we've completed all inspections
    if (current_inspection_index_ >= optimized_route_.size()) {
        returnToHome();
        return;
    }
    
    auto& current_site = optimized_route_[current_inspection_index_];
    auto current_pose = slam_controller_->getCurrentPose();
    auto current_map = slam_controller_->getCurrentMap();
    
    // Calculate distance to current site
    double dx = current_site.position.x - current_pose.position.x;
    double dy = current_site.position.y - current_pose.position.y;
    double distance_to_site = std::sqrt(dx*dx + dy*dy);
    
    // Check if we've reached the site
    bool site_reached = (distance_to_site < SITE_TOLERANCE) || 
                       (site_path_completed_ && distance_to_site < SITE_REACHED_THRESHOLD);
    
    if (site_reached) {
        // If we haven't started reading the tag yet, start now
        if (!is_reading_tag_) {
            RCLCPP_INFO(node_->get_logger(), "✓ Reached site: %s (%.3fm) - Reading AprilTag...", 
                       current_site.name.c_str(), distance_to_site);
            
            // Stop all motion
            geometry_msgs::msg::TwistStamped stop_cmd;
            stop_cmd.header.stamp = node_->now();
            stop_cmd.header.frame_id = "base_footprint";
            stop_cmd.twist.linear.x = 0.0;
            stop_cmd.twist.angular.z = 0.0;
            
            auto cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
            cmd_vel_pub->publish(stop_cmd);
            
            is_reading_tag_ = true;
            tag_detected_at_site_ = false;
            tag_reading_start_time_ = node_->now();
            return;
        }
        
        // Wait for tag detection or timeout
        double reading_elapsed = (node_->now() - tag_reading_start_time_).seconds();
        
        if (tag_detected_at_site_) {
            // Successfully read tag
            RCLCPP_INFO(node_->get_logger(), "✓ AprilTag ID %d detected at %s", 
                       last_detected_tag_id_, current_site.name.c_str());
            
            // Update site with detected tag ID
            current_site.apriltag_id = last_detected_tag_id_;
            
            // Save inspection record
            InspectionRecord record;
            record.site_name = current_site.name;
            record.apriltag_id = last_detected_tag_id_;
            record.position = current_site.position;
            record.start_time = std::chrono::system_clock::now();
            record.end_time = std::chrono::system_clock::now();
            record.distance_traveled = total_distance_;
            record.duration_seconds = reading_elapsed;
            record.success = true;
            record.notes = "AprilTag detected successfully";
            
            saveInspectionRecord(record);
            
            // Reset flags and move to next site
            is_reading_tag_ = false;
            in_site_docking_mode_ = false;
            site_path_completed_ = false;
            current_inspection_index_++;
            
            publishStatus("Inspected: " + current_site.name);
            
            // Small pause
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            
        } else if (reading_elapsed > TAG_DETECTION_TIMEOUT) {
            // Timeout - no tag detected
            RCLCPP_WARN(node_->get_logger(), "⚠ No AprilTag detected at %s after %.1fs", 
                       current_site.name.c_str(), reading_elapsed);
            
            // Save failed inspection record
            InspectionRecord record;
            record.site_name = current_site.name;
            record.apriltag_id = -1;
            record.position = current_site.position;
            record.start_time = std::chrono::system_clock::now();
            record.end_time = std::chrono::system_clock::now();
            record.distance_traveled = total_distance_;
            record.duration_seconds = reading_elapsed;
            record.success = false;
            record.notes = "No AprilTag detected (timeout)";
            
            saveInspectionRecord(record);
            
            // Reset flags and move to next site
            is_reading_tag_ = false;
            in_site_docking_mode_ = false;
            site_path_completed_ = false;
            current_inspection_index_++;
            
            publishStatus("Failed: " + current_site.name + " (no tag)");
        } else {
            // Still waiting for tag
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                "Waiting for AprilTag... %.1fs", reading_elapsed);
        }
        
        return;
    }
    
    // Navigate to site using precise docking when close
    if (distance_to_site < 1.0) {
        bool has_clear_path = hasLineOfSight(current_pose.position, current_site.position, *current_map);
        
        if (has_clear_path) {
            if (!in_site_docking_mode_) {
                RCLCPP_INFO(node_->get_logger(), "Entering precise site docking mode (%.3fm from %s)", 
                           distance_to_site, current_site.name.c_str());
                in_site_docking_mode_ = true;
                motion_controller_->clearPath();
            }
            
            preciseSiteDocking(current_pose, current_site, distance_to_site);
            return;
        }
    }
    
    // Normal navigation to site
    if (!navigateToSite(current_site)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to navigate to site: %s", current_site.name.c_str());
        stopInspections();
    }
}

bool InspectionRobot::navigateToSite(const DamageSite& site) {
    auto current_pose = slam_controller_->getCurrentPose();
    auto current_map = slam_controller_->getCurrentMap();
    
    if (!current_map) {
        return false;
    }
    
    // Execute motion if we have a path
    if (motion_controller_->hasPath()) {
        motion_controller_->computeVelocityCommand(current_pose, *current_map);
        
        // Check if we've reached the path goal
        if (motion_controller_->isAtGoal() && !site_path_completed_) {
            site_path_completed_ = true;
            double dx = site.position.x - current_pose.position.x;
            double dy = site.position.y - current_pose.position.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            RCLCPP_INFO(node_->get_logger(), 
                       "Path complete to %s (%.3fm from exact position)", 
                       site.name.c_str(), distance);
        }
        
        return true;  // Continue following path
    }
    
    // No path - need to plan one
    if (!motion_controller_->hasPath()) {
        RCLCPP_INFO(node_->get_logger(), "Planning path to site: %s", site.name.c_str());
        
        // Plan path using A*
        GridCell start = PathPlanner::worldToGrid(*current_map, current_pose.position);
        GridCell goal = PathPlanner::worldToGrid(*current_map, site.position);
        
        auto [cspace, cspace_cells] = PathPlanner::calcCSpace(*current_map, false);
        cv::Mat cost_map = PathPlanner::calcCostMap(*current_map);
        
        auto [path, cost, actual_start, actual_goal] = 
            PathPlanner::aStar(cspace, cost_map, start, goal);
        
        // If path planning fails, try to find nearest accessible point
        if (path.empty()) {
            RCLCPP_WARN(node_->get_logger(),
                       "Cannot reach exact site position, searching for nearest accessible point...");
            
            // Search in expanding radius around goal
            bool found_goal = false;
            std::vector<std::pair<int,int>> directions = {
                {1,0}, {-1,0}, {0,1}, {0,-1},
                {1,1}, {1,-1}, {-1,1}, {-1,-1}
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
                            geometry_msgs::msg::Point alt_pos = PathPlanner::gridToWorld(*current_map, candidate);
                            RCLCPP_INFO(node_->get_logger(), 
                                       "Found accessible point near %s at (%.2f, %.2f)",
                                       site.name.c_str(), alt_pos.x, alt_pos.y);
                            break;
                        }
                    }
                }
            }
            
            if (!found_goal) {
                RCLCPP_ERROR(node_->get_logger(),
                            "Failed to plan path to %s - site unreachable", 
                            site.name.c_str());
                return false;
            }
        }
        
        if (!path.empty()) {
            auto path_msg = PathPlanner::pathToMessage(*current_map, path);
            motion_controller_->setPath(path_msg);
            
            RCLCPP_INFO(node_->get_logger(), "Navigating to %s", site.name.c_str());
            publishStatus("Navigating to: " + site.name);
            site_path_completed_ = false;
        }
    }
    
    return true;
}

DamageSite* InspectionRobot::findSite(const std::string& site_name) {
    for (auto& site : damage_sites_) {
        if (site.name == site_name) {
            return &site;
        }
    }
    return nullptr;
}

bool InspectionRobot::isAtSite(const DamageSite& site) {
    auto current_pose = slam_controller_->getCurrentPose();
    double dx = site.position.x - current_pose.position.x;
    double dy = site.position.y - current_pose.position.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    return distance < SITE_REACHED_THRESHOLD;
}


void InspectionRobot::returnToHome() {
    auto current_pose = slam_controller_->getCurrentPose();
    auto current_map = slam_controller_->getCurrentMap();
    
    if (!current_map) {
        RCLCPP_ERROR(node_->get_logger(), "No map available for return home");
        stopInspections();
        return;
    }
    
    geometry_msgs::msg::Point home_position;
    home_position.x = 0.0;
    home_position.y = 0.0;
    home_position.z = 0.0;
    
    double dx = home_position.x - current_pose.position.x;
    double dy = home_position.y - current_pose.position.y;
    double distance_to_home = std::sqrt(dx*dx + dy*dy);
    
    // Check if already at home
    if (distance_to_home < HOME_TOLERANCE) {
        RCLCPP_INFO(node_->get_logger(), "✓ All inspections complete! Robot at home.");
        stopInspections();
        
        // Create completion marker
        std::ofstream marker("/tmp/inspection_complete.marker");
        marker << "complete";
        marker.close();
        
        return;
    }
    
    // Enter precise docking mode when close to home
    if (distance_to_home < DOCKING_DISTANCE) {
        bool has_clear_path = hasLineOfSight(current_pose.position, home_position, *current_map);
        
        if (has_clear_path) {
            if (!in_docking_mode_) {
                RCLCPP_INFO(node_->get_logger(), "Entering precise home docking mode (%.3fm from home)", 
                           distance_to_home);
                in_docking_mode_ = true;
                motion_controller_->clearPath();
            }
            
            preciseDocking(current_pose, distance_to_home);
            return;
        }
    }
    
    // Execute motion if we have a path
    if (motion_controller_->hasPath()) {
        motion_controller_->computeVelocityCommand(current_pose, *current_map);
        return;
    }
    
    // Plan path to home if needed
    if (!motion_controller_->hasPath()) {
        RCLCPP_INFO(node_->get_logger(), "Planning path home from (%.2f, %.2f)...",
                   current_pose.position.x, current_pose.position.y);
        
        // Plan path using A*
        GridCell start = PathPlanner::worldToGrid(*current_map, current_pose.position);
        GridCell goal = PathPlanner::worldToGrid(*current_map, home_position);
        
        auto [cspace, cspace_cells] = PathPlanner::calcCSpace(*current_map, false);
        cv::Mat cost_map = PathPlanner::calcCostMap(*current_map);
        
        auto [path, cost, actual_start, actual_goal] = 
            PathPlanner::aStar(cspace, cost_map, start, goal);
        
        if (path.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "Cannot find path home!");
            stopInspections();
            return;
        }
        
        auto path_msg = PathPlanner::pathToMessage(*current_map, path);
        motion_controller_->setPath(path_msg);
        RCLCPP_INFO(node_->get_logger(), "Path to home planned with %zu waypoints", path_msg.poses.size());
        publishStatus("Returning home");
    }
}

void InspectionRobot::preciseDocking(const geometry_msgs::msg::Pose& current_pose, double distance_to_home) {
    geometry_msgs::msg::Point home_position;
    home_position.x = 0.0;
    home_position.y = 0.0;
    
    double dx = home_position.x - current_pose.position.x;
    double dy = home_position.y - current_pose.position.y;
    double angle_to_home = std::atan2(dy, dx);
    
    tf2::Quaternion q(
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    double angle_diff = angle_to_home - yaw;
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
    
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = node_->now();
    cmd_vel.header.frame_id = "base_footprint";
    
    if (std::abs(angle_diff) > 0.175) {
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = (angle_diff > 0) ? DOCKING_ANGULAR_SPEED : -DOCKING_ANGULAR_SPEED;
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "Docking: Aligning (angle error: %.1f°)", angle_diff * 180.0 / M_PI);
    } else {
        cmd_vel.twist.linear.x = DOCKING_LINEAR_SPEED;
        cmd_vel.twist.angular.z = angle_diff * 0.5;
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "Docking: Moving forward (%.3fm remaining)", distance_to_home);
    }
    
    auto cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    cmd_vel_pub->publish(cmd_vel);
}

void InspectionRobot::preciseSiteDocking(const geometry_msgs::msg::Pose& current_pose, 
                                        const DamageSite& site, 
                                        double distance_to_site) {
    double dx = site.position.x - current_pose.position.x;
    double dy = site.position.y - current_pose.position.y;
    double angle_to_site = std::atan2(dy, dx);
    
    tf2::Quaternion q(
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    double angle_diff = angle_to_site - yaw;
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
    
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = node_->now();
    cmd_vel.header.frame_id = "base_footprint";
    
    if (std::abs(angle_diff) > 0.175) {
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = (angle_diff > 0) ? DOCKING_ANGULAR_SPEED : -DOCKING_ANGULAR_SPEED;
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "Site docking: Aligning (angle error: %.1f°)", angle_diff * 180.0 / M_PI);
    } else {
        cmd_vel.twist.linear.x = DOCKING_LINEAR_SPEED;
        cmd_vel.twist.angular.z = angle_diff * 0.5;
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "Site docking: Moving forward (%.3fm remaining)", distance_to_site);
    }
    
    auto cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    cmd_vel_pub->publish(cmd_vel);
}

bool InspectionRobot::hasLineOfSight(const geometry_msgs::msg::Point& from, 
                                    const geometry_msgs::msg::Point& to,
                                    const nav_msgs::msg::OccupancyGrid& map) {
    GridCell start = PathPlanner::worldToGrid(map, from);
    GridCell goal = PathPlanner::worldToGrid(map, to);
    
    int x0 = start.first;
    int y0 = start.second;
    int x1 = goal.first;
    int y1 = goal.second;
    
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    const int robot_radius_cells = 2;
    
    while (true) {
        for (int dx_check = -robot_radius_cells; dx_check <= robot_radius_cells; ++dx_check) {
            for (int dy_check = -robot_radius_cells; dy_check <= robot_radius_cells; ++dy_check) {
                if (dx_check * dx_check + dy_check * dy_check <= robot_radius_cells * robot_radius_cells) {
                    GridCell check_cell = {x0 + dx_check, y0 + dy_check};
                    if (!PathPlanner::isCellInBounds(map, check_cell) || 
                        !PathPlanner::isCellWalkable(map, check_cell)) {
                        return false;
                    }
                }
            }
        }
        
        if (x0 == x1 && y0 == y1) {
            break;
        }
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
    
    return true;
}

double InspectionRobot::checkMinDistanceToWalls(const geometry_msgs::msg::Point& position,
                                               const nav_msgs::msg::OccupancyGrid& map) {
    GridCell center = PathPlanner::worldToGrid(map, position);
    
    double min_distance = std::numeric_limits<double>::max();
    
    std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };
    
    for (const auto& [dx, dy] : directions) {
        for (int step = 1; step < 100; ++step) {
            GridCell check_cell = {center.first + dx * step, center.second + dy * step};
            
            if (!PathPlanner::isCellInBounds(map, check_cell)) {
                break;
            }
            
            if (!PathPlanner::isCellWalkable(map, check_cell)) {
                geometry_msgs::msg::Point obstacle_pos = PathPlanner::gridToWorld(map, check_cell);
                double dist_x = obstacle_pos.x - position.x;
                double dist_y = obstacle_pos.y - position.y;
                double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);
                
                min_distance = std::min(min_distance, distance);
                break;
            }
        }
    }
    
    return min_distance;
}

// Route optimization methods (simplified versions)
std::vector<DamageSite> InspectionRobot::optimizeRoute(
    const geometry_msgs::msg::Point& start,
    const std::vector<InspectionRequest>& requests) {
    
    std::vector<DamageSite> route;
    
    for (const auto& request : requests) {
        auto* site = findSite(request.site_name);
        if (site) {
            route.push_back(*site);
        }
    }
    
    return route;
}

std::vector<DamageSite> InspectionRobot::optimizeRouteTSP(
    const geometry_msgs::msg::Point& start,
    const std::vector<InspectionRequest>& requests) {
    
    if (requests.empty()) {
        return {};
    }
    
    std::vector<geometry_msgs::msg::Point> points;
    std::vector<DamageSite> sites;
    
    points.push_back(start);
    
    for (const auto& req : requests) {
        auto* site = findSite(req.site_name);
        if (site) {
            points.push_back(site->position);
            sites.push_back(*site);
        }
    }
    
    if (sites.empty()) {
        return {};
    }
    
    // Build distance matrix using A*
    auto distance_matrix = buildDistanceMatrix(start, points);
    
    // Run simulated annealing TSP solver
    auto tour = simulatedAnnealing(distance_matrix, 0);
    
    // Convert tour indices to sites
    std::vector<DamageSite> optimized_route;
    for (size_t i = 1; i < tour.size(); ++i) {  // Skip start (index 0)
        int site_idx = tour[i] - 1;  // Adjust for start position
        if (site_idx >= 0 && site_idx < static_cast<int>(sites.size())) {
            optimized_route.push_back(sites[site_idx]);
        }
    }
    
    return optimized_route;
}

std::vector<std::vector<double>> InspectionRobot::buildDistanceMatrix(
    const geometry_msgs::msg::Point& start,
    const std::vector<geometry_msgs::msg::Point>& points) {
    
    size_t n = points.size();
    std::vector<std::vector<double>> matrix(n, std::vector<double>(n, 0.0));
    
    auto current_map = slam_controller_->getCurrentMap();
    if (!current_map) {
        return matrix;
    }
    
    // Calculate distances between all pairs
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            double dx = points[j].x - points[i].x;
            double dy = points[j].y - points[i].y;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            matrix[i][j] = distance;
            matrix[j][i] = distance;
        }
    }
    
    return matrix;
}

std::vector<int> InspectionRobot::simulatedAnnealing(
    const std::vector<std::vector<double>>& distance_matrix,
    int start_idx,
    double initial_temp,
    double cooling_rate,
    int max_iterations) {
    
    int n = distance_matrix.size();
    if (n <= 1) {
        return {start_idx};
    }
    
    // Initialize with greedy nearest neighbor
    std::vector<int> current_tour = {start_idx};
    std::vector<bool> visited(n, false);
    visited[start_idx] = true;
    
    int current_city = start_idx;
    for (int i = 1; i < n; ++i) {
        int nearest = -1;
        double min_dist = std::numeric_limits<double>::max();
        
        for (int j = 0; j < n; ++j) {
            if (!visited[j] && distance_matrix[current_city][j] < min_dist) {
                min_dist = distance_matrix[current_city][j];
                nearest = j;
            }
        }
        
        if (nearest != -1) {
            current_tour.push_back(nearest);
            visited[nearest] = true;
            current_city = nearest;
        }
    }
    
    double current_cost = calculateTourCost(current_tour, distance_matrix);
    std::vector<int> best_tour = current_tour;
    double best_cost = current_cost;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    
    double temperature = initial_temp;
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        // Generate neighbor by swapping two random cities (excluding start)
        std::uniform_int_distribution<> city_dis(1, n - 1);
        int i = city_dis(gen);
        int j = city_dis(gen);
        
        if (i == j) continue;
        
        std::vector<int> new_tour = current_tour;
        std::swap(new_tour[i], new_tour[j]);
        
        double new_cost = calculateTourCost(new_tour, distance_matrix);
        double delta = new_cost - current_cost;
        
        if (delta < 0 || dis(gen) < std::exp(-delta / temperature)) {
            current_tour = new_tour;
            current_cost = new_cost;
            
            if (current_cost < best_cost) {
                best_tour = current_tour;
                best_cost = current_cost;
            }
        }
        
        temperature *= cooling_rate;
    }
    
    return best_tour;
}

double InspectionRobot::calculateTourCost(
    const std::vector<int>& tour,
    const std::vector<std::vector<double>>& distance_matrix) {
    
    double cost = 0.0;
    for (size_t i = 0; i < tour.size() - 1; ++i) {
        cost += distance_matrix[tour[i]][tour[i + 1]];
    }
    return cost;
}

void InspectionRobot::saveInspectionRecord(const InspectionRecord& record) {
    std::ofstream log(inspection_log_file_, std::ios::app);
    
    auto time_t_start = std::chrono::system_clock::to_time_t(record.start_time);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t_start), "%Y-%m-%d %H:%M:%S");
    
    log << ss.str() << ","
        << record.site_name << ","
        << record.apriltag_id << ","
        << std::fixed << std::setprecision(3)
        << record.position.x << ","
        << record.position.y << ","
        << record.distance_traveled << ","
        << record.duration_seconds << ","
        << (record.success ? "Success" : "Failed") << ","
        << record.notes << "\n";
    
    log.close();
    
    RCLCPP_INFO(node_->get_logger(), "Inspection record saved: %s", record.site_name.c_str());
}

void InspectionRobot::publishStatus(const std::string& status) {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
}

std::string InspectionRobot::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

// ============================================================================
// EXPLORATION MODE IMPLEMENTATION
// ============================================================================

void InspectionRobot::startExploration() {
    if (exploration_mode_) {
        RCLCPP_INFO(node_->get_logger(), "🔍 Starting inspection exploration patrol...");
        is_inspecting_ = true;  // Reuse inspection flag to enable update loop
        publishStatus("Starting exploration patrol");
    } else {
        RCLCPP_WARN(node_->get_logger(), "Not in exploration mode!");
    }
}

void InspectionRobot::generatePatrolPoints(const nav_msgs::msg::OccupancyGrid& map) {
    patrol_points_.clear();
    
    // Grid-based patrol pattern - smaller spacing for thorough coverage
    double grid_spacing = 0.8;  // 0.8 meter spacing for better coverage
    
    // Get map bounds
    double map_width = map.info.width * map.info.resolution;
    double map_height = map.info.height * map.info.resolution;
    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;
    
    RCLCPP_INFO(node_->get_logger(), "🗺️  Map bounds: %.2f x %.2f meters, origin: (%.2f, %.2f)", 
               map_width, map_height, origin_x, origin_y);
    RCLCPP_INFO(node_->get_logger(), "📐 Grid spacing: %.1fm", grid_spacing);
    
    // Generate grid points with smaller footprint check
    int points_generated = 0;
    int points_rejected = 0;
    
    for (double y = origin_y + 0.3; y < origin_y + map_height - 0.3; y += grid_spacing) {
        for (double x = origin_x + 0.3; x < origin_x + map_width - 0.3; x += grid_spacing) {
            geometry_msgs::msg::Point point;
            point.x = x;
            point.y = y;
            point.z = 0.0;
            
            // Check if point is in free space
            GridCell cell = PathPlanner::worldToGrid(map, point);
            if (!PathPlanner::isCellInBounds(map, cell) || 
                !PathPlanner::isCellWalkable(map, cell)) {
                points_rejected++;
                continue;
            }
            
            // Check surrounding area is also free (smaller robot footprint check)
            bool area_clear = true;
            for (int dx = -1; dx <= 1 && area_clear; dx++) {
                for (int dy = -1; dy <= 1 && area_clear; dy++) {
                    GridCell check_cell = {cell.first + dx, cell.second + dy};
                    if (!PathPlanner::isCellInBounds(map, check_cell) ||
                        !PathPlanner::isCellWalkable(map, check_cell)) {
                        area_clear = false;
                    }
                }
            }
            
            if (area_clear) {
                patrol_points_.push_back(point);
                points_generated++;
            } else {
                points_rejected++;
            }
        }
    }
    
    RCLCPP_INFO(node_->get_logger(), "✅ Generated %d patrol points (rejected %d due to obstacles)", 
               points_generated, points_rejected);
    
    if (patrol_points_.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "❌ No valid patrol points generated! Map may be too constrained.");
    } else {
        RCLCPP_INFO(node_->get_logger(), "🎯 Patrol coverage: ~%.1f square meters", 
                   points_generated * grid_spacing * grid_spacing);
    }
}

bool InspectionRobot::navigateToPatrolPoint(const geometry_msgs::msg::Point& point) {
    auto current_pose = slam_controller_->getCurrentPose();
    auto current_map = slam_controller_->getCurrentMap();
    
    if (!current_map) {
        return false;
    }
    
    // Execute motion if we have a path
    if (motion_controller_->hasPath()) {
        motion_controller_->computeVelocityCommand(current_pose, *current_map);
        return true;
    }
    
    // Plan path to patrol point
    RCLCPP_INFO(node_->get_logger(), "Planning path to patrol point (%.2f, %.2f)", 
               point.x, point.y);
    
    GridCell start = PathPlanner::worldToGrid(*current_map, current_pose.position);
    GridCell goal = PathPlanner::worldToGrid(*current_map, point);
    
    auto [cspace, cspace_cells] = PathPlanner::calcCSpace(*current_map, false);
    cv::Mat cost_map = PathPlanner::calcCostMap(*current_map);
    
    auto [path, cost, actual_start, actual_goal] = 
        PathPlanner::aStar(cspace, cost_map, start, goal);
    
    if (path.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Cannot reach patrol point (%.2f, %.2f)", 
                   point.x, point.y);
        return false;
    }
    
    auto path_msg = PathPlanner::pathToMessage(*current_map, path);
    motion_controller_->setPath(path_msg);
    
    return true;
}

void InspectionRobot::processAprilTagDetections() {
    // This method is called continuously to check for new AprilTag detections
    // The actual detection handling is in onAprilTagDetection callback
}

void InspectionRobot::saveDiscoveredSite(int tag_id, const geometry_msgs::msg::Point& position) {
    // Create new damage site
    DamageSite site;
    site.name = "Damage_" + std::to_string(damage_sites_.size() + 1);
    site.apriltag_id = tag_id;
    site.position = position;
    site.description = "Automatically discovered during patrol";
    site.discovered_time = std::chrono::system_clock::now();
    
    // Add to list
    damage_sites_.push_back(site);
    
    // Save to file
    try {
        saveSitesToFile(sites_file_);
        RCLCPP_INFO(node_->get_logger(), "✓ Discovered and saved damage site: %s (AprilTag ID: %d)", 
                   site.name.c_str(), tag_id);
        publishStatus("Discovered: " + site.name);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to save discovered site: %s", e.what());
    }
    
    // Log to exploration CSV
    std::ofstream log("inspection_exploration_log.csv", std::ios::app);
    if (log.is_open()) {
        // Create header if file is new
        std::ifstream check("inspection_exploration_log.csv");
        bool file_exists = check.good();
        check.close();
        
        if (!file_exists) {
            log << "Timestamp,SiteName,AprilTagID,Position_X,Position_Y,PatrolPoint,Notes\n";
        }
        
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S");
        
        log << ss.str() << ","
            << site.name << ","
            << tag_id << ","
            << std::fixed << std::setprecision(3)
            << position.x << ","
            << position.y << ","
            << current_patrol_index_ << ","
            << "Discovered during systematic patrol\n";
        
        log.close();
    }
}
