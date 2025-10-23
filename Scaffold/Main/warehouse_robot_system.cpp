// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: warehouse_robot_system.cpp
// Author(s): Dylan George
//
// Implementation file for polymorphic warehouse robot system with factory manager

#include "turtlebot3_node/warehouse_robot_system.hpp"
#include <cmath>
#include <cstdlib>

// ============================================================================
// DATA STRUCTURES IMPLEMENTATION
// ============================================================================
Pose::Pose(double x_val, double y_val, double theta_val) 
    : x(x_val), y(y_val), theta(theta_val) {}

// ============================================================================
// ROBOT BASE CLASS IMPLEMENTATION
// ============================================================================
Robot::Robot(int id, RobotType type) 
    : robot_id_(id), robot_type_(type), current_state_(IDLE), 
      battery_level_(MAX_BATTERY_LEVEL), is_charging_(false) {}

void Robot::moveToLocation(const Pose& target) {
    current_state_ = MOVING_TO_TASK;
    // Pseudocode: Calculate velocity commands for differential drive
    // Use PID controller for smooth movement
    // Publish cmd_vel messages to turtlebot3
    double distance = calculateDistance(current_pose_, target);
    double angle = calculateAngle(current_pose_, target);
    
    // Simple movement controller
    while (!reachedTarget(target)) {
        publishVelocityCommand(0.2, angle * 0.5); // Move forward and turn
        updateOdometry();
    }
    stopMotors();
}

void Robot::updateBatteryLevel(double usage_rate) {
    if (!is_charging_) {
        battery_level_ -= usage_rate;
        if (battery_level_ <= 0) battery_level_ = 0;
        if (battery_level_ <= LOW_BATTERY_THRESHOLD) current_state_ = LOW_BATTERY;
    }
}

bool Robot::needsCharging() const { 
    return battery_level_ <= LOW_BATTERY_THRESHOLD; 
}

void Robot::returnToDockingStation() { 
    current_state_ = RETURNING_TO_DOCK; 
    moveToLocation(Pose(0.0, 0.0, 0.0)); // Dock at origin
}

void Robot::startCharging() { 
    is_charging_ = true; 
    current_state_ = CHARGING;
    // Simulate charging process
    while (is_charging_ && battery_level_ < MAX_BATTERY_LEVEL) {
        battery_level_ += 2; // Charging rate
        if (battery_level_ >= MAX_BATTERY_LEVEL) {
            battery_level_ = MAX_BATTERY_LEVEL;
            is_charging_ = false;
            current_state_ = IDLE;
        }
    }
}

// Getters
RobotState Robot::getCurrentState() const { return current_state_; }
int Robot::getBatteryLevel() const { return battery_level_; }
int Robot::getRobotId() const { return robot_id_; }
RobotType Robot::getRobotType() const { return robot_type_; }
Pose Robot::getCurrentPose() const { return current_pose_; }

// Protected methods
void Robot::publishVelocityCommand(double linear_x, double angular_z) {
    // Pseudocode: Publish to /cmd_vel topic for turtlebot movement
    // geometry_msgs::Twist msg;
    // msg.linear.x = linear_x;
    // msg.angular.z = angular_z;
    // cmd_vel_publisher_.publish(msg);
}

void Robot::stopMotors() { 
    publishVelocityCommand(0.0, 0.0); 
}

bool Robot::reachedTarget(const Pose& target, double tolerance) {
    double distance = calculateDistance(current_pose_, target);
    return distance < tolerance;
}

void Robot::updateOdometry() {
    // Pseudocode: Subscribe to /odom topic and update current_pose_
    // nav_msgs::Odometry odom_msg = odom_subscriber_.getLatestMessage();
    // current_pose_.x = odom_msg.pose.pose.position.x;
    // current_pose_.y = odom_msg.pose.pose.position.y;
}

double Robot::calculateDistance(const Pose& from, const Pose& to) {
    return sqrt(pow(from.x - to.x, 2) + pow(from.y - to.y, 2));
}

double Robot::calculateAngle(const Pose& from, const Pose& to) {
    return atan2(to.y - from.y, to.x - from.x);
}

// ============================================================================
// INSPECTION ROBOT IMPLEMENTATION
// ============================================================================
InspectionRobot::InspectionRobot(int id) 
    : Robot(id, INSPECTION), inspection_state_(PLANNING_PATH),
      current_waypoint_index_(0), inspection_complete_(false), next_damage_id_(1) {
    generateInspectionPath();
}

void InspectionRobot::executeTask() {
    processInspectionState();
    updateBatteryLevel(0.5); // Camera and processing usage
}

void InspectionRobot::processStateTransition() {
    switch (inspection_state_) {
        case PLANNING_PATH:
            generateInspectionPath();
            inspection_state_ = MOVING_TO_WAYPOINT;
            break;
            
        case MOVING_TO_WAYPOINT:
            if (reachedTarget(inspection_waypoints_[current_waypoint_index_])) {
                inspection_state_ = SCANNING_AREA;
            }
            break;
            
        case SCANNING_AREA:
            scanArea360Degrees();
            inspection_state_ = ANALYZING_DAMAGE;
            break;
            
        case ANALYZING_DAMAGE:
            if (detectDamage()) {
                inspection_state_ = RECORDING_DAMAGE;
            } else {
                moveToNextWaypoint();
            }
            break;
            
        case RECORDING_DAMAGE:
            recordCurrentDamage();
            moveToNextWaypoint();
            break;
            
        case INSPECTION_COMPLETE:
            saveDataToDisk();
            returnToDockingStation();
            break;
    }
}

bool InspectionRobot::isTaskComplete() { 
    return inspection_complete_ && current_waypoint_index_ >= inspection_waypoints_.size(); 
}

void InspectionRobot::saveDataToDisk() {
    std::ofstream file("warehouse_damage_report.txt");
    file << "=== WAREHOUSE DAMAGE INSPECTION REPORT ===" << std::endl;
    file << "Robot ID: " << robot_id_ << std::endl;
    file << "Total Damage Sites Found: " << damage_reports_.size() << std::endl;
    file << "Inspection Status: " << (inspection_complete_ ? "COMPLETE" : "IN PROGRESS") << std::endl;
    file << std::endl;
    
    for (const auto& damage : damage_reports_) {
        file << "Damage ID: " << damage.damage_id << std::endl;
        file << "Location: (" << damage.location.x << ", " << damage.location.y << ")" << std::endl;
        file << "Description: " << damage.description << std::endl;
        file << "Severity Score: " << damage.severity_score << "/10" << std::endl;
        file << "Timestamp: " << damage.timestamp << std::endl;
        file << "Image File: " << damage.image_path << std::endl;
        file << "-------------------------------------------" << std::endl;
    }
    file.close();
}

std::string InspectionRobot::getStatusReport() {
    return "InspectionRobot " + std::to_string(robot_id_) + 
           " - Waypoint: " + std::to_string(current_waypoint_index_) + 
           "/" + std::to_string(inspection_waypoints_.size()) +
           " - Damage Found: " + std::to_string(damage_reports_.size()) +
           " - Battery: " + std::to_string(battery_level_) + "%" +
           " - State: " + getInspectionStateString();
}

void InspectionRobot::startFullAreaInspection() { 
    inspection_state_ = PLANNING_PATH; 
    current_waypoint_index_ = 0;
    inspection_complete_ = false;
    damage_reports_.clear();
    generateInspectionPath();
}

void InspectionRobot::navigateToSpecificDamage(int damage_id) {
    for (const auto& damage : damage_reports_) {
        if (damage.damage_id == damage_id) {
            moveToLocation(damage.location);
            break;
        }
    }
}

std::vector<DamageReport> InspectionRobot::getDamageReports() const { 
    return damage_reports_; 
}

// Private methods
void InspectionRobot::processInspectionState() {
    switch (inspection_state_) {
        case MOVING_TO_WAYPOINT:
            moveToLocation(inspection_waypoints_[current_waypoint_index_]);
            break;
        case SCANNING_AREA:
            scanArea360Degrees();
            break;
        case ANALYZING_DAMAGE:
            if (detectDamage()) recordCurrentDamage();
            break;
        default:
            break;
    }
}

void InspectionRobot::generateInspectionPath() {
    // Create systematic grid pattern to cover entire warehouse space
    inspection_waypoints_.clear();
    
    // Grid pattern covering warehouse area
    for (double x = -5.0; x <= 5.0; x += 1.5) {
        for (double y = -5.0; y <= 5.0; y += 1.5) {
            inspection_waypoints_.push_back(Pose(x, y, 0.0));
        }
    }
    
    // Add perimeter inspection for wall damage
    std::vector<Pose> perimeter = {
        Pose(-5.0, -5.0, 0.0), Pose(5.0, -5.0, 0.0),   // Bottom wall
        Pose(5.0, 5.0, 1.57), Pose(-5.0, 5.0, 1.57),  // Top wall
    };
    inspection_waypoints_.insert(inspection_waypoints_.end(), perimeter.begin(), perimeter.end());
}

void InspectionRobot::moveToNextWaypoint() {
    current_waypoint_index_++;
    if (current_waypoint_index_ >= inspection_waypoints_.size()) {
        inspection_state_ = INSPECTION_COMPLETE;
        inspection_complete_ = true;
    } else {
        inspection_state_ = MOVING_TO_WAYPOINT;
    }
}

bool InspectionRobot::detectDamage() {
    // CAMERA-BASED DAMAGE DETECTION
    // Uses computer vision or simplified marker detection
    return analyzeCameraImage();
}

bool InspectionRobot::analyzeCameraImage() {
    // Pseudocode for image analysis using camera
    // Method 1: Use OpenCV for crack detection
    // Method 2: Use color markers (red = damage)
    // Method 3: Use ArUco markers for simplified detection
    
    // Simplified implementation - look for specific damage markers
    // cv::Mat image = camera.capture();
    // cv::Mat hsv;
    // cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    // 
    // // Look for red markers indicating damage
    // cv::Scalar lower_red(0, 50, 50);
    // cv::Scalar upper_red(10, 255, 255);
    // cv::Mat mask;
    // cv::inRange(hsv, lower_red, upper_red, mask);
    // 
    // std::vector<std::vector<cv::Point>> contours;
    // cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // 
    // return contours.size() > 0; // Damage found if red markers detected
    
    // For demonstration - randomly detect damage
    return (rand() % 10 == 0); // 10% chance of finding damage
}

void InspectionRobot::scanArea360Degrees() {
    // Rotate robot 360 degrees while taking images at different angles
    for (int angle = 0; angle < 360; angle += 45) {
        publishVelocityCommand(0.0, 0.1); // Rotate slowly
        captureImage("scan_" + std::to_string(current_waypoint_index_) + "_" + std::to_string(angle) + ".jpg");
        // Small delay for image processing
    }
    stopMotors();
}

void InspectionRobot::captureImage(const std::string& filename) {
    // Pseudocode: Capture image from turtlebot camera
    // sensor_msgs::Image::ConstPtr image_msg = camera_subscriber_.getLatestMessage();
    // cv::Mat image = cv_bridge::toCvShare(image_msg, "bgr8")->image;
    // cv::imwrite(filename, image);
}

void InspectionRobot::recordCurrentDamage() {
    DamageReport damage;
    damage.damage_id = next_damage_id_++;
    damage.location = current_pose_;
    damage.description = "Structural damage detected by camera analysis";
    damage.timestamp = getCurrentTimestamp();
    damage.image_path = "damage_" + std::to_string(damage.damage_id) + ".jpg";
    damage.severity_score = calculateDamageSeverity();
    
    damage_reports_.push_back(damage);
    captureImage(damage.image_path);
    
    std::cout << "DAMAGE DETECTED: ID " << damage.damage_id 
              << " at (" << damage.location.x << ", " << damage.location.y << ")" << std::endl;
}

std::string InspectionRobot::getCurrentTimestamp() {
    // Return current timestamp
    return "2025-10-20T12:00:00Z"; // Simplified
}

double InspectionRobot::calculateDamageSeverity() {
    // Analyze image to determine damage severity (1-10 scale)
    // Based on size, color intensity, crack patterns, etc.
    return 3.0 + (rand() % 5); // Random severity 3-7
}

std::string InspectionRobot::getInspectionStateString() {
    switch (inspection_state_) {
        case PLANNING_PATH: return "PLANNING";
        case MOVING_TO_WAYPOINT: return "MOVING";
        case SCANNING_AREA: return "SCANNING";
        case ANALYZING_DAMAGE: return "ANALYZING";
        case RECORDING_DAMAGE: return "RECORDING";
        case INSPECTION_COMPLETE: return "COMPLETE";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// DELIVERY ROBOT IMPLEMENTATION
// ============================================================================
DeliveryRobot::DeliveryRobot(int id) 
    : Robot(id, DELIVERY), delivery_state_(PLANNING_ROUTE),
      current_delivery_index_(0), package_loaded_(false) {
    initializeDeliveryRoutes();
}

void DeliveryRobot::executeTask() {
    processDeliveryState();
    updateBatteryLevel(0.8); // Higher usage due to package payload
}

void DeliveryRobot::processStateTransition() {
    switch (delivery_state_) {
        case PLANNING_ROUTE:
            optimizeDeliveryOrder();
            if (!delivery_queue_.empty()) {
                delivery_state_ = MOVING_TO_PICKUP;
            }
            break;
            
        case MOVING_TO_PICKUP:
            if (reachedTarget(getCurrentDelivery().pickup_location)) {
                delivery_state_ = LOADING_PACKAGE;
            }
            break;
            
        case LOADING_PACKAGE:
            simulatePackagePickup();
            delivery_state_ = MOVING_TO_DELIVERY;
            break;
            
        case MOVING_TO_DELIVERY:
            if (reachedTarget(getCurrentDelivery().delivery_location)) {
                delivery_state_ = UNLOADING_PACKAGE;
            }
            break;
            
        case UNLOADING_PACKAGE:
            simulatePackageDelivery();
            markCurrentDeliveryComplete();
            if (hasMoreDeliveries()) {
                current_delivery_index_++;
                delivery_state_ = MOVING_TO_PICKUP;
            } else {
                delivery_state_ = DELIVERY_COMPLETE;
            }
            break;
            
        case DELIVERY_COMPLETE:
            saveDataToDisk();
            returnToDockingStation();
            break;
    }
}

bool DeliveryRobot::isTaskComplete() { 
    return delivery_state_ == DELIVERY_COMPLETE && delivery_queue_.empty(); 
}

void DeliveryRobot::saveDataToDisk() {
    std::ofstream file("warehouse_delivery_log.txt");
    file << "=== WAREHOUSE DELIVERY LOG ===" << std::endl;
    file << "Robot ID: " << robot_id_ << std::endl;
    file << "Total Deliveries Completed: " << completed_deliveries_.size() << std::endl;
    file << "Deliveries Pending: " << delivery_queue_.size() << std::endl;
    file << std::endl;
    
    file << "COMPLETED DELIVERIES:" << std::endl;
    for (const auto& delivery : completed_deliveries_) {
        file << "Delivery ID: " << delivery.delivery_id << std::endl;
        file << "Package: " << delivery.package_description << std::endl;
        file << "Route: (" << delivery.pickup_location.x << ", " << delivery.pickup_location.y 
             << ") -> (" << delivery.delivery_location.x << ", " << delivery.delivery_location.y << ")" << std::endl;
        file << "Priority Level: " << delivery.priority_level << std::endl;
        file << "Completed: " << delivery.timestamp << std::endl;
        file << "-------------------------------------------" << std::endl;
    }
    
    if (!delivery_queue_.empty()) {
        file << std::endl << "PENDING DELIVERIES:" << std::endl;
        for (const auto& delivery : delivery_queue_) {
            file << "ID: " << delivery.delivery_id << " - " << delivery.package_description 
                 << " (Priority: " << delivery.priority_level << ")" << std::endl;
        }
    }
    file.close();
}

std::string DeliveryRobot::getStatusReport() {
    return "DeliveryRobot " + std::to_string(robot_id_) + 
           " - Delivery: " + std::to_string(current_delivery_index_) + 
           "/" + std::to_string(delivery_queue_.size()) +
           " - Package Loaded: " + (package_loaded_ ? "Yes" : "No") +
           " - Battery: " + std::to_string(battery_level_) + "%" +
           " - State: " + getDeliveryStateString();
}

void DeliveryRobot::addDeliveryRequest(const DeliveryRequest& request) {
    delivery_queue_.push_back(request);
}

void DeliveryRobot::startDeliveryRoute() {
    if (!delivery_queue_.empty()) {
        delivery_state_ = PLANNING_ROUTE;
        current_delivery_index_ = 0;
    }
}

std::vector<DeliveryRequest> DeliveryRobot::getPendingDeliveries() const { 
    return delivery_queue_; 
}

std::vector<DeliveryRequest> DeliveryRobot::getCompletedDeliveries() const { 
    return completed_deliveries_; 
}

// Private methods
void DeliveryRobot::processDeliveryState() {
    switch (delivery_state_) {
        case MOVING_TO_PICKUP:
            moveToLocation(getCurrentDelivery().pickup_location);
            break;
        case MOVING_TO_DELIVERY:
            moveToLocation(getCurrentDelivery().delivery_location);
            break;
        default:
            break;
    }
}

void DeliveryRobot::initializeDeliveryRoutes() {
    // Define common delivery routes in warehouse for efficiency
    
    // Route 1: Main corridor route
    std::vector<Pose> main_route = {
        Pose(0.0, 0.0, 0.0),    // Depot
        Pose(2.0, 0.0, 0.0),    // Checkpoint A
        Pose(4.0, 0.0, 0.0),    // Checkpoint B
        Pose(4.0, 2.0, 1.57),   // Turn point
        Pose(4.0, 4.0, 1.57),   // Storage Area 1
    };
    
    // Route 2: Side corridor route  
    std::vector<Pose> side_route = {
        Pose(0.0, 0.0, 0.0),    // Depot
        Pose(-2.0, 0.0, 3.14),  // West corridor
        Pose(-2.0, -3.0, -1.57), // South section
        Pose(-4.0, -3.0, 3.14), // Storage Area 2
    };
    
    delivery_routes_.push_back(main_route);
    delivery_routes_.push_back(side_route);
}

void DeliveryRobot::optimizeDeliveryOrder() {
    // INTELLIGENT DELIVERY OPTIMIZATION
    // Sort deliveries by priority first, then by location efficiency
    std::sort(delivery_queue_.begin(), delivery_queue_.end(),
             [this](const DeliveryRequest& a, const DeliveryRequest& b) {
                 // Higher priority first
                 if (a.priority_level != b.priority_level) {
                     return a.priority_level > b.priority_level;
                 }
                 // If same priority, choose closer pickup location (TSP heuristic)
                 double dist_a = calculateDistance(current_pose_, a.pickup_location);
                 double dist_b = calculateDistance(current_pose_, b.pickup_location);
                 return dist_a < dist_b;
             });
}

DeliveryRequest& DeliveryRobot::getCurrentDelivery() { 
    return delivery_queue_[current_delivery_index_]; 
}

bool DeliveryRobot::hasMoreDeliveries() {
    return current_delivery_index_ < delivery_queue_.size() - 1;
}

void DeliveryRobot::simulatePackagePickup() {
    // Simulate time and process for package loading
    package_loaded_ = true;
    // In real implementation: 
    // - Extend robotic arm
    // - Grip package using sensors
    // - Secure package in storage compartment
    // - Verify package is properly loaded
    std::cout << "Package loaded for delivery " << getCurrentDelivery().delivery_id << std::endl;
}

void DeliveryRobot::simulatePackageDelivery() {
    // Simulate time and process for package unloading
    package_loaded_ = false;
    // In real implementation:
    // - Navigate to precise delivery location
    // - Extend arm to delivery position
    // - Release package
    // - Confirm delivery completion
    // - Retract arm
    std::cout << "Package delivered for delivery " << getCurrentDelivery().delivery_id << std::endl;
}

void DeliveryRobot::markCurrentDeliveryComplete() {
    DeliveryRequest completed = getCurrentDelivery();
    completed.completed = true;
    completed.timestamp = getCurrentTimestamp();
    completed_deliveries_.push_back(completed);
    
    // Remove from queue
    delivery_queue_.erase(delivery_queue_.begin() + current_delivery_index_);
    if (current_delivery_index_ > 0) current_delivery_index_--;
}

std::string DeliveryRobot::getCurrentTimestamp() {
    return "2025-10-20T12:00:00Z"; // Simplified
}

std::string DeliveryRobot::getDeliveryStateString() {
    switch (delivery_state_) {
        case PLANNING_ROUTE: return "PLANNING";
        case MOVING_TO_PICKUP: return "TO_PICKUP";
        case LOADING_PACKAGE: return "LOADING";
        case MOVING_TO_DELIVERY: return "TO_DELIVERY";
        case UNLOADING_PACKAGE: return "UNLOADING";
        case DELIVERY_COMPLETE: return "COMPLETE";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// FACTORY MANAGER IMPLEMENTATION
// ============================================================================
FactoryManager::FactoryManager() : next_robot_id_(1) {
    initializeChargingStations();
}

FactoryManager::~FactoryManager() {
    // Cleanup - send all robots to dock before shutdown
    for (auto& pair : active_robots_) {
        pair.second->returnToDockingStation();
    }
    active_robots_.clear();
}

std::shared_ptr<Robot> FactoryManager::createRobot(RobotType type, int robot_id) {
    if (robot_id == -1) robot_id = next_robot_id_++;
    
    std::shared_ptr<Robot> robot;
    switch (type) {
        case INSPECTION: 
            robot = std::make_shared<InspectionRobot>(robot_id); 
            std::cout << "Created InspectionRobot with ID " << robot_id << std::endl;
            break;
        case DELIVERY: 
            robot = std::make_shared<DeliveryRobot>(robot_id); 
            std::cout << "Created DeliveryRobot with ID " << robot_id << std::endl;
            break;
        default:
            std::cout << "Error: Unknown robot type" << std::endl;
            return nullptr;
    }
    
    if (robot) {
        active_robots_[robot_id] = robot;
    }
    return robot;
}

void FactoryManager::destroyRobot(int robot_id) {
    auto it = active_robots_.find(robot_id);
    if (it != active_robots_.end()) {
        std::cout << "Destroying robot " << robot_id << " - sending to dock" << std::endl;
        it->second->returnToDockingStation();
        active_robots_.erase(it);
    }
}

std::shared_ptr<Robot> FactoryManager::getRobot(int robot_id) {
    auto it = active_robots_.find(robot_id);
    return (it != active_robots_.end()) ? it->second : nullptr;
}

void FactoryManager::assignInspectionTask(int robot_id) {
    auto robot = getRobot(robot_id);
    if (robot && robot->getRobotType() == INSPECTION) {
        // Safe downcast to access specialized functionality
        auto inspection_robot = std::dynamic_pointer_cast<InspectionRobot>(robot);
        if (inspection_robot) {
            inspection_robot->startFullAreaInspection();
            std::cout << "Assigned inspection task to robot " << robot_id << std::endl;
        }
    } else {
        std::cout << "Error: Robot " << robot_id << " is not an inspection robot" << std::endl;
    }
}

void FactoryManager::assignDeliveryTask(int robot_id, const DeliveryRequest& request) {
    auto robot = getRobot(robot_id);
    if (robot && robot->getRobotType() == DELIVERY) {
        // Safe downcast to access specialized functionality
        auto delivery_robot = std::dynamic_pointer_cast<DeliveryRobot>(robot);
        if (delivery_robot) {
            delivery_robot->addDeliveryRequest(request);
            delivery_robot->startDeliveryRoute();
            std::cout << "Assigned delivery task " << request.delivery_id 
                      << " to robot " << robot_id << std::endl;
        }
    } else {
        std::cout << "Error: Robot " << robot_id << " is not a delivery robot" << std::endl;
    }
}

void FactoryManager::updateAllRobots() {
    for (auto& pair : active_robots_) {
        auto robot = pair.second;
        
        // POLYMORPHIC CALLS - each robot type implements these differently
        robot->executeTask();           // Virtual function call
        robot->processStateTransition(); // Virtual function call
        
        // Handle low battery universally
        if (robot->needsCharging()) {
            sendRobotToCharging(robot->getRobotId());
        }
    }
}

void FactoryManager::sendRobotToCharging(int robot_id) {
    auto robot = getRobot(robot_id);
    if (robot) {
        std::cout << "Sending robot " << robot_id << " to charging station" << std::endl;
        robot->returnToDockingStation();
        assignChargingStation(robot_id);
    }
}

void FactoryManager::emergencyStopAllRobots() {
    std::cout << "EMERGENCY STOP - All robots returning to dock" << std::endl;
    for (auto& pair : active_robots_) {
        pair.second->returnToDockingStation();
    }
}

void FactoryManager::printSystemStatus() {
    std::cout << "=== WAREHOUSE ROBOT SYSTEM STATUS ===" << std::endl;
    std::cout << "Active Robots: " << active_robots_.size() << std::endl;
    std::cout << "Charging Stations: " << charging_stations_.size() << std::endl;
    std::cout << std::endl;
    
    for (const auto& pair : active_robots_) {
        auto robot = pair.second;
        // POLYMORPHIC CALL - each robot provides different status info
        std::cout << robot->getStatusReport() << std::endl;
    }
    std::cout << "=====================================" << std::endl;
}

bool FactoryManager::changeRobotType(int robot_id, RobotType new_type) {
    auto old_robot = getRobot(robot_id);
    if (!old_robot) {
        std::cout << "Error: Robot " << robot_id << " not found" << std::endl;
        return false;
    }
    
    std::cout << "Changing robot " << robot_id << " type..." << std::endl;
    
    // Save current position for reference
    Pose current_pos = old_robot->getCurrentPose();
    
    // Send to docking station first (required for type change)
    old_robot->returnToDockingStation();
    
    // Wait for robot to reach dock (in real implementation, would check status)
    // while (old_robot->getCurrentState() != IDLE) { sleep(100ms); }
    
    // Destroy old robot instance
    destroyRobot(robot_id);
    
    // Create new robot of different type at same ID
    auto new_robot = createRobot(new_type, robot_id);
    if (new_robot) {
        std::cout << "Successfully changed robot " << robot_id << " to new type" << std::endl;
        return true;
    }
    
    std::cout << "Failed to create new robot instance" << std::endl;
    return false;
}

int FactoryManager::getActiveRobotCount() const { 
    return active_robots_.size(); 
}

std::vector<int> FactoryManager::getActiveRobotIds() const {
    std::vector<int> ids;
    for (const auto& pair : active_robots_) {
        ids.push_back(pair.first);
    }
    return ids;
}

void FactoryManager::initializeChargingStations() {
    // Define multiple charging station locations
    charging_stations_.push_back(Pose(0.0, 0.0, 0.0));     // Primary station
    charging_stations_.push_back(Pose(-1.0, 0.0, 0.0));    // Secondary station  
    charging_stations_.push_back(Pose(1.0, 0.0, 0.0));     // Backup station
}

void FactoryManager::assignChargingStation(int robot_id) {
    // Simple assignment strategy - round robin
    int station_index = robot_id % charging_stations_.size();
    // In real implementation: check station availability, queue management
    std::cout << "Robot " << robot_id << " assigned to charging station " << station_index << std::endl;
}

// ============================================================================
// WAREHOUSE ROBOT SYSTEM IMPLEMENTATION
// ============================================================================
WarehouseRobotSystem::WarehouseRobotSystem() : system_running_(false) {
    factory_manager_ = std::make_unique<FactoryManager>();
}

WarehouseRobotSystem::~WarehouseRobotSystem() {
    shutdown();
}

void WarehouseRobotSystem::initialize() {
    std::cout << "Initializing Warehouse Robot System..." << std::endl;
    
    // Initialize ROS nodes (pseudocode)
    initializeROS();
    
    // Create initial robot fleet
    auto inspection_robot = factory_manager_->createRobot(INSPECTION);
    auto delivery_robot = factory_manager_->createRobot(DELIVERY);
    
    system_running_ = true;
    std::cout << "System initialized successfully!" << std::endl;
    std::cout << "Ready to accept inspection and delivery requests." << std::endl;
}

void WarehouseRobotSystem::run() {
    std::cout << "Starting main system control loop..." << std::endl;
    
    while (system_running_) {
        // POLYMORPHIC SYSTEM UPDATE
        factory_manager_->updateAllRobots(); // Calls virtual functions on all robots
        
        // Process user commands and requests
        processUserCommands();
        
        // System monitoring and status updates
        publishRobotStates();
        
        // Control loop delay (in real system)
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void WarehouseRobotSystem::shutdown() {
    std::cout << "Shutting down warehouse robot system..." << std::endl;
    system_running_ = false;
    factory_manager_->emergencyStopAllRobots();
    factory_manager_.reset(); // Clean shutdown
    std::cout << "System shutdown complete." << std::endl;
}

void WarehouseRobotSystem::requestDamageInspection() {
    std::cout << "Processing damage inspection request..." << std::endl;
    
    // Find available inspection robot or create one
    auto robot_ids = factory_manager_->getActiveRobotIds();
    for (int id : robot_ids) {
        auto robot = factory_manager_->getRobot(id);
        if (robot->getRobotType() == INSPECTION && robot->getCurrentState() == IDLE) {
            factory_manager_->assignInspectionTask(id);
            return;
        }
    }
    
    // No available inspection robot, create new one
    auto new_robot = factory_manager_->createRobot(INSPECTION);
    if (new_robot) {
        factory_manager_->assignInspectionTask(new_robot->getRobotId());
    }
}

void WarehouseRobotSystem::requestPackageDelivery(const DeliveryRequest& request) {
    std::cout << "Processing delivery request for: " << request.package_description << std::endl;
    
    // Find available delivery robot
    auto robot_ids = factory_manager_->getActiveRobotIds();
    for (int id : robot_ids) {
        auto robot = factory_manager_->getRobot(id);
        if (robot->getRobotType() == DELIVERY && robot->getCurrentState() == IDLE) {
            factory_manager_->assignDeliveryTask(id, request);
            return;
        }
    }
    
    // No available delivery robot, create new one
    auto new_robot = factory_manager_->createRobot(DELIVERY);
    if (new_robot) {
        factory_manager_->assignDeliveryTask(new_robot->getRobotId(), request);
    }
}

void WarehouseRobotSystem::displaySystemStatus() {
    factory_manager_->printSystemStatus();
}

void WarehouseRobotSystem::requestRobotTypeChange(int robot_id, RobotType new_type) {
    std::cout << "Processing robot type change request..." << std::endl;
    factory_manager_->changeRobotType(robot_id, new_type);
}

void WarehouseRobotSystem::processUserCommands() {
    // Pseudocode for processing various user interface commands
    // - Web interface requests
    // - Command line input  
    // - ROS service calls
    // - Emergency stop signals
    // - Robot reconfiguration requests
}

void WarehouseRobotSystem::initializeROS() {
    // Pseudocode for ROS system initialization
    // rclcpp::init(argc, argv);
    // Create node handles for different components
    // Set up publishers for robot commands (/cmd_vel topics)
    // Set up subscribers for sensor data (/odom, /imu, /camera topics)
    // Create service servers for user requests
    // Initialize transform listeners for localization
}

void WarehouseRobotSystem::publishRobotStates() {
    // Pseudocode: Publish robot states to ROS topics
    // for visualization in RViz, web interfaces, or monitoring systems
    // Each robot's status, battery level, current task, etc.
}
