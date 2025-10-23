// MTRX3760 2025 Project 2: Warehouse Robot 
// File: warehouse_robot_system.hpp
// Author(s): Aryan Rai
//
// Header file for polymorphic warehouse robot system with factory manager

#ifndef WAREHOUSE_ROBOT_SYSTEM_HPP
#define WAREHOUSE_ROBOT_SYSTEM_HPP

#include <memory>
#include <vector>
#include <string>
#include <map>
#include <fstream>
#include <iostream>
#include <algorithm>

#define MAX_BATTERY_LEVEL 100
#define LOW_BATTERY_THRESHOLD 20

// Forward declarations
class Robot;
class InspectionRobot;
class DeliveryRobot;
class FactoryManager;
class WarehouseRobotSystem;

// Data structures
struct Pose {
    double x, y, theta;
    Pose(double x_val = 0.0, double y_val = 0.0, double theta_val = 0.0);
};

struct DamageReport {
    int damage_id;
    Pose location;
    std::string description;
    std::string timestamp;
    std::string image_path;
    double severity_score;
};

struct DeliveryRequest {
    int delivery_id;
    Pose pickup_location;
    Pose delivery_location;
    std::string package_description;
    bool completed;
    std::string timestamp;
    int priority_level;
};

// Enumerations
enum RobotType { INSPECTION, DELIVERY };
enum RobotState { IDLE, MOVING_TO_TASK, EXECUTING_TASK, RETURNING_TO_DOCK, CHARGING, LOW_BATTERY, ERROR };
enum InspectionState { PLANNING_PATH, MOVING_TO_WAYPOINT, SCANNING_AREA, ANALYZING_DAMAGE, RECORDING_DAMAGE, INSPECTION_COMPLETE };
enum DeliveryState { PLANNING_ROUTE, MOVING_TO_PICKUP, LOADING_PACKAGE, MOVING_TO_DELIVERY, UNLOADING_PACKAGE, DELIVERY_COMPLETE };

// ============================================================================
// ABSTRACT BASE CLASS - IMPLEMENTS POLYMORPHISM FOR SHARED ROBOT FUNCTIONALITY
// ============================================================================
class Robot {
public:
    Robot(int id, RobotType type);
    virtual ~Robot() = default;

    // PURE VIRTUAL FUNCTIONS - POLYMORPHIC INTERFACE
    // Each robot type must implement these differently
    virtual void executeTask() = 0;
    virtual void processStateTransition() = 0;
    virtual bool isTaskComplete() = 0;
    virtual void saveDataToDisk() = 0;
    virtual std::string getStatusReport() = 0;

    // SHARED FUNCTIONALITY - Common to all robot types
    void moveToLocation(const Pose& target);
    void updateBatteryLevel(double usage_rate);
    bool needsCharging() const;
    void returnToDockingStation();
    void startCharging();
    void stopMotors();

    // Getters
    RobotState getCurrentState() const;
    int getBatteryLevel() const;
    int getRobotId() const;
    RobotType getRobotType() const;
    Pose getCurrentPose() const;

protected:
    int robot_id_;
    RobotType robot_type_;
    RobotState current_state_;
    Pose current_pose_;
    int battery_level_;
    bool is_charging_;

    // Shared movement functions
    void publishVelocityCommand(double linear_x, double angular_z);
    bool reachedTarget(const Pose& target, double tolerance = 0.1);
    void updateOdometry();
    double calculateDistance(const Pose& from, const Pose& to);
    double calculateAngle(const Pose& from, const Pose& to);
};

// ============================================================================
// INSPECTION ROBOT - SPECIALIZED FOR DAMAGE DETECTION USING CAMERA
// ============================================================================
class InspectionRobot : public Robot {
public:
    InspectionRobot(int id);

    // OVERRIDE POLYMORPHIC INTERFACE - Inspection-specific implementations
    void executeTask() override;
    void processStateTransition() override;
    bool isTaskComplete() override;
    void saveDataToDisk() override;
    std::string getStatusReport() override;

    // INSPECTION-SPECIFIC FUNCTIONALITY
    void startFullAreaInspection();
    void navigateToSpecificDamage(int damage_id);
    std::vector<DamageReport> getDamageReports() const;

private:
    // Inspection-specific state variables
    InspectionState inspection_state_;
    std::vector<Pose> inspection_waypoints_;
    std::vector<DamageReport> damage_reports_;
    int current_waypoint_index_;
    bool inspection_complete_;
    int next_damage_id_;

    // Private methods
    void processInspectionState();
    void generateInspectionPath();
    void moveToNextWaypoint();
    bool detectDamage();
    bool analyzeCameraImage();
    void scanArea360Degrees();
    void captureImage(const std::string& filename);
    void recordCurrentDamage();
    std::string getCurrentTimestamp();
    double calculateDamageSeverity();
    std::string getInspectionStateString();
};

// ============================================================================
// DELIVERY ROBOT - SPECIALIZED FOR PACKAGE DELIVERY (NO CAMERA, LIDAR ONLY)
// ============================================================================
class DeliveryRobot : public Robot {
public:
    DeliveryRobot(int id);

    // OVERRIDE POLYMORPHIC INTERFACE - Delivery-specific implementations
    void executeTask() override;
    void processStateTransition() override;
    bool isTaskComplete() override;
    void saveDataToDisk() override;
    std::string getStatusReport() override;

    // DELIVERY-SPECIFIC FUNCTIONALITY
    void addDeliveryRequest(const DeliveryRequest& request);
    void startDeliveryRoute();
    std::vector<DeliveryRequest> getPendingDeliveries() const;
    std::vector<DeliveryRequest> getCompletedDeliveries() const;

private:
    // Delivery-specific state variables
    DeliveryState delivery_state_;
    std::vector<DeliveryRequest> delivery_queue_;
    std::vector<DeliveryRequest> completed_deliveries_;
    int current_delivery_index_;
    bool package_loaded_;
    std::vector<std::vector<Pose>> delivery_routes_; // Pre-defined efficient routes

    // Private methods
    void processDeliveryState();
    void initializeDeliveryRoutes();
    void optimizeDeliveryOrder();
    DeliveryRequest& getCurrentDelivery();
    bool hasMoreDeliveries();
    void simulatePackagePickup();
    void simulatePackageDelivery();
    void markCurrentDeliveryComplete();
    std::string getCurrentTimestamp();
    std::string getDeliveryStateString();
};

// ============================================================================
// FACTORY MANAGER - USES POLYMORPHISM TO MANAGE DIFFERENT ROBOT TYPES
// ============================================================================
class FactoryManager {
public:
    FactoryManager();
    ~FactoryManager();

    // FACTORY PATTERN - Creates robots polymorphically
    std::shared_ptr<Robot> createRobot(RobotType type, int robot_id = -1);
    void destroyRobot(int robot_id);
    std::shared_ptr<Robot> getRobot(int robot_id);

    // POLYMORPHIC TASK ASSIGNMENT
    void assignInspectionTask(int robot_id);
    void assignDeliveryTask(int robot_id, const DeliveryRequest& request);

    // SYSTEM MANAGEMENT USING POLYMORPHISM
    void updateAllRobots();
    void sendRobotToCharging(int robot_id);
    void emergencyStopAllRobots();
    void printSystemStatus();

    // ROBOT TYPE SWITCHING - Demonstrates factory flexibility
    bool changeRobotType(int robot_id, RobotType new_type);

    // Getters
    int getActiveRobotCount() const;
    std::vector<int> getActiveRobotIds() const;

private:
    std::map<int, std::shared_ptr<Robot>> active_robots_;
    std::vector<Pose> charging_stations_;
    int next_robot_id_;

    void initializeChargingStations();
    void assignChargingStation(int robot_id);
};

// ============================================================================
// MAIN SYSTEM CLASS - ORCHESTRATES THE ENTIRE WAREHOUSE OPERATION
// ============================================================================
class WarehouseRobotSystem {
public:
    WarehouseRobotSystem();
    ~WarehouseRobotSystem();

    void initialize();
    void run();
    void shutdown();

    // USER INTERFACE FUNCTIONS
    void requestDamageInspection();
    void requestPackageDelivery(const DeliveryRequest& request);
    void displaySystemStatus();
    void requestRobotTypeChange(int robot_id, RobotType new_type);

private:
    std::unique_ptr<FactoryManager> factory_manager_;
    bool system_running_;

    void processUserCommands();
    void initializeROS();
    void publishRobotStates();
};

#endif // WAREHOUSE_ROBOT_SYSTEM_HPP