// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: WarehouseRobot.hpp
// Author(s): Dylan George, Aryan Rai, Inez Dumas
//
// Description: Header for WarehouseRobot base class. Provides common
//              functionality for delivery and inspection robots including
//              TSP optimization, docking, relocalization, and navigation utilities.

#ifndef WAREHOUSE_ROBOT_HPP
#define WAREHOUSE_ROBOT_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include "SLAM/SlamController.hpp"
#include "SLAM/MotionController.hpp"
#include "SLAM/PathPlanner.hpp"
#include <memory>
#include <vector>
#include <chrono>
#include <string>

/**
 * @brief Robot type enumeration for polymorphic identification
 */
enum class RobotType {
    INSPECTION,
    DELIVERY
};

/**
 * @brief Base class for warehouse robots with common navigation and optimization functionality
 * 
 * Provides shared capabilities for delivery and inspection robots including:
 * - SLAM controller and motion controller integration
 * - TSP-based route optimization using simulated annealing
 * - Precise docking behavior for home position and targets
 * - Relocalization with 360° spin
 * - Line-of-sight and obstacle checking
 * - Status publishing and logging
 * - Battery monitoring with emergency return
 */
class WarehouseRobot {
public:
    /**
     * @brief Constructor
     * @param node Shared pointer to ROS2 node
     */
    explicit WarehouseRobot(rclcpp::Node::SharedPtr node);
    
    /**
     * @brief Virtual destructor for proper cleanup of derived classes
     */
    virtual ~WarehouseRobot() = default;
    
    /**
     * @brief Main update loop - must be implemented by derived classes
     */
    virtual void update() = 0;
    
    /**
     * @brief Get robot type for identification
     * @return Robot type enumeration
     */
    virtual RobotType getType() const = 0;
    
    /**
     * @brief Start robot-specific operations (inspection/delivery)
     */
    virtual void startOperations() = 0;
    
    /**
     * @brief Stop robot-specific operations
     */
    virtual void stopOperations() = 0;
    
    /**
     * @brief Check if robot is currently performing operations
     * @return True if operations in progress
     */
    virtual bool isOperating() const = 0;
    
    /**
     * @brief Check if robot has a valid map
     * @return True if valid map available
     */
    bool hasValidMap() const;
    
    /**
     * @brief Set route optimization mode
     * @param use_tsp True to use TSP optimization, false for ordered sequence
     */
    void setOptimizationMode(bool use_tsp) { use_tsp_optimization_ = use_tsp; }
    
    /**
     * @brief Check if using TSP optimization
     * @return True if TSP enabled
     */
    bool isUsingTSP() const { return use_tsp_optimization_; }

    /**
     * @brief Get ROS2 node
     * @return Shared pointer to ROS2 node
     */
    rclcpp::Node::SharedPtr getNode() const { return node_; }
    
protected:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<SlamController> slam_controller_;
    std::unique_ptr<MotionController> motion_controller_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    bool in_docking_mode_;
    bool in_target_docking_mode_;  // For zone/site docking
    double initial_yaw_;
    bool has_relocalized_;
    rclcpp::Time relocalization_start_time_;
    bool use_tsp_optimization_;
    bool path_completed_;  // Generic path completion tracking
    double total_distance_;
    
    // Battery monitoring
    float battery_level_;
    double battery_low_threshold_;
    bool battery_monitoring_enabled_;
    bool low_battery_return_triggered_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

    static constexpr double DOCKING_DISTANCE = 0.5;       // Enter docking mode within 50cm
    static constexpr double HOME_TOLERANCE = 0.05;        // Success within 5cm
    static constexpr double TARGET_TOLERANCE = 0.08;      // Success within 8cm for targets
    static constexpr double TARGET_REACHED_THRESHOLD = 0.3; // 30cm threshold
    static constexpr double DOCKING_LINEAR_SPEED = 0.05;  // Slow speed for docking
    static constexpr double DOCKING_ANGULAR_SPEED = 0.3;  // Moderate rotation for alignment
    static constexpr double RELOCALIZATION_DURATION = 8.0; // 8 seconds for 2 full rotations
    static constexpr double RELOCALIZATION_SPEED = 1.57;  // rad/s (~90°/s)
    static constexpr double UPDATE_RATE = 10.0;           // Hz
    
    
    /**
     * @brief Build distance matrix for TSP optimization using A*
     * @param start Starting position
     * @param points Vector of target points
     * @return 2D matrix of distances between all points
     */
    std::vector<std::vector<double>> buildDistanceMatrix(
        const geometry_msgs::msg::Point& start,
        const std::vector<geometry_msgs::msg::Point>& points);
    
    /**
     * @brief Optimize route using simulated annealing TSP solver
     * @param distance_matrix Pre-computed distance matrix
     * @param start_idx Index of starting position
     * @param initial_temp Initial temperature for annealing
     * @param cooling_rate Temperature reduction rate
     * @param max_iterations Maximum iterations
     * @return Optimized tour as vector of indices
     */
    std::vector<int> simulatedAnnealing(
        const std::vector<std::vector<double>>& distance_matrix,
        int start_idx,
        double initial_temp = 10000.0,
        double cooling_rate = 0.995,
        int max_iterations = 10000);
    
    /**
     * @brief Calculate total cost of a tour
     * @param tour Vector of indices representing the tour
     * @param distance_matrix Distance matrix
     * @return Total distance of tour
     */
    double calculateTourCost(
        const std::vector<int>& tour,
        const std::vector<std::vector<double>>& distance_matrix);
    
    
    /**
     * @brief Return robot to home position
     * Can be overridden by derived classes for custom behavior
     */
    virtual void returnToHome();
    
    /**
     * @brief Precise docking behavior for final approach to home
     * @param current_pose Current robot pose
     * @param distance_to_home Distance to home position
     */
    void preciseDocking(
        const geometry_msgs::msg::Pose& current_pose, 
        double distance_to_home);
    
    /**
     * @brief Precise docking behavior for final approach to target (zone/site)
     * @param current_pose Current robot pose
     * @param target_position Target position
     * @param distance_to_target Distance to target
     * @param target_name Name of target for logging
     */
    void preciseTargetDocking(
        const geometry_msgs::msg::Pose& current_pose,
        const geometry_msgs::msg::Point& target_position,
        double distance_to_target,
        const std::string& target_name);
    
    /**
     * @brief Perform relocalization spin at start
     * @return True if relocalization complete, false if still spinning
     */
    bool performRelocalization();
    
    /**
     * @brief Check line of sight between two points
     * @param from Starting point
     * @param to Ending point
     * @param map Occupancy grid map
     * @return True if clear line of sight exists
     */
    bool hasLineOfSight(
        const geometry_msgs::msg::Point& from,
        const geometry_msgs::msg::Point& to,
        const nav_msgs::msg::OccupancyGrid& map);
    
    /**
     * @brief Check minimum distance to walls from a position
     * @param position Position to check from
     * @param map Occupancy grid map
     * @return Minimum distance to nearest wall
     */
    double checkMinDistanceToWalls(
        const geometry_msgs::msg::Point& position,
        const nav_msgs::msg::OccupancyGrid& map);
    
    
    /**
     * @brief Initialize battery monitoring
     * @param threshold Battery percentage threshold for emergency return (-1 to disable)
     */
    void initializeBatteryMonitoring(double threshold);
    
    /**
     * @brief Battery state callback
     * @param msg Battery state message
     */
    void onBatteryState(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    
    /**
     * @brief Check if battery is low and trigger emergency return if needed
     * @return True if low battery return was triggered
     */
    bool checkLowBattery();
    
    /**
     * @brief Handle emergency low battery return to home
     * Can be overridden by derived classes
     */
    virtual void emergencyReturnHome();
    
    
    /**
     * @brief Publish status message
     * @param status Status string to publish
     */
    void publishStatus(const std::string& status);
    
    /**
     * @brief Get current timestamp as string
     * @return Formatted timestamp string
     */
    std::string getCurrentTimestamp();
    
    /**
     * @brief Calculate Euclidean distance between two points
     * @param p1 First point
     * @param p2 Second point
     * @return Distance between points
     */
    double calculateDistance(
        const geometry_msgs::msg::Point& p1,
        const geometry_msgs::msg::Point& p2) const;
    
    /**
     * @brief Extract yaw angle from quaternion
     * @param orientation Quaternion orientation
     * @return Yaw angle in radians
     */
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& orientation) const;
    
    /**
     * @brief Stop all robot motion
     */
    void stopMotion();
};

#endif // WAREHOUSE_ROBOT_HPP