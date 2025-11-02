// ============================================================================
// MTRX3760 Project 2 - 
// File: WarehouseRobotBase.hpp
// Description: Base class for warehouse robots (Delivery & Inspection).
//              Contains common functionality including SLAM integration,
//              TSP route optimization, docking behavior, and navigation.
//              Inherits from rclcpp::Node for direct ROS2 integration.
// Author(s): Dylan George
// Last Edited: 2025-11-02
// ============================================================================

#ifndef WAREHOUSE_ROBOT_BASE_HPP
#define WAREHOUSE_ROBOT_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include "SLAM/SlamController.hpp"
#include "SLAM/MotionController.hpp"
#include "SLAM/PathPlanner.hpp"
#include <memory>
#include <vector>
#include <chrono>

/**
 * @brief Base class for warehouse robots with common navigation and optimization functionality
 * 
 * Inherits from rclcpp::Node to provide ROS2 node functionality.
 * Provides shared capabilities for delivery and inspection robots including:
 * - SLAM controller and motion controller integration
 * - TSP-based route optimization using simulated annealing
 * - Precise docking behavior for home position
 * - Line-of-sight and obstacle checking
 * - Status publishing and logging
 */
class Robot : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     * @param node_name Name of the ROS2 node
     */
    explicit Robot(const std::string& node_name);
    
    /**
     * @brief Virtual destructor for proper cleanup of derived classes
     */
    virtual ~Robot() = default;
    
    /**
     * @brief Initialize SLAM and motion controllers
     * Must be called after construction (cannot use shared_from_this in constructor)
     */
    void initialize();
    
    /**
     * @brief Main update loop - must be implemented by derived classes
     */
    virtual void update() = 0;
    
    /**
     * @brief Check if robot has a valid map
     * @return True if valid map available
     */
    bool hasValidMap() const;
    
protected:
    // ========================================================================
    // ROS and SLAM Components
    // ========================================================================
    std::unique_ptr<SlamController> slam_controller_;
    std::unique_ptr<MotionController> motion_controller_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    // ========================================================================
    // Common State Variables
    // ========================================================================
    bool in_docking_mode_;
    double initial_yaw_;
    bool has_relocalized_;
    rclcpp::Time relocalization_start_time_;
    bool use_tsp_optimization_;
    
    // Battery monitoring
    float battery_level_;
    double battery_low_threshold_;
    bool battery_monitoring_enabled_;
    bool low_battery_return_triggered_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    
    // ========================================================================
    // Common Constants
    // ========================================================================
    static constexpr double DOCKING_DISTANCE = 0.5;       // Enter docking mode within 50cm
    static constexpr double HOME_TOLERANCE = 0.05;        // Success within 5cm
    static constexpr double DOCKING_LINEAR_SPEED = 0.05;  // Slow speed for docking
    static constexpr double DOCKING_ANGULAR_SPEED = 0.3;  // Moderate rotation for alignment
    static constexpr double RELOCALIZATION_DURATION = 8.0; // 8 seconds for 2 full rotations
    static constexpr double RELOCALIZATION_SPEED = 1.57;  // rad/s (~90°/s)
    static constexpr double UPDATE_RATE = 10.0;           // Hz
    
    // ========================================================================
    // TSP Route Optimization
    // ========================================================================
    
    /**
     * @brief Build distance matrix for TSP optimization
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
    
    // ========================================================================
    // Navigation and Docking
    // ========================================================================
    
    /**
     * @brief Return robot to home position
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
    
    // ========================================================================
    // Battery Monitoring
    // ========================================================================
    
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
     */
    virtual void emergencyReturnHome();
    
    // ========================================================================
    // Utility Methods
    // ========================================================================
    
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
};

#endif // WAREHOUSE_ROBOT_BASE_HPP
