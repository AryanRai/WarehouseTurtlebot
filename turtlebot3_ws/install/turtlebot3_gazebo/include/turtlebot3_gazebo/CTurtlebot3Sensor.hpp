/*
 * File: CTurtlebot3Sensor.hpp
 * Description: Declares the CTurtlebot3Sensor class for managing sensor data subscriptions and processing in TurtleBot3 Gazebo simulation.
 * 
 * Date: 2025-10-10
 *
 * This header defines the interface for subscribing to laser scan and odometry topics,
 * processing sensor data, and providing access to sensor readings for other components.
 */

#ifndef TURTLEBOT3_GAZEBO__CTURTLEBOT3_SENSOR_HPP_
#define TURTLEBOT3_GAZEBO__CTURTLEBOT3_SENSOR_HPP_

#include <cmath>      
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

/**
 * @class CTurtlebot3Sensor
 * @brief Manages sensor data subscriptions and processing
 * 
 * Subscribes to laser scan and odometry topics, processes the data,
 * and provides clean interface for other components to access sensor readings.
 */
class CTurtlebot3Sensor
{
  public:
    /**
     * @enum eScanIndex
     * @brief Indices for processed laser scan sectors
     */
    enum eScanIndex : int {
      FORWARD      = 0,
      FRONT_RIGHT  = 1,
      RIGHT        = 2,
      FRONT_LEFT   = 3,
      LEFT         = 4,
      SCAN_COUNT   = 5  // Total number of scan sectors
    };

    /**
     * @brief Constructor
     * @param aNode Pointer to parent ROS2 node (not owned)
     */
    explicit CTurtlebot3Sensor(rclcpp::Node* aNode);
    
    /**
     * @brief Destructor
     */
    ~CTurtlebot3Sensor();

    /**
     * @brief Get processed laser scan distance for a sector
     * @param aIndex Sector index (see eScanIndex enum)
     * @return Minimum distance in sector (meters)
     */
    double get_scan_data(eScanIndex aIndex) const;
    
    /**
     * @brief Get robot X position from odometry
     * @return X position in world frame (meters)
     */
    double get_pose_x() const;
    
    /**
     * @brief Get robot Y position from odometry
     * @return Y position in world frame (meters)
     */
    double get_pose_y() const;
    
    /**
     * @brief Get robot yaw angle (wrapped to [-pi, pi])
     * @return Yaw angle (radians)
     */
    double get_robot_pose() const;
    
    /**
     * @brief Get unwrapped yaw angle (can exceed [-pi, pi])
     * @return Cumulative yaw angle (radians)
     */
    double get_yaw_unwrapped() const;

  private:
    // Mathematical constants
    static constexpr double DEG_TO_RAD = M_PI / 180.0;
    static constexpr double RAD_TO_DEG = 180.0 / M_PI;
    
    // Sensor processing parameters
    static constexpr double SECTOR_HALF_ANGLE_DEG = 10.0;
    static constexpr double MIN_VALID_RANGE = 0.20;  // Clamp readings below this (meters)
    static constexpr double MAX_YAW_DELTA = 0.8;     // Reject yaw deltas larger than this (radians)

    // ROS node pointer (not owned by this class)
    rclcpp::Node* node_;

    // ROS subscriptions
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Processed sensor data
    double scan_data_[SCAN_COUNT];
    double pose_x_;
    double pose_y_;
    double robot_pose_;
    
    // Yaw unwrapping state
    double prev_yaw_;
    double yaw_unwrapped_;
    bool first_odom_;

    /**
     * @brief Callback for laser scan messages
     * @param msg Laser scan message
     */
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr aMsg);
    
    /**
     * @brief Callback for odometry messages
     * @param msg Odometry message
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr aMsg);

    /**
     * @brief Wrap angle to [-pi, pi] range
     * @param aAngle Input angle (radians)
     * @return Wrapped angle (radians)
     */
    double wrap_to_pi(double aAngle) const;
};

#endif  // TURTLEBOT3_GAZEBO__CTURTLEBOT3_SENSOR_HPP_