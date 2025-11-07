// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: LidarController.hpp
// Author(s): Inez Dumas
//
// Description: Simple abstraction class for LiDAR sensor management.
//              Subscribes to laser scan data and republishes it for other
//              components to use, avoiding duplicate subscriptions.

#ifndef LIDAR_CONTROLLER_HPP
#define LIDAR_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

/**
 * @brief Simple LiDAR republisher node
 * 
 * Subscribes to laser scan data and republishes it to avoid
 * duplicate subscriptions across multiple components.
 */
class LidarController : public rclcpp::Node
{
public:
    /**
     * @brief Constructor
     */
    LidarController();
    
    /**
     * @brief Destructor
     */
    ~LidarController() = default;
    
private:
    // Subscriber for raw laser scan data
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    
    // Publisher for republished laser scan data
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
    
    /**
     * @brief Laser scan callback - simply republishes the data
     * @param msg Laser scan message
     */
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

#endif // LIDAR_CONTROLLER_HPP