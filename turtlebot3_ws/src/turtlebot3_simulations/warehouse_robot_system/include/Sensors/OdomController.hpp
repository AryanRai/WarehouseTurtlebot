// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: OdomController.hpp
// Author(s): Inez Dumas
//
// Description: Simple abstraction class for odometry sensor management.
//              Subscribes to odometry data and republishes it for other
//              components to use, avoiding duplicate subscriptions.

#ifndef ODOM_CONTROLLER_HPP
#define ODOM_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

/**
 * @brief Simple odometry republisher node
 * 
 * Subscribes to odometry data and republishes it to avoid
 * duplicate subscriptions across multiple components.
 */
class OdomController : public rclcpp::Node
{
public:
    /**
     * @brief Constructor
     */
    OdomController();
    
    /**
     * @brief Destructor
     */
    ~OdomController() = default;
    
private:
    // Subscriber for raw odometry data
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // Publisher for republished odometry data
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    
    /**
     * @brief Odometry callback - simply republishes the data
     * @param msg Odometry message
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif // ODOM_CONTROLLER_HPP