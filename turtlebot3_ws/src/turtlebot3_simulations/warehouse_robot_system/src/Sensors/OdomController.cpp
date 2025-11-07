// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: OdomController.cpp
// Author(s): Inez Dumas
//
// Description: Implementation of OdomController class

#include "Sensors/OdomController.hpp"

OdomController::OdomController()
    : Node("odom_controller")
{
    // Subscribe to raw odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        rclcpp::SensorDataQoS(),
        std::bind(&OdomController::odomCallback, this, std::placeholders::_1)
    );
    
    // Republish odometry
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/sensors/odom",
        rclcpp::SensorDataQoS()
    );
    
    RCLCPP_INFO(this->get_logger(), "OdomController initialized");
}

void OdomController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Simply republish
    odom_pub_->publish(*msg);
}

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<OdomController>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}