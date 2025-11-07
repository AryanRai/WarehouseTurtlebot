// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: LidarController.cpp
// Author(s): Inez Dumas
//
// Description: Implementation of LidarController class

#include "Sensors/LidarController.hpp"

LidarController::LidarController()
    : Node("lidar_controller")
{
    // Subscribe to raw laser scan
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&LidarController::laserCallback, this, std::placeholders::_1)
    );
    
    // Republish laser scan
    laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "/lidar/scan",
        rclcpp::SensorDataQoS()
    );
    
    RCLCPP_INFO(this->get_logger(), "LidarController initialized");
}

void LidarController::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Simply republish
    laser_pub_->publish(*msg);
}

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<LidarController>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}