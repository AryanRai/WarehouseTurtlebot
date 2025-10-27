// MTRX3760 2025 Project 2: Warehouse Robot
// File: SlamController.cpp
// Author(s): Aryan Rai
//
// SLAM Controller - Handles mapping and localization only

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "SlamController.hpp"

namespace slam {

SlamController::SlamController() : Node("slam_controller") 
{
    RCLCPP_INFO(this->get_logger(), "SLAM Controller starting...");
    
    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    // Subscribers - receive from Lidar and Odometer nodes
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, 
        std::bind(&SlamController::lidarCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&SlamController::odomCallback, this, std::placeholders::_1));
    
    // Publishers - provide map and pose to other nodes
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", 10);
    
    // Timer for publishing pose updates
    pose_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&SlamController::publishPose, this));
    
    RCLCPP_INFO(this->get_logger(), "SLAM Controller initialized");
}

void SlamController::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
{
    // Process laser scan data for SLAM
    // This would integrate with slam_toolbox or your SLAM implementation
    current_scan_ = msg;
    
    // Update map based on scan and odometry
    updateMap();
}

void SlamController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) 
{
    // Store odometry for SLAM processing
    current_odom_ = msg;
}

void SlamController::updateMap() 
{
    if (!current_scan_ || !current_odom_) {
        return;
    }
    
    // Perform SLAM update using scan and odometry
    // This is where you'd call slam_toolbox functions or your SLAM algorithm
    
    // For now, publish the updated map
    if (current_map_) {
        current_map_->header.stamp = this->now();
        map_pub_->publish(*current_map_);
    }
}

void SlamController::publishPose() 
{
    // Get current robot pose from SLAM/TF
    try {
        geometry_msgs::msg::TransformStamped transform = 
            tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
        
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = transform.transform.translation.x;
        pose.pose.position.y = transform.transform.translation.y;
        pose.pose.position.z = transform.transform.translation.z;
        pose.pose.orientation = transform.transform.rotation;
        
        pose_pub_->publish(pose);
        
    } catch (tf2::TransformException& ex) {
        RCLCPP_DEBUG(this->get_logger(), "Could not get transform: %s", ex.what());
    }
}

} // namespace slam