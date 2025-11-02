// ============================================================================
// MTRX3760 Project 2 - 
// File: SlamController.hpp
// Description: Header for SlamController class. Defines SLAM Toolbox
//              integration for mapping, localization, and map management
//              operations in warehouse environments.
// Author(s): Dylan George
// Last Edited: 2025-11-02
// ============================================================================

#ifndef SLAM_CONTROLLER_HPP
#define SLAM_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>
#include <string>

class SlamController {
public:
    SlamController(rclcpp::Node::SharedPtr node);
    ~SlamController() = default;
    
    // Map access
    nav_msgs::msg::OccupancyGrid::SharedPtr getCurrentMap() const;
    bool hasValidMap() const;
    
    // Pose access
    geometry_msgs::msg::Pose getCurrentPose() const;
    bool hasValidPose() const;
    
    // Map saving
    void saveMap(const std::string& map_name = "warehouse_map");
    
    // Exploration status
    bool isExplorationComplete() const;
    void setExplorationComplete(bool complete);
    
private:
    rclcpp::Node::SharedPtr node_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // State
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    geometry_msgs::msg::Pose current_pose_;
    bool has_valid_map_;
    bool has_valid_pose_;
    bool exploration_complete_;
    
    // Callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // Helper functions
    void updatePoseFromTF();
};

#endif // SLAM_CONTROLLER_HPP
