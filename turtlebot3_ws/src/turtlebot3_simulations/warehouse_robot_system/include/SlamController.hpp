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

namespace slam {

class SlamController : public rclcpp::Node {
public:
    SlamController();

private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    void updateMap();
    
    void publishPose();
    
    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr pose_timer_;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // SLAM data
    sensor_msgs::msg::LaserScan::SharedPtr current_scan_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
};

} // namespace slam
