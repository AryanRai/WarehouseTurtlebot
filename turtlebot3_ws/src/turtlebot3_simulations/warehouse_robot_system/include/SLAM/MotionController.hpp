// ============================================================================
// MTRX3760 Project 2 - 
// File: MotionController.hpp
// Description: Header for MotionController class. Defines Pure Pursuit path
//              following with obstacle avoidance and adaptive navigation for
//              precise robot motion control in warehouse environments.
// Author(s): Dylan George
// Last Edited: 2025-11-02
// ============================================================================

#ifndef MOTION_CONTROLLER_HPP
#define MOTION_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include "SLAM/PathPlanner.hpp"
#include <memory>

class MotionController {
public:
    MotionController(rclcpp::Node::SharedPtr node);
    ~MotionController() = default;
    
    // Path following
    void setPath(const nav_msgs::msg::Path& path);
    void clearPath();
    bool hasPath() const;
    
    // Control
    geometry_msgs::msg::Twist computeVelocityCommand(
        const geometry_msgs::msg::Pose& current_pose,
        const nav_msgs::msg::OccupancyGrid& map
    );
    
    // Status
    bool isAtGoal() const;
    void setEnabled(bool enabled);
    bool isEnabled() const;
    
private:
    rclcpp::Node::SharedPtr node_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_pub_;
    rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr fov_cells_pub_;
    rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr close_wall_cells_pub_;
    
    // Pure pursuit parameters
    static constexpr double LOOKAHEAD_DISTANCE = 0.18;  // m
    static constexpr double WHEEL_BASE = 0.16;  // m
    static constexpr double MAX_DRIVE_SPEED = 0.06;  // m/s - slower for better AprilTag detection
    static constexpr double MAX_TURN_SPEED = 0.8;  // rad/s - slower turns
    static constexpr double TURN_SPEED_KP = 1.25;
    static constexpr double DISTANCE_TOLERANCE = 0.1;  // m
    
    // Obstacle avoidance parameters
    static constexpr double OBSTACLE_AVOIDANCE_GAIN = 0.3;
    static constexpr double OBSTACLE_AVOIDANCE_MAX_SLOW_DOWN_DISTANCE = 0.16;  // m
    static constexpr double OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_DISTANCE = 0.12;  // m
    static constexpr double OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_FACTOR = 0.25;
    static constexpr int FOV = 200;  // degrees
    static constexpr int FOV_DISTANCE = 25;  // grid cells
    static constexpr int FOV_DEADZONE = 80;  // degrees
    static constexpr int SMALL_FOV = 300;  // degrees
    static constexpr int SMALL_FOV_DISTANCE = 10;  // grid cells
    
    // State
    nav_msgs::msg::Path current_path_;
    bool has_path_;
    bool enabled_;
    bool reversed_;
    double alpha_;
    double closest_distance_;
    bool debug_mode_;
    
    // Helper functions
    int findNearestWaypointIndex(const geometry_msgs::msg::Pose& pose) const;
    geometry_msgs::msg::Point findLookahead(const geometry_msgs::msg::Pose& pose, 
                                            int nearest_waypoint_index) const;
    geometry_msgs::msg::Point getGoal() const;
    double getDistanceToWaypoint(const geometry_msgs::msg::Pose& pose, int index) const;
    double calculateSteeringAdjustment(const geometry_msgs::msg::Pose& pose,
                                       const nav_msgs::msg::OccupancyGrid& map);
    static double distance(double x0, double y0, double x1, double y1);
};

#endif // MOTION_CONTROLLER_HPP
