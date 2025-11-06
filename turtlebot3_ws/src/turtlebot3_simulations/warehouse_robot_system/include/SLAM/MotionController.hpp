// ============================================================================
// MTRX3760 Project 2 - 
// File: MotionController.hpp
// Description: MotionController ROS2 node for Pure Pursuit path following.
//              Subscribes to paths and publishes velocity commands with 
//              obstacle avoidance for precise warehouse navigation.
// Author(s): Dylan George
// Last Edited: 2025-11-06
// ============================================================================

#ifndef MOTION_CONTROLLER_HPP
#define MOTION_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "SLAM/PathPlanner.hpp"
#include <memory>

class MotionController : public rclcpp::Node {
public:
    MotionController();
    ~MotionController() = default;
    
    // Public methods for backwards compatibility
    void setPath(const nav_msgs::msg::Path& path);
    void clearPath();
    bool hasPath() const { return has_path_; }
    bool isAtGoal() const { return !has_path_; }
    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }
    
    geometry_msgs::msg::Twist computeVelocityCommand(
        const geometry_msgs::msg::Pose& current_pose,
        const nav_msgs::msg::OccupancyGrid& map
    );
    
    // Speed configuration
    void setMaxSpeeds(double linear_speed, double angular_speed);
    void setInspectionSpeeds();
    void setDeliverySpeeds();
    void setExplorationSpeeds();
    
private:
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr speed_profile_sub_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_pub_;
    rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr fov_cells_pub_;
    rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr close_wall_cells_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // State
    nav_msgs::msg::Path current_path_;
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    geometry_msgs::msg::Pose current_pose_;
    bool has_path_;
    bool has_valid_map_;
    bool has_valid_pose_;
    bool enabled_;
    bool reversed_;
    double alpha_;
    double closest_distance_;
    
    // Pure pursuit parameters
    static constexpr double LOOKAHEAD_DISTANCE = 0.18; 
    static constexpr double WHEEL_BASE = 0.16;  
    static constexpr double TURN_SPEED_KP = 1.25;
    static constexpr double DISTANCE_TOLERANCE = 0.1; 
    static constexpr double CONTROL_RATE = 20.0; 
    
    // Speed limits (configurable per robot type)
    double max_drive_speed_;   
    double max_turn_speed_;    
    
    // Default speed profiles
    static constexpr double INSPECTION_LINEAR_SPEED = 0.05;  
    static constexpr double INSPECTION_ANGULAR_SPEED = 0.6;  
    static constexpr double DELIVERY_LINEAR_SPEED = 0.10;     
    static constexpr double DELIVERY_ANGULAR_SPEED = 1.25;    
    static constexpr double EXPLORATION_LINEAR_SPEED = 0.10;  
    static constexpr double EXPLORATION_ANGULAR_SPEED = 1.25; 
    
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
    
    bool debug_mode_;
    
    // Callbacks
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void speedProfileCallback(const std_msgs::msg::String::SharedPtr msg);
    void controlTimerCallback();
    
    // Control functions
    geometry_msgs::msg::Twist computeVelocityCommand();
    
    // Helper functions
    int findNearestWaypointIndex(const geometry_msgs::msg::Pose& pose) const;
    geometry_msgs::msg::Point findLookahead(const geometry_msgs::msg::Pose& pose, 
                                            int nearest_waypoint_index) const;
    geometry_msgs::msg::Point getGoal() const;
    double getDistanceToWaypoint(const geometry_msgs::msg::Pose& pose, int index) const;
    double calculateSteeringAdjustment(const geometry_msgs::msg::Pose& pose);
    static double distance(double x0, double y0, double x1, double y1);
    void declareParameters();
};

#endif // MOTION_CONTROLLER_HPP