/*
 * File: CTurtlebot3Drive.hpp
 * Description: Declares the CTurtlebot3Drive class, the main ROS2 node for TurtleBot3 wall-following and navigation in Gazebo simulation.
 * 
 * Date: 2025-10-10
 *
 * This header defines the interface for managing sensor processing, wall-following controller,
 * and publishing velocity commands to the robot.
*/

#ifndef TURTLEBOT3_GAZEBO__CTURTLEBOT3_DRIVE_HPP_
#define TURTLEBOT3_GAZEBO__CTURTLEBOT3_DRIVE_HPP_

#include <cmath>
#include <memory>
#include <utility>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <cstddef>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

/**
 * @class CTurtlebot3Drive
 * @brief Main ROS2 node that orchestrates all components
 * 
 * Owns and manages sensor processing, wall-following controller,
 * and publishes velocity commands to robot.
 */
class CTurtlebot3Drive : public rclcpp::Node
{
  public:
    /**
     * @brief Constructor - initializes all components
     */
    CTurtlebot3Drive();
    
    /**
     * @brief Destructor
     */
    ~CTurtlebot3Drive();

  private:
    // Control loop rate
    static constexpr int CONTROL_RATE_MS = 10;

    // Component instances (owned by this class)
    std::unique_ptr<CTurtlebot3Sensor> cSensor_;
    std::unique_ptr<CWallFollowingController> cController_;

    // ROS publisher
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

    // Control loop timer
    rclcpp::TimerBase::SharedPtr update_timer_;

    // Timing state
    rclcpp::Time prev_ctrl_time_;

    /**
     * @brief Main control loop callback
     * 
     * Called at fixed rate, computes velocity and publishes command
     */
    void update_callback();
    
    /**
     * @brief Publish velocity command to robot
     * @param aLinear Linear velocity (m/s)
     * @param aAngular Angular velocity (rad/s)
     */
    void update_cmd_vel(double aLinear, double aAngular);
};

#endif  // TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_