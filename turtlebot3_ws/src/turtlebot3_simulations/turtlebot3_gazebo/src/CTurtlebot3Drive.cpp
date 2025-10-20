/*
 * File: CTurtlebot3Drive.cpp
 * Description: Implements the CTurtlebot3Drive class, 
   the main ROS2 node for TurtleBot3 wall-following and 
   navigation in Gazebo simulation.
 * 
 * Date: 2025-10-10
 *
 * This file manages sensor processing, wall-following 
   controller, and publishes velocity commands to the robot.
 */

#include "turtlebot3_gazebo/CWallFollowingController.hpp"
#include "turtlebot3_gazebo/CTurtlebot3Sensor.hpp"
#include "turtlebot3_gazebo/CTurtlebot3Drive.hpp"
#include "turtlebot3_gazebo/CPIDController.hpp"
#include <algorithm>
#include <cmath>
#include <memory>


// CTurtlebot3Drive Constructor
CTurtlebot3Drive::CTurtlebot3Drive()
  : Node("turtlebot3_drive_node"),
    prev_ctrl_time_(0, 0, RCL_ROS_TIME)
{
  // Create sensor manager
  cSensor_ = std::make_unique<CTurtlebot3Sensor>(this);

  // Create wall following controller
  cController_ = std::make_unique<CWallFollowingController>(this, cSensor_.get());

  // Initialize publisher
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 
      qos);

  // Initialize timer for control loop
  update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(CONTROL_RATE_MS), 
      std::bind(&CTurtlebot3Drive::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been initialized");
}

// CTurtlebot3Drive Destructor
CTurtlebot3Drive::~CTurtlebot3Drive()
{
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been terminated");
}

// Publish velocity command
void CTurtlebot3Drive::update_cmd_vel(double aLinear, double aAngular)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = this->get_clock()->now();
  cmd_vel.twist.linear.x = aLinear;
  cmd_vel.twist.angular.z = aAngular;
  cmd_vel_pub_->publish(cmd_vel);
}

// Control loop callback
void CTurtlebot3Drive::update_callback()
{
  const rclcpp::Time now = this->now();
  
  // Compute time step
  double dt = 0.05;
  if (prev_ctrl_time_.nanoseconds() > 0) 
  {
    dt = (now - prev_ctrl_time_).seconds();
  }
  prev_ctrl_time_ = now;

  // Get desired velocity from controller
  const CWallFollowingController::SVelocityCommand sCmd = 
      cController_->compute_velocity(dt);

  // Publish velocity command
  update_cmd_vel(sCmd.mLinear, sCmd.mAngular);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CTurtlebot3Drive>());
  rclcpp::shutdown();
  
  return 0;
}