/*
 * File: CTurtlebot3Sensor.cpp
 * Description: Implements the CTurtlebot3Sensor class for managing 
   sensor data subscriptions and processing in TurtleBot3 Gazebo simulation.
 * 
 * Date: 2025-10-10
 *
 * This file subscribes to laser scan and odometry topics, processes sensor 
   data, and provides access to sensor readings for other components.
 */

#include "turtlebot3_gazebo/CWallFollowingController.hpp"
#include "turtlebot3_gazebo/CTurtlebot3Sensor.hpp"
#include "turtlebot3_gazebo/CTurtlebot3Drive.hpp"
#include "turtlebot3_gazebo/CPIDController.hpp"
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

// CTurtlebot3Sensor Constructor
CTurtlebot3Sensor::CTurtlebot3Sensor(rclcpp::Node* aNode)
  : node_(aNode),
    pose_x_(0.0),
    pose_y_(0.0),
    robot_pose_(0.0),
    prev_yaw_(0.0),
    yaw_unwrapped_(0.0),
    first_odom_(true)
{
  // Validate input
  if (node_ == nullptr)
  {
    throw std::invalid_argument("CTurtlebot3Sensor: node pointer cannot be null");
  }

  // Initialize scan data array
  for (int i = 0; i < SCAN_COUNT; ++i)
  {
    scan_data_[i] = 0.0;
  }

  // Create QoS profile
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Subscribe to laser scan
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 
      rclcpp::SensorDataQoS(), 
      std::bind(&CTurtlebot3Sensor::scan_callback, this, std::placeholders::_1));

  // Subscribe to odometry
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 
      qos, 
      std::bind(&CTurtlebot3Sensor::odom_callback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "CTurtlebot3Sensor initialized");
}

// CTurtlebot3Sensor Destructor
CTurtlebot3Sensor::~CTurtlebot3Sensor()
{
  if (node_ != nullptr)
  {
    RCLCPP_INFO(node_->get_logger(), "CTurtlebot3Sensor destroyed");
  }
}

// Get scan data for specified direction
double CTurtlebot3Sensor::get_scan_data(eScanIndex aIndex) const
{
  if (aIndex < 0 || aIndex >= SCAN_COUNT)
  {
    return 0.0;
  }
  return scan_data_[aIndex];
}

// Get robot's x position
double CTurtlebot3Sensor::get_pose_x() const
{
  return pose_x_;
}

// Get robot's y position
double CTurtlebot3Sensor::get_pose_y() const
{
  return pose_y_;
}

// Get robot's yaw orientation (radians)
double CTurtlebot3Sensor::get_robot_pose() const
{
  return robot_pose_;
}

// Get robot's unwrapped yaw (cumulative rotation in radians)
double CTurtlebot3Sensor::get_yaw_unwrapped() const
{
  return yaw_unwrapped_;
}

// Laser scan callback
void CTurtlebot3Sensor::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr aMsg)
{
  // Validate message
  if (aMsg == nullptr || aMsg->ranges.empty())
  {
    return;
  }

  // Convert angle in degrees to scan index
  auto idx_from_deg = [&](double deg) -> int {
    const double rad = deg * DEG_TO_RAD;
    const int idx = static_cast<int>(
        std::lround((rad - aMsg->angle_min) / aMsg->angle_increment));
    const int n = static_cast<int>(aMsg->ranges.size());
    const int wrapped = ((idx % n) + n) % n;
    return wrapped;
  };

  // Find minimum range in a sector
  auto sector_min = [&](double center_deg, double half_deg) -> double {
    double rmin = aMsg->range_max;
    const int half_deg_int = static_cast<int>(half_deg);
    
    for (int d = -half_deg_int; d <= half_deg_int; ++d) 
    {
      const int idx = idx_from_deg(center_deg + static_cast<double>(d));
      float r = aMsg->ranges.at(idx);
      
      if (std::isfinite(r)) 
      {
        // Clamp small values to minimum valid range
        if (r < MIN_VALID_RANGE) 
        {
          r = MIN_VALID_RANGE;
        }
        
        if (r < rmin) 
        {
          rmin = r;
        }
      }
    }
    return rmin;
  };

  // Process all scan sectors
  scan_data_[FORWARD]     = sector_min(0.0,   SECTOR_HALF_ANGLE_DEG);
  scan_data_[FRONT_RIGHT] = sector_min(315.0, SECTOR_HALF_ANGLE_DEG);
  scan_data_[RIGHT]       = sector_min(270.0, SECTOR_HALF_ANGLE_DEG);
  scan_data_[FRONT_LEFT]  = sector_min(45.0,  SECTOR_HALF_ANGLE_DEG);
  scan_data_[LEFT]        = sector_min(90.0,  SECTOR_HALF_ANGLE_DEG);
}

// Odometry callback
void CTurtlebot3Sensor::odom_callback(const nav_msgs::msg::Odometry::SharedPtr aMsg)
{
  // Validate message
  if (aMsg == nullptr)
  {
    return;
  }

  // Extract position
  pose_x_ = aMsg->pose.pose.position.x;
  pose_y_ = aMsg->pose.pose.position.y;

  // Extract orientation and convert to yaw
  tf2::Quaternion q(
      aMsg->pose.pose.orientation.x,
      aMsg->pose.pose.orientation.y,
      aMsg->pose.pose.orientation.z,
      aMsg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll;
  double pitch;
  double yaw;
  m.getRPY(roll, pitch, yaw);
  robot_pose_ = yaw;

  // Unwrap yaw to track cumulative rotation
  if (first_odom_) 
  {
    prev_yaw_ = robot_pose_;
    yaw_unwrapped_ = robot_pose_;
    first_odom_ = false;
    return;
  }

  // Compute shortest signed angle difference
  double dyaw = wrap_to_pi(robot_pose_ - prev_yaw_);
  
  // Reject unrealistic spikes that can occur on simulation start
  if (std::abs(dyaw) > MAX_YAW_DELTA) 
  {
    dyaw = (dyaw > 0.0) ? MAX_YAW_DELTA : -MAX_YAW_DELTA;
  }
  
  prev_yaw_ = robot_pose_;
  yaw_unwrapped_ += dyaw;
}

// Wrap angle to (-pi, pi]
double CTurtlebot3Sensor::wrap_to_pi(double aAngle) const
{
  double result = aAngle;
  while (result > M_PI) 
  {
    result -= 2.0 * M_PI;
  }
  while (result < -M_PI) 
  {
    result += 2.0 * M_PI;
  }
  return result;
}