#include "Odometer.hpp"

OdometryNode::OdometryNode()
: Node("odometry_node")
{
  // Create publisher for odometry data
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_out", 10);

  // Create subscriber for odometry data
  // No callback needed — could be connected later if processing is required
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    [](const nav_msgs::msg::Odometry::SharedPtr) {
      // Empty lambda callback - no processing
    }
  );

  RCLCPP_INFO(this->get_logger(), "OdometryNode initialised: subscribed to /odom, publishing on /odom_out");
}

OdometryNode::~OdometryNode()
{
  RCLCPP_INFO(this->get_logger(), "OdometryNode shutting down");
}
