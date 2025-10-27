#ifndef ODOMETRY_NODE_HPP
#define ODOMETRY_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdometryNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Odometry Node
   *
   * Creates both subscriber and publisher for odometry data.
   */
  OdometryNode();

  /**
   * @brief Destroy the Odometry Node
   */
  ~OdometryNode();

private:
  // Subscriber for incoming odometry data
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publisher for republishing or relaying odometry data
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

#endif // ODOMETRY_NODE_HPP
