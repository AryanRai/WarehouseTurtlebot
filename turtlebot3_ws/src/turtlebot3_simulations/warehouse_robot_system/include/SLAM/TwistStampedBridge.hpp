// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: TwistStampedBridge.hpp
// Author(s): Aryan Rai
//
// Description: ROS2 node that bridges geometry_msgs/Twist messages to
//              geometry_msgs/TwistStamped messages for compatibility with
//              MotionController which expects stamped messages.

#ifndef TWIST_STAMPED_BRIDGE_HPP
#define TWIST_STAMPED_BRIDGE_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Bridge node for converting Twist to TwistStamped messages
 * 
 * Subscribes to unstamped Twist messages and republishes them as
 * TwistStamped messages with proper header information.
 */
class TwistStampedBridge : public rclcpp::Node
{
public:
    /**
     * @brief Constructor
     */
    TwistStampedBridge();
    
    /**
     * @brief Destructor
     */
    ~TwistStampedBridge() = default;
    
private:
    // Subscriber for unstamped Twist messages
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    
    // Publisher for TwistStamped messages
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
        twist_stamped_pub_;
    
    /**
     * @brief Twist callback - converts and republishes as TwistStamped
     * @param msg Twist message
     */
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
};

#endif // TWIST_STAMPED_BRIDGE_HPP