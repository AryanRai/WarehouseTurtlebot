// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: twist_stamped_bridge.cpp
// Author(s): Aryan Rai
//
// Description: ROS2 node that bridges geometry_msgs/Twist messages to
//              geometry_msgs/TwistStamped messages for compatibility with
//              MotionController which expects stamped messages.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class TwistStampedBridge : public rclcpp::Node {
public:
    TwistStampedBridge() : Node("twist_stamped_bridge") {
        // Subscribe to regular Twist messages
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_unstamped", 10,
            std::bind(&TwistStampedBridge::twistCallback, this, std::placeholders::_1)
        );
        
        // Publish TwistStamped messages
        twist_stamped_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10
        );
        
        RCLCPP_INFO(this->get_logger(), "Twist to TwistStamped bridge started");
    }

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        geometry_msgs::msg::TwistStamped stamped_msg;
        stamped_msg.header.stamp = this->now();
        stamped_msg.header.frame_id = "base_footprint";
        stamped_msg.twist = *msg;
        twist_stamped_pub_->publish(stamped_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistStampedBridge>());
    rclcpp::shutdown();
    return 0;
}
