// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: Camera_Node.cpp
// Author(s): Dylan George
//
// Description: Implementation of unified camera node that subscribes to multiple
//              camera topics and republishes images to a single output topic.

#include "Camera/Camera_Node.hpp"

CCameraNode::CCameraNode()
    : rclcpp::Node("camera_node")
{
    // Create publisher for unified camera output
    mpUnifiedPublisher = this->create_publisher<sensor_msgs::msg::Image>(
        kUnifiedTopicName,
        kQueueSize
    );

    RCLCPP_INFO(this->get_logger(), "Camera node initialised");
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", kUnifiedTopicName.c_str());

    // Load camera topics from parameters
    LoadCameraTopics();

    // Create subscriptions to all camera sources
    CreateSubscriptions();
}

CCameraNode::~CCameraNode()
{
    RCLCPP_INFO(this->get_logger(), "Camera node shutting down");
}

void CCameraNode::LoadCameraTopics()
{
    // Attempt to load camera topics from ROS parameter
    // If parameter not set, use default camera topic
    std::vector<std::string> paramTopics;
    
    if (this->has_parameter("camera_topics")) {
        paramTopics = this->get_parameter("camera_topics").as_string_array();
    } 
    else {
        // Declare parameter with default value
        paramTopics = this->declare_parameter<std::vector<std::string>>(
            "camera_topics",
            std::vector<std::string>{"/camera/image_raw"}
        );
    }

    // Validate and copy to member variable
    if (paramTopics.empty()) {
        RCLCPP_WARN(this->get_logger(), "No camera topics specified, using default");
        mCameraTopics.push_back("/camera/image_raw");
    } 
    else {
        mCameraTopics = paramTopics;
        RCLCPP_INFO(this->get_logger(), "Loaded %zu camera topics", mCameraTopics.size());
        
        for (const auto &topic : mCameraTopics) {
            RCLCPP_INFO(this->get_logger(), "  - %s", topic.c_str());
        }
    }
}

void CCameraNode::CreateSubscriptions()
{
    // Create a subscriber for each camera topic
    for (const auto &topic : mCameraTopics) {
        auto subscription = this->create_subscription<sensor_msgs::msg::Image>(
            topic,
            kQueueSize,
            std::bind(&CCameraNode::ImageCallback, this, std::placeholders::_1)
        );

        mCameraSubscribers.push_back(subscription);
        RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", topic.c_str());
    }
}

void CCameraNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr aMsg)
{
    // Validate incoming message
    if (aMsg == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "Received null image message");
        return;
    }

    if (aMsg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty image data");
        return;
    }

    // Forward message to unified output topic
    mpUnifiedPublisher->publish(*aMsg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto cCameraNode = std::make_shared<CCameraNode>();
    rclcpp::spin(cCameraNode);
    rclcpp::shutdown();
    return 0;
}