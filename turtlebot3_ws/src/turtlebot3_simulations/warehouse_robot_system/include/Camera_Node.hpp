// ============================================================================
// File: CCameraNode.hpp
// Description: ROS2 node that subscribes to all available camera topics and
//              republishes image data on a unified topic for modular access.
//              Provides single point of camera data distribution.
// Author(s): Dylan George
// Last Edited: 2025-10-27
// ============================================================================

#ifndef CCAMERA_NODE_HPP
#define CCAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>
#include <string>

// CCameraNode
// Aggregates multiple camera topics and republishes on a single unified topic.
// Subscribes to all camera sources defined in ROS parameters and forwards
// image messages to downstream processing nodes. This decouples the camera
// hardware interface from processing logic.
// Ownership: Manages its own publishers and subscribers via rclcpp.
class CCameraNode : public rclcpp::Node
{
    public:
        // Constructor
        // Initialises the node, loads camera topic parameters, creates subscribers
        // for each camera source, and sets up the unified publisher.
        CCameraNode();

        // Destructor
        // Cleans up ROS resources (handled automatically by rclcpp).
        ~CCameraNode();

    private:
        // ImageCallback
        // Callback invoked when a new image arrives from any subscribed camera topic.
        // aMsg: incoming image message from camera
        // Simply forwards the message to the unified output topic.
        void ImageCallback(const sensor_msgs::msg::Image::SharedPtr aMsg);

        // LoadCameraTopics
        // Reads camera topic names from ROS parameters and populates mCameraTopics.
        // If no parameter is set, uses default topic "/camera/image_raw".
        void LoadCameraTopics();

        // CreateSubscriptions
        // Creates a subscriber for each topic in mCameraTopics.
        // All subscribers use the same callback (ImageCallback).
        void CreateSubscriptions();

        // Member Variables

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mpUnifiedPublisher; 
        // Publishes to unified camera topic; owned by this node, created in ctor

        std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> mCameraSubscribers;
        // List of subscribers to various camera topics; owned by this node

        std::vector<std::string> mCameraTopics;
        // List of camera topic names to subscribe to; loaded from parameters

        const std::string kUnifiedTopicName = "/camera/unified";
        // Output topic name for republished images [string constant]

        const int kQueueSize = 10;
        // ROS publisher/subscriber queue depth [messages]
};

#endif // CCAMERA_NODE_HPP