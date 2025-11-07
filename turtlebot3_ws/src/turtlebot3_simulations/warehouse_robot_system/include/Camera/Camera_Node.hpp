// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: Camera_Node.hpp
// Author(s): Dylan George
//
// Description: Aggregates multiple camera topics and republishes on a single
// unified topic.

#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>

/**
 * @class CCameraNode
 * @brief Aggregates multiple camera topics and republishes on a single unified
 * topic.
 * @details Subscribes to all camera sources defined in ROS parameters and
 * forwards image messages to downstream processing nodes. This decouples the
 * camera hardware interface from processing logic.
 */
class CCameraNode : public rclcpp::Node
{
    public:
        /**
         * @brief Constructor - Initialises the node, loads camera topic
         * parameters, creates subscribers for each camera source, and sets up
         * the unified publisher.
         */
        CCameraNode();

        /**
         * @brief Destructor - Cleans up ROS resources (handled automatically by
         * rclcpp).
         */
        ~CCameraNode();

        /// @}

    private:
        /**
         * @brief Callback invoked when a new image arrives from any subscribed
         * camera topic.
         * @param aMsg incoming image message from camera
         * @details Simply forwards the message to the unified output topic.
         */
        void ImageCallback(const sensor_msgs::msg::Image::SharedPtr aMsg);

        /**
         * @brief Reads camera topic names from ROS parameters and populates
         * mCameraTopics.
         * @details If no parameter is set, uses default topic
         * "/camera/image_raw".
         */
        void LoadCameraTopics();

        /**
         * @brief Creates a subscriber for each topic in mCameraTopics.
         * @details All subscribers use the same callback (ImageCallback).
         */
        void CreateSubscriptions();

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
            mpUnifiedPublisher;
        ///< Publishes to unified camera topic; owned by this node, created in
        ///< ctor

        std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
            mCameraSubscribers;
        ///< List of subscribers to various camera topics; owned by this node

        std::vector<std::string> mCameraTopics;
        ///< List of camera topic names to subscribe to; loaded from parameters

        const std::string kUnifiedTopicName = "/camera/unified"; ///< Output topic name for republished images
        const int kQueueSize = 10; ///< ROS publisher/subscriber queue depth [messages]
};

#endif // CCAMERA_NODE_HPP