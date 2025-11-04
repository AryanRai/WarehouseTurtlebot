// ============================================================================
// MTRX3760 Project 2 - 
// File: Camera_Node.hpp
// Description: Header for CCameraNode class. Defines ROS2 node that subscribes
//              to all available camera topics and republishes image data on a
//              unified topic for modular access and distribution.
// Author(s): Dylan George
// Last Edited: 2025-11-02
// ============================================================================

#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>
#include <string>

/**
 * @class CCameraNode
 * @brief Aggregates multiple camera topics and republishes on a single unified topic.
 * @details Subscribes to all camera sources defined in ROS parameters and forwards
 *          image messages to downstream processing nodes. This decouples the camera
 *          hardware interface from processing logic.
 */
class CCameraNode : public rclcpp::Node
{
    public:
        // ====================================================================
        /// @name Constructor & Destructor
        // ====================================================================
        /// @{
        
        /**
         * @brief Constructor - Initialises the node, loads camera topic parameters, creates subscribers
         *        for each camera source, and sets up the unified publisher.
         */
        CCameraNode();

        /**
         * @brief Destructor - Cleans up ROS resources (handled automatically by rclcpp).
         */
        ~CCameraNode();
        
        /// @}

    private:
        // ====================================================================
        /// @name Core Callback Methods
        // ====================================================================
        /// @{
        
        /**
         * @brief Callback invoked when a new image arrives from any subscribed camera topic.
         * @param aMsg incoming image message from camera
         * @details Simply forwards the message to the unified output topic.
         */
        void ImageCallback(const sensor_msgs::msg::Image::SharedPtr aMsg);
        
        /// @}

        // ====================================================================
        /// @name Initialization & Setup Methods
        // ====================================================================
        /// @{
        
        /**
         * @brief Reads camera topic names from ROS parameters and populates mCameraTopics.
         * @details If no parameter is set, uses default topic "/camera/image_raw".
         */
        void LoadCameraTopics();

        /**
         * @brief Creates a subscriber for each topic in mCameraTopics.
         * @details All subscribers use the same callback (ImageCallback).
         */
        void CreateSubscriptions();
        
        /// @}

        // ====================================================================
        /// @name Member Variables - ROS Communication
        // ====================================================================
        /// @{

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mpUnifiedPublisher; 
        ///< Publishes to unified camera topic; owned by this node, created in ctor

        std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> mCameraSubscribers;
        ///< List of subscribers to various camera topics; owned by this node
        
        /// @}

        // ====================================================================
        /// @name Member Variables - Configuration
        // ====================================================================
        /// @{

        std::vector<std::string> mCameraTopics;
        ///< List of camera topic names to subscribe to; loaded from parameters
        
        /// @}

        // ====================================================================
        /// @name Constants - Configuration
        // ====================================================================
        /// @{

        const std::string kUnifiedTopicName = "/camera/unified";  ///< Output topic name for republished images
        const int kQueueSize = 10;  ///< ROS publisher/subscriber queue depth [messages]
        
        /// @}
};

#endif // CCAMERA_NODE_HPP