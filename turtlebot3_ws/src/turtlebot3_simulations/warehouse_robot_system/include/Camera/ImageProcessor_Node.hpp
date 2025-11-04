// ============================================================================
// MTRX3760 Project 2 - 
// File: ImageProcessor_Node.hpp
// Description: Header for CImageProcessorNode base class. Defines ROS2 node
//              for image processing with unified camera topic subscription
//              and shared infrastructure for derived detector nodes.
// Author(s): Dylan George
// Last Edited: 2025-11-02
// ============================================================================

#ifndef IMAGE_PROCESSOR_NODE_HPP
#define IMAGE_PROCESSOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

/**
 * @class CImageProcessorNode
 * @brief Abstract base class for image processing nodes.
 * @details Provides common functionality for subscribing to camera data, converting ROS images 
 *          to OpenCV format, and basic image validation. Derived classes implement ProcessImage() 
 *          to define specific detection/processing algorithms.
 */
class CImageProcessorNode : public rclcpp::Node
{
    public:
        // ====================================================================
        /// @name Constructor & Destructor
        // ====================================================================
        /// @{
        
        /**
         * @brief Constructor - Initialises the node and subscribes to the unified camera topic.
         * @param aNodeName name for this ROS node
         */
        explicit CImageProcessorNode(const std::string &aNodeName);

        /**
         * @brief Virtual destructor to ensure proper cleanup in derived classes.
         */
        virtual ~CImageProcessorNode();
        
        /// @}

    protected:
        // ====================================================================
        /// @name Pure Virtual Methods (Must be Implemented by Derived Classes)
        // ====================================================================
        /// @{
        
        /**
         * @brief Processes a single OpenCV image. Must be implemented by derived classes.
         * @param aImage OpenCV image in BGR8 format
         * @param aTimestamp ROS timestamp from original image message
         * @details Called after image validation and conversion.
         */
        virtual void ProcessImage(const cv::Mat &aImage, 
                                 const rclcpp::Time &aTimestamp) = 0;
        
        /// @}

        // ====================================================================
        /// @name Utility Methods for Derived Classes
        // ====================================================================
        /// @{
        
        /**
         * @brief Checks if an OpenCV image is valid for processing.
         * @param aImage image to validate
         * @return true if image is non-empty and has valid dimensions
         */
        bool ValidateImage(const cv::Mat &aImage) const;

        /**
         * @brief Provides access to ROS logger for derived classes.
         * @return reference to this node's logger
         */
        rclcpp::Logger GetLogger() const;
        
        /// @}

    private:
        // ====================================================================
        /// @name Core Callback Methods
        // ====================================================================
        /// @{
        
        /**
         * @brief Callback for incoming camera images.
         * @param aMsg incoming image message from unified camera topic
         * @details Converts ROS image to OpenCV format, validates it, and calls ProcessImage() if valid.
         */
        void ImageCallback(const sensor_msgs::msg::Image::SharedPtr aMsg);
        
        /// @}

        // ====================================================================
        /// @name Member Variables - ROS Communication
        // ====================================================================
        /// @{

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mpImageSubscriber;
        ///< Subscribes to unified camera topic; owned by this node, created in ctor
        
        /// @}

        // ====================================================================
        /// @name Constants - Configuration
        // ====================================================================
        /// @{

        const std::string kCameraTopicName = "/camera/image_raw";  ///< Input topic for camera data
        const int kQueueSize = 10;  ///< Subscriber queue depth [messages]
        const int kMinImageWidth = 10;  ///< Minimum acceptable image width [pixels]
        const int kMinImageHeight = 10;  ///< Minimum acceptable image height [pixels]
        
        /// @}
};

#endif // IMAGE_PROCESSOR_NODE_HPP