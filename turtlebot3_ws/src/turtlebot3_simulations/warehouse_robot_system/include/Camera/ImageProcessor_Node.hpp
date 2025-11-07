// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: ImageProcessor_Node.hpp
// Author(s): Dylan George
//
// Description: Abstract base class for image processing nodes.

#ifndef IMAGE_PROCESSOR_NODE_HPP
#define IMAGE_PROCESSOR_NODE_HPP

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

/**
 * @class ImageProcessorNode
 * @brief Abstract base class for image processing nodes.
 * @details Provides common functionality for subscribing to camera data,
 * converting ROS images to OpenCV format, and basic image validation. Derived
 * classes implement ProcessImage() to define specific detection/processing
 * algorithms.
 */
class ImageProcessorNode : public rclcpp::Node
{
    public:
        /**
         * @brief Constructor - Initialises the node and subscribes to the
         * unified camera topic.
         * @param aNodeName name for this ROS node
         */
        explicit ImageProcessorNode(const std::string &aNodeName);

        /**
         * @brief Virtual destructor to ensure proper cleanup in derived
         * classes.
         */
        virtual ~ImageProcessorNode();

    protected:
        /**
         * @brief Processes a single OpenCV image. Must be implemented by
         * derived classes.
         * @param aImage OpenCV image in BGR8 format
         * @param aTimestamp ROS timestamp from original image message
         * @details Called after image validation and conversion.
         */
        virtual void ProcessImage(const cv::Mat &aImage,
                                  const rclcpp::Time &aTimestamp) = 0;

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

    private:
        /**
         * @brief Callback for incoming camera images.
         * @param aMsg incoming image message from unified camera topic
         * @details Converts ROS image to OpenCV format, validates it, and calls
         * ProcessImage() if valid.
         */
        void ImageCallback(const sensor_msgs::msg::Image::SharedPtr aMsg);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
            mpImageSubscriber;

        const std::string kCameraTopicName = "/camera/image_raw";   ///< Input topic for camera data
        const int kQueueSize = 10; ///< Subscriber queue depth [messages]
        const int kMinImageWidth = 10; ///< Minimum acceptable image width [pixels]
        const int kMinImageHeight = 10; ///< Minimum acceptable image height [pixels]

        /// @}
};

#endif // IMAGE_PROCESSOR_NODE_HPP