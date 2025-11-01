// ============================================================================
// File: CImageProcessorNode.hpp
// Description: Base ROS2 node for image processing. Subscribes to unified
//              camera topic and provides shared infrastructure for derived
//              detector nodes. Handles image conversion and basic validation.
// Author(s): Dylan George
// Last Edited: 2025-10-27
// ============================================================================

#ifndef IMAGE_PROCESSOR_NODE_HPP
#define IMAGE_PROCESSOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

// CImageProcessorNode
// Abstract base class for image processing nodes. Provides common functionality
// for subscribing to camera data, converting ROS images to OpenCV format, and
// basic image validation. Derived classes implement ProcessImage() to define
// specific detection/processing algorithms.
// Ownership: Manages camera subscription via rclcpp. Derived classes own their
// specific publishers and processing logic.
class CImageProcessorNode : public rclcpp::Node
{
    public:
        // Constructor
        // aNodeName: name for this ROS node
        // Initialises the node and subscribes to the unified camera topic.
        explicit CImageProcessorNode(const std::string &aNodeName);

        // Destructor
        // Virtual to ensure proper cleanup in derived classes.
        virtual ~CImageProcessorNode();

    protected:
        // ProcessImage (pure virtual)
        // Processes a single OpenCV image. Must be implemented by derived classes.
        // aImage: OpenCV image in BGR8 format
        // aTimestamp: ROS timestamp from original image message
        // Called after image validation and conversion.
        virtual void ProcessImage(const cv::Mat &aImage, 
                                 const rclcpp::Time &aTimestamp) = 0;

        // ValidateImage
        // Checks if an OpenCV image is valid for processing.
        // aImage: image to validate
        // Returns true if image is non-empty and has valid dimensions.
        bool ValidateImage(const cv::Mat &aImage) const;

        // GetLogger
        // Provides access to ROS logger for derived classes.
        // Returns reference to this node's logger.
        rclcpp::Logger GetLogger() const;

    private:
        // ImageCallback
        // Callback for incoming camera images. Converts ROS image to OpenCV format,
        // validates it, and calls ProcessImage() if valid.
        // aMsg: incoming image message from unified camera topic
        void ImageCallback(const sensor_msgs::msg::Image::SharedPtr aMsg);

        // Member Variables

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mpImageSubscriber;
        // Subscribes to unified camera topic; owned by this node, created in ctor

        const std::string kCameraTopicName = "/camera/image_raw";
        // Input topic for camera data [string constant]

        const int kQueueSize = 10;
        // Subscriber queue depth [messages]

        const int kMinImageWidth = 10;
        // Minimum acceptable image width [pixels]

        const int kMinImageHeight = 10;
        // Minimum acceptable image height [pixels]
};

#endif // IMAGE_PROCESSOR_NODE_HPP