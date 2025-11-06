// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: ImageProcessor_Node.cpp
// Author(s): Dylan George
//
// Description: Implementation of base image processor node for camera image processing.

#include "Camera/ImageProcessor_Node.hpp"

CImageProcessorNode::CImageProcessorNode(const std::string &aNodeName)
    : rclcpp::Node(aNodeName)
{
    // Create subscription to unified camera topic
    mpImageSubscriber = this->create_subscription<sensor_msgs::msg::Image>(
        kCameraTopicName,
        kQueueSize,
        std::bind(&CImageProcessorNode::ImageCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Image processor node '%s' initialised", aNodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", kCameraTopicName.c_str());
}

CImageProcessorNode::~CImageProcessorNode()
{
    RCLCPP_INFO(this->get_logger(), "Image processor node shutting down");
}

void CImageProcessorNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr aMsg)
{
    // Validate incoming message pointer
    if (aMsg == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "Received null image message");
        return;
    }

    // Convert ROS image to OpenCV format
    cv_bridge::CvImagePtr cvPointer;
    try {
        cvPointer = cv_bridge::toCvCopy(aMsg, sensor_msgs::image_encodings::BGR8);
    } 
    catch (const cv_bridge::Exception &aException) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", aException.what());
        return;
    }

    // Validate converted image
    if (!ValidateImage(cvPointer->image)) {
        RCLCPP_WARN(this->get_logger(), "Image validation failed");
        return;
    }

    // Extract timestamp from message header
    rclcpp::Time timestamp(aMsg->header.stamp);

    // Delegate to derived class for specific processing
    ProcessImage(cvPointer->image, timestamp);
}

bool CImageProcessorNode::ValidateImage(const cv::Mat &aImage) const
{
    // Check if image is empty
    if (aImage.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty image");
        return false;
    }

    // Check if image dimensions are reasonable
    if (aImage.cols < kMinImageWidth || aImage.rows < kMinImageHeight) {
        RCLCPP_WARN(this->get_logger(), 
                    "Image dimensions too small: %dx%d", 
                    aImage.cols, aImage.rows);
        return false;
    }

    // Image is valid
    return true;
}

rclcpp::Logger CImageProcessorNode::GetLogger() const
{
    return this->get_logger();
}