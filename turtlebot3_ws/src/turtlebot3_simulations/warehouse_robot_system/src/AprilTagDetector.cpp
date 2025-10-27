// ============================================================================
// File: AprilTagDetector.cpp
// Description: Implementation of CAprilTagDetector. Subscribes to apriltag_ros
//              detections and republishes with additional formatting for colour
//              detection downstream. Processes 16h5 AprilTags.
// Author(s): Dylan George
// Last Edited: 2025-10-27
// ============================================================================

#include "AprilTagDetector.hpp"

CAprilTagDetector::CAprilTagDetector()
    : CImageProcessorNode("apriltag_detector_node")
{
    // Create publisher for processed AprilTag detections
    mpTagPublisher = this->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
        kTagDetectionTopic,
        kPublisherQueueSize
    );

    // Subscribe to apriltag_ros output
    mpTagSubscriber = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        kAprilTagRosTopic,
        kPublisherQueueSize,
        [this](const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr aMsg) {
            // Simply republish for now - could add processing here
            if (aMsg != nullptr) {
                mpTagPublisher->publish(*aMsg);
            }
        }
    );

    RCLCPP_INFO(GetLogger(), "AprilTag detector initialised");
    RCLCPP_INFO(GetLogger(), "Publishing to: %s", kTagDetectionTopic.c_str());
    RCLCPP_INFO(GetLogger(), "Subscribing to: %s", kAprilTagRosTopic.c_str());
}

CAprilTagDetector::~CAprilTagDetector()
{
    RCLCPP_INFO(GetLogger(), "AprilTag detector shutting down");
}

void CAprilTagDetector::ProcessImage(const cv::Mat &aImage, 
                                     const rclcpp::Time &aTimestamp)
{
    // Convert to grayscale for detection
    cv::Mat grayImage = ConvertToGrayscale(aImage);

    // Detect AprilTags
    std::vector<SAprilTagData> detections = DetectTags(grayImage, aTimestamp);

    // Store detections for potential later use
    mLastDetections = detections;

    // Publish detection results
    if (!detections.empty()) {
        PublishDetections(detections);
        RCLCPP_DEBUG(GetLogger(), "Detected %zu AprilTags", detections.size());
    }
}

cv::Mat CAprilTagDetector::ConvertToGrayscale(const cv::Mat &aColorImage) const
{
    cv::Mat grayImage;

    // Check if already grayscale
    if (aColorImage.channels() == 1) {
        grayImage = aColorImage;
    } else {
        // Convert BGR to grayscale
        cv::cvtColor(aColorImage, grayImage, cv::COLOR_BGR2GRAY);
    }

    return grayImage;
}

std::vector<SAprilTagData> CAprilTagDetector::DetectTags(
    const cv::Mat &aGrayImage,
    const rclcpp::Time &aTimestamp)
{
    std::vector<SAprilTagData> detections;

    // Note: Actual detection is handled by apriltag_ros node
    // This function is here for potential future custom detection logic
    // Currently detections come through the subscriber callback

    return detections;
}

void CAprilTagDetector::PublishDetections(const std::vector<SAprilTagData> &aDetections)
{
    // Create detection array message
    apriltag_msgs::msg::AprilTagDetectionArray msgArray;
    msgArray.header.stamp = this->now();
    msgArray.header.frame_id = "camera_frame";

    // Convert internal detection format to ROS message format
    for (const auto &sDetection : aDetections) {
        apriltag_msgs::msg::AprilTagDetection msgDetection;
        
        msgDetection.id = sDetection.mTagId;
        msgDetection.pose.pose.pose = sDetection.mPose;
        msgDetection.pose.header.stamp = sDetection.mTimestamp;
        msgDetection.pose.header.frame_id = "camera_frame";

        msgArray.detections.push_back(msgDetection);
    }

    // Publish the array
    if (!msgArray.detections.empty()) {
        mpTagPublisher->publish(msgArray);
    }
}

std::vector<cv::Point2d> CAprilTagDetector::ExtractCorners(
    const apriltag_msgs::msg::AprilTagDetection &aDetection) const
{
    std::vector<cv::Point2d> corners;

    // AprilTag detections contain corner information
    // Extract the four corners from the detection message
    // Note: Actual implementation depends on apriltag_msgs format
    
    return corners;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto cAprilTagDetector = std::make_shared<CAprilTagDetector>();
    rclcpp::spin(cAprilTagDetector);
    rclcpp::shutdown();
    return 0;
}