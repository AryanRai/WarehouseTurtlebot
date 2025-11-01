// ============================================================================
// File: CAprilTagDetector.hpp
// Description: AprilTag detection node that inherits from CImageProcessorNode.
//              Detects 16h5 AprilTags in camera images and publishes their
//              pose, ID, and corner positions for downstream colour analysis.
// Author(s): Dylan George
// Last Edited: 2025-10-27
// ============================================================================

#ifndef APRIL_TAG_DETECTOR_HPP
#define APRIL_TAG_DETECTOR_HPP

#include "ImageProcessor_Node.hpp"
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>

// SAprilTagData
// Stores detection results for a single AprilTag including pose and corner info.
struct SAprilTagData
{
    int mTagId;                        // AprilTag ID number
    geometry_msgs::msg::Pose mPose;    // 6DOF pose of tag in camera frame
    std::vector<cv::Point2d> mCorners; // Four corner positions in image [pixels]
    rclcpp::Time mTimestamp;           // Detection timestamp
};

// CAprilTagDetector
// Detects 16h5 AprilTags in images and publishes detection data. Inherits image
// processing infrastructure from CImageProcessorNode. Uses apriltag_ros package
// for detection and publishes results for colour analysis.
// Ownership: Manages AprilTag detector instance and ROS publisher.
class CAprilTagDetector : public CImageProcessorNode
{
    public:
        // Constructor
        // Initialises AprilTag detector and creates publisher for detection results.
        CAprilTagDetector();

        // Destructor
        // Cleans up detector resources.
        ~CAprilTagDetector();

    protected:
        // ProcessImage (override)
        // Detects AprilTags in the provided image and publishes results.
        // aImage: OpenCV image in BGR8 format
        // aTimestamp: timestamp from original image message
        void ProcessImage(const cv::Mat &aImage, 
                         const rclcpp::Time &aTimestamp) override;

    private:
        // DetectTags
        // Performs AprilTag detection on a grayscale image.
        // aGrayImage: grayscale OpenCV image
        // aTimestamp: timestamp for detected tags
        // Returns vector of detected tag data.
        std::vector<SAprilTagData> DetectTags(const cv::Mat &aGrayImage,
                                              const rclcpp::Time &aTimestamp);

        // PublishDetections
        // Publishes detected AprilTag data to ROS topic.
        // aDetections: vector of tag detections to publish
        void PublishDetections(const std::vector<SAprilTagData> &aDetections);

        // ConvertToGrayscale
        // Converts BGR image to grayscale for AprilTag detection.
        // aColorImage: input BGR8 image
        // Returns grayscale image.
        cv::Mat ConvertToGrayscale(const cv::Mat &aColorImage) const;

        // ExtractCorners
        // Extracts corner positions from AprilTag detection result.
        // aDetection: detection result from apriltag library
        // Returns vector of four corner points.
        std::vector<cv::Point2d> ExtractCorners(
            const apriltag_msgs::msg::AprilTagDetection &aDetection) const;

        // Member Variables

        rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr mpTagPublisher;
        // Publishes AprilTag detections; owned by this node, created in ctor

        rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr mpTagSubscriber;
        // Subscribes to apriltag_ros detections; owned by this node, created in ctor

        std::vector<SAprilTagData> mLastDetections;
        // Most recent tag detections; cached for potential reprocessing

        const std::string kTagDetectionTopic = "/apriltag/detections";
        // Topic for publishing tag detection data [string constant]

        const std::string kAprilTagRosTopic = "/apriltag_ros/detections";
        // Input topic from apriltag_ros package [string constant]

        const int kPublisherQueueSize = 10;
        // Publisher queue depth [messages]
};

#endif // APRIL_TAG_DETECTOR_HPP