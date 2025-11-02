// ============================================================================
// MTRX3760 Project 2 - 
// File: AprilTagDetector.hpp
// Description: Header for AprilTag detection node for 16h5 family tags.
//              Defines independent AprilTag detection using native library
//              with pose extraction and GUI visualization capabilities.
// Author(s): Dylan George
// Last Edited: 2025-11-02
// ============================================================================

#ifndef APRIL_TAG_DETECTOR_HPP
#define APRIL_TAG_DETECTOR_HPP

#include "Camera/ImageProcessor_Node.hpp"
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>

// Native AprilTag library headers
extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/tag16h5.h>
#include <apriltag/common/zarray.h>
#include <apriltag/common/image_u8.h>
}

// CAprilTagDetector
// Independent AprilTag detection using native apriltag library.
// Detects 16h5 family tags, extracts pose/orientation, shows GUI visualization,
// and publishes detection data for downstream nodes (like color detection).
// Ownership: Manages apriltag detector lifecycle and native library resources.
class CAprilTagDetector : public CImageProcessorNode
{
    public:
        // Constructor
        // Initializes native AprilTag detector for 16h5 family.
        CAprilTagDetector();

        // Destructor
        // Cleans up apriltag detector and family resources.
        ~CAprilTagDetector();

    protected:
        // ProcessImage (override)
        // Detects 16h5 AprilTags and publishes results with visualization.
        // aImage: OpenCV image in BGR8 format
        // aTimestamp: timestamp from image message
        void ProcessImage(const cv::Mat &aImage, 
                         const rclcpp::Time &aTimestamp) override;

    private:
        // InitializeDetector
        // Sets up native apriltag detector with 16h5 family.
        // Returns true if successful, false otherwise.
        bool InitializeDetector();

        // DetectTagsNative
        // Performs native AprilTag detection on grayscale image.
        // aGrayImage: input image in grayscale format
        // Returns apriltag detections from native library.
        zarray_t* DetectTagsNative(const cv::Mat &aGrayImage);

        // ConvertToImageU8
        // Converts OpenCV grayscale image to apriltag image_u8 format.
        // aGrayImage: OpenCV grayscale image
        // Returns image_u8 structure for apriltag processing.
        image_u8_t* ConvertToImageU8(const cv::Mat &aGrayImage);

        // ConvertDetectionToROS
        // Converts native apriltag detection to ROS message format.
        // aDetection: native apriltag_detection_t structure
        // aTimestamp: detection timestamp
        // Returns ROS AprilTagDetection message.
        apriltag_msgs::msg::AprilTagDetection ConvertDetectionToROS(
            apriltag_detection_t* aDetection, const rclcpp::Time &aTimestamp);

        // CalculateOrientation
        // Computes tag orientation from homography matrix.
        // aDetection: native apriltag detection with homography
        // Returns roll, pitch, yaw angles in radians.
        std::vector<double> CalculateOrientation(apriltag_detection_t* aDetection);

        // PrintDetectionInfo
        // Prints comprehensive detection info including ID, position, orientation.
        // aDetection: ROS detection message
        // aOrientation: roll, pitch, yaw angles [radians]
        void PrintDetectionInfo(const apriltag_msgs::msg::AprilTagDetection &aDetection,
                               const std::vector<double> &aOrientation);

        // DrawDetectionBox
        // Draws green bounding box around detected tag with ID label.
        // aImage: image to annotate [in-place modification]
        // aDetection: ROS detection message with corners
        void DrawDetectionBox(cv::Mat &aImage, 
                             const apriltag_msgs::msg::AprilTagDetection &aDetection);

        // ShowVisualizationWindow
        // Displays image with detection overlays in GUI window.
        // aImage: annotated image to display
        void ShowVisualizationWindow(const cv::Mat &aImage);

        // PublishDetections
        // Publishes detection array to ROS topic.
        // aDetections: vector of ROS detection messages
        // aTimestamp: detection timestamp
        void PublishDetections(
            const std::vector<apriltag_msgs::msg::AprilTagDetection> &aDetections,
            const rclcpp::Time &aTimestamp);

        // Member Variables

        rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr mpDetectionPublisher;
        // Publishes AprilTag detections; owned by this node

        apriltag_detector_t* mpAprilTagDetector;
        // Native AprilTag detector instance; owned by this node

        apriltag_family_t* mpTag16h5Family;
        // 16h5 tag family configuration; owned by this node

        bool mShowVisualization;
        // Flag to enable/disable GUI visualization [ROS parameter]

        bool mPrintDetections;
        // Flag to enable/disable console detection output [ROS parameter]

        bool mEnableTemporalFiltering;
        // Flag to enable/disable temporal filtering [ROS parameter]

        std::string mVisualizationWindowName;
        // OpenCV window name for detection display

        // Temporal filtering for stable detections
        struct TagTrackingInfo {
            rclcpp::Time first_seen;
            rclcpp::Time last_seen;
            int consecutive_frames;
            apriltag_msgs::msg::AprilTagDetection last_detection;
        };
        std::map<int, TagTrackingInfo> mTrackedTags;
        // Map of tag ID to tracking info for temporal filtering

        // Constants
        const std::string kDetectionOutputTopic = "/apriltag_detections";
        const int kQueueSize = 10;
        const double kTagSize = 0.0778; // Tag size in meters (77.8mm standard)
        const double kDecimation = 1.5; // Image decimation for performance (lower = more accurate)
        const double kBlur = 0.0; // Gaussian blur sigma (0 = disabled)
        const int kThreads = 1; // Number of detection threads
        const double kRefineEdges = 1; // Subpixel edge refinement
        const double kDecodeSharpening = 0.25; // Decode sharpening
        
        // Quality thresholds to reduce false positives
        const double kMinDecisionMargin = 35.0; // Minimum decision margin (higher = stricter)
        const int kMaxHammingDistance = 0; // Maximum bit errors allowed (0 = perfect match only)
        
        // Temporal filtering thresholds
        const double kMinDetectionDuration = 1.0; // Minimum duration in seconds for stable detection
        const double kMaxTimeSinceLastSeen = 1.0; // Max time gap before resetting tracking (seconds)
};

#endif // APRIL_TAG_DETECTOR_HPP