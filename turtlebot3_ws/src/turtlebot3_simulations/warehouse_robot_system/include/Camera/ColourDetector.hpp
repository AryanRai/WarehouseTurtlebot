// ============================================================================
// File: CColourDetector.hpp
// Description: Colour detection node that analyses regions around detected
//              AprilTags to identify damage type. Uses HSV colour space to
//              classify damage (green=mould, blue=water, red=blood).
//              Includes interactive calibration mode with visual feedback.
// Author(s): Dylan George
// Last Edited: 2025-11-01
// ============================================================================

#ifndef COLOUR_DETECTOR_HPP
#define COLOUR_DETECTOR_HPP

#include "Camera/ImageProcessor_Node.hpp"
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

// eDamageType
// Enumeration of damage types identified by colour analysis.
enum eDamageType
{
    DAMAGE_NONE = 0,
    DAMAGE_MOULD = 1,   // Green colour
    DAMAGE_WATER = 2,   // Blue colour
    DAMAGE_BLOOD = 3,   // Red colour
    DAMAGE_UNKNOWN = -1  // Unrecognised colour
};

// SDamageReport
// Complete damage report including location, type, and identification data.
struct SDamageReport
{
    int mTagId;                               // AprilTag ID marking damage location
    geometry_msgs::msg::Pose mLocalPose;     // Pose relative to robot when detected
    geometry_msgs::msg::Pose mGlobalPose;    // Global world frame pose (updated by robot)
    eDamageType mDamageType;                  // Classification of damage type
    rclcpp::Time mTimestamp;                  // Time of detection
    double mConfidence;                       // Detection confidence [0.0 to 1.0]
};

// CColourDetector
// Analyses colour around AprilTags to classify damage type. Subscribes to both
// camera images and AprilTag detections, performs HSV-based colour analysis,
// and publishes damage reports for warehouse inspection system.
// Ownership: Manages subscribers, publishers, and processing objects.
class CColourDetector : public CImageProcessorNode
{
    public:
        // Constructor
        // Initialises colour detector, sets up subscribers and publishers.
        CColourDetector();

        // Destructor
        // Cleans up resources.
        ~CColourDetector();

    protected:
        // ProcessImage (override)
        // Stores latest image for processing when AprilTag detections arrive.
        // aImage: OpenCV image in BGR8 format
        // aTimestamp: timestamp from image message
        void ProcessImage(const cv::Mat &aImage, 
                          const rclcpp::Time &aTimestamp) override;

    private:
        // TagDetectionCallback
        // Processes AprilTag detections and analyses colour around each tag.
        // aMsg: array of AprilTag detections
        void TagDetectionCallback(
            const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr aMsg);

        // AnalyseColourAroundTag
        // Examines pixels surrounding an AprilTag to determine damage type.
        // aImage: source image in BGR format
        // aTagCenterX: X coordinate of tag center [pixels]
        // aTagCenterY: Y coordinate of tag center [pixels]
        // aTagSize: approximate size of tag [pixels]
        // Returns detected damage type.
        eDamageType AnalyseColourAroundTag(const cv::Mat &aImage,
                                           int aTagCenterX,
                                           int aTagCenterY,
                                           int aTagSize);

        // ExtractSamplingRegions
        // Defines four sampling regions (above, below, left, right) around tag.
        // aTagCenterX: X coordinate of tag center [pixels]
        // aTagCenterY: Y coordinate of tag center [pixels]
        // aTagSize: size of tag [pixels]
        // aImageWidth: image width for boundary checking [pixels]
        // aImageHeight: image height for boundary checking [pixels]
        // Returns vector of four sampling rectangles.
        std::vector<cv::Rect> ExtractSamplingRegions(int aTagCenterX,
                                                     int aTagCenterY,
                                                     int aTagSize,
                                                     int aImageWidth,
                                                     int aImageHeight);

        // ClassifyColour
        // Classifies dominant colour in a region using HSV analysis.
        // aRegion: image region to analyse
        // Returns detected damage type based on colour.
        eDamageType ClassifyColour(const cv::Mat &aRegion);

        // ConvertToHSV
        // Converts BGR image region to HSV colour space.
        // aBGRImage: input image in BGR format
        // Returns image in HSV format.
        cv::Mat ConvertToHSV(const cv::Mat &aBGRImage) const;

        // CountColouredPixels
        // Counts pixels within specified HSV range.
        // aHSVImage: image in HSV format
        // aLowerBound: lower HSV threshold
        // aUpperBound: upper HSV threshold
        // Returns count of pixels in range.
        int CountColouredPixels(const cv::Mat &aHSVImage,
                                const cv::Scalar &aLowerBound,
                                const cv::Scalar &aUpperBound) const;

        // PublishDamageReport
        // Publishes damage report to output topic.
        // aReport: complete damage report data
        void PublishDamageReport(const SDamageReport &aReport);

        // DamageTypeToString
        // Converts damage type enum to human-readable string.
        // aDamageType: damage type to convert
        // Returns string representation.
        std::string DamageTypeToString(eDamageType aDamageType) const;

        // Calibration UI: Display annotated image with sampling regions and HSV stats
        // aImage: current frame [BGR]
        // aDetections: AprilTag detections to overlay
        void DisplayCalibrationWindow(
            const cv::Mat &aImage,
            const std::vector<apriltag_msgs::msg::AprilTagDetection> &aDetections);

        // Draw four sampling rectangles around tag and colour-code borders by class
        // aImage: image to annotate [in-place]
        // aTagCenterX/Y: tag centre in pixels
        // aTagSize: approx tag size in pixels
        void DrawSamplingRegions(cv::Mat &aImage,
                                 int aTagCenterX,
                                 int aTagCenterY,
                                 int aTagSize);

        // Overlay mean HSV statistics above a region
        // aImage: image to annotate [in-place]
        // aRegion: ROI rectangle in pixels
        // aLabel: text label to prefix
        void OverlayHSVStatistics(cv::Mat &aImage,
                                  const cv::Rect &aRegion,
                                  const std::string &aLabel);

        // Save current HSV thresholds and sampling params to YAML
        // aFilePath: destination file path (supports '~' home expansion)
        void SaveCalibrationToYAML(const std::string &aFilePath);

        // Handle keyboard input for calibration controls
        // aKeyCode: key from cv::waitKey()
        void HandleKeyboardInput(int aKeyCode);

        // Member Variables

        rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr mpTagSubscriber;
        // Subscribes to AprilTag detections; owned by this node, created in ctor

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mpDamagePublisher;
        // Publishes damage reports as JSON strings; owned by this node, created in ctor

        cv::Mat mLatestImage;
        // Most recent camera image; stored for processing when tags detected

        rclcpp::Time mLatestImageTimestamp;
        // Timestamp of mLatestImage

        bool mHasImage;
        // Flag indicating whether valid image is available [boolean]

        // Calibration mode flag (ROS parameter: calibration_mode)
        bool mCalibrationMode;
        // Calibration window name string
        std::string mCalibrationWindowName;

        // Cache last received detections for calibration display
        std::vector<apriltag_msgs::msg::AprilTagDetection> mLastDetections;

                const std::string kTagInputTopic = "/apriltag_detections";
        // Input topic for AprilTag detections [string constant]

        const std::string kDamageOutputTopic = "/warehouse/damage_reports";

        const int kQueueSize = 10;

        const int kSamplingOffset = 10;

        const int kSamplingWidth = 20;

        const int kSamplingHeight = 20;

        const double kColourZoneScale = 0.4; // Zone size relative to tag size
        
        const cv::Scalar kGreenLower = cv::Scalar(35, 40, 40);        
        const cv::Scalar kGreenUpper = cv::Scalar(85, 255, 255);

        const cv::Scalar kBlueLower = cv::Scalar(90, 50, 50);        
        const cv::Scalar kBlueUpper = cv::Scalar(130, 255, 255);

        const cv::Scalar kRedLower1 = cv::Scalar(0, 50, 50);        
        const cv::Scalar kRedUpper1 = cv::Scalar(10, 255, 255);        
        const cv::Scalar kRedLower2 = cv::Scalar(170, 50, 50);        
        const cv::Scalar kRedUpper2 = cv::Scalar(180, 255, 255);

        const double kMinConfidenceThreshold = 0.5;

        const int kMinPixelCount = 50;
};

#endif // COLOUR_DETECTOR_HPP