// ============================================================================
// MTRX3760 Project 2 - 
// File: ColourDetector.hpp
// Description: Header for CColourDetector class. Defines colour detection node
//              that analyses regions around AprilTags to identify damage types
//              using HSV colour space classification with calibration mode.
// Author(s): Dylan George
// Last Edited: 2025-11-02
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

/**
 * @enum eDamageType
 * @brief Enumeration of damage types identified by colour analysis.
 */
enum eDamageType
{
    DAMAGE_NONE = 0,      ///< No damage detected
    DAMAGE_MOULD = 1,     ///< Green colour - mould damage
    DAMAGE_WATER = 2,     ///< Blue colour - water damage  
    DAMAGE_BLOOD = 3,     ///< Red colour - blood damage
    DAMAGE_UNKNOWN = -1   ///< Unrecognised colour
};

/**
 * @struct SDamageReport
 * @brief Complete damage report including location, type, and identification data.
 */
struct SDamageReport
{
    int mTagId;                               ///< AprilTag ID marking damage location
    geometry_msgs::msg::Pose mLocalPose;     ///< Pose relative to robot when detected
    geometry_msgs::msg::Pose mGlobalPose;    ///< Global world frame pose (updated by robot)
    eDamageType mDamageType;                  ///< Classification of damage type
    rclcpp::Time mTimestamp;                  ///< Time of detection
    double mConfidence;                       ///< Detection confidence [0.0 to 1.0]
};

/**
 * @class CColourDetector
 * @brief Analyses colour around AprilTags to classify damage type.
 * @details Subscribes to both camera images and AprilTag detections, performs HSV-based 
 *          colour analysis, and publishes damage reports for warehouse inspection system.
 */
class CColourDetector : public CImageProcessorNode
{
    public:
        // ====================================================================
        /// @name Constructor & Destructor
        // ====================================================================
        /// @{
        
        /**
         * @brief Constructor - Initialises colour detector, sets up subscribers and publishers.
         */
        CColourDetector();

        /**
         * @brief Destructor - Cleans up resources.
         */
        ~CColourDetector();
        
        /// @}

    protected:
        // ====================================================================
        /// @name Core Processing (Override from Base Class)
        // ====================================================================
        /// @{
        
        /**
         * @brief ProcessImage (override) - Stores latest image for processing when AprilTag detections arrive.
         * @param aImage OpenCV image in BGR8 format
         * @param aTimestamp timestamp from image message
         */
        void ProcessImage(const cv::Mat &aImage, 
                          const rclcpp::Time &aTimestamp) override;
        
        /// @}

    private:
        // ====================================================================
        /// @name Core Detection Methods
        // ====================================================================
        /// @{
        
        /**
         * @brief Processes AprilTag detections and analyses colour around each tag.
         * @param aMsg array of AprilTag detections
         */
        void TagDetectionCallback(
            const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr aMsg);

        /**
         * @brief Examines pixels surrounding an AprilTag to determine damage type.
         * @param aImage source image in BGR format
         * @param aTagCenterX X coordinate of tag center [pixels]
         * @param aTagCenterY Y coordinate of tag center [pixels]
         * @param aTagSize approximate size of tag [pixels]
         * @return detected damage type
         */
        eDamageType AnalyseColourAroundTag(const cv::Mat &aImage,
                                           int aTagCenterX,
                                           int aTagCenterY,
                                           int aTagSize);
        
        /// @}

        // ====================================================================
        /// @name Region Extraction & Analysis Methods
        // ====================================================================
        /// @{
        
        /**
         * @brief Defines four sampling regions (above, below, left, right) around tag.
         * @param aTagCenterX X coordinate of tag center [pixels]
         * @param aTagCenterY Y coordinate of tag center [pixels]
         * @param aTagSize size of tag [pixels]
         * @param aImageWidth image width for boundary checking [pixels]
         * @param aImageHeight image height for boundary checking [pixels]
         * @return vector of four sampling rectangles
         */
        std::vector<cv::Rect> ExtractSamplingRegions(int aTagCenterX,
                                                     int aTagCenterY,
                                                     int aTagSize,
                                                     int aImageWidth,
                                                     int aImageHeight);

        /**
         * @brief Classifies dominant colour in a region using HSV analysis.
         * @param aRegion image region to analyse
         * @return detected damage type based on colour
         */
        eDamageType ClassifyColour(const cv::Mat &aRegion);
        
        /// @}

        // ====================================================================
        /// @name Image Processing Utility Methods
        // ====================================================================
        /// @{
        
        /**
         * @brief Converts BGR image region to HSV colour space.
         * @param aBGRImage input image in BGR format
         * @return image in HSV format
         */
        cv::Mat ConvertToHSV(const cv::Mat &aBGRImage) const;

        /**
         * @brief Counts pixels within specified HSV range.
         * @param aHSVImage image in HSV format
         * @param aLowerBound lower HSV threshold
         * @param aUpperBound upper HSV threshold
         * @return count of pixels in range
         */
        int CountColouredPixels(const cv::Mat &aHSVImage,
                                const cv::Scalar &aLowerBound,
                                const cv::Scalar &aUpperBound) const;
        
        /// @}

        // ====================================================================
        /// @name Output & Reporting Methods
        // ====================================================================
        /// @{
        
        /**
         * @brief Publishes damage report to output topic.
         * @param aReport complete damage report data
         */
        void PublishDamageReport(const SDamageReport &aReport);

        /**
         * @brief Converts damage type enum to human-readable string.
         * @param aDamageType damage type to convert
         * @return string representation
         */
        std::string DamageTypeToString(eDamageType aDamageType) const;
        
        /// @}

        // ====================================================================
        /// @name Calibration & Visualization Methods
        // ====================================================================
        /// @{
        
        /**
         * @brief Calibration UI: Display annotated image with sampling regions and HSV stats
         * @param aImage current frame [BGR]
         * @param aDetections AprilTag detections to overlay
         */
        void DisplayCalibrationWindow(
            const cv::Mat &aImage,
            const std::vector<apriltag_msgs::msg::AprilTagDetection> &aDetections);

        /**
         * @brief Draw four sampling rectangles around tag and colour-code borders by class
         * @param aImage image to annotate [in-place]
         * @param aTagCenterX tag centre X coordinate in pixels
         * @param aTagCenterY tag centre Y coordinate in pixels
         * @param aTagSize approx tag size in pixels
         */
        void DrawSamplingRegions(cv::Mat &aImage,
                                 int aTagCenterX,
                                 int aTagCenterY,
                                 int aTagSize);

        /**
         * @brief Overlay mean HSV statistics above a region
         * @param aImage image to annotate [in-place]
         * @param aRegion ROI rectangle in pixels
         * @param aLabel text label to prefix
         */
        void OverlayHSVStatistics(cv::Mat &aImage,
                                  const cv::Rect &aRegion,
                                  const std::string &aLabel);

        /**
         * @brief Save current HSV thresholds and sampling params to YAML
         * @param aFilePath destination file path (supports '~' home expansion)
         */
        void SaveCalibrationToYAML(const std::string &aFilePath);

        /**
         * @brief Handle keyboard input for calibration controls
         * @param aKeyCode key from cv::waitKey()
         */
        void HandleKeyboardInput(int aKeyCode);
        
        /// @}

        // ====================================================================
        /// @name Member Variables - ROS Communication
        // ====================================================================
        /// @{

        rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr mpTagSubscriber;
        ///< Subscribes to AprilTag detections; owned by this node, created in ctor

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mpDamagePublisher;
        ///< Publishes damage reports as JSON strings; owned by this node, created in ctor
        
        /// @}

        // ====================================================================
        /// @name Member Variables - Image Processing State
        // ====================================================================
        /// @{

        cv::Mat mLatestImage;
        ///< Most recent camera image; stored for processing when tags detected

        rclcpp::Time mLatestImageTimestamp;
        ///< Timestamp of mLatestImage

        bool mHasImage;
        ///< Flag indicating whether valid image is available [boolean]
        
        /// @}

        // ====================================================================
        /// @name Member Variables - Calibration Mode
        // ====================================================================
        /// @{

        bool mCalibrationMode;
        ///< Calibration mode flag (ROS parameter: calibration_mode)

        std::string mCalibrationWindowName;
        ///< Calibration window name string

        std::vector<apriltag_msgs::msg::AprilTagDetection> mLastDetections;
        ///< Cache last received detections for calibration display
        
        /// @}

        // ====================================================================
        /// @name Constants - ROS Communication
        // ====================================================================
        /// @{

        const std::string kTagInputTopic = "/apriltag_detections";  ///< Input topic for AprilTag detections
        const std::string kDamageOutputTopic = "/warehouse/damage_reports";  ///< Output topic for damage reports
        const int kQueueSize = 10;  ///< ROS publisher/subscriber queue depth [messages]
        
        /// @}

        // ====================================================================
        /// @name Constants - Sampling Parameters
        // ====================================================================
        /// @{

        const int kSamplingOffset = 10;  ///< Distance from tag edge to start sampling [pixels]
        const int kSamplingWidth = 20;  ///< Width of sampling regions [pixels]
        const int kSamplingHeight = 20;  ///< Height of sampling regions [pixels]
        const double kColourZoneScale = 0.4;  ///< Zone size relative to tag size [ratio]
        
        /// @}
        
        // ====================================================================
        /// @name Constants - HSV Color Thresholds
        // ====================================================================
        /// @{
        
        const cv::Scalar kGreenLower = cv::Scalar(35, 40, 40);   ///< Green lower HSV threshold     
        const cv::Scalar kGreenUpper = cv::Scalar(85, 255, 255);  ///< Green upper HSV threshold

        const cv::Scalar kBlueLower = cv::Scalar(90, 50, 50);    ///< Blue lower HSV threshold    
        const cv::Scalar kBlueUpper = cv::Scalar(130, 255, 255);  ///< Blue upper HSV threshold

        const cv::Scalar kRedLower1 = cv::Scalar(0, 50, 50);     ///< Red lower HSV threshold (range 1)    
        const cv::Scalar kRedUpper1 = cv::Scalar(10, 255, 255);   ///< Red upper HSV threshold (range 1)    
        const cv::Scalar kRedLower2 = cv::Scalar(170, 50, 50);    ///< Red lower HSV threshold (range 2)    
        const cv::Scalar kRedUpper2 = cv::Scalar(180, 255, 255);  ///< Red upper HSV threshold (range 2)
        
        /// @}

        // ====================================================================
        /// @name Constants - Detection Thresholds
        // ====================================================================
        /// @{

        const double kMinConfidenceThreshold = 0.5;  ///< Minimum confidence level for damage classification [0.0 to 1.0]
        const int kMinPixelCount = 50;  ///< Minimum number of colored pixels required for detection [pixels]
        
        /// @}
};

#endif // COLOUR_DETECTOR_HPP