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

/**
 * @class CAprilTagDetector
 * @brief Independent AprilTag detection using native apriltag library.
 * @details Detects 16h5 family tags, extracts pose/orientation, shows GUI visualization,
 *          and publishes detection data for downstream nodes (like color detection).
 *          Manages apriltag detector lifecycle and native library resources.
 */
class CAprilTagDetector : public CImageProcessorNode
{
    public:
        // ====================================================================
        /// @name Constructor & Destructor
        // ====================================================================
        /// @{
        
        /**
         * @brief Constructor - Initializes native AprilTag detector for 16h5 family.
         */
        CAprilTagDetector();

        /**
         * @brief Destructor - Cleans up apriltag detector and family resources.
         */
        ~CAprilTagDetector();
        
        /// @}

    protected:
        // ====================================================================
        /// @name Core Processing (Override from Base Class)
        // ====================================================================
        /// @{
        
        /**
         * @brief ProcessImage (override) - Detects 16h5 AprilTags and publishes results with visualization.
         * @param aImage OpenCV image in BGR8 format
         * @param aTimestamp timestamp from image message
         */
        void ProcessImage(const cv::Mat &aImage, 
                         const rclcpp::Time &aTimestamp) override;
        
        /// @}

    private:
        // ====================================================================
        /// @name Native AprilTag Detection Methods
        // ====================================================================
        /// @{
        
        /**
         * @brief Sets up native apriltag detector with 16h5 family.
         * @return true if successful, false otherwise
         */
        bool InitializeDetector();

        /**
         * @brief Performs native AprilTag detection on grayscale image.
         * @param aGrayImage input image in grayscale format
         * @return apriltag detections from native library
         */
        zarray_t* DetectTagsNative(const cv::Mat &aGrayImage);

        /**
         * @brief Converts OpenCV grayscale image to apriltag image_u8 format.
         * @param aGrayImage OpenCV grayscale image
         * @return image_u8 structure for apriltag processing
         */
        image_u8_t* ConvertToImageU8(const cv::Mat &aGrayImage);
        
        /// @}

        // ====================================================================
        /// @name Data Conversion & Processing Methods
        // ====================================================================
        /// @{
        
        /**
         * @brief Converts native apriltag detection to ROS message format.
         * @param aDetection native apriltag_detection_t structure
         * @param aTimestamp detection timestamp
         * @return ROS AprilTagDetection message
         */
        apriltag_msgs::msg::AprilTagDetection ConvertDetectionToROS(
            apriltag_detection_t* aDetection, const rclcpp::Time &aTimestamp);

        /**
         * @brief Computes tag orientation from homography matrix.
         * @param aDetection native apriltag detection with homography
         * @return roll, pitch, yaw angles in radians
         */
        std::vector<double> CalculateOrientation(apriltag_detection_t* aDetection);
        
        /// @}

        // ====================================================================
        /// @name Visualization & Output Methods
        // ====================================================================
        /// @{
        
        /**
         * @brief Prints comprehensive detection info including ID, position, orientation.
         * @param aDetection ROS detection message
         * @param aOrientation roll, pitch, yaw angles [radians]
         */
        void PrintDetectionInfo(const apriltag_msgs::msg::AprilTagDetection &aDetection,
                               const std::vector<double> &aOrientation);

        /**
         * @brief Draws green bounding box around detected tag with ID label.
         * @param aImage image to annotate [in-place modification]
         * @param aDetection ROS detection message with corners
         */
        void DrawDetectionBox(cv::Mat &aImage, 
                             const apriltag_msgs::msg::AprilTagDetection &aDetection);

        /**
         * @brief Displays image with detection overlays in GUI window.
         * @param aImage annotated image to display
         */
        void ShowVisualizationWindow(const cv::Mat &aImage);

        /**
         * @brief Publishes detection array to ROS topic.
         * @param aDetections vector of ROS detection messages
         * @param aTimestamp detection timestamp
         */
        void PublishDetections(
            const std::vector<apriltag_msgs::msg::AprilTagDetection> &aDetections,
            const rclcpp::Time &aTimestamp);
            
        /// @}

        // ====================================================================
        /// @name Member Variables - ROS Communication
        // ====================================================================
        /// @{

        rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr mpDetectionPublisher;
        ///< Publishes AprilTag detections; owned by this node
        
        /// @}

        // ====================================================================
        /// @name Member Variables - Native AprilTag Library
        // ====================================================================
        /// @{

        apriltag_detector_t* mpAprilTagDetector;
        ///< Native AprilTag detector instance; owned by this node

        apriltag_family_t* mpTag16h5Family;
        ///< 16h5 tag family configuration; owned by this node
        
        /// @}

        // ====================================================================
        /// @name Member Variables - Configuration Flags
        // ====================================================================
        /// @{

        bool mShowVisualization;
        ///< Flag to enable/disable GUI visualization [ROS parameter]

        bool mPrintDetections;
        ///< Flag to enable/disable console detection output [ROS parameter]

        bool mEnableTemporalFiltering;
        ///< Flag to enable/disable temporal filtering [ROS parameter]

        std::string mVisualizationWindowName;
        ///< OpenCV window name for detection display
        
        /// @}

        // ====================================================================
        /// @name Member Variables - Temporal Filtering System
        // ====================================================================
        /// @{

        /**
         * @brief Temporal filtering for stable detections
         */
        struct TagTrackingInfo {
            rclcpp::Time first_seen;               ///< First detection timestamp
            rclcpp::Time last_seen;                ///< Last detection timestamp  
            int consecutive_frames;                ///< Number of consecutive frames detected
            apriltag_msgs::msg::AprilTagDetection last_detection;  ///< Last detection data
        };
        std::map<int, TagTrackingInfo> mTrackedTags;
        ///< Map of tag ID to tracking info for temporal filtering
        
        /// @}

        // ====================================================================
        /// @name Constants - ROS Communication
        // ====================================================================
        /// @{
        
        const std::string kDetectionOutputTopic = "/apriltag_detections";  ///< Output topic name
        const int kQueueSize = 10;  ///< ROS publisher queue depth
        
        /// @}

        // ====================================================================
        /// @name Constants - AprilTag Physical Properties
        // ====================================================================
        /// @{
        
        const double kTagSize = 0.0778; ///< Tag size in meters (77.8mm standard)
        
        /// @}

        // ====================================================================
        /// @name Constants - Detection Performance Parameters
        // ====================================================================
        /// @{
        
        const double kDecimation = 1.5; ///< Image decimation for performance (lower = more accurate)
        const double kBlur = 0.0; ///< Gaussian blur sigma (0 = disabled)
        const int kThreads = 1; ///< Number of detection threads
        const double kRefineEdges = 1; ///< Subpixel edge refinement
        const double kDecodeSharpening = 0.25; ///< Decode sharpening
        
        /// @}
        
        // ====================================================================
        /// @name Constants - Quality Thresholds
        // ====================================================================
        /// @{
        
        const double kMinDecisionMargin = 35.0; ///< Minimum decision margin (higher = stricter)
        const int kMaxHammingDistance = 0; ///< Maximum bit errors allowed (0 = perfect match only)
        
        /// @}
        
        // ====================================================================
        /// @name Constants - Temporal Filtering Thresholds
        // ====================================================================
        /// @{
        
        const double kMinDetectionDuration = 1.0; ///< Minimum duration in seconds for stable detection
        const double kMaxTimeSinceLastSeen = 1.0; ///< Max time gap before resetting tracking (seconds)
        
        /// @}
};

#endif // APRIL_TAG_DETECTOR_HPP