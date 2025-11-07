// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: AprilTagDetector.cpp
// Author(s): Dylan George
//
// Description: Implementation of AprilTag 16h5 detector node using native
// AprilTag library.

#include "Camera/AprilTagDetector.hpp"
#include <cmath>
#include <iomanip>
#include <map>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <set>

AprilTagDetector::AprilTagDetector()
    : ImageProcessorNode("apriltag_detector_node"),
      mpAprilTagDetector(nullptr), mpTag16h5Family(nullptr),
      mVisualizationWindowName("AprilTag 16h5 Detection")
{
    // Initialize native AprilTag detector
    if (!InitializeDetector())
    {
        RCLCPP_ERROR(GetLogger(), "Failed to initialize AprilTag detector");
        return;
    }

    // Create publisher for AprilTag detections
    mpDetectionPublisher =
        this->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
            kDetectionOutputTopic, kQueueSize);

    // Declare ROS parameters
    mShowVisualization =
        this->declare_parameter<bool>("show_visualization", true);
    mPrintDetections = this->declare_parameter<bool>("print_detections", true);
    mEnableTemporalFiltering =
        this->declare_parameter<bool>("enable_temporal_filtering", false);

    RCLCPP_INFO(GetLogger(), "️ AprilTag 16h5 detector initialized");
    RCLCPP_INFO(GetLogger(), "Publishing to: %s",
                kDetectionOutputTopic.c_str());
    RCLCPP_INFO(GetLogger(), "Visualization: %s",
                mShowVisualization ? "ON" : "OFF");
    RCLCPP_INFO(GetLogger(), "Console output: %s",
                mPrintDetections ? "ON" : "OFF");
    RCLCPP_INFO(GetLogger(), "Temporal filtering: %s",
                mEnableTemporalFiltering ? "ON" : "OFF");
}

AprilTagDetector::~AprilTagDetector()
{
    RCLCPP_INFO(GetLogger(), "AprilTag detector shutting down");

    // Clean up native AprilTag resources
    if (mpAprilTagDetector)
    {
        apriltag_detector_destroy(mpAprilTagDetector);
    }
    if (mpTag16h5Family)
    {
        tag16h5_destroy(mpTag16h5Family);
    }

    // Close OpenCV windows
    if (mShowVisualization)
    {
        cv::destroyAllWindows();
    }
}

bool AprilTagDetector::InitializeDetector()
{
    try
    {
        // Create 16h5 tag family
        mpTag16h5Family = tag16h5_create();
        if (!mpTag16h5Family)
        {
            RCLCPP_ERROR(GetLogger(), "Failed to create 16h5 tag family");
            return false;
        }

        // Create AprilTag detector
        mpAprilTagDetector = apriltag_detector_create();
        if (!mpAprilTagDetector)
        {
            RCLCPP_ERROR(GetLogger(), "Failed to create AprilTag detector");
            tag16h5_destroy(mpTag16h5Family);
            mpTag16h5Family = nullptr;
            return false;
        }

        // Configure detector parameters
        mpAprilTagDetector->quad_decimate = kDecimation;
        mpAprilTagDetector->quad_sigma = kBlur;
        mpAprilTagDetector->nthreads = kThreads;
        mpAprilTagDetector->refine_edges = kRefineEdges;
        mpAprilTagDetector->decode_sharpening = kDecodeSharpening;

        // Add 16h5 family to detector
        apriltag_detector_add_family(mpAprilTagDetector, mpTag16h5Family);

        RCLCPP_INFO(GetLogger(),
                    " Native AprilTag detector configured for 16h5 family");
        return true;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(GetLogger(),
                     "Exception during detector initialization: %s", e.what());
        return false;
    }
}

void AprilTagDetector::ProcessImage(const cv::Mat &aImage,
                                     const rclcpp::Time &aTimestamp)
{
    if (aImage.empty())
    {
        RCLCPP_WARN(GetLogger(), "Empty image received");
        return;
    }

    // Convert to grayscale
    cv::Mat grayImage;
    if (aImage.channels() == 3)
    {
        cv::cvtColor(aImage, grayImage, cv::COLOR_BGR2GRAY);
    }
    else
    {
        grayImage = aImage.clone();
    }

    // Perform native AprilTag detection
    zarray_t *detections = DetectTagsNative(grayImage);
    if (!detections)
    {
        return;
    }

    // Track currently detected tag IDs in this frame
    std::set<int> currentFrameTagIds;

    // Convert to ROS messages and update tracking
    std::vector<apriltag_msgs::msg::AprilTagDetection> candidateDetections;
    std::vector<apriltag_msgs::msg::AprilTagDetection>
        immediateDetections; // For non-temporal mode
    cv::Mat visualizationImage;

    // Only clone image if we have detections and visualization is enabled
    bool hasDetections = zarray_size(detections) > 0;
    if (mShowVisualization && hasDetections)
    {
        visualizationImage = aImage.clone();
    }

    for (int i = 0; i < zarray_size(detections); i++)
    {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        // Filter out low-quality detections to reduce false positives
        if (det->decision_margin < kMinDecisionMargin)
        {
            RCLCPP_INFO_THROTTLE(
                GetLogger(), *this->get_clock(), 2000,
                " Rejected tag ID %d: low decision margin (%.1f < %.1f)",
                det->id, det->decision_margin, kMinDecisionMargin);
            continue;
        }

        if (det->hamming > kMaxHammingDistance)
        {
            RCLCPP_INFO_THROTTLE(
                GetLogger(), *this->get_clock(), 2000,
                " Rejected tag ID %d: high hamming distance (%d > %d)",
                det->id, det->hamming, kMaxHammingDistance);
            continue;
        }

        // Convert to ROS message
        apriltag_msgs::msg::AprilTagDetection rosDetection =
            ConvertDetectionToROS(det, aTimestamp);
        currentFrameTagIds.insert(det->id);

        // If temporal filtering is disabled, add to immediate detections
        if (!mEnableTemporalFiltering)
        {
            immediateDetections.push_back(rosDetection);

            // Calculate orientation
            std::vector<double> orientation = CalculateOrientation(det);

            // Print detection info if enabled
            if (mPrintDetections)
            {
                PrintDetectionInfo(rosDetection, orientation);
            }

            // Draw visualization if enabled
            if (mShowVisualization && !visualizationImage.empty())
            {
                DrawDetectionBox(visualizationImage, rosDetection);
            }

            continue; // Skip temporal tracking
        }

        // Update temporal tracking (only if enabled)
        auto it = mTrackedTags.find(det->id);
        if (it == mTrackedTags.end())
        {
            // First time seeing this tag
            TagTrackingInfo info;
            info.first_seen = aTimestamp;
            info.last_seen = aTimestamp;
            info.consecutive_frames = 1;
            info.last_detection = rosDetection;
            mTrackedTags[det->id] = info;

            RCLCPP_INFO(GetLogger(), "️ Started tracking tag ID %d", det->id);
        }
        else
        {
            // Tag seen before - check if continuous
            double time_since_last =
                (aTimestamp - it->second.last_seen).seconds();

            if (time_since_last > kMaxTimeSinceLastSeen)
            {
                // Gap too large, reset tracking
                it->second.first_seen = aTimestamp;
                it->second.consecutive_frames = 1;
                RCLCPP_INFO(GetLogger(),
                            " Reset tracking for tag ID %d (gap: %.2fs)",
                            det->id, time_since_last);
            }
            else
            {
                // Continuous detection
                it->second.consecutive_frames++;
            }

            it->second.last_seen = aTimestamp;
            it->second.last_detection = rosDetection;
        }

        candidateDetections.push_back(rosDetection);
    }

    // If temporal filtering is disabled, publish immediately
    if (!mEnableTemporalFiltering)
    {
        if (!immediateDetections.empty())
        {
            PublishDetections(immediateDetections, aTimestamp);
        }

        // Show visualization window
        if (mShowVisualization && !visualizationImage.empty())
        {
            ShowVisualizationWindow(visualizationImage);
        }

        // Clean up and return
        apriltag_detections_destroy(detections);
        return;
    }

    // Temporal filtering mode - continue with tracking logic
    // Remove tags that haven't been seen recently
    auto it = mTrackedTags.begin();
    while (it != mTrackedTags.end())
    {
        double time_since_last = (aTimestamp - it->second.last_seen).seconds();
        if (time_since_last > kMaxTimeSinceLastSeen)
        {
            RCLCPP_INFO(
                GetLogger(),
                "️ Removed tag ID %d from tracking (not seen for %.2fs)",
                it->first, time_since_last);
            it = mTrackedTags.erase(it);
        }
        else
        {
            ++it;
        }
    }

    // Only publish and visualize tags that have been stable for minimum
    // duration
    std::vector<apriltag_msgs::msg::AprilTagDetection> stableDetections;

    for (const auto &detection : candidateDetections)
    {
        auto it = mTrackedTags.find(detection.id);
        if (it != mTrackedTags.end())
        {
            double duration = (aTimestamp - it->second.first_seen).seconds();

            if (duration >= kMinDetectionDuration)
            {
                // Tag has been stable long enough
                stableDetections.push_back(detection);

                // Calculate orientation
                std::vector<double> orientation = {0.0, 0.0,
                                                   0.0}; // Simplified for now

                // Print detection info if enabled (throttled)
                if (mPrintDetections)
                {
                    PrintDetectionInfo(detection, orientation);
                }

                // Draw visualization if enabled
                if (mShowVisualization && !visualizationImage.empty())
                {
                    DrawDetectionBox(visualizationImage, detection);
                }
            }
            else
            {
                // Still tracking, not stable yet
                RCLCPP_INFO_THROTTLE(
                    GetLogger(), *this->get_clock(), 1000,
                    " Tag ID %d tracking: %.2fs / %.2fs (frames: %d)",
                    detection.id, duration, kMinDetectionDuration,
                    it->second.consecutive_frames);
            }
        }
    }

    // Publish only stable detections
    if (!stableDetections.empty())
    {
        PublishDetections(stableDetections, aTimestamp);
    }

    // Show visualization window only when we have detections
    if (mShowVisualization && !visualizationImage.empty())
    {
        ShowVisualizationWindow(visualizationImage);
    }

    // Clean up native detection array
    apriltag_detections_destroy(detections);
}

zarray_t *AprilTagDetector::DetectTagsNative(const cv::Mat &aGrayImage)
{
    if (!mpAprilTagDetector || aGrayImage.empty())
    {
        return nullptr;
    }

    // Convert OpenCV image to apriltag image_u8 format
    image_u8_t *img = ConvertToImageU8(aGrayImage);
    if (!img)
    {
        return nullptr;
    }

    // Perform detection
    zarray_t *detections = apriltag_detector_detect(mpAprilTagDetector, img);

    // Clean up image_u8
    image_u8_destroy(img);

    return detections;
}

image_u8_t *AprilTagDetector::ConvertToImageU8(const cv::Mat &aGrayImage)
{
    // Converts OpenCV grayscale image to apriltag image_u8 format.
    // aGrayImage: OpenCV grayscale image
    // Returns image_u8 structure
    if (aGrayImage.empty() || aGrayImage.type() != CV_8UC1)
    {
        return nullptr;
    }

    image_u8_t *img = image_u8_create(aGrayImage.cols, aGrayImage.rows);
    if (!img)
    {
        return nullptr;
    }

    // Copy data from OpenCV Mat to image_u8
    for (int y = 0; y < aGrayImage.rows; y++)
    {
        memcpy(&img->buf[y * img->stride], aGrayImage.ptr(y), aGrayImage.cols);
    }

    return img;
}

apriltag_msgs::msg::AprilTagDetection
AprilTagDetector::ConvertDetectionToROS(apriltag_detection_t *aDetection,
                                         const rclcpp::Time & /*aTimestamp*/)
{
    apriltag_msgs::msg::AprilTagDetection detection;

    // Set ID
    detection.id = aDetection->id;

    // Set center
    detection.centre.x = aDetection->c[0];
    detection.centre.y = aDetection->c[1];

    // Set corners (corners is a std::array<Point, 4>, not a vector)
    for (int i = 0; i < 4; i++)
    {
        detection.corners[i].x = aDetection->p[i][0];
        detection.corners[i].y = aDetection->p[i][1];
    }

    // Set confidence metrics
    detection.decision_margin = aDetection->decision_margin;
    detection.goodness = aDetection->decision_margin / 100.0f; // Normalize to 0-1
    detection.hamming = aDetection->hamming;
    detection.family = "16h5";

    // Copy homography matrix
    for (int i = 0; i < 9; i++)
    {
        detection.homography[i] = MATD_EL(aDetection->H, i / 3, i % 3);
    }

    return detection;
}

std::vector<double>
AprilTagDetector::CalculateOrientation(apriltag_detection_t *aDetection)
{
    std::vector<double> orientation(3, 0.0); // roll, pitch, yaw

    if (!aDetection || !aDetection->H)
    {
        return orientation;
    }

    // Extract rotation from homography matrix
    // This is a simplified approach - for full 3D pose estimation,
    // camera calibration parameters would be needed

    double h11 = MATD_EL(aDetection->H, 0, 0);
    double h21 = MATD_EL(aDetection->H, 1, 0);

    // Calculate yaw (rotation around Z-axis) from homography
    orientation[2] = atan2(h21, h11); // yaw in radians

    // For this simplified case, assume roll and pitch are small
    orientation[0] = 0.0; // roll
    orientation[1] = 0.0; // pitch

    return orientation;
}

void AprilTagDetector::PrintDetectionInfo(
    const apriltag_msgs::msg::AprilTagDetection &aDetection,
    const std::vector<double> &aOrientation)
{
    // Throttle console output to once per second to reduce overhead
    RCLCPP_INFO_THROTTLE(GetLogger(), *this->get_clock(), 1000,
                         "️ APRILTAG 16h5 DETECTED:");
    RCLCPP_INFO_THROTTLE(GetLogger(), *this->get_clock(), 1000, "    ID: %d",
                         aDetection.id);
    RCLCPP_INFO_THROTTLE(GetLogger(), *this->get_clock(), 1000,
                         "    Center: (%.1f, %.1f) pixels",
                         aDetection.centre.x, aDetection.centre.y);

    // Calculate tag size
    double dx = aDetection.corners[0].x - aDetection.corners[2].x;
    double dy = aDetection.corners[0].y - aDetection.corners[2].y;
    double diagonal = sqrt(dx * dx + dy * dy);

    RCLCPP_INFO_THROTTLE(GetLogger(), *this->get_clock(), 1000,
                         "    Size: %.1f pixels (diagonal)", diagonal);
    RCLCPP_INFO_THROTTLE(GetLogger(), *this->get_clock(), 1000,
                         "    Orientation: %.1f° (yaw)",
                         aOrientation[2] * 180.0 / M_PI);
    RCLCPP_INFO_THROTTLE(GetLogger(), *this->get_clock(), 1000,
                         "    Quality: margin=%.1f, hamming=%d",
                         aDetection.decision_margin, aDetection.hamming);
}

void AprilTagDetector::DrawDetectionBox(
    cv::Mat &aImage, const apriltag_msgs::msg::AprilTagDetection &aDetection)
{
    if (aImage.empty())
    {
        return;
    }

    // Draw green bounding box around tag
    std::vector<cv::Point> corners;
    for (const auto &corner : aDetection.corners)
    {
        corners.push_back(
            cv::Point(static_cast<int>(corner.x), static_cast<int>(corner.y)));
    }

    // Draw lines connecting corners
    for (size_t i = 0; i < corners.size(); i++)
    {
        cv::line(aImage, corners[i], corners[(i + 1) % corners.size()],
                 cv::Scalar(0, 255, 0), 3); // Green lines, thickness 3
    }

    // Draw center point
    cv::Point center(static_cast<int>(aDetection.centre.x),
                     static_cast<int>(aDetection.centre.y));
    cv::circle(aImage, center, 5, cv::Scalar(0, 255, 0), cv::FILLED);

    // Draw ID label with background
    std::string id_text = "ID:" + std::to_string(aDetection.id);
    cv::Size text_size =
        cv::getTextSize(id_text, cv::FONT_HERSHEY_SIMPLEX, 0.8, 2, nullptr);
    cv::Point text_pos(center.x - text_size.width / 2, center.y - 40);

    // Draw black background for text
    cv::rectangle(aImage,
                  cv::Point(text_pos.x - 5, text_pos.y - text_size.height - 5),
                  cv::Point(text_pos.x + text_size.width + 5, text_pos.y + 5),
                  cv::Scalar(0, 0, 0), cv::FILLED);

    // Draw white text
    cv::putText(aImage, id_text, text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(255, 255, 255), 2);
}

void AprilTagDetector::ShowVisualizationWindow(const cv::Mat &aImage)
{
    try
    {
        // Create window if it doesn't exist (first call)
        static bool window_created = false;
        if (!window_created)
        {
            cv::namedWindow(mVisualizationWindowName, cv::WINDOW_AUTOSIZE);
            window_created = true;
        }

        cv::imshow(mVisualizationWindowName, aImage);
        cv::waitKey(1);
    }
    catch (const std::exception &e)
    {
        // Fallback to saving image if GUI fails
        static int frame_count = 0;
        std::string filename =
            "/tmp/apriltag_detection_" + std::to_string(frame_count++) + ".jpg";
        cv::imwrite(filename, aImage);
        RCLCPP_INFO_THROTTLE(GetLogger(), *this->get_clock(), 3000,
                             "GUI failed, saved detection to: %s",
                             filename.c_str());
    }
}

void AprilTagDetector::PublishDetections(
    const std::vector<apriltag_msgs::msg::AprilTagDetection> &aDetections,
    const rclcpp::Time &aTimestamp)
{
    apriltag_msgs::msg::AprilTagDetectionArray msgArray;
    msgArray.header.stamp = aTimestamp;
    msgArray.header.frame_id = "camera_frame";
    msgArray.detections = aDetections;

    mpDetectionPublisher->publish(msgArray);

    RCLCPP_DEBUG(GetLogger(), "Published %zu AprilTag detections",
                 aDetections.size());
}

// ============================================================================
// Main entry point for AprilTag detector node
// ============================================================================
int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create AprilTag detector node
    auto node = std::make_shared<AprilTagDetector>();

    // Log startup
    RCLCPP_INFO(node->get_logger(),
                "️ Starting independent AprilTag 16h5 detector...");

    try
    {
        // Spin the node
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception in AprilTag detector: %s",
                     e.what());
    }

    // Cleanup
    rclcpp::shutdown();
    return 0;
}