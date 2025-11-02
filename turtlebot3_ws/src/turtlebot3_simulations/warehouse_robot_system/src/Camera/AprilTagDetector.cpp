// ============================================================================
// MTRX3760 Project 2 - 
// File: AprilTagDetector.cpp
// Description: Implementation of independent AprilTag detection using native
//              apriltag library. Detects 16h5 tags, computes orientation,
//              provides GUI visualization, and publishes for color detection.
// Author(s): Dylan George
// Last Edited: 2025-11-02
// ============================================================================

#include "Camera/AprilTagDetector.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cmath>
#include <iomanip>

CAprilTagDetector::CAprilTagDetector()
    : CImageProcessorNode("apriltag_detector_node"),
      mpAprilTagDetector(nullptr),
      mpTag16h5Family(nullptr),
      mVisualizationWindowName("AprilTag 16h5 Detection")
{
    // Initialize native AprilTag detector
    if (!InitializeDetector()) {
        RCLCPP_ERROR(GetLogger(), "Failed to initialize AprilTag detector");
        return;
    }

    // Create publisher for AprilTag detections
    mpDetectionPublisher = this->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
        kDetectionOutputTopic,
        kQueueSize
    );

    // Declare ROS parameters
    mShowVisualization = this->declare_parameter<bool>("show_visualization", true);
    mPrintDetections = this->declare_parameter<bool>("print_detections", true);

    RCLCPP_INFO(GetLogger(), "🏷️ AprilTag 16h5 detector initialized");
    RCLCPP_INFO(GetLogger(), "Publishing to: %s", kDetectionOutputTopic.c_str());
    RCLCPP_INFO(GetLogger(), "Visualization: %s", mShowVisualization ? "ON" : "OFF");
    RCLCPP_INFO(GetLogger(), "Console output: %s", mPrintDetections ? "ON" : "OFF");
}

CAprilTagDetector::~CAprilTagDetector()
{
    RCLCPP_INFO(GetLogger(), "AprilTag detector shutting down");
    
    // Clean up native AprilTag resources
    if (mpAprilTagDetector) {
        apriltag_detector_destroy(mpAprilTagDetector);
    }
    if (mpTag16h5Family) {
        tag16h5_destroy(mpTag16h5Family);
    }
    
    // Close OpenCV windows
    if (mShowVisualization) {
        cv::destroyAllWindows();
    }
}

bool CAprilTagDetector::InitializeDetector()
{
    try {
        // Create 16h5 tag family
        mpTag16h5Family = tag16h5_create();
        if (!mpTag16h5Family) {
            RCLCPP_ERROR(GetLogger(), "Failed to create 16h5 tag family");
            return false;
        }

        // Create AprilTag detector
        mpAprilTagDetector = apriltag_detector_create();
        if (!mpAprilTagDetector) {
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

        RCLCPP_INFO(GetLogger(), "✅ Native AprilTag detector configured for 16h5 family");
        return true;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(GetLogger(), "Exception during detector initialization: %s", e.what());
        return false;
    }
}

void CAprilTagDetector::ProcessImage(const cv::Mat &aImage, 
                                    const rclcpp::Time &aTimestamp)
{
    if (aImage.empty()) {
        RCLCPP_WARN(GetLogger(), "Empty image received");
        return;
    }

    // Convert to grayscale
    cv::Mat grayImage;
    if (aImage.channels() == 3) {
        cv::cvtColor(aImage, grayImage, cv::COLOR_BGR2GRAY);
    } else {
        grayImage = aImage.clone();
    }

    // Perform native AprilTag detection
    zarray_t* detections = DetectTagsNative(grayImage);
    if (!detections) {
        return;
    }

    // Convert to ROS messages and process
    std::vector<apriltag_msgs::msg::AprilTagDetection> rosDetections;
    cv::Mat visualizationImage = aImage.clone();

    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        // Convert to ROS message
        apriltag_msgs::msg::AprilTagDetection rosDetection = ConvertDetectionToROS(det, aTimestamp);
        rosDetections.push_back(rosDetection);

        // Calculate orientation
        std::vector<double> orientation = CalculateOrientation(det);

        // Print detection info if enabled
        if (mPrintDetections) {
            PrintDetectionInfo(rosDetection, orientation);
        }

        // Draw visualization if enabled
        if (mShowVisualization) {
            DrawDetectionBox(visualizationImage, rosDetection);
        }
    }

    // Publish detections
    if (!rosDetections.empty()) {
        PublishDetections(rosDetections, aTimestamp);
    }

    // Show visualization window
    if (mShowVisualization) {
        ShowVisualizationWindow(visualizationImage);
    }

    // Clean up native detection array
    apriltag_detections_destroy(detections);
}

zarray_t* CAprilTagDetector::DetectTagsNative(const cv::Mat &aGrayImage)
{
    if (!mpAprilTagDetector || aGrayImage.empty()) {
        return nullptr;
    }

    // Convert OpenCV image to apriltag image_u8 format
    image_u8_t* img = ConvertToImageU8(aGrayImage);
    if (!img) {
        return nullptr;
    }

    // Perform detection
    zarray_t* detections = apriltag_detector_detect(mpAprilTagDetector, img);

    // Clean up image_u8
    image_u8_destroy(img);

    return detections;
}

image_u8_t* CAprilTagDetector::ConvertToImageU8(const cv::Mat &aGrayImage)
{
    // Converts OpenCV grayscale image to apriltag image_u8 format.
    // aGrayImage: OpenCV grayscale image
    // Returns image_u8 structure
    if (aGrayImage.empty() || aGrayImage.type() != CV_8UC1) {
        return nullptr;
    }

    image_u8_t* img = image_u8_create(aGrayImage.cols, aGrayImage.rows);
    if (!img) {
        return nullptr;
    }

    // Copy data from OpenCV Mat to image_u8
    for (int y = 0; y < aGrayImage.rows; y++) {
        memcpy(&img->buf[y * img->stride], aGrayImage.ptr(y), aGrayImage.cols);
    }

    return img;
}

apriltag_msgs::msg::AprilTagDetection CAprilTagDetector::ConvertDetectionToROS(
    apriltag_detection_t* aDetection, const rclcpp::Time & /*aTimestamp*/)
{
    apriltag_msgs::msg::AprilTagDetection detection;
    
    // Set ID
    detection.id = aDetection->id;
    
    // Set center
    detection.centre.x = aDetection->c[0];
    detection.centre.y = aDetection->c[1];
    
    // Set corners (corners is a std::array<Point, 4>, not a vector)
    for (int i = 0; i < 4; i++) {
        detection.corners[i].x = aDetection->p[i][0];
        detection.corners[i].y = aDetection->p[i][1];
    }
    
    // Set confidence metrics
    detection.decision_margin = aDetection->decision_margin;
    detection.goodness = aDetection->decision_margin / 100.0f; // Normalize to 0-1
    detection.hamming = aDetection->hamming;
    detection.family = "16h5";
    
    // Copy homography matrix
    for (int i = 0; i < 9; i++) {
        detection.homography[i] = MATD_EL(aDetection->H, i/3, i%3);
    }
    
    return detection;
}

std::vector<double> CAprilTagDetector::CalculateOrientation(apriltag_detection_t* aDetection)
{
    std::vector<double> orientation(3, 0.0); // roll, pitch, yaw
    
    if (!aDetection || !aDetection->H) {
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

void CAprilTagDetector::PrintDetectionInfo(
    const apriltag_msgs::msg::AprilTagDetection &aDetection,
    const std::vector<double> &aOrientation)
{
    RCLCPP_INFO(GetLogger(), "🏷️ APRILTAG 16h5 DETECTED:");
    RCLCPP_INFO(GetLogger(), "   📍 ID: %d", aDetection.id);
    RCLCPP_INFO(GetLogger(), "   📐 Center: (%.1f, %.1f) pixels", 
               aDetection.centre.x, aDetection.centre.y);
    
    // Calculate tag size
    double dx = aDetection.corners[0].x - aDetection.corners[2].x;
    double dy = aDetection.corners[0].y - aDetection.corners[2].y;
    double diagonal = sqrt(dx*dx + dy*dy);
    
    RCLCPP_INFO(GetLogger(), "   📏 Size: %.1f pixels (diagonal)", diagonal);
    RCLCPP_INFO(GetLogger(), "   🔄 Orientation: %.1f° (yaw)", aOrientation[2] * 180.0 / M_PI);
    
    RCLCPP_INFO(GetLogger(), "   📊 Corners:");
    for (size_t i = 0; i < aDetection.corners.size(); i++) {
        RCLCPP_INFO(GetLogger(), "      [%zu]: (%.1f, %.1f)", i, 
                   aDetection.corners[i].x, aDetection.corners[i].y);
    }
}

void CAprilTagDetector::DrawDetectionBox(cv::Mat &aImage, 
                                        const apriltag_msgs::msg::AprilTagDetection &aDetection)
{
    if (aImage.empty()) {
        return;
    }

    // Draw green bounding box around tag
    std::vector<cv::Point> corners;
    for (const auto& corner : aDetection.corners) {
        corners.push_back(cv::Point(static_cast<int>(corner.x), static_cast<int>(corner.y)));
    }
    
    // Draw lines connecting corners
    for (size_t i = 0; i < corners.size(); i++) {
        cv::line(aImage, corners[i], corners[(i+1) % corners.size()], 
                cv::Scalar(0, 255, 0), 3); // Green lines, thickness 3
    }
    
    // Draw center point
    cv::Point center(static_cast<int>(aDetection.centre.x), 
                    static_cast<int>(aDetection.centre.y));
    cv::circle(aImage, center, 5, cv::Scalar(0, 255, 0), cv::FILLED);
    
    // Draw ID label with background
    std::string id_text = "ID:" + std::to_string(aDetection.id);
    cv::Size text_size = cv::getTextSize(id_text, cv::FONT_HERSHEY_SIMPLEX, 0.8, 2, nullptr);
    cv::Point text_pos(center.x - text_size.width/2, center.y - 40);
    
    // Draw black background for text
    cv::rectangle(aImage, 
                 cv::Point(text_pos.x - 5, text_pos.y - text_size.height - 5),
                 cv::Point(text_pos.x + text_size.width + 5, text_pos.y + 5),
                 cv::Scalar(0, 0, 0), cv::FILLED);
    
    // Draw white text
    cv::putText(aImage, id_text, text_pos,
               cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
}

void CAprilTagDetector::ShowVisualizationWindow(const cv::Mat &aImage)
{
    try {
        cv::imshow(mVisualizationWindowName, aImage);
        cv::waitKey(1);
    } catch (const std::exception &e) {
        // Fallback to saving image if GUI fails
        static int frame_count = 0;
        std::string filename = "/tmp/apriltag_detection_" + std::to_string(frame_count++) + ".jpg";
        cv::imwrite(filename, aImage);
        RCLCPP_INFO_THROTTLE(GetLogger(), *this->get_clock(), 3000, 
                             "GUI failed, saved detection to: %s", filename.c_str());
    }
}

void CAprilTagDetector::PublishDetections(
    const std::vector<apriltag_msgs::msg::AprilTagDetection> &aDetections,
    const rclcpp::Time &aTimestamp)
{
    apriltag_msgs::msg::AprilTagDetectionArray msgArray;
    msgArray.header.stamp = aTimestamp;
    msgArray.header.frame_id = "camera_frame";
    msgArray.detections = aDetections;

    mpDetectionPublisher->publish(msgArray);
    
    RCLCPP_DEBUG(GetLogger(), "Published %zu AprilTag detections", aDetections.size());
}

// ============================================================================
// Main entry point for AprilTag detector node
// ============================================================================
int main(int argc, char** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Create AprilTag detector node
    auto node = std::make_shared<CAprilTagDetector>();
    
    // Log startup
    RCLCPP_INFO(node->get_logger(), "🏷️ Starting independent AprilTag 16h5 detector...");
    
    try {
        // Spin the node
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in AprilTag detector: %s", e.what());
    }
    
    // Cleanup
    rclcpp::shutdown();
    return 0;
}