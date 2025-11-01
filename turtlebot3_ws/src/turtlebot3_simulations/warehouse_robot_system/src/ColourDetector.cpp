// ============================================================================
// File: ColourDetector.cpp
// Description: Implementation of CColourDetector. Analyses colour in regions
//              surrounding AprilTags to classify warehouse damage types.
//              Combines tag location data with HSV colour analysis to generate
//              complete damage reports for inspection robot.
// Author(s): Dylan George
// Last Edited: 2025-11-01
// ============================================================================

#include "ColourDetector.hpp"
#include <sstream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <cstring>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

CColourDetector::CColourDetector()
    : CImageProcessorNode("colour_detector_node"),
      mHasImage(false),
      mCalibrationMode(false),
      mCalibrationWindowName("HSV Calibration - Press 's'=save, 'c'=capture, 'q'=quit")
{
    // Create subscriber for AprilTag detections
    mpTagSubscriber = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        kTagInputTopic,
        kQueueSize,
        std::bind(&CColourDetector::TagDetectionCallback, this, std::placeholders::_1)
    );

    // Create publisher for damage reports
    mpDamagePublisher = this->create_publisher<std_msgs::msg::String>(
        kDamageOutputTopic,
        kQueueSize
    );

    // Declare and read calibration mode parameter
    mCalibrationMode = this->declare_parameter<bool>("calibration_mode", false);

    if (mCalibrationMode) {
        // Try to create OpenCV window for real-time calibration feedback
        try {
            // Check if we have a display available
            const char* display = getenv("DISPLAY");
            if (display == nullptr || strlen(display) == 0) {
                RCLCPP_WARN(GetLogger(), "No DISPLAY environment variable - use 'ssh -X' for GUI");
                RCLCPP_INFO(GetLogger(), "Running in FILE mode - saving frames to /tmp/");
            } else {
                cv::namedWindow(mCalibrationWindowName, cv::WINDOW_AUTOSIZE);
                RCLCPP_INFO(GetLogger(), "🎥 CALIBRATION mode - Live camera window enabled");
                RCLCPP_INFO(GetLogger(), "📸 Controls: 's'=save, 'c'=capture values, 'q'=quit");
                RCLCPP_INFO(GetLogger(), "🔍 Starting calibration mode...");
                RCLCPP_INFO(GetLogger(), "🎯 CALIBRATION INSTRUCTIONS:");
                RCLCPP_INFO(GetLogger(), "1. Place AprilTag 16h5 in camera view");
                RCLCPP_INFO(GetLogger(), "2. Add colored patches around the tag:");
                RCLCPP_INFO(GetLogger(), "   🟩 GREEN patch (mould simulation) - above tag");
                RCLCPP_INFO(GetLogger(), "   🟦 BLUE patch (water simulation) - below tag");
                RCLCPP_INFO(GetLogger(), "   🟥 RED patch (blood simulation) - left/right of tag");
                RCLCPP_INFO(GetLogger(), "3. Watch the colored rectangles appear around detected tag");
                RCLCPP_INFO(GetLogger(), "4. Check HSV values displayed above each region");
                RCLCPP_INFO(GetLogger(), "5. Press 'c' to print current values for adjustment");
                RCLCPP_INFO(GetLogger(), "6. Press 's' to save final calibration");
            }
        } catch (const std::exception &aEx) {
            RCLCPP_WARN(GetLogger(), "Failed to create window: %s - using file mode", aEx.what());
        }
    }

    RCLCPP_INFO(GetLogger(), "Colour detector initialised");
    RCLCPP_INFO(GetLogger(), "Subscribed to: %s", kTagInputTopic.c_str());
    RCLCPP_INFO(GetLogger(), "Publishing to: %s", kDamageOutputTopic.c_str());
}

CColourDetector::~CColourDetector()
{
    RCLCPP_INFO(GetLogger(), "Colour detector shutting down");
}

void CColourDetector::ProcessImage(const cv::Mat &aImage, 
                                   const rclcpp::Time &aTimestamp)
{
    // Store latest image for processing when tag detections arrive
    if (aImage.empty()) {
        RCLCPP_WARN(GetLogger(), "ProcessImage received empty frame");
        mHasImage = false;
        return;
    }

    // Deep copy to ensure data persistence
    mLatestImage = aImage.clone();
    mLatestImageTimestamp = aTimestamp;
    mHasImage = true;

    // In calibration mode, show overlay if we have detections
    if (mCalibrationMode) {
        if (!mLastDetections.empty()) {
            DisplayCalibrationWindow(mLatestImage, mLastDetections);
            int key = cv::waitKey(1);
            if (key >= 0) {
                HandleKeyboardInput(key);
            }
        } else {
            // No tags detected yet; still show raw image to confirm feed
            try {
                cv::imshow(mCalibrationWindowName, mLatestImage);
                int key = cv::waitKey(1);
                if (key >= 0) {
                    HandleKeyboardInput(key);
                }
            } catch (const std::exception &aEx) {
                // Fall back to file saving if GUI fails
                static int raw_frame_count = 0;
                std::string filename = "/tmp/raw_frame_" + std::to_string(raw_frame_count++) + ".jpg";
                cv::imwrite(filename, mLatestImage);
                RCLCPP_INFO_THROTTLE(GetLogger(), *this->get_clock(), 5000, 
                                     "GUI failed, saving to: %s", filename.c_str());
            }
        }
    }
}

void CColourDetector::TagDetectionCallback(
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr aMsg)
{
    // Validate message
    if (aMsg == nullptr) {
        RCLCPP_ERROR(GetLogger(), "TagDetectionCallback received null message");
        return;
    }

    // Cache detections for calibration display
    mLastDetections = aMsg->detections;

    // If no image available yet, nothing else to do
    if (!mHasImage || mLatestImage.empty()) {
        return;
    }

    if (mCalibrationMode) {
        // In calibration mode, just update the display (no publishing of damage)
        DisplayCalibrationWindow(mLatestImage, mLastDetections);
        int key = cv::waitKey(1);
        if (key >= 0) {
            HandleKeyboardInput(key);
        }
        return;
    }

    // === NORMAL DETECTION MODE WITH CAMERA VISUALIZATION ===
    // Show camera feed with AprilTag detection overlays in normal mode too
    cv::Mat display_image = mLatestImage.clone();
    
    // Draw detection overlays for normal mode
    for (const auto &detection : aMsg->detections) {
        int centerX = static_cast<int>(detection.centre.x);
        int centerY = static_cast<int>(detection.centre.y);
        
        // Calculate tag size from corners
        double corner_dist = std::sqrt(
            std::pow(detection.corners[0].x - detection.corners[2].x, 2) +
            std::pow(detection.corners[0].y - detection.corners[2].y, 2)
        );
        int tagSize = static_cast<int>(corner_dist);
        
        // Draw AprilTag box (blue for normal mode)
        std::vector<cv::Point> corners;
        for (const auto& corner : detection.corners) {
            corners.push_back(cv::Point(static_cast<int>(corner.x), static_cast<int>(corner.y)));
        }
        
        for (size_t i = 0; i < corners.size(); ++i) {
            cv::line(display_image, corners[i], corners[(i+1) % corners.size()], 
                     cv::Scalar(255, 0, 0), 2); // Blue border for normal mode
        }
        
        // Draw ID label
        std::string id_text = "ID:" + std::to_string(detection.id);
        cv::putText(display_image, id_text, 
                   cv::Point(centerX - 20, centerY - tagSize/2 - 15), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
    }
    
    // Show detection window (try GUI, fallback to file)
    try {
        cv::imshow("AprilTag Detection", display_image);
        cv::waitKey(1);
    } catch (const std::exception &ex) {
        static int detection_frame_count = 0;
        std::string filename = "/tmp/detection_frame_" + std::to_string(detection_frame_count++) + ".jpg";
        cv::imwrite(filename, display_image);
        RCLCPP_INFO_THROTTLE(GetLogger(), *this->get_clock(), 3000, 
                             "Saved detection frame: %s", filename.c_str());
    }

    // Normal detection path (existing logic)
    // Process each detected tag
    for (const auto &detection : aMsg->detections) {
        // Extract tag center from centre field (in pixels)
        int tagCenterX = static_cast<int>(detection.centre.x);
        int tagCenterY = static_cast<int>(detection.centre.y);
        
        // Calculate tag size from corners
        int tagSize = static_cast<int>(std::abs(detection.corners[0].x - detection.corners[2].x));

        // Analyse colour around this tag
        eDamageType damageType = AnalyseColourAroundTag(
            mLatestImage,
            tagCenterX,
            tagCenterY,
            tagSize
        );

        // Create and publish damage report if damage detected
        if (damageType != DAMAGE_NONE && damageType != DAMAGE_UNKNOWN) {
            SDamageReport sReport;
            sReport.mTagId = detection.id;
            
            // Create pose from centre point (2D detection in image plane)
            sReport.mPose.position.x = detection.centre.x;
            sReport.mPose.position.y = detection.centre.y;
            sReport.mPose.position.z = 0.0;
            sReport.mPose.orientation.x = 0.0;
            sReport.mPose.orientation.y = 0.0;
            sReport.mPose.orientation.z = 0.0;
            sReport.mPose.orientation.w = 1.0;
            
            sReport.mDamageType = damageType;
            sReport.mTimestamp = this->now();
            sReport.mConfidence = 0.8; // Placeholder - calculate from analysis

            PublishDamageReport(sReport);

            RCLCPP_INFO(GetLogger(), 
                       "Detected %s damage at tag ID %d",
                       DamageTypeToString(damageType).c_str(),
                       detection.id);
        }
    }
}

eDamageType CColourDetector::AnalyseColourAroundTag(const cv::Mat &aImage,
                                                     int aTagCenterX,
                                                     int aTagCenterY,
                                                     int aTagSize)
{
    // Validate inputs
    if (aImage.empty()) {
        RCLCPP_ERROR(GetLogger(), "Cannot analyse empty image");
        return DAMAGE_UNKNOWN;
    }

    // Extract sampling regions around tag
    std::vector<cv::Rect> samplingRegions = ExtractSamplingRegions(
        aTagCenterX,
        aTagCenterY,
        aTagSize,
        aImage.cols,
        aImage.rows
    );

    // Analyse each region and vote for damage type
    int mouldVotes = 0;
    int waterVotes = 0;
    int bloodVotes = 0;

    for (const auto &region : samplingRegions) {
        // Extract region from image
        cv::Mat regionImage = aImage(region);

        // Classify colour in this region
        eDamageType regionDamage = ClassifyColour(regionImage);

        // Accumulate votes
        if (regionDamage == DAMAGE_MOULD) {
            mouldVotes++;
        } else if (regionDamage == DAMAGE_WATER) {
            waterVotes++;
        } else if (regionDamage == DAMAGE_BLOOD) {
            bloodVotes++;
        }
    }

    // Determine overall damage type by majority vote
    int maxVotes = 0;
    eDamageType finalDamage = DAMAGE_NONE;

    if (mouldVotes > maxVotes) {
        maxVotes = mouldVotes;
        finalDamage = DAMAGE_MOULD;
    }
    if (waterVotes > maxVotes) {
        maxVotes = waterVotes;
        finalDamage = DAMAGE_WATER;
    }
    if (bloodVotes > maxVotes) {
        maxVotes = bloodVotes;
        finalDamage = DAMAGE_BLOOD;
    }

    // Require at least 2 votes for confident detection
    if (maxVotes < 2) {
        return DAMAGE_NONE;
    }

    return finalDamage;
}

std::vector<cv::Rect> CColourDetector::ExtractSamplingRegions(int aTagCenterX,
                                                               int aTagCenterY,
                                                               int aTagSize,
                                                               int aImageWidth,
                                                               int aImageHeight)
{
    std::vector<cv::Rect> regions;

    int halfSize = aTagSize / 2;
    
    // === ADAPTIVE SAMPLING BASED ON TAG SIZE ===
    // Larger tags (closer to camera) get larger sampling regions
    // Scale sampling parameters based on tag size
    double scale_factor = std::max(0.5, std::min(2.0, aTagSize / 80.0)); // Scale relative to 80px baseline
    
    int adaptive_offset = static_cast<int>(kSamplingOffset * scale_factor);
    int adaptive_width = static_cast<int>(kSamplingWidth * scale_factor);
    int adaptive_height = static_cast<int>(kSamplingHeight * scale_factor);
    
    // Ensure minimum sampling size for reliable color detection
    adaptive_width = std::max(adaptive_width, 20);
    adaptive_height = std::max(adaptive_height, 20);
    
    // Region above tag (GREEN area - for mould detection)
    int topX = aTagCenterX - adaptive_width / 2;
    int topY = aTagCenterY - halfSize - adaptive_offset - adaptive_height;
    if (topX >= 0 && topY >= 0 && 
        topX + adaptive_width <= aImageWidth && 
        topY + adaptive_height <= aImageHeight) {
        regions.push_back(cv::Rect(topX, topY, adaptive_width, adaptive_height));
    }

    // Region below tag (BLUE area - for water damage detection)
    int bottomX = aTagCenterX - adaptive_width / 2;
    int bottomY = aTagCenterY + halfSize + adaptive_offset;
    if (bottomX >= 0 && bottomY >= 0 && 
        bottomX + adaptive_width <= aImageWidth && 
        bottomY + adaptive_height <= aImageHeight) {
        regions.push_back(cv::Rect(bottomX, bottomY, adaptive_width, adaptive_height));
    }

    // Region left of tag (RED area - for blood damage detection)
    int leftX = aTagCenterX - halfSize - adaptive_offset - adaptive_width;
    int leftY = aTagCenterY - adaptive_height / 2;
    if (leftX >= 0 && leftY >= 0 && 
        leftX + adaptive_width <= aImageWidth && 
        leftY + adaptive_height <= aImageHeight) {
        regions.push_back(cv::Rect(leftX, leftY, adaptive_width, adaptive_height));
    }

    // Region right of tag (RED area - for blood damage detection)
    int rightX = aTagCenterX + halfSize + adaptive_offset;
    int rightY = aTagCenterY - adaptive_height / 2;
    if (rightX >= 0 && rightY >= 0 && 
        rightX + adaptive_width <= aImageWidth && 
        rightY + adaptive_height <= aImageHeight) {
        regions.push_back(cv::Rect(rightX, rightY, adaptive_width, adaptive_height));
    }

    return regions;
}

eDamageType CColourDetector::ClassifyColour(const cv::Mat &aRegion)
{
    // Convert region to HSV
    cv::Mat hsvRegion = ConvertToHSV(aRegion);

    // Count pixels matching each damage colour
    int greenPixels = CountColouredPixels(hsvRegion, kGreenLower, kGreenUpper);
    int bluePixels = CountColouredPixels(hsvRegion, kBlueLower, kBlueUpper);
    
    // Red wraps around HSV hue circle, need two ranges
    int redPixels1 = CountColouredPixels(hsvRegion, kRedLower1, kRedUpper1);
    int redPixels2 = CountColouredPixels(hsvRegion, kRedLower2, kRedUpper2);
    int redPixels = redPixels1 + redPixels2;

    // Determine which colour is most prominent
    int maxPixels = 0;
    eDamageType detectedType = DAMAGE_NONE;

    if (greenPixels > maxPixels && greenPixels >= kMinPixelCount) {
        maxPixels = greenPixels;
        detectedType = DAMAGE_MOULD;
    }
    if (bluePixels > maxPixels && bluePixels >= kMinPixelCount) {
        maxPixels = bluePixels;
        detectedType = DAMAGE_WATER;
    }
    if (redPixels > maxPixels && redPixels >= kMinPixelCount) {
        maxPixels = redPixels;
        detectedType = DAMAGE_BLOOD;
    }

    return detectedType;
}

cv::Mat CColourDetector::ConvertToHSV(const cv::Mat &aBGRImage) const
{
    cv::Mat hsvImage;
    cv::cvtColor(aBGRImage, hsvImage, cv::COLOR_BGR2HSV);
    return hsvImage;
}

int CColourDetector::CountColouredPixels(const cv::Mat &aHSVImage,
                                        const cv::Scalar &aLowerBound,
                                        const cv::Scalar &aUpperBound) const
{
    cv::Mat mask;
    cv::inRange(aHSVImage, aLowerBound, aUpperBound, mask);
    return cv::countNonZero(mask);
}

void CColourDetector::PublishDamageReport(const SDamageReport &aReport)
{
    // Create JSON-formatted damage report
    std::ostringstream jsonStream;
    jsonStream << "{";
    jsonStream << "\"tag_id\": " << aReport.mTagId << ", ";
    jsonStream << "\"damage_type\": \"" << DamageTypeToString(aReport.mDamageType) << "\", ";
    jsonStream << "\"position\": {";
    jsonStream << "\"x\": " << aReport.mPose.position.x << ", ";
    jsonStream << "\"y\": " << aReport.mPose.position.y << ", ";
    jsonStream << "\"z\": " << aReport.mPose.position.z;
    jsonStream << "}, ";
    jsonStream << "\"confidence\": " << aReport.mConfidence;
    jsonStream << "}";

    // Publish as string message
    std_msgs::msg::String msgReport;
    msgReport.data = jsonStream.str();
    mpDamagePublisher->publish(msgReport);

    RCLCPP_DEBUG(GetLogger(), "Published damage report: %s", msgReport.data.c_str());
}

void CColourDetector::DisplayCalibrationWindow(
    const cv::Mat &aImage,
    const std::vector<apriltag_msgs::msg::AprilTagDetection> &aDetections)
{
    if (aImage.empty()) {
        RCLCPP_WARN(GetLogger(), "DisplayCalibrationWindow: empty image");
        return;
    }

    cv::Mat annotated = aImage.clone();

    // For each detection, extract centre and tag size directly from message
    for (const auto &detection : aDetections) {
        // Extract center from centre field (in pixels)
        int centerX = static_cast<int>(detection.centre.x);
        int centerY = static_cast<int>(detection.centre.y);
        
        // Calculate tag size from corners (diagonal distance)
        double corner_dist = std::sqrt(
            std::pow(detection.corners[0].x - detection.corners[2].x, 2) +
            std::pow(detection.corners[0].y - detection.corners[2].y, 2)
        );
        int tagSize = static_cast<int>(corner_dist);
        
        // Ensure minimum tag size for sampling regions
        if (tagSize < 20) tagSize = 40;

        // === DRAW APRILTAG BOUNDING BOX ===
        // Draw box around detected AprilTag using corners
        std::vector<cv::Point> corners;
        for (const auto& corner : detection.corners) {
            corners.push_back(cv::Point(static_cast<int>(corner.x), static_cast<int>(corner.y)));
        }
        
        // Draw the tag boundary as a green quadrilateral
        for (size_t i = 0; i < corners.size(); ++i) {
            cv::line(annotated, corners[i], corners[(i+1) % corners.size()], 
                     cv::Scalar(0, 255, 0), 3); // Green thick border
        }
        
        // Draw AprilTag ID label with background
        std::string id_text = "ID:" + std::to_string(detection.id);
        cv::Size text_size = cv::getTextSize(id_text, cv::FONT_HERSHEY_SIMPLEX, 0.8, 2, nullptr);
        cv::Point text_pos(centerX - text_size.width/2, centerY - tagSize/2 - 10);
        
        // Draw text background rectangle
        cv::rectangle(annotated, 
                     cv::Point(text_pos.x - 5, text_pos.y - text_size.height - 5),
                     cv::Point(text_pos.x + text_size.width + 5, text_pos.y + 5),
                     cv::Scalar(0, 0, 0), cv::FILLED); // Black background
        
        // Draw white text on black background
        cv::putText(annotated, id_text, text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.8, 
                    cv::Scalar(255, 255, 255), 2); // White text

        // === DRAW ADAPTIVE SAMPLING REGIONS ===
        // Make sampling regions scale with tag size and distance
        // Larger tags (closer) = larger sampling regions
        DrawSamplingRegions(annotated, centerX, centerY, tagSize);

        // Compute sampling regions again to overlay HSV stats
        std::vector<cv::Rect> regions = ExtractSamplingRegions(
            centerX, centerY, tagSize, annotated.cols, annotated.rows);

        const std::vector<std::string> labels = {"ABOVE", "BELOW", "LEFT", "RIGHT"};
        for (size_t i = 0; i < regions.size() && i < labels.size(); ++i) {
            OverlayHSVStatistics(annotated, regions[i], labels[i]);
        }

        // Draw a small circle at the computed tag centre
        if (centerX >= 0 && centerY >= 0) {
            cv::circle(annotated, cv::Point(centerX, centerY), 3, cv::Scalar(0, 255, 255), cv::FILLED);
        }
    }

    try {
        cv::imshow(mCalibrationWindowName, annotated);
    } catch (const std::exception &aEx) {
        // Fall back to saving file if GUI display fails
        static int frame_count = 0;
        std::string filename = "/tmp/calibration_frame_" + std::to_string(frame_count++) + ".jpg";
        cv::imwrite(filename, annotated);
        RCLCPP_INFO_THROTTLE(GetLogger(), *this->get_clock(), 2000, 
                             "GUI failed, saved to: %s", filename.c_str());
    }
}

void CColourDetector::DrawSamplingRegions(cv::Mat &aImage,
                                          int aTagCenterX,
                                          int aTagCenterY,
                                          int aTagSize)
{
    if (aImage.empty() || aTagSize <= 0) {
        return;
    }

    // Calculate regions using existing helper to ensure consistency
    std::vector<cv::Rect> regions = ExtractSamplingRegions(
        aTagCenterX, aTagCenterY, aTagSize, aImage.cols, aImage.rows);

    for (const auto &roi : regions) {
        if (roi.x < 0 || roi.y < 0 ||
            roi.x + roi.width > aImage.cols ||
            roi.y + roi.height > aImage.rows) {
            continue; // safety bounds check
        }

        cv::Mat sub = aImage(roi);
        eDamageType cls = ClassifyColour(sub);

        // Choose border colour (BGR) by class
        cv::Scalar border = cv::Scalar(255, 255, 255); // default white
        switch (cls) {
            case DAMAGE_MOULD: border = cv::Scalar(0, 255, 0); break;   // green
            case DAMAGE_WATER: border = cv::Scalar(255, 0, 0); break;   // blue
            case DAMAGE_BLOOD: border = cv::Scalar(0, 0, 255); break;   // red
            case DAMAGE_NONE:
            case DAMAGE_UNKNOWN:
            default: border = cv::Scalar(255, 255, 255); break;
        }

        cv::rectangle(aImage, roi, border, 2);
    }
}

void CColourDetector::OverlayHSVStatistics(cv::Mat &aImage,
                                           const cv::Rect &aRegion,
                                           const std::string &aLabel)
{
    if (aImage.empty()) {
        return;
    }

    // Validate region bounds
    if (aRegion.x < 0 || aRegion.y < 0 ||
        aRegion.x + aRegion.width > aImage.cols ||
        aRegion.y + aRegion.height > aImage.rows) {
        return;
    }

    cv::Mat roiBGR = aImage(aRegion).clone();
    if (roiBGR.empty()) {
        return;
    }

    cv::Mat roiHSV;
    cv::cvtColor(roiBGR, roiHSV, cv::COLOR_BGR2HSV);

    // Compute mean HSV
    cv::Scalar meanHSV = cv::mean(roiHSV);

    // Compute min/max per channel for range info
    std::vector<cv::Mat> channels;
    cv::split(roiHSV, channels);
    double minH = 0.0, maxH = 0.0, minS = 0.0, maxS = 0.0, minV = 0.0, maxV = 0.0;
    if (channels.size() == 3) {
        cv::minMaxLoc(channels[0], &minH, &maxH);
        cv::minMaxLoc(channels[1], &minS, &maxS);
        cv::minMaxLoc(channels[2], &minV, &maxV);
    }

    // Enhanced format with more detail for calibration
    std::ostringstream ss;
    ss << aLabel << " [" << aRegion.width << "x" << aRegion.height << "px]" << std::endl;
    ss << "HSV: ["
       << static_cast<int>(std::round(meanHSV[0])) << "±" << static_cast<int>(maxH - minH) << ", "
       << static_cast<int>(std::round(meanHSV[1])) << "±" << static_cast<int>(maxS - minS) << ", "
       << static_cast<int>(std::round(meanHSV[2])) << "±" << static_cast<int>(maxV - minV) << "]";

    // === COLOR PLACEMENT GUIDANCE ===
    std::string color_instruction;
    if (aLabel.find("ABOVE") != std::string::npos) {
        color_instruction = "📍 Place GREEN object here (mould)";
    } else if (aLabel.find("BELOW") != std::string::npos) {
        color_instruction = "📍 Place BLUE object here (water)";
    } else if (aLabel.find("LEFT") != std::string::npos || aLabel.find("RIGHT") != std::string::npos) {
        color_instruction = "📍 Place RED object here (blood)";
    }

    // Calculate text placement (above the region with black background for readability)
    std::vector<std::string> lines = {ss.str(), color_instruction};
    int line_height = 15;
    int start_y = std::max(15, aRegion.y - (static_cast<int>(lines.size()) * line_height) - 5);
    
    for (size_t i = 0; i < lines.size(); ++i) {
        if (lines[i].empty()) continue;
        
        int text_y = start_y + static_cast<int>(i) * line_height;
        cv::Size text_size = cv::getTextSize(lines[i], cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, nullptr);
        
        // Draw black background for text readability
        cv::rectangle(aImage, 
                     cv::Point(aRegion.x - 2, text_y - text_size.height - 2),
                     cv::Point(aRegion.x + text_size.width + 2, text_y + 2),
                     cv::Scalar(0, 0, 0), cv::FILLED);
        
        // Draw white text
        cv::putText(aImage, lines[i], cv::Point(aRegion.x, text_y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
}

void CColourDetector::HandleKeyboardInput(int aKeyCode)
{
    if (aKeyCode == 's' || aKeyCode == 'S') {
        SaveCalibrationToYAML(std::string("~/hsv_calibration.yaml"));
        RCLCPP_INFO(GetLogger(), "Calibration saved to ~/hsv_calibration.yaml");
    } else if (aKeyCode == 'c' || aKeyCode == 'C') {
        // Print current HSV values with detailed color guidance
        RCLCPP_INFO(GetLogger(), "🎨 CURRENT HSV VALUES CAPTURED:");
        RCLCPP_INFO(GetLogger(), "================================");
        
        if (!mHasImage || mLatestImage.empty()) {
            RCLCPP_WARN(GetLogger(), "❌ No image available - ensure camera is running");
            return;
        }
        if (mLastDetections.empty()) {
            RCLCPP_WARN(GetLogger(), "❌ No AprilTags detected - place tag in camera view");
            return;
        }

        // Print current calibration ranges for reference
        RCLCPP_INFO(GetLogger(), "📊 Current HSV Thresholds:");
        RCLCPP_INFO(GetLogger(), "  🟩 GREEN (Mould):  H=%d-%d, S=%d-%d, V=%d-%d", 
                   (int)kGreenLower[0], (int)kGreenUpper[0],
                   (int)kGreenLower[1], (int)kGreenUpper[1], 
                   (int)kGreenLower[2], (int)kGreenUpper[2]);
        RCLCPP_INFO(GetLogger(), "  🟦 BLUE (Water):   H=%d-%d, S=%d-%d, V=%d-%d",
                   (int)kBlueLower[0], (int)kBlueUpper[0],
                   (int)kBlueLower[1], (int)kBlueUpper[1],
                   (int)kBlueLower[2], (int)kBlueUpper[2]);
        RCLCPP_INFO(GetLogger(), "  🟥 RED (Blood):    H=%d-%d OR %d-%d, S=%d-%d, V=%d-%d",
                   (int)kRedLower1[0], (int)kRedUpper1[0],
                   (int)kRedLower2[0], (int)kRedUpper2[0],
                   (int)kRedLower1[1], (int)kRedUpper1[1],
                   (int)kRedLower1[2], (int)kRedUpper1[2]);
        
        RCLCPP_INFO(GetLogger(), "📋 Color detection thresholds loaded");

        for (const auto &det : mLastDetections) {
            // Compute centre/size via corners if available
            bool hasCorners = false;
            std::vector<cv::Point2f> cornersPx;
            try {
                const auto &c = det.corners;
                if (!c.empty()) {
                    hasCorners = true;
                    for (const auto &p : c) {
                        cornersPx.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));
                    }
                }
            } catch (...) {
                hasCorners = false;
            }
            if (!hasCorners || cornersPx.size() < 4) {
                continue;
            }

            double sx = 0.0, sy = 0.0;
            for (const auto &pt : cornersPx) { sx += pt.x; sy += pt.y; }
            int cx = static_cast<int>(sx / static_cast<double>(cornersPx.size()));
            int cy = static_cast<int>(sy / static_cast<double>(cornersPx.size()));
            double d01 = cv::norm(cornersPx[0] - cornersPx[1]);
            double d12 = cv::norm(cornersPx[1] - cornersPx[2]);
            double d23 = cv::norm(cornersPx[2] - cornersPx[3]);
            double d30 = cv::norm(cornersPx[3] - cornersPx[0]);
            int tagSize = static_cast<int>(std::max(std::max(d01, d12), std::max(d23, d30)));

            std::vector<cv::Rect> regions = ExtractSamplingRegions(
                cx, cy, tagSize, mLatestImage.cols, mLatestImage.rows);

            auto meanRegion = [&](const cv::Rect &r)->cv::Scalar {
                if (r.x < 0 || r.y < 0 ||
                    r.x + r.width > mLatestImage.cols ||
                    r.y + r.height > mLatestImage.rows) {
                    return cv::Scalar(0,0,0,0);
                }
                cv::Mat bgr = mLatestImage(r).clone();
                cv::Mat hsv;
                cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
                return cv::mean(hsv);
            };

            cv::Scalar mAbove(0,0,0,0), mBelow(0,0,0,0), mLeft(0,0,0,0), mRight(0,0,0,0);
            if (regions.size() >= 4) {
                mAbove = meanRegion(regions[0]);
                mBelow = meanRegion(regions[1]);
                mLeft  = meanRegion(regions[2]);
                mRight = meanRegion(regions[3]);
            }

            int tagId = 0;
            try { tagId = det.id; } catch (...) { tagId = 0; }

            auto fmtHSV = [](const cv::Scalar &s)->std::string {
                return "[" + std::to_string((int)s[0]) + ", " + 
                            std::to_string((int)s[1]) + ", " + 
                            std::to_string((int)s[2]) + "]";
            };

            RCLCPP_INFO(GetLogger(), "🏷️ Tag ID %d sampling regions:", tagId);
            RCLCPP_INFO(GetLogger(), "  🟩 ABOVE (place GREEN here):  HSV %s", fmtHSV(mAbove).c_str());
            RCLCPP_INFO(GetLogger(), "  🟦 BELOW (place BLUE here):   HSV %s", fmtHSV(mBelow).c_str());
            RCLCPP_INFO(GetLogger(), "  🟥 LEFT  (place RED here):    HSV %s", fmtHSV(mLeft).c_str());
            RCLCPP_INFO(GetLogger(), "  🟥 RIGHT (place RED here):    HSV %s", fmtHSV(mRight).c_str());
            
            // Give specific guidance on what to do
            RCLCPP_INFO(GetLogger(), "📋 NEXT STEPS:");
            RCLCPP_INFO(GetLogger(), "1. Place colored objects around the AprilTag:");
            RCLCPP_INFO(GetLogger(), "   • GREEN object ABOVE the tag (mould simulation)");
            RCLCPP_INFO(GetLogger(), "   • BLUE object BELOW the tag (water simulation)");
            RCLCPP_INFO(GetLogger(), "   • RED objects LEFT and RIGHT of tag (blood simulation)");
            RCLCPP_INFO(GetLogger(), "2. Watch the colored rectangles change in the camera window");
            RCLCPP_INFO(GetLogger(), "3. When satisfied with detection, press 's' to save calibration");
        }
    } else if (aKeyCode == 'q' || aKeyCode == 'Q') {
        RCLCPP_INFO(GetLogger(), "Exiting calibration mode");
        rclcpp::shutdown();
    }
}

void CColourDetector::SaveCalibrationToYAML(const std::string &aFilePath)
{
    // Expand '~' to home directory if present
    auto expandPath = [](const std::string &p)->std::string {
        if (!p.empty() && p[0] == '~') {
            const char *home = std::getenv("HOME");
            if (home != nullptr) {
                return std::string(home) + p.substr(1);
            }
        }
        return p;
    };

    const std::string path = expandPath(aFilePath);

    try {
        std::ofstream ofs(path);
        if (!ofs.is_open()) {
            RCLCPP_ERROR(GetLogger(), "Failed to open file for writing: %s", path.c_str());
            return;
        }

        // Timestamp string
        auto now = std::chrono::system_clock::now();
        std::time_t t_c = std::chrono::system_clock::to_time_t(now);

        ofs << "# HSV Calibration for Warehouse Damage Detection\n";
        ofs << "# Generated: " << std::put_time(std::localtime(&t_c), "%Y-%m-%d %H:%M:%S") << "\n\n";

        ofs << "green_mould:\n";
        ofs << "  lower: [" << static_cast<int>(kGreenLower[0]) << ", "
                         << static_cast<int>(kGreenLower[1]) << ", "
                         << static_cast<int>(kGreenLower[2]) << "]\n";
        ofs << "  upper: [" << static_cast<int>(kGreenUpper[0]) << ", "
                         << static_cast<int>(kGreenUpper[1]) << ", "
                         << static_cast<int>(kGreenUpper[2]) << "]\n\n";

        ofs << "blue_water:\n";
        ofs << "  lower: [" << static_cast<int>(kBlueLower[0]) << ", "
                         << static_cast<int>(kBlueLower[1]) << ", "
                         << static_cast<int>(kBlueLower[2]) << "]\n";
        ofs << "  upper: [" << static_cast<int>(kBlueUpper[0]) << ", "
                         << static_cast<int>(kBlueUpper[1]) << ", "
                         << static_cast<int>(kBlueUpper[2]) << "]\n\n";

        ofs << "red_blood:\n";
        ofs << "  lower_1: [" << static_cast<int>(kRedLower1[0]) << ", "
                              << static_cast<int>(kRedLower1[1]) << ", "
                              << static_cast<int>(kRedLower1[2]) << "]\n";
        ofs << "  upper_1: [" << static_cast<int>(kRedUpper1[0]) << ", "
                              << static_cast<int>(kRedUpper1[1]) << ", "
                              << static_cast<int>(kRedUpper1[2]) << "]\n";
        ofs << "  lower_2: [" << static_cast<int>(kRedLower2[0]) << ", "
                              << static_cast<int>(kRedLower2[1]) << ", "
                              << static_cast<int>(kRedLower2[2]) << "]\n";
        ofs << "  upper_2: [" << static_cast<int>(kRedUpper2[0]) << ", "
                              << static_cast<int>(kRedUpper2[1]) << ", "
                              << static_cast<int>(kRedUpper2[2]) << "]\n\n";

        ofs << "# Sampling parameters\n";
        ofs << "sampling_offset: " << kSamplingOffset << "\n";
        ofs << "sampling_width: " << kSamplingWidth << "\n";
        ofs << "sampling_height: " << kSamplingHeight << "\n";

        ofs.close();
    } catch (const std::exception &aEx) {
        RCLCPP_ERROR(GetLogger(), "Exception while saving YAML: %s", aEx.what());
    }
}

std::string CColourDetector::DamageTypeToString(eDamageType aDamageType) const
{
    switch (aDamageType) {
        case DAMAGE_NONE:
            return "none";
        case DAMAGE_MOULD:
            return "mould";
        case DAMAGE_WATER:
            return "water";
        case DAMAGE_BLOOD:
            return "blood";
        case DAMAGE_UNKNOWN:
            return "unknown";
        default:
            return "unknown";
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto cColourDetector = std::make_shared<CColourDetector>();
    rclcpp::spin(cColourDetector);
    rclcpp::shutdown();
    return 0;
}