// ============================================================================
// File: ColourDetector.cpp
// Description: Implementation of CColourDetector. Analyses colour in regions
//              surrounding AprilTags to classify warehouse damage types.
//              Combines tag location data with HSV colour analysis to generate
//              complete damage reports for inspection robot.
// Author(s): Dylan George
// Last Edited: 2025-10-27
// ============================================================================

#include "ColourDetector.hpp"
#include <sstream>

CColourDetector::CColourDetector()
    : CImageProcessorNode("colour_detector_node"),
      mHasImage(false)
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
        RCLCPP_WARN(GetLogger(), "Received empty image, not storing");
        return;
    }

    // Deep copy to ensure data persistence
    mLatestImage = aImage.clone();
    mLatestImageTimestamp = aTimestamp;
    mHasImage = true;
}

void CColourDetector::TagDetectionCallback(
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr aMsg)
{
    // Validate message
    if (aMsg == nullptr) {
        RCLCPP_ERROR(GetLogger(), "Received null tag detection message");
        return;
    }

    // Check if we have an image to process
    if (!mHasImage) {
        RCLCPP_WARN(GetLogger(), "No image available for colour analysis");
        return;
    }

    if (mLatestImage.empty()) {
        RCLCPP_WARN(GetLogger(), "Latest image is empty");
        return;
    }

    // Process each detected tag
    for (const auto &detection : aMsg->detections) {
        // Extract tag center position from pose
        // Note: This assumes 2D projection; adjust if using full 3D
        int tagCenterX = mLatestImage.cols / 2; // Placeholder - extract from detection
        int tagCenterY = mLatestImage.rows / 2; // Placeholder - extract from detection
        int tagSize = 100; // Placeholder - calculate from detection corners

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
            sReport.mPose = detection.pose.pose.pose;
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
    int offset = kSamplingOffset;

    // Region above tag
    int topX = aTagCenterX - kSamplingWidth / 2;
    int topY = aTagCenterY - halfSize - offset - kSamplingHeight;
    if (topX >= 0 && topY >= 0 && 
        topX + kSamplingWidth <= aImageWidth && 
        topY + kSamplingHeight <= aImageHeight) {
        regions.push_back(cv::Rect(topX, topY, kSamplingWidth, kSamplingHeight));
    }

    // Region below tag
    int bottomX = aTagCenterX - kSamplingWidth / 2;
    int bottomY = aTagCenterY + halfSize + offset;
    if (bottomX >= 0 && bottomY >= 0 && 
        bottomX + kSamplingWidth <= aImageWidth && 
        bottomY + kSamplingHeight <= aImageHeight) {
        regions.push_back(cv::Rect(bottomX, bottomY, kSamplingWidth, kSamplingHeight));
    }

    // Region left of tag
    int leftX = aTagCenterX - halfSize - offset - kSamplingWidth;
    int leftY = aTagCenterY - kSamplingHeight / 2;
    if (leftX >= 0 && leftY >= 0 && 
        leftX + kSamplingWidth <= aImageWidth && 
        leftY + kSamplingHeight <= aImageHeight) {
        regions.push_back(cv::Rect(leftX, leftY, kSamplingWidth, kSamplingHeight));
    }

    // Region right of tag
    int rightX = aTagCenterX + halfSize + offset;
    int rightY = aTagCenterY - kSamplingHeight / 2;
    if (rightX >= 0 && rightY >= 0 && 
        rightX + kSamplingWidth <= aImageWidth && 
        rightY + kSamplingHeight <= aImageHeight) {
        regions.push_back(cv::Rect(rightX, rightY, kSamplingWidth, kSamplingHeight));
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
            return "invalid";
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