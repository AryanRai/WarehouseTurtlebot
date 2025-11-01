// ============================================================================
// File: apriltag_detector_main.cpp
// Description: Main entry point for independent AprilTag detection node.
//              Creates and runs CAprilTagDetector instance.
// Author(s): Dylan George
// Last Edited: 2025-11-01
// ============================================================================

#include "AprilTagDetector.hpp"
#include <rclcpp/rclcpp.hpp>

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
