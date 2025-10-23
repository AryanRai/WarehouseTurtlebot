// MTRX3760 2025 Project 2: Autonomous SLAM Main
// File: autonomous_slam_main.cpp
// Author(s): Aryan Rai
//
// Main executable for autonomous SLAM system

#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include "autonomous_slam_controller.hpp"

std::shared_ptr<slam::AutonomousSlamController> slam_controller;

void signalHandler(int signum) {
    if (slam_controller) {
        RCLCPP_INFO(rclcpp::get_logger("autonomous_slam_main"), 
                    "Received signal %d, shutting down gracefully...", signum);
        slam_controller.reset();
    }
    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Set up signal handler for graceful shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        // Create and run SLAM controller
        slam_controller = std::make_shared<slam::AutonomousSlamController>();
        
        RCLCPP_INFO(rclcpp::get_logger("autonomous_slam_main"), 
                    "=== MTRX3760 Autonomous SLAM System ===");
        RCLCPP_INFO(rclcpp::get_logger("autonomous_slam_main"), 
                    "Starting autonomous exploration and mapping...");
        RCLCPP_INFO(rclcpp::get_logger("autonomous_slam_main"), 
                    "Robot will explore until all frontiers are mapped,");
        RCLCPP_INFO(rclcpp::get_logger("autonomous_slam_main"), 
                    "then return to origin (0,0) for warehouse operations.");
        RCLCPP_INFO(rclcpp::get_logger("autonomous_slam_main"), 
                    "Press Ctrl+C to stop.");
        RCLCPP_INFO(rclcpp::get_logger("autonomous_slam_main"), 
                    "======================================");
        
        slam_controller->run();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("autonomous_slam_main"), 
                     "Exception in SLAM system: %s", e.what());
        return 1;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("autonomous_slam_main"), 
                "Autonomous SLAM system shutdown complete");
    return 0;
}