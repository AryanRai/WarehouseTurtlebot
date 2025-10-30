// Autonomous SLAM Exploration Node
// Standalone node for autonomous mapping using frontier exploration

#include <rclcpp/rclcpp.hpp>
#include "AutonomousExplorationRobot.hpp"
#include <memory>
#include <signal.h>

std::shared_ptr<AutonomousExplorationRobot> g_robot;

void signalHandler(int signum) {
    if (g_robot) {
        RCLCPP_INFO(rclcpp::get_logger("autonomous_slam"), 
                   "Interrupt signal (%d) received. Stopping exploration...", signum);
        g_robot->stopExploration();
        g_robot->saveMap("warehouse_map_final");
    }
    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Create node
    auto node = std::make_shared<rclcpp::Node>("autonomous_slam_node");
    
    // Register signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    RCLCPP_INFO(node->get_logger(), "===========================================");
    RCLCPP_INFO(node->get_logger(), "  Autonomous SLAM Exploration System");
    RCLCPP_INFO(node->get_logger(), "  Based on Frontier Exploration");
    RCLCPP_INFO(node->get_logger(), "===========================================");
    
    // Create autonomous exploration robot
    g_robot = std::make_shared<AutonomousExplorationRobot>(node);
    
    // Wait for SLAM to initialize and build initial map
    RCLCPP_INFO(node->get_logger(), "Waiting for SLAM Toolbox to initialize...");
    RCLCPP_INFO(node->get_logger(), "This may take 15-30 seconds for SLAM to build initial map...");
    rclcpp::sleep_for(std::chrono::seconds(15));
    
    // Start exploration
    RCLCPP_INFO(node->get_logger(), "Starting autonomous exploration!");
    g_robot->startExploration();
    
    // Main loop
    rclcpp::Rate rate(20);  // 20 Hz
    while (rclcpp::ok()) {
        // Update robot
        g_robot->update();
        
        // Check if exploration is complete
        if (g_robot->isExplorationComplete()) {
            RCLCPP_INFO(node->get_logger(), "Exploration complete!");
            g_robot->saveMap("warehouse_map_complete");
            break;
        }
        
        // Spin once
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    // Cleanup
    RCLCPP_INFO(node->get_logger(), "Autonomous SLAM node shutting down");
    g_robot->stopExploration();
    g_robot.reset();  // Release the robot before shutdown
    
    rclcpp::shutdown();
    return 0;
}
