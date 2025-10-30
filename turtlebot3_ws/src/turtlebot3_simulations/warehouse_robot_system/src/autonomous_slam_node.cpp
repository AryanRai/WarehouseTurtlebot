// Autonomous SLAM Exploration Node
// Standalone node for autonomous mapping using frontier exploration

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "AutonomousExplorationRobot.hpp"
#include <std_msgs/msg/float32.hpp>
#include "std_srvs/srv/trigger.hpp"
#include <memory>
#include <signal.h>

std::shared_ptr<AutonomousExplorationRobot> g_robot;
std::shared_ptr<rclcpp::Node> g_node;
float g_battery_level = 100.0;

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

// Battery callback
void batteryCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    g_battery_level = msg->data;
    
    // Auto return home if battery < 20%
    if (g_battery_level < 20.0 && g_robot && g_robot->isExploring()) {
        RCLCPP_WARN(g_node->get_logger(), 
                   "Low battery detected (%.1f%%)! Initiating return to home...", 
                   g_battery_level);
        g_robot->pauseExploration();
        // The robot will automatically return home through the exploration complete logic
    }
}

// Service callback for manual return home
void returnHomeCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    if (g_robot) {
        RCLCPP_INFO(g_node->get_logger(), "Manual return home requested");
        g_robot->pauseExploration();
        response->success = true;
        response->message = "Return home initiated";
    } else {
        response->success = false;
        response->message = "Robot not initialized";
    }
}

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Create node
    g_node = std::make_shared<rclcpp::Node>("autonomous_slam_node");
    
    // Register signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    RCLCPP_INFO(g_node->get_logger(), "===========================================");
    RCLCPP_INFO(g_node->get_logger(), "  Autonomous SLAM Exploration System");
    RCLCPP_INFO(g_node->get_logger(), "  Based on Frontier Exploration");
    RCLCPP_INFO(g_node->get_logger(), "===========================================");
    
    // Create battery subscriber
    auto battery_sub = g_node->create_subscription<std_msgs::msg::Float32>(
        "/battery/percentage", 10, batteryCallback);
    
    // Create return home service
    auto return_home_service = g_node->create_service<std_srvs::srv::Trigger>(
        "/return_home", returnHomeCallback);
    
    RCLCPP_INFO(g_node->get_logger(), "Return home service available at /return_home");
    RCLCPP_INFO(g_node->get_logger(), "Monitoring battery level on /battery/percentage");
    
    // Create autonomous exploration robot
    g_robot = std::make_shared<AutonomousExplorationRobot>(g_node);
    
    // Wait for SLAM to initialize and build initial map
    RCLCPP_INFO(g_node->get_logger(), "Waiting for SLAM Toolbox to initialize...");
    RCLCPP_INFO(g_node->get_logger(), "This may take 15-30 seconds for SLAM to build initial map...");
    rclcpp::sleep_for(std::chrono::seconds(15));
    
    // Start exploration
    RCLCPP_INFO(g_node->get_logger(), "Starting autonomous exploration!");
    g_robot->startExploration();
    
    // Main loop
    rclcpp::Rate rate(20);  // 20 Hz
    bool exploration_finished = false;
    
    while (rclcpp::ok()) {
        // Update robot
        g_robot->update();
        
        // Check if exploration is complete
        if (!exploration_finished && g_robot->isExplorationComplete()) {
            RCLCPP_INFO(g_node->get_logger(), "Exploration complete!");
            g_robot->saveMap("warehouse_map_complete");
            exploration_finished = true;
            
            // Stay alive in idle mode - wait for external signal to shutdown
            RCLCPP_INFO(g_node->get_logger(), "");
            RCLCPP_INFO(g_node->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            RCLCPP_INFO(g_node->get_logger(), "🎉 EXPLORATION COMPLETE - READY FOR MODE SWITCH");
            RCLCPP_INFO(g_node->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            RCLCPP_INFO(g_node->get_logger(), "");
            RCLCPP_INFO(g_node->get_logger(), "Node staying alive for mode transition...");
            RCLCPP_INFO(g_node->get_logger(), "Press Ctrl+C or send SIGTERM to shutdown");
            RCLCPP_INFO(g_node->get_logger(), "");
        }
        
        // Spin once
        rclcpp::spin_some(g_node);
        rate.sleep();
    }
    
    // Cleanup
    RCLCPP_INFO(g_node->get_logger(), "Autonomous SLAM node shutting down");
    g_robot->stopExploration();
    g_robot.reset();  // Release the robot before shutdown
    g_node.reset();
    
    rclcpp::shutdown();
    return 0;
}
