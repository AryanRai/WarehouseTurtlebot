// Delivery Robot Node
// Handles delivery operations using saved map from SLAM

#include <rclcpp/rclcpp.hpp>
#include "DeliveryRobot.hpp"
#include <signal.h>

std::shared_ptr<DeliveryRobot> g_robot;

void signalHandler(int signum) {
    if (g_robot) {
        RCLCPP_INFO(rclcpp::get_logger("delivery_robot"), 
                   "Interrupt signal (%d) received. Stopping deliveries...", signum);
        g_robot->stopDeliveries();
    }
    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Create node
    auto node = std::make_shared<rclcpp::Node>("delivery_robot_node");
    
    // Register signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    RCLCPP_INFO(node->get_logger(), "===========================================");
    RCLCPP_INFO(node->get_logger(), "  Delivery Robot System");
    RCLCPP_INFO(node->get_logger(), "  Multi-Point Delivery with Route Optimization");
    RCLCPP_INFO(node->get_logger(), "===========================================");
    RCLCPP_INFO(node->get_logger(), "");
    RCLCPP_INFO(node->get_logger(), "Setup Instructions:");
    RCLCPP_INFO(node->get_logger(), "1. Click points in RViz to define delivery zones");
    RCLCPP_INFO(node->get_logger(), "2. Call service: ros2 service call /save_delivery_zones std_srvs/srv/Trigger");
    RCLCPP_INFO(node->get_logger(), "3. Add delivery requests via code or service");
    RCLCPP_INFO(node->get_logger(), "4. Call service: ros2 service call /start_deliveries std_srvs/srv/Trigger");
    RCLCPP_INFO(node->get_logger(), "");
    
    // Create delivery robot
    g_robot = std::make_shared<DeliveryRobot>(node);
    
    // Add delivery requests programmatically
    DeliveryRequest req1;
    req1.id = "DEL001";
    req1.from_zone = "Zone_1";
    req1.to_zone = "Zone_2";
    req1.item_description = "Package A";
    req1.priority = 1;
    
    DeliveryRequest req2;
    req2.id = "DEL002";
    req2.from_zone = "Zone_2";
    req2.to_zone = "Zone_1";
    req2.item_description = "Package B";
    req2.priority = 2;
    
    // Add the requests
    g_robot->addDeliveryRequest(req1);
    g_robot->addDeliveryRequest(req2);
    
    RCLCPP_INFO(node->get_logger(), "Added 2 delivery requests");
    
    // Auto-start deliveries after 5 seconds
    auto start_timer = node->create_wall_timer(
        std::chrono::seconds(5),
        [&]() {
            RCLCPP_INFO(node->get_logger(), "Auto-starting deliveries...");
            g_robot->startDeliveries();
        });
    start_timer->cancel();  // Will be called once by executor
    start_timer->reset();
    
    // Create executor for proper service handling
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    // Create timer for robot updates
    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(100),  // 10 Hz
        [&]() {
            g_robot->update();
        });
    
    // Spin with executor (handles services properly)
    executor.spin();
    
    // Cleanup
    RCLCPP_INFO(node->get_logger(), "Delivery Robot node shutting down");
    g_robot.reset();
    
    rclcpp::shutdown();
    return 0;
}
