// ============================================================================
// MTRX3760 Project 2 -
// File: inspection_robot_node.cpp
// Description: ROS2 node entry point for inspection robot operations. Handles
//              damage detection operations using camera and AprilTag markers
//              to identify and classify warehouse infrastructure damage.
// Author(s): Dylan George
// Last Edited: 2025-11-02
// ============================================================================
// Last Edited: 2025-11-01
// ============================================================================


#include <rclcpp/rclcpp.hpp>
#include "Robot/InspectionRobot.hpp"
#include <signal.h>


std::shared_ptr<InspectionRobot> g_robot;


void signalHandler(int signum) {
   if (g_robot) {
       RCLCPP_INFO(rclcpp::get_logger("inspection_robot"),
                  "Interrupt signal (%d) received. Stopping inspections...", signum);
       g_robot->stopInspections();
   }
   rclcpp::shutdown();
   exit(signum);
}


int main(int argc, char** argv) {
   // Initialize ROS 2
   rclcpp::init(argc, argv);
  
   // Create node
   auto node = std::make_shared<rclcpp::Node>("inspection_robot_node");
  
   // Register signal handler
   signal(SIGINT, signalHandler);
   signal(SIGTERM, signalHandler);
  
   RCLCPP_INFO(node->get_logger(), "===========================================");
   RCLCPP_INFO(node->get_logger(), "  Inspection Robot System");
   RCLCPP_INFO(node->get_logger(), "  Damage Detection with Camera & AprilTags");
   RCLCPP_INFO(node->get_logger(), "===========================================");
   RCLCPP_INFO(node->get_logger(), " ");
   RCLCPP_INFO(node->get_logger(), "Setup Instructions:");
   RCLCPP_INFO(node->get_logger(), "1. Click points in RViz to define damage sites");
   RCLCPP_INFO(node->get_logger(), "2. Call service: ros2 service call /save_damage_sites std_srvs/srv/Trigger");
   RCLCPP_INFO(node->get_logger(), "3. Add inspection requests via code or service");
   RCLCPP_INFO(node->get_logger(), "4. Call service: ros2 service call /start_inspections std_srvs/srv/Trigger");
   RCLCPP_INFO(node->get_logger(), " ");
   RCLCPP_INFO(node->get_logger(), "Camera & AprilTag Detection:");
   RCLCPP_INFO(node->get_logger(), "• Subscribes to /apriltag_detections");
   RCLCPP_INFO(node->get_logger(), "• Reads AprilTag IDs at each damage site");
   RCLCPP_INFO(node->get_logger(), "• Logs inspection results to CSV");
   RCLCPP_INFO(node->get_logger(), " ");
  
   // Create inspection robot
   g_robot = std::make_shared<InspectionRobot>(node);
  
   // Check for inspection mode from environment variable
   const char* mode_env = std::getenv("INSPECTION_MODE");
   bool exploration_mode = false;
  
   if (mode_env != nullptr) {
       std::string mode(mode_env);
       if (mode == "exploration") {
           exploration_mode = true;
           RCLCPP_INFO(node->get_logger(), "🔍 Mode: Inspection Exploration (Systematic Patrol)");
           RCLCPP_INFO(node->get_logger(), "   Will patrol all accessible areas to discover AprilTags");
       } else {
           RCLCPP_INFO(node->get_logger(), "📋 Mode: Inspection (Visit Pre-defined Sites)");
       }
   } else {
       RCLCPP_INFO(node->get_logger(), "📋 Mode: Inspection (Visit Pre-defined Sites) - Default");
   }
  
   // Check for optimization mode from environment variable
   const char* opt_mode_env = std::getenv("INSPECTION_OPTIMIZATION");
   bool use_tsp = false;
  
   if (opt_mode_env != nullptr) {
       std::string opt_mode(opt_mode_env);
       if (opt_mode == "tsp" || opt_mode == "TSP" || opt_mode == "optimized") {
           use_tsp = true;
           RCLCPP_INFO(node->get_logger(), "🎯 Route Optimization: TSP (A* + Simulated Annealing)");
       } else {
           RCLCPP_INFO(node->get_logger(), "🎯 Route Optimization: Ordered (Sequential)");
       }
   } else {
       RCLCPP_INFO(node->get_logger(), "🎯 Route Optimization: Ordered (Sequential) - Default");
       if (!exploration_mode) {
           RCLCPP_INFO(node->get_logger(), "   Set INSPECTION_OPTIMIZATION=tsp for optimized routing");
       }
   }
  
   g_robot->setOptimizationMode(use_tsp);
  
   // Set exploration mode if specified
   if (exploration_mode) {
       g_robot->setExplorationMode(true);
   }
  
   // Generate inspection requests for all damage sites (only in inspection mode)
   if (!exploration_mode) {
       auto sites = g_robot->getSites();
      
       if (!sites.empty()) {
           for (size_t i = 0; i < sites.size(); i++) {
               InspectionData::InspectionRequest req;
               req.site_name = sites[i].name;
               req.priority = i + 1;
              
               g_robot->addInspectionRequest(req);
           }
          
           RCLCPP_INFO(node->get_logger(), "Added %zu inspection requests for all damage sites", sites.size());
       } else {
           RCLCPP_WARN(node->get_logger(), "No damage sites defined. Please define sites first.");
       }
   }
  
   // Wait for map to be available before starting
   RCLCPP_INFO(node->get_logger(), "Waiting for map to be available...");
  
   // Auto-start after map is ready (check every second)
   auto start_timer = node->create_wall_timer(
       std::chrono::seconds(1),
       [&, node, exploration_mode]() {
           static bool started = false;
           static int wait_count = 0;
          
           if (!started) {
               // Check if we have a valid map
               if (g_robot && g_robot->hasValidMap()) {
                   if (exploration_mode) {
                       RCLCPP_INFO(node->get_logger(), "Map is ready! Starting exploration patrol...");
                       g_robot->startInspections();
                   } else {
                       RCLCPP_INFO(node->get_logger(), "Map is ready! Auto-starting inspections...");
                       g_robot->startInspections();
                   }
                   started = true;
               } else {
                   wait_count++;
                   if (wait_count % 5 == 0) {
                       RCLCPP_INFO(node->get_logger(), "Still waiting for map... (%ds)", wait_count);
                   }
                  
                   // Timeout after 30 seconds
                   if (wait_count >= 30) {
                       RCLCPP_ERROR(node->get_logger(),
                                   "Timeout waiting for map! Cannot start.");
                       RCLCPP_ERROR(node->get_logger(),
                                   "Check that SLAM Toolbox is running and publishing /map");
                       started = true;  // Stop trying
                   }
               }
           }
       });
  
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
   RCLCPP_INFO(node->get_logger(), "Inspection Robot node shutting down");
   g_robot.reset();
  
   rclcpp::shutdown();
   return 0;
}
