// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: delivery_robot_node.cpp
// Author(s): Inez Dumas, Aryan Rai
//
// Description: Implementation of delivery robot node for multi-point delivery
//              with route optimization via TSP.

#include "Robot/DeliveryRobot.hpp"
#include <rclcpp/rclcpp.hpp>
#include <signal.h>

std::shared_ptr<DeliveryRobot> g_robot;

void signalHandler(int signum)
{
    if (g_robot)
    {
        RCLCPP_INFO(rclcpp::get_logger("delivery_robot"),
                    "Interrupt signal (%d) received. Stopping deliveries...",
                    signum);
        g_robot->stopDeliveries();
    }
    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create node
    auto node = std::make_shared<rclcpp::Node>("delivery_robot_node");

    // Register signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    RCLCPP_INFO(node->get_logger(),
                "===========================================");
    RCLCPP_INFO(node->get_logger(), "  Delivery Robot System");
    RCLCPP_INFO(node->get_logger(),
                "  Multi-Point Delivery with Route Optimization");
    RCLCPP_INFO(node->get_logger(),
                "===========================================");
    RCLCPP_INFO(node->get_logger(), "");
    RCLCPP_INFO(node->get_logger(), "Setup Instructions:");
    RCLCPP_INFO(node->get_logger(),
                "1. Click points in RViz to define delivery zones");
    RCLCPP_INFO(node->get_logger(),
                "2. Call service: ros2 service call /save_delivery_zones "
                "std_srvs/srv/Trigger");
    RCLCPP_INFO(node->get_logger(),
                "3. Add delivery requests via code or service");
    RCLCPP_INFO(node->get_logger(), "4. Call service: ros2 service call "
                                    "/start_deliveries std_srvs/srv/Trigger");
    RCLCPP_INFO(node->get_logger(), "");

    // Create delivery robot
    g_robot = std::make_shared<DeliveryRobot>(node);

    // Check for optimization mode from environment variable
    const char *opt_mode_env = std::getenv("DELIVERY_OPTIMIZATION");
    bool use_tsp = false;

    if (opt_mode_env != nullptr)
    {
        std::string opt_mode(opt_mode_env);
        if (opt_mode == "tsp" || opt_mode == "TSP" || opt_mode == "optimized")
        {
            use_tsp = true;
            RCLCPP_INFO(
                node->get_logger(),
                " Route Optimization: TSP (A* + Simulated Annealing)");
        }
        else
        {
            RCLCPP_INFO(node->get_logger(),
                        " Route Optimization: Ordered (Sequential)");
        }
    }
    else
    {
        RCLCPP_INFO(node->get_logger(),
                    " Route Optimization: Ordered (Sequential) - Default");
        RCLCPP_INFO(node->get_logger(),
                    "   Set DELIVERY_OPTIMIZATION=tsp for optimized routing");
    }

    g_robot->setOptimizationMode(use_tsp);

    // Generate delivery requests for all zones
    // Creates a sequential path: Zone_1 -> Zone_2 -> Zone_3 -> ... -> Zone_N ->
    // Home
    auto zones = g_robot->getZones();

    if (zones.size() >= 2)
    {
        // Create delivery chain through all zones (no return to first zone)
        for (size_t i = 0; i < zones.size() - 1; i++)
        {
            DeliveryData::DeliveryRequest req;
            req.id = "DEL" + std::to_string(i + 1);
            req.from_zone = zones[i].name;
            req.to_zone = zones[i + 1].name; // Next zone in sequence
            req.item_description = "Package " + std::to_string(i + 1);
            req.priority = i + 1;

            g_robot->addDeliveryRequest(req);
        }

        RCLCPP_INFO(
            node->get_logger(),
            "Added %zu delivery requests for sequential path through all zones",
            zones.size() - 1);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(),
                    "Need at least 2 zones to create deliveries. Please define "
                    "zones first.");
    }

    // Wait for map to be available before starting deliveries
    RCLCPP_INFO(node->get_logger(), "Waiting for map to be available...");

    // Auto-start deliveries after map is ready (check every second)
    auto start_timer = node->create_wall_timer(
        std::chrono::seconds(1),
        [&, node]()
        {
            static bool started = false;
            static int wait_count = 0;

            if (!started)
            {
                // Check if we have a valid map
                if (g_robot && g_robot->hasValidMap())
                {
                    RCLCPP_INFO(node->get_logger(),
                                "Map is ready! Auto-starting deliveries...");
                    g_robot->startDeliveries();
                    started = true;
                }
                else
                {
                    wait_count++;
                    if (wait_count % 5 == 0)
                    {
                        RCLCPP_INFO(node->get_logger(),
                                    "Still waiting for map... (%ds)",
                                    wait_count);
                    }

                    // Timeout after 30 seconds
                    if (wait_count >= 30)
                    {
                        RCLCPP_ERROR(node->get_logger(),
                                     "Timeout waiting for map! Cannot start "
                                     "deliveries.");
                        RCLCPP_ERROR(node->get_logger(),
                                     "Check that SLAM Toolbox is running and "
                                     "publishing /map");
                        started = true; // Stop trying
                    }
                }
            }
        });

    // Create executor for proper service handling
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // Create timer for robot updates
    auto timer =
        node->create_wall_timer(std::chrono::milliseconds(100), // 10 Hz
                                [&]() { g_robot->update(); });

    // Spin with executor (handles services properly)
    executor.spin();

    // Cleanup
    RCLCPP_INFO(node->get_logger(), "Delivery Robot node shutting down");
    g_robot.reset();

    rclcpp::shutdown();
    return 0;
}
