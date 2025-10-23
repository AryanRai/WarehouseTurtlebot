// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: warehouse_robot_main.cpp
// Author(s): Dylan George
//
// Main function demonstrating the warehouse robot system

#include "turtlebot3_node/warehouse_robot_system.hpp"

int main() {
    // Create and initialize the warehouse robot system
    WarehouseRobotSystem system;
    system.initialize();
    
    // Example 1: Request damage inspection
    std::cout << "\n=== EXAMPLE 1: DAMAGE INSPECTION ===" << std::endl;
    system.requestDamageInspection();
    
    // Example 2: Request package deliveries with different priorities
    std::cout << "\n=== EXAMPLE 2: PACKAGE DELIVERIES ===" << std::endl;
    
    DeliveryRequest urgent_medical;
    urgent_medical.delivery_id = 1;
    urgent_medical.pickup_location = Pose(2.0, 2.0, 0.0);
    urgent_medical.delivery_location = Pose(4.0, 4.0, 0.0);
    urgent_medical.package_description = "Urgent medical supplies";
    urgent_medical.priority_level = 9; // High priority
    urgent_medical.completed = false;
    
    DeliveryRequest routine_parts;
    routine_parts.delivery_id = 2;
    routine_parts.pickup_location = Pose(-1.0, 1.0, 0.0);
    routine_parts.delivery_location = Pose(-3.0, -2.0, 0.0);
    routine_parts.package_description = "Routine spare parts";
    routine_parts.priority_level = 3; // Lower priority
    routine_parts.completed = false;
    
    system.requestPackageDelivery(urgent_medical);
    system.requestPackageDelivery(routine_parts);
    
    // Example 3: Monitor system status
    std::cout << "\n=== EXAMPLE 3: SYSTEM STATUS ===" << std::endl;
    system.displaySystemStatus();
    
    // Example 4: Demonstrate robot type switching
    std::cout << "\n=== EXAMPLE 4: ROBOT TYPE SWITCHING ===" << std::endl;
    system.requestRobotTypeChange(1, DELIVERY); // Change inspection robot to delivery robot
    
    // Display final status
    std::cout << "\n=== FINAL SYSTEM STATUS ===" << std::endl;
    system.displaySystemStatus();
    
    // Note: In a real application, you would call system.run() here
    // which would run the main control loop continuously
    
    return 0;
}
