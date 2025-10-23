// MTRX3760 2025 Project 2: Warehouse Robot
// File: warehouse_robot_main.cpp
// Author(s): Aryan Rai 
//
// Main entry point for the warehouse robot system

#include "warehouse_robot_system.hpp"
#include <iostream>
#include <thread>

int main(int argc, char** argv) {
    std::cout << "=== MTRX3760 Warehouse Robot DevKit ===" << std::endl;
    std::cout << "Polymorphic Robot System with SLAM Integration" << std::endl;
    std::cout << "=========================================" << std::endl;

    try {
        // Create and initialize the warehouse robot system
        WarehouseRobotSystem system;
        system.initialize();
        
        // Run the system
        system.run();
        
        // Shutdown gracefully
        system.shutdown();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    std::cout << "System terminated successfully." << std::endl;
    return 0;
}