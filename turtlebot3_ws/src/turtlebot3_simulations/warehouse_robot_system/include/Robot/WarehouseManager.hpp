// ============================================================================
// MTRX3760 Project 2 - 
// File: WarehouseManager.hpp
// Description: Warehouse Manager class implementing polymorphic robot control.
//              Manages base class pointers that can be dynamically cast to
//              InspectionRobot or DeliveryRobot for proper polymorphism.
// Author(s): GitHub Copilot Refactoring
// Last Edited: 2025-11-05
// ============================================================================

#ifndef WAREHOUSE_MANAGER_HPP
#define WAREHOUSE_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include "Robot/WarehouseRobot.hpp"
#include <memory>
#include <string>
#include <optional>

// Forward declarations to avoid circular dependencies and missing headers
class InspectionRobot;
class DeliveryRobot;

/**
 * @brief Warehouse Manager class for polymorphic robot control
 * 
 * Implements proper polymorphism by maintaining a base class pointer
 * (WarehouseRobot*) that can be dynamically cast to either InspectionRobot
 * or DeliveryRobot. Provides a unified interface for robot operations
 * while maintaining all existing functionality.
 * 
 * Key Features:
 * - Polymorphic base pointer: std::unique_ptr<WarehouseRobot>
 * - Dynamic robot type switching at runtime
 * - Unified interface for all robot operations
 * - Maintains existing node-based architecture
 * - Zero functional changes to existing behavior
 */
class WarehouseManager {
public:
    /**
     * @brief Constructor
     * @param node Shared pointer to ROS2 node
     */
    explicit WarehouseManager(rclcpp::Node::SharedPtr node);
    
    /**
     * @brief Destructor
     */
    ~WarehouseManager() = default;
    
    // ========================================================================
    // Polymorphic Robot Management
    // ========================================================================
    
    /**
     * @brief Set robot type and create appropriate robot instance
     * @param type Robot type (INSPECTION or DELIVERY)
     * 
     * Creates polymorphic base pointer cast to derived class:
     * robot_ = std::make_unique<InspectionRobot>(node_);  // Polymorphic cast
     * robot_ = std::make_unique<DeliveryRobot>(node_);    // Polymorphic cast
     */
    void setRobotType(RobotType type);
    
    /**
     * @brief Get current robot type
     * @return Current robot type, or nullopt if no robot active
     */
    std::optional<RobotType> getCurrentRobotType() const;
    
    /**
     * @brief Check if robot is active
     * @return True if robot pointer is valid
     */
    bool hasActiveRobot() const { return robot_ != nullptr; }
    
    // ========================================================================
    // Polymorphic Interface (Unified Robot Control)
    // ========================================================================
    
    /**
     * @brief Update robot - polymorphic call
     * Calls robot_->update() which resolves to correct derived implementation
     */
    void update();
    
    /**
     * @brief Start robot operations - polymorphic call
     * Calls robot_->startOperations() which resolves to:
     * - InspectionRobot::startInspections() for inspection robots
     * - DeliveryRobot::startDeliveries() for delivery robots
     */
    void startOperations();
    
    /**
     * @brief Stop robot operations - polymorphic call
     */
    void stopOperations();
    
    /**
     * @brief Check if robot is operating - polymorphic call
     * @return True if robot is currently performing operations
     */
    bool isOperating() const;
    
    /**
     * @brief Check if robot has valid map - polymorphic call
     * @return True if robot has valid SLAM map
     */
    bool hasValidMap() const;
    
    // ========================================================================
    // Type-Safe Robot Access (Optional)
    // ========================================================================
    
    /**
     * @brief Get inspection robot (type-safe cast)
     * @return Pointer to InspectionRobot if current type is INSPECTION, nullptr otherwise
     */
    InspectionRobot* getInspectionRobot();
    
    /**
     * @brief Get delivery robot (type-safe cast)
     * @return Pointer to DeliveryRobot if current type is DELIVERY, nullptr otherwise
     */
    DeliveryRobot* getDeliveryRobot();
    
    /**
     * @brief Get base robot pointer (for advanced use)
     * @return Raw pointer to base WarehouseRobot
     */
    WarehouseRobot* getRobot() const { return robot_.get(); }

private:
    // ========================================================================
    // Core Components
    // ========================================================================
    
    rclcpp::Node::SharedPtr node_;                    ///< ROS2 node
    std::unique_ptr<WarehouseRobot> robot_;          ///< Polymorphic base pointer
    
    // ========================================================================
    // Validation Helpers
    // ========================================================================
    
    /**
     * @brief Validate robot pointer before operations
     * @throws std::runtime_error if no active robot
     */
    void validateRobot() const;
};

#endif // WAREHOUSE_MANAGER_HPP