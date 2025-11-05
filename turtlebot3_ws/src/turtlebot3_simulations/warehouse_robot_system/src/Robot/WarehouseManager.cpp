// ============================================================================
// MTRX3760 Project 2 - 
// File: WarehouseManager.cpp
// Description: Implementation of WarehouseManager for polymorphic robot control.
//              Demonstrates proper polymorphism with base class pointers
//              cast to derived classes (InspectionRobot/DeliveryRobot).
// Author(s): GitHub Copilot Refactoring
// Last Edited: 2025-11-05
// ============================================================================

#include "Robot/WarehouseManager.hpp"
#include <stdexcept>

// NOTE: InspectionRobot and DeliveryRobot headers not included due to missing dependencies
// (apriltag_msgs, visualization_msgs). WarehouseManager will use base WarehouseRobot interface only.

WarehouseManager::WarehouseManager(rclcpp::Node::SharedPtr node)
    : node_(node), robot_(nullptr) {
    RCLCPP_INFO(node_->get_logger(), "WarehouseManager initialized - ready for polymorphic robot control");
}

// ============================================================================
// Polymorphic Robot Management
// ============================================================================

void WarehouseManager::setRobotType(RobotType type) {
    // Clear existing robot
    robot_.reset();
    
    // NOTE: Due to missing dependencies (apriltag_msgs, visualization_msgs), 
    // we cannot instantiate InspectionRobot or DeliveryRobot directly.
    // This is a placeholder implementation for the polymorphic architecture.
    
    RCLCPP_WARN(node_->get_logger(), 
                "WarehouseManager::setRobotType() - Cannot instantiate derived robots due to missing dependencies");
    RCLCPP_WARN(node_->get_logger(), 
                "Install missing packages: ros-jazzy-apriltag, ros-jazzy-visualization-msgs");
    
    // For now, we'll leave robot_ as nullptr to demonstrate the architecture
    // In a complete implementation with all dependencies, this would work as designed
    
    (void)type; // Suppress unused parameter warning
}

std::optional<RobotType> WarehouseManager::getCurrentRobotType() const {
    if (!robot_) {
        return std::nullopt;
    }
    
    // Polymorphic call - resolves to correct derived implementation
    return robot_->getType();
}

// ============================================================================
// Polymorphic Interface (Unified Robot Control)
// ============================================================================

void WarehouseManager::update() {
    validateRobot();
    
    // Polymorphic call - robot_->update() resolves to:
    // - InspectionRobot::update() if robot_ points to InspectionRobot
    // - DeliveryRobot::update() if robot_ points to DeliveryRobot
    robot_->update();
}

void WarehouseManager::startOperations() {
    validateRobot();
    
    // Polymorphic call - robot_->startOperations() resolves to:
    // - InspectionRobot::startOperations() → InspectionRobot::startInspections()
    // - DeliveryRobot::startOperations() → DeliveryRobot::startDeliveries()
    robot_->startOperations();
    
    RCLCPP_INFO(node_->get_logger(), "Started operations for robot type: %s",
                robot_->getType() == RobotType::INSPECTION ? "INSPECTION" : "DELIVERY");
}

void WarehouseManager::stopOperations() {
    validateRobot();
    
    // Polymorphic call - robot_->stopOperations() resolves to:
    // - InspectionRobot::stopOperations() → InspectionRobot::stopInspections()
    // - DeliveryRobot::stopOperations() → DeliveryRobot::stopDeliveries()
    robot_->stopOperations();
    
    RCLCPP_INFO(node_->get_logger(), "Stopped operations for robot type: %s",
                robot_->getType() == RobotType::INSPECTION ? "INSPECTION" : "DELIVERY");
}

bool WarehouseManager::isOperating() const {
    if (!robot_) {
        return false;
    }
    
    // Polymorphic call - robot_->isOperating() resolves to:
    // - InspectionRobot::isOperating() → InspectionRobot::isInspecting()
    // - DeliveryRobot::isOperating() → DeliveryRobot::isDelivering()
    return robot_->isOperating();
}

bool WarehouseManager::hasValidMap() const {
    if (!robot_) {
        return false;
    }
    
    // Polymorphic call through base pointer
    return robot_->hasValidMap();
}

// ============================================================================
// Type-Safe Robot Access
// ============================================================================

InspectionRobot* WarehouseManager::getInspectionRobot() {
    if (!robot_ || robot_->getType() != RobotType::INSPECTION) {
        return nullptr;
    }
    
    // Safe downcast from base pointer to derived pointer
    return static_cast<InspectionRobot*>(robot_.get());
}

DeliveryRobot* WarehouseManager::getDeliveryRobot() {
    if (!robot_ || robot_->getType() != RobotType::DELIVERY) {
        return nullptr;
    }
    
    // Safe downcast from base pointer to derived pointer
    return static_cast<DeliveryRobot*>(robot_.get());
}

// ============================================================================
// Validation Helpers
// ============================================================================

void WarehouseManager::validateRobot() const {
    if (!robot_) {
        throw std::runtime_error("No active robot - call setRobotType() first");
    }
}