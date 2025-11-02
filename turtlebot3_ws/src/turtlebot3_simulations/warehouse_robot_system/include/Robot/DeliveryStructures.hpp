// ============================================================================
// MTRX3760 Project 2 - 
// File: DeliveryStructures.hpp
// Description: Data structures for delivery robot operations. Defines zones,
//              tasks, and delivery state information for warehouse package
//              management and route optimization systems.
// Author(s): Dylan George
// Last Edited: 2025-11-02
// ============================================================================

#ifndef DELIVERY_STRUCTURES_HPP
#define DELIVERY_STRUCTURES_HPP

#include <string>
#include <vector>
#include <chrono>
#include <geometry_msgs/msg/point.hpp>

struct DeliveryZone {
    std::string name;
    geometry_msgs::msg::Point position;
    std::string description;
    
    DeliveryZone() = default;
    DeliveryZone(const std::string& n, const geometry_msgs::msg::Point& p, const std::string& d = "")
        : name(n), position(p), description(d) {}
};

struct DeliveryRequest {
    std::string id;
    std::string from_zone;
    std::string to_zone;
    std::string item_description;
    int priority;  // 1 = highest, 5 = lowest
    bool completed;
    
    DeliveryRequest() : priority(3), completed(false) {}
};

struct DeliveryRecord {
    std::string timestamp;
    std::string request_id;
    std::string from_zone;
    std::string to_zone;
    double distance_traveled;
    double time_taken_seconds;
    bool success;
    std::string notes;
    
    std::string toLogString() const {
        return timestamp + "," + request_id + "," + from_zone + "," + 
               to_zone + "," + std::to_string(distance_traveled) + "," + 
               std::to_string(time_taken_seconds) + "," + 
               (success ? "SUCCESS" : "FAILED") + "," + notes;
    }
};

#endif // DELIVERY_STRUCTURES_HPP