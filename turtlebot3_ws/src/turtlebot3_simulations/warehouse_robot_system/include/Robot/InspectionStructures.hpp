// ============================================================================
// MTRX3760 Project 2 - 
// File: InspectionStructures.hpp
// Description: Data structures for inspection robot operations including
//              damage sites (AprilTag locations), inspection requests, and
//              inspection records for warehouse damage management logging.
// Author(s): Dylan George
// Last Edited: 2025-11-02
// ============================================================================

#ifndef INSPECTION_STRUCTURES_HPP
#define INSPECTION_STRUCTURES_HPP

#include <string>
#include <chrono>
#include <geometry_msgs/msg/point.hpp>

// DamageSite - Represents a discovered damage location marked by AprilTag
struct DamageSite {
    std::string name;              // e.g., "Damage_1", "Damage_2"
    int apriltag_id;               // AprilTag ID detected at this site
    geometry_msgs::msg::Point position;  // Position in map frame
    std::string description;       // Optional description of damage
    std::chrono::system_clock::time_point discovered_time;  // When it was found
    
    DamageSite() : apriltag_id(-1) {}
    
    DamageSite(const std::string& n, int tag_id, const geometry_msgs::msg::Point& pos)
        : name(n), apriltag_id(tag_id), position(pos), 
          discovered_time(std::chrono::system_clock::now()) {}
};

// InspectionRequest - Request to inspect a specific damage site
struct InspectionRequest {
    std::string site_name;         // Name of damage site to inspect
    int priority;                  // Priority level (1=highest)
    std::chrono::system_clock::time_point request_time;
    
    InspectionRequest() : priority(1) {}
    
    InspectionRequest(const std::string& name, int prio = 1)
        : site_name(name), priority(prio),
          request_time(std::chrono::system_clock::now()) {}
};

// InspectionRecord - Log entry for completed inspection
struct InspectionRecord {
    std::string site_name;
    int apriltag_id;
    geometry_msgs::msg::Point position;
    std::chrono::system_clock::time_point start_time;
    std::chrono::system_clock::time_point end_time;
    double distance_traveled;      // Total distance in meters
    double duration_seconds;       // Time taken in seconds
    bool success;                  // Whether inspection completed successfully
    std::string notes;             // Additional notes or error messages
    
    InspectionRecord() : apriltag_id(-1), distance_traveled(0.0), 
                        duration_seconds(0.0), success(false) {}
};

#endif // INSPECTION_STRUCTURES_HPP
