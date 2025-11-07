// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: ZoneMarkerNode.hpp
// Author(s): Aryan Rai
//
// Description: ROS2 node for managing delivery zone markers in RViz.
//              Subscribes to clicked points to add zones, publishes markers,
//              and provides service to clear all zones.

#ifndef ZONE_MARKER_NODE_HPP
#define ZONE_MARKER_NODE_HPP

#include "Robot/DeliveryStructures.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/**
 * @brief Node for managing delivery zone markers in RViz
 * 
 * Allows users to click points in RViz to add delivery zones,
 * visualizes zones with markers, and provides services to manage zones.
 */
class ZoneMarkerNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor
     */
    ZoneMarkerNode();
    
    /**
     * @brief Destructor
     */
    ~ZoneMarkerNode() = default;
    
private:
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
        clicked_point_sub_;
    
    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_pub_;
    
    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_zones_srv_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr marker_timer_;
    
    // State
    std::vector<DeliveryData::DeliveryZone> zones_;
    
    /**
     * @brief Callback for clicked points from RViz
     * @param msg Point stamped message
     */
    void onPointClicked(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    
    /**
     * @brief Load zones from YAML file
     */
    void loadZones();
    
    /**
     * @brief Save zones to YAML file
     */
    void saveZones();
    
    /**
     * @brief Service callback to clear all zones
     */
    void onClearZones(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    /**
     * @brief Publish zone markers to RViz
     */
    void publishMarkers();
};

#endif // ZONE_MARKER_NODE_HPP