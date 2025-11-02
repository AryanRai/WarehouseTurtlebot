// ============================================================================
// MTRX3760 Project 2 - 
// File: zone_marker_node.cpp
// Description: ROS2 node for interactive zone definition with visualization.
//              Subscribes to clicked points and publishes zone markers for
//              warehouse area definition and robot task assignment.
// Author(s): Dylan George
// Last Edited: 2025-11-02
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <string>
#include "Robot/DeliveryStructures.hpp"

class ZoneMarkerNode : public rclcpp::Node {
public:
    ZoneMarkerNode() : Node("zone_marker_node") {
        // Subscribe to clicked points
        clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10,
            std::bind(&ZoneMarkerNode::onPointClicked, this, std::placeholders::_1));
        
        // Publisher for zone markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/delivery_zones/markers", 10);
        
        // Service to clear all zones
        clear_zones_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/clear_zones",
            std::bind(&ZoneMarkerNode::onClearZones, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // Timer to republish markers periodically
        marker_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ZoneMarkerNode::publishMarkers, this));
        
        // Load existing zones
        loadZones();
        
        RCLCPP_INFO(this->get_logger(), "Zone Marker Node started");
        RCLCPP_INFO(this->get_logger(), "Publishing markers to: /delivery_zones/markers");
        RCLCPP_INFO(this->get_logger(), "Click points in RViz to add delivery zones");
        RCLCPP_INFO(this->get_logger(), "Currently have %zu zones", zones_.size());
        
        // Publish existing zones immediately
        if (!zones_.empty()) {
            publishMarkers();
            RCLCPP_INFO(this->get_logger(), "Published %zu existing zone markers", zones_.size());
        }
    }
    
private:
    void onPointClicked(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        DeliveryData::DeliveryZone zone;
        zone.name = "Zone_" + std::to_string(zones_.size() + 1);
        zone.position.x = msg->point.x;
        zone.position.y = msg->point.y;
        zone.position.z = 0.0;
        zone.description = "Delivery zone added via RViz";
        
        zones_.push_back(zone);
        
        RCLCPP_INFO(this->get_logger(), "✓ Added %s at (%.2f, %.2f)", 
                   zone.name.c_str(), zone.position.x, zone.position.y);
        
        saveZones();
        publishMarkers();
    }
    
    void loadZones() {
        try {
            YAML::Node config = YAML::LoadFile("delivery_zones.yaml");
            zones_.clear();
            
            if (config["delivery_zones"]) {
                for (const auto& zone_node : config["delivery_zones"]) {
                    DeliveryData::DeliveryZone zone;
                    zone.name = zone_node["name"].as<std::string>();
                    zone.position.x = zone_node["x"].as<double>();
                    zone.position.y = zone_node["y"].as<double>();
                    zone.position.z = 0.0;
                    zone.description = zone_node["description"].as<std::string>("");
                    zones_.push_back(zone);
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_INFO(this->get_logger(), "No existing zones file, starting fresh");
        }
    }
    
    void saveZones() {
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "delivery_zones";
        out << YAML::Value << YAML::BeginSeq;
        
        for (const auto& zone : zones_) {
            out << YAML::BeginMap;
            out << YAML::Key << "name" << YAML::Value << zone.name;
            out << YAML::Key << "x" << YAML::Value << zone.position.x;
            out << YAML::Key << "y" << YAML::Value << zone.position.y;
            out << YAML::Key << "description" << YAML::Value << zone.description;
            out << YAML::EndMap;
        }
        
        out << YAML::EndSeq;
        out << YAML::EndMap;
        
        std::ofstream fout("delivery_zones.yaml");
        fout << out.c_str();
        fout.close();
    }
    
    void onClearZones(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        RCLCPP_INFO(this->get_logger(), "Clearing all zones...");
        
        zones_.clear();
        saveZones();
        
        // Publish empty marker array to clear visualization
        visualization_msgs::msg::MarkerArray marker_array;
        
        // Send delete markers for all previous markers
        for (int i = 0; i < 100; i++) {  // Clear up to 100 zones
            visualization_msgs::msg::Marker delete_marker;
            delete_marker.header.frame_id = "map";
            delete_marker.header.stamp = this->now();
            delete_marker.ns = "delivery_zones";
            delete_marker.id = i;
            delete_marker.action = visualization_msgs::msg::Marker::DELETE;
            marker_array.markers.push_back(delete_marker);
        }
        
        marker_pub_->publish(marker_array);
        
        response->success = true;
        response->message = "All zones cleared";
        
        RCLCPP_INFO(this->get_logger(), "✓ All zones cleared");
    }
    
    void publishMarkers() {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // Create markers for each zone
        for (size_t i = 0; i < zones_.size(); i++) {
            const auto& zone = zones_[i];
            
            // Cylinder marker for the zone
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "delivery_zones";
            marker.id = i * 2;  // Even IDs for cylinders
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = zone.position.x;
            marker.pose.position.y = zone.position.y;
            marker.pose.position.z = 0.15;  // Half height above ground
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = 0.3;  // Diameter
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;  // Height
            
            // Color based on zone number (cycle through colors)
            float hue = (i * 60.0f) / 360.0f;  // 60° apart in HSV
            marker.color.r = (i % 3 == 0) ? 1.0f : 0.3f;
            marker.color.g = (i % 3 == 1) ? 1.0f : 0.3f;
            marker.color.b = (i % 3 == 2) ? 1.0f : 0.3f;
            marker.color.a = 0.7;
            
            marker.lifetime = rclcpp::Duration::from_seconds(0);  // Persistent
            
            marker_array.markers.push_back(marker);
            
            // Text marker for zone name
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = "map";
            text_marker.header.stamp = this->now();
            text_marker.ns = "delivery_zones";
            text_marker.id = i * 2 + 1;  // Odd IDs for text
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            
            text_marker.pose.position.x = zone.position.x;
            text_marker.pose.position.y = zone.position.y;
            text_marker.pose.position.z = 0.5;  // Above the cylinder
            text_marker.pose.orientation.w = 1.0;
            
            text_marker.scale.z = 0.15;  // Text height
            
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            
            text_marker.text = zone.name;
            text_marker.lifetime = rclcpp::Duration::from_seconds(0);
            
            marker_array.markers.push_back(text_marker);
        }
        
        marker_pub_->publish(marker_array);
    }
    
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_zones_srv_;
    rclcpp::TimerBase::SharedPtr marker_timer_;
    std::vector<DeliveryData::DeliveryZone> zones_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZoneMarkerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
