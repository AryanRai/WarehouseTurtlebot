// ============================================================================
// MTRX3760 Project 2 - 
// File: DeliveryRobot.hpp
// Description: Header for DeliveryRobot class (inherits from WarehouseRobot).
//              Handles autonomous warehouse delivery operations including
//              package pickup, routing via TSP optimization, and precise
//              AprilTag-based docking for deliveries.
// Author(s): Aryan
// Last Edited: 2025-11-02
// ============================================================================

#ifndef DELIVERY_ROBOT_HPP
#define DELIVERY_ROBOT_HPP

#include "Robot/WarehouseRobot.hpp"
#include "Robot/DeliveryStructures.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <fstream>
#include <memory>
#include <vector>

/**
 * @brief Delivery robot for warehouse package delivery operations
 * 
 * Inherits common functionality from WarehouseRobot and adds delivery-specific features:
 * - Delivery zone management (add, load, save zones)
 * - Delivery request queue management
 * - Route optimization (ordered or TSP)
 * - Multi-zone sequential delivery execution
 * - Delivery record logging
 */
class DeliveryRobot : public WarehouseRobot {
public:
    /**
     * @brief Constructor
     * @param node Shared pointer to ROS2 node
     */
    explicit DeliveryRobot(rclcpp::Node::SharedPtr node);
    
    /**
     * @brief Destructor
     */
    ~DeliveryRobot() = default;
    
    // ========================================================================
    // Main Control (Override from base)
    // ========================================================================
    
    /**
     * @brief Main update loop for delivery operations
     */
    void update() override;
    
    /**
     * @brief Start delivery operations
     */
    void startDeliveries();
    
    /**
     * @brief Stop delivery operations
     */
    void stopDeliveries();
    
    /**
     * @brief Check if currently delivering
     * @return True if delivery in progress
     */
    bool isDelivering() const { return is_delivering_; }
    
    // ========================================================================
    // Zone Management
    // ========================================================================
    
    /**
     * @brief Load delivery zones from YAML file
     * @param filename Path to YAML file
     */
    void loadZonesFromFile(const std::string& filename);
    
    /**
     * @brief Save delivery zones to YAML file
     * @param filename Path to YAML file
     */
    void saveZonesToFile(const std::string& filename);
    
    /**
     * @brief Add a new delivery zone
     * @param zone Zone to add
     */
    void addZone(const DeliveryData::DeliveryZone& zone);
    
    /**
     * @brief Clear all delivery zones
     */
    void clearZones();
    
    /**
     * @brief Get all delivery zones
     * @return Vector of delivery zones
     */
    std::vector<DeliveryData::DeliveryZone> getZones() const { return zones_; }
    
    // ========================================================================
    // Delivery Management
    // ========================================================================
    
    /**
     * @brief Add a delivery request to queue
     * @param request Delivery request to add
     */
    void addDeliveryRequest(const DeliveryData::DeliveryRequest& request);
    
    /**
     * @brief Clear delivery queue
     */
    void clearDeliveryQueue();
    
    /**
     * @brief Get current delivery queue
     * @return Vector of delivery requests
     */
    std::vector<DeliveryData::DeliveryRequest> getDeliveryQueue() const { return delivery_queue_; }
    
    /**
     * @brief Save delivery record to log file
     * @param record Delivery record to save
     */
    void saveDeliveryRecord(const DeliveryData::DeliveryRecord& record);
    
private:
    // ========================================================================
    // Delivery-Specific Data
    // ========================================================================
    std::vector<DeliveryData::DeliveryZone> zones_;
    std::vector<DeliveryData::DeliveryRequest> delivery_queue_;
    std::vector<DeliveryData::DeliveryRecord> delivery_history_;
    
    // Current delivery state
    bool is_delivering_;
    size_t current_delivery_index_;
    std::vector<DeliveryData::DeliveryZone> optimized_route_;
    
    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_delivery_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_zones_srv_;
    
    // Configuration
    std::string zones_file_;
    std::string delivery_log_file_;
    
    // ========================================================================
    // Helper Methods
    // ========================================================================
    
    /**
     * @brief Handle clicked point from RViz (for zone marking)
     * @param msg Point stamped message
     */
    void onPointClicked(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    
    /**
     * @brief Service callback to start deliveries
     */
    void onStartDeliveryService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    /**
     * @brief Service callback to save zones
     */
    void onSaveZonesService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    /**
     * @brief Optimize route (ordered sequence mode)
     * @param start Starting position
     * @param requests Delivery requests
     * @return Optimized route
     */
    std::vector<DeliveryData::DeliveryZone> optimizeRoute(
        const geometry_msgs::msg::Point& start,
        const std::vector<DeliveryData::DeliveryRequest>& requests);
    
    /**
     * @brief Optimize route using TSP
     * @param start Starting position
     * @param requests Delivery requests
     * @return TSP-optimized route
     */
    std::vector<DeliveryData::DeliveryZone> optimizeRouteTSP(
        const geometry_msgs::msg::Point& start,
        const std::vector<DeliveryData::DeliveryRequest>& requests);
    
    /**
     * @brief Find zone by name
     * @param zone_name Name of zone
     * @return Pointer to zone or nullptr if not found
     */
    DeliveryData::DeliveryZone* findZone(const std::string& zone_name);
    
    /**
     * @brief Check if robot is at zone
     * @param zone Zone to check
     * @return True if at zone
     */
    bool isAtZone(const DeliveryData::DeliveryZone& zone);
};

#endif // DELIVERY_ROBOT_HPP