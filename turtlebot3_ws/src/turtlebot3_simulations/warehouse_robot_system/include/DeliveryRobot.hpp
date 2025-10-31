#ifndef DELIVERY_ROBOT_HPP
#define DELIVERY_ROBOT_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include "DeliveryStructures.hpp"
#include "SlamController.hpp"
#include "MotionController.hpp"
#include "PathPlanner.hpp"
#include <fstream>
#include <memory>
#include <vector>
#include <queue>

class DeliveryRobot {
public:
    DeliveryRobot(rclcpp::Node::SharedPtr node);
    ~DeliveryRobot() = default;
    
    // Main control
    void update();
    void startDeliveries();
    void stopDeliveries();
    bool isDelivering() const { return is_delivering_; }
    bool hasValidMap() const;
    
    // Zone management
    void loadZonesFromFile(const std::string& filename);
    void saveZonesToFile(const std::string& filename);
    void addZone(const DeliveryZone& zone);
    void clearZones();
    
    // Delivery management
    void addDeliveryRequest(const DeliveryRequest& request);
    void clearDeliveryQueue();
    std::vector<DeliveryRequest> getDeliveryQueue() const { return delivery_queue_; }
    
    // Records
    void saveDeliveryRecord(const DeliveryRecord& record);
    
private:
    rclcpp::Node::SharedPtr node_;
    
    // SLAM components (reuse existing)
    std::unique_ptr<SlamController> slam_controller_;
    std::unique_ptr<MotionController> motion_controller_;
    
    // Delivery data
    std::vector<DeliveryZone> zones_;
    std::vector<DeliveryRequest> delivery_queue_;
    std::vector<DeliveryRecord> delivery_history_;
    
    // Current delivery state
    bool is_delivering_;
    size_t current_delivery_index_;
    std::vector<DeliveryZone> optimized_route_;
    DeliveryRequest current_request_;
    std::chrono::steady_clock::time_point delivery_start_time_;
    geometry_msgs::msg::Point delivery_start_position_;
    double total_distance_;
    
    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_delivery_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_zones_srv_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    // Configuration
    std::string zones_file_;
    std::string delivery_log_file_;
    static constexpr double ZONE_REACHED_THRESHOLD = 0.3;  // 30cm
    static constexpr double UPDATE_RATE = 10.0;  // Hz
    
    // Docking parameters (same as exploration robot)
    static constexpr double DOCKING_DISTANCE = 0.5;  // Enter docking mode within 50cm
    static constexpr double HOME_TOLERANCE = 0.05;   // Success within 5cm
    static constexpr double DOCKING_LINEAR_SPEED = 0.05;  // Slow speed for docking
    static constexpr double DOCKING_ANGULAR_SPEED = 0.3;  // Moderate rotation for alignment
    
    // Docking state
    bool in_docking_mode_;
    double initial_yaw_;  // Store initial orientation to return to
    bool has_relocalized_;  // Track if we've done initial spin
    rclcpp::Time relocalization_start_time_;
    static constexpr double RELOCALIZATION_DURATION = 8.0;  // 8 seconds for 2 full rotations
    static constexpr double RELOCALIZATION_SPEED = 1.57;  // rad/s (~90°/s, 2 full rotations in 8s)
    
    // Helper methods
    void onPointClicked(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void onStartDeliveryService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void onSaveZonesService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    std::vector<DeliveryZone> optimizeRoute(
        const geometry_msgs::msg::Point& start,
        const std::vector<DeliveryRequest>& requests);
    
    DeliveryZone* findZone(const std::string& zone_name);
    bool navigateToZone(const DeliveryZone& zone);
    bool isAtZone(const DeliveryZone& zone);
    void returnToHome();
    void preciseDocking(const geometry_msgs::msg::Pose& current_pose, double distance_to_home);
    void publishStatus(const std::string& status);
    std::string getCurrentTimestamp();
};

#endif // DELIVERY_ROBOT_HPP
