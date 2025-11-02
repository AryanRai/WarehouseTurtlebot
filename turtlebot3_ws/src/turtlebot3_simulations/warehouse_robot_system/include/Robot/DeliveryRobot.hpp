#ifndef DELIVERY_ROBOT_HPP
#define DELIVERY_ROBOT_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include "Robot/DeliveryStructures.hpp"
#include "SLAM/SlamController.hpp"
#include "SLAM/MotionController.hpp"
#include "SLAM/PathPlanner.hpp"
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
    std::vector<DeliveryZone> getZones() const { return zones_; }
    
    // Delivery management
    void addDeliveryRequest(const DeliveryRequest& request);
    void clearDeliveryQueue();
    std::vector<DeliveryRequest> getDeliveryQueue() const { return delivery_queue_; }
    
    // Route optimization mode
    void setOptimizationMode(bool use_tsp) { use_tsp_optimization_ = use_tsp; }
    bool isUsingTSP() const { return use_tsp_optimization_; }
    
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
    bool zone_path_completed_;  // Track if we've completed path to current zone
    
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
    static constexpr double ZONE_TOLERANCE = 0.08;   // Success within 8cm for delivery zones
    static constexpr double DOCKING_LINEAR_SPEED = 0.05;  // Slow speed for docking
    static constexpr double DOCKING_ANGULAR_SPEED = 0.3;  // Moderate rotation for alignment
    static constexpr double ZONE_DOCKING_DISTANCE = 0.25;  // Enter zone docking within 25cm (larger than lookahead)
    
    // Docking state
    bool in_docking_mode_;
    bool in_zone_docking_mode_;  // Separate flag for zone docking
    double initial_yaw_;  // Store initial orientation to return to
    bool has_relocalized_;  // Track if we've done initial spin
    rclcpp::Time relocalization_start_time_;
    static constexpr double RELOCALIZATION_DURATION = 8.0;  // 8 seconds for 2 full rotations
    static constexpr double RELOCALIZATION_SPEED = 1.57;  // rad/s (~90°/s, 2 full rotations in 8s)
    
    // Route optimization mode
    bool use_tsp_optimization_;
    
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
    
    std::vector<DeliveryZone> optimizeRouteTSP(
        const geometry_msgs::msg::Point& start,
        const std::vector<DeliveryRequest>& requests);
    
    std::vector<std::vector<double>> buildDistanceMatrix(
        const geometry_msgs::msg::Point& start,
        const std::vector<geometry_msgs::msg::Point>& points);
    
    std::vector<int> simulatedAnnealing(
        const std::vector<std::vector<double>>& distance_matrix,
        int start_idx,
        double initial_temp = 10000.0,
        double cooling_rate = 0.995,
        int max_iterations = 10000);
    
    double calculateTourCost(
        const std::vector<int>& tour,
        const std::vector<std::vector<double>>& distance_matrix);
    
    DeliveryZone* findZone(const std::string& zone_name);
    bool navigateToZone(const DeliveryZone& zone);
    bool isAtZone(const DeliveryZone& zone);
    void returnToHome();
    void preciseDocking(const geometry_msgs::msg::Pose& current_pose, double distance_to_home);
    void preciseZoneDocking(const geometry_msgs::msg::Pose& current_pose, 
                           const DeliveryZone& zone, 
                           double distance_to_zone);
    bool hasLineOfSight(const geometry_msgs::msg::Point& from, 
                       const geometry_msgs::msg::Point& to,
                       const nav_msgs::msg::OccupancyGrid& map);
    double checkMinDistanceToWalls(const geometry_msgs::msg::Point& position,
                                   const nav_msgs::msg::OccupancyGrid& map);
    void publishStatus(const std::string& status);
    std::string getCurrentTimestamp();
};

#endif // DELIVERY_ROBOT_HPP