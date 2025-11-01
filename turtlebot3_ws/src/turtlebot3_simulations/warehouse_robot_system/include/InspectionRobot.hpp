// ============================================================================
// File: InspectionRobot.hpp
// Description: Inspection robot for warehouse damage detection using camera
//              and AprilTag markers. Supports exploration mode to discover
//              damage sites and inspection mode to visit selected sites.
// Author(s): Warehouse Robot System Team
// Last Edited: 2025-11-01
// ============================================================================

#ifndef INSPECTION_ROBOT_HPP
#define INSPECTION_ROBOT_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include "InspectionStructures.hpp"
#include "SlamController.hpp"
#include "MotionController.hpp"
#include "PathPlanner.hpp"
#include <fstream>
#include <memory>
#include <vector>
#include <queue>

class InspectionRobot {
public:
    InspectionRobot(rclcpp::Node::SharedPtr node);
    ~InspectionRobot() = default;
    
    // Main control
    void update();
    void startInspections();
    void stopInspections();
    bool isInspecting() const { return is_inspecting_; }
    bool hasValidMap() const;
    
    // Damage site management
    void loadSitesFromFile(const std::string& filename);
    void saveSitesToFile(const std::string& filename);
    void addSite(const DamageSite& site);
    void clearSites();
    std::vector<DamageSite> getSites() const { return damage_sites_; }
    
    // Inspection management
    void addInspectionRequest(const InspectionRequest& request);
    void clearInspectionQueue();
    std::vector<InspectionRequest> getInspectionQueue() const { return inspection_queue_; }
    
    // Route optimization mode
    void setOptimizationMode(bool use_tsp) { use_tsp_optimization_ = use_tsp; }
    bool isUsingTSP() const { return use_tsp_optimization_; }
    
    // Exploration mode
    void setExplorationMode(bool exploration) { exploration_mode_ = exploration; }
    bool isExplorationMode() const { return exploration_mode_; }
    void startExploration();
    
    // Records
    void saveInspectionRecord(const InspectionRecord& record);
    
private:
    rclcpp::Node::SharedPtr node_;
    
    // SLAM components (reuse existing)
    std::unique_ptr<SlamController> slam_controller_;
    std::unique_ptr<MotionController> motion_controller_;
    
    // Inspection data
    std::vector<DamageSite> damage_sites_;
    std::vector<InspectionRequest> inspection_queue_;
    std::vector<InspectionRecord> inspection_history_;
    
    // Current inspection state
    bool is_inspecting_;
    size_t current_inspection_index_;
    std::vector<DamageSite> optimized_route_;
    InspectionRequest current_request_;
    std::chrono::steady_clock::time_point inspection_start_time_;
    geometry_msgs::msg::Point inspection_start_position_;
    double total_distance_;
    bool site_path_completed_;  // Track if we've completed path to current site
    
    // AprilTag detection
    bool tag_detected_at_site_;
    int last_detected_tag_id_;
    rclcpp::Time last_tag_detection_time_;
    
    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_inspection_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_sites_srv_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    // Configuration
    std::string sites_file_;
    std::string inspection_log_file_;
    static constexpr double SITE_REACHED_THRESHOLD = 0.3;  // 30cm
    static constexpr double UPDATE_RATE = 10.0;  // Hz
    
    // Docking parameters (same as delivery robot)
    static constexpr double DOCKING_DISTANCE = 0.5;  // Enter docking mode within 50cm
    static constexpr double HOME_TOLERANCE = 0.05;   // Success within 5cm
    static constexpr double SITE_TOLERANCE = 0.08;   // Success within 8cm for damage sites
    static constexpr double DOCKING_LINEAR_SPEED = 0.05;  // Slow speed for docking
    static constexpr double DOCKING_ANGULAR_SPEED = 0.3;  // Moderate rotation for alignment
    static constexpr double SITE_DOCKING_DISTANCE = 0.25;  // Enter site docking within 25cm
    
    // Tag reading parameters
    static constexpr double TAG_READING_DURATION = 3.0;  // Seconds to read tag at site
    static constexpr double TAG_DETECTION_TIMEOUT = 5.0;  // Max seconds to wait for tag
    
    // Docking state
    bool in_docking_mode_;
    bool in_site_docking_mode_;  // Separate flag for site docking
    double initial_yaw_;  // Store initial orientation to return to
    bool has_relocalized_;  // Track if we've done initial spin
    rclcpp::Time relocalization_start_time_;
    bool is_reading_tag_;
    rclcpp::Time tag_reading_start_time_;
    static constexpr double RELOCALIZATION_DURATION = 8.0;  // 8 seconds for 2 full rotations
    static constexpr double RELOCALIZATION_SPEED = 1.57;  // rad/s (~90°/s)
    
    // Route optimization mode
    bool use_tsp_optimization_;
    
    // Exploration mode
    bool exploration_mode_;
    std::vector<geometry_msgs::msg::Point> patrol_points_;
    size_t current_patrol_index_;
    
    // Helper methods
    void onPointClicked(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void onAprilTagDetection(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
    void onStartInspectionService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void onSaveSitesService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    std::vector<DamageSite> optimizeRoute(
        const geometry_msgs::msg::Point& start,
        const std::vector<InspectionRequest>& requests);
    
    std::vector<DamageSite> optimizeRouteTSP(
        const geometry_msgs::msg::Point& start,
        const std::vector<InspectionRequest>& requests);
    
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
    
    DamageSite* findSite(const std::string& site_name);
    bool navigateToSite(const DamageSite& site);
    bool isAtSite(const DamageSite& site);
    bool readAprilTag();
    void returnToHome();
    void preciseDocking(const geometry_msgs::msg::Pose& current_pose, double distance_to_home);
    void preciseSiteDocking(const geometry_msgs::msg::Pose& current_pose, 
                           const DamageSite& site, 
                           double distance_to_site);
    bool hasLineOfSight(const geometry_msgs::msg::Point& from, 
                       const geometry_msgs::msg::Point& to,
                       const nav_msgs::msg::OccupancyGrid& map);
    
    // Exploration mode helpers
    void generatePatrolPoints(const nav_msgs::msg::OccupancyGrid& map);
    bool navigateToPatrolPoint(const geometry_msgs::msg::Point& point);
    void processAprilTagDetections();
    void saveDiscoveredSite(int tag_id, const geometry_msgs::msg::Point& position);
    double checkMinDistanceToWalls(const geometry_msgs::msg::Point& position,
                                   const nav_msgs::msg::OccupancyGrid& map);
    void publishStatus(const std::string& status);
    std::string getCurrentTimestamp();
};

#endif // INSPECTION_ROBOT_HPP
