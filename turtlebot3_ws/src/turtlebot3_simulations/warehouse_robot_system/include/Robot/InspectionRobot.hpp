// ============================================================================
// MTRX3760 Project 2 - 
// File: InspectionRobot.hpp
// Description: Header for InspectionRobot class (inherits from WarehouseRobot).
//              Handles autonomous warehouse inspection operations including
//              damage site inspection, AprilTag detection, exploration mode,
//              and Tier 1 Safety features.
// Author(s): Dylan George, Inez Dumas
// Last Edited: 2025-11-02
// ============================================================================

#ifndef INSPECTION_ROBOT_HPP
#define INSPECTION_ROBOT_HPP

#include "Robot/WarehouseRobot.hpp"
#include "Robot/InspectionStructures.hpp"
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <fstream>
#include <memory>
#include <vector>
#include <chrono>

/**
 * @brief Inspection robot for warehouse damage site inspection operations
 * 
 * Inherits common functionality from WarehouseRobot and adds inspection-specific features:
 * - Damage site management (add, load, save sites)
 * - Inspection request queue management
 * - AprilTag detection and recording
 * - 360° scanning at inspection sites
 * - Exploration mode with patrol point generation
 * - Tier 1 Safety: TF health monitoring and obstacle avoidance
 * - RViz marker visualization
 */
class InspectionRobot : public WarehouseRobot {
public:
    /**
     * @brief Constructor
     * @param node Shared pointer to ROS2 node
     */
    explicit InspectionRobot(rclcpp::Node::SharedPtr node);
    
    /**
     * @brief Destructor
     */
    ~InspectionRobot() = default;
    
    // ========================================================================
    // Main Control (Override from base)
    // ========================================================================
    
    /**
     * @brief Main update loop for inspection operations
     */
    void update() override;
    
    /**
     * @brief Start inspection operations
     */
    void startInspections();
    
    /**
     * @brief Stop inspection operations
     */
    void stopInspections();
    
    /**
     * @brief Check if currently inspecting
     * @return True if inspection in progress
     */
    bool isInspecting() const { return is_inspecting_; }
    
    // ========================================================================
    // Site Management
    // ========================================================================
    
    /**
     * @brief Load damage sites from YAML file
     * @param filename Path to YAML file
     */
    void loadSitesFromFile(const std::string& filename);
    
    /**
     * @brief Save damage sites to YAML file
     * @param filename Path to YAML file
     */
    void saveSitesToFile(const std::string& filename);
    
    /**
     * @brief Add a new damage site
     * @param site Site to add
     */
    void addSite(const InspectionData::DamageSite& site);
    
    /**
     * @brief Clear all damage sites
     */
    void clearSites();
    
    /**
     * @brief Get all damage sites
     * @return Vector of damage sites
     */
    std::vector<InspectionData::DamageSite> getSites() const { return damage_sites_; }
    
    // ========================================================================
    // Inspection Management
    // ========================================================================
    
    /**
     * @brief Add an inspection request to queue
     * @param request Inspection request to add
     */
    void addInspectionRequest(const InspectionData::InspectionRequest& request);
    
    /**
     * @brief Clear inspection queue
     */
    void clearInspectionQueue();
    
    /**
     * @brief Get current inspection queue
     * @return Vector of inspection requests
     */
    std::vector<InspectionData::InspectionRequest> getInspectionQueue() const { 
        return inspection_queue_; 
    }
    
    // ========================================================================
    // Exploration Mode
    // ========================================================================
    
    /**
     * @brief Enable exploration mode
     * @param enable True to enable, false to disable
     */
    void setExplorationMode(bool enable) { exploration_mode_ = enable; }
    
    /**
     * @brief Check if exploration mode is enabled
     * @return True if exploration mode enabled
     */
    bool isExplorationMode() const { return exploration_mode_; }
    
    /**
     * @brief Generate patrol points for exploration
     * @param map Occupancy grid map
     * @return Vector of patrol points
     */
    std::vector<geometry_msgs::msg::Point> generatePatrolPoints(
        const nav_msgs::msg::OccupancyGrid& map);
    
private:
    // ========================================================================
    // Inspection-Specific Data
    // ========================================================================
    std::vector<InspectionData::DamageSite> damage_sites_;
    std::vector<InspectionData::InspectionRequest> inspection_queue_;
    std::vector<InspectionData::InspectionRecord> inspection_history_;
    
    // Current inspection state
    bool is_inspecting_;
    size_t current_inspection_index_;
    std::vector<InspectionData::DamageSite> optimized_route_;
    
    // Scanning state
    bool is_scanning_;
    double scan_start_yaw_;
    rclcpp::Time scan_start_time_;
    static constexpr double SCAN_DURATION = 8.0;  // 8 seconds for full 360° scan
    static constexpr double SCAN_SPEED = 0.785;   // rad/s (45°/s for smooth scan)
    
    // Exploration mode
    bool exploration_mode_;
    std::vector<geometry_msgs::msg::Point> patrol_points_;
    size_t current_patrol_index_;
    
    // AprilTag detection
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_sub_;
    std::vector<int> detected_tags_;
    
    // RViz visualization
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_inspection_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_sites_srv_;
    
    // Tier 1 Safety - TF Health Monitoring
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Time last_tf_check_time_;
    bool tf_health_ok_;
    static constexpr double TF_CHECK_INTERVAL = 1.0;  // Check TF every 1 second
    static constexpr double TF_TIMEOUT = 2.0;          // TF considered stale after 2s
    
    // Tier 1 Safety - Obstacle Avoidance
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    bool obstacle_detected_;
    double min_obstacle_distance_;
    static constexpr double OBSTACLE_STOP_DISTANCE = 0.3;  // Stop if obstacle within 30cm
    static constexpr double OBSTACLE_SLOW_DISTANCE = 0.5;  // Slow down if within 50cm
    
    // Configuration
    std::string sites_file_;
    std::string inspection_log_file_;
    
    // ========================================================================
    // Scanning Methods
    // ========================================================================
    
    /**
     * @brief Perform 360° scan at current site
     * @return True if scan complete, false if still scanning
     */
    bool performSiteScan();
    
    /**
     * @brief Check if robot is at site
     * @param site Site to check
     * @return True if at site
     */
    bool isAtSite(const InspectionData::DamageSite& site);
    
    // ========================================================================
    // AprilTag Detection
    // ========================================================================
    
    /**
     * @brief AprilTag detection callback
     * @param msg AprilTag detection array message
     */
    void onAprilTagDetection(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
    
    /**
     * @brief Check if tag has already been detected
     * @param tag_id Tag ID to check
     * @return True if already detected
     */
    bool isTagAlreadyDetected(int tag_id) const;
    
    // ========================================================================
    // Tier 1 Safety Methods
    // ========================================================================
    
    /**
     * @brief Check TF transform health
     * @return True if TF is healthy
     */
    bool checkTFHealth();
    
    /**
     * @brief Laser scan callback for obstacle detection
     * @param msg Laser scan message
     */
    void onLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    /**
     * @brief Check for obstacles in path
     * @return True if safe to proceed
     */
    bool isSafeToMove();
    
    /**
     * @brief Emergency stop due to safety issue
     * @param reason Reason for emergency stop
     */
    void emergencyStop(const std::string& reason);
    
    // ========================================================================
    // Route Optimization
    // ========================================================================
    
    /**
     * @brief Optimize route (ordered sequence mode)
     * @param start Starting position
     * @param requests Inspection requests
     * @return Optimized route
     */
    std::vector<InspectionData::DamageSite> optimizeRoute(
        const geometry_msgs::msg::Point& start,
        const std::vector<InspectionData::InspectionRequest>& requests);
    
    /**
     * @brief Optimize route using TSP
     * @param start Starting position
     * @param requests Inspection requests
     * @return TSP-optimized route
     */
    std::vector<InspectionData::DamageSite> optimizeRouteTSP(
        const geometry_msgs::msg::Point& start,
        const std::vector<InspectionData::InspectionRequest>& requests);
    
    // ========================================================================
    // Exploration Methods
    // ========================================================================
    
    /**
     * @brief Execute exploration behavior
     */
    void executeExploration();
    
    /**
     * @brief Check if point is valid for patrol
     * @param point Point to check
     * @param map Occupancy grid map
     * @return True if valid patrol point
     */
    bool isValidPatrolPoint(
        const geometry_msgs::msg::Point& point,
        const nav_msgs::msg::OccupancyGrid& map);
    
    // ========================================================================
    // Visualization Methods
    // ========================================================================
    
    /**
     * @brief Publish RViz markers for damage sites
     */
    void publishSiteMarkers();
    
    /**
     * @brief Create marker for damage site
     * @param site Damage site
     * @param id Marker ID
     * @return Visualization marker
     */
    visualization_msgs::msg::Marker createSiteMarker(
        const InspectionData::DamageSite& site, 
        int id);
    
    // ========================================================================
    // Helper Methods
    // ========================================================================
    
    /**
     * @brief Handle clicked point from RViz (for site marking)
     * @param msg Point stamped message
     */
    void onPointClicked(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    
    /**
     * @brief Service callback to start inspections
     */
    void onStartInspectionService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    /**
     * @brief Service callback to save sites
     */
    void onSaveSitesService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    /**
     * @brief Find site by name
     * @param site_name Name of site
     * @return Pointer to site or nullptr if not found
     */
    InspectionData::DamageSite* findSite(const std::string& site_name);
};

#endif // INSPECTION_ROBOT_HPP