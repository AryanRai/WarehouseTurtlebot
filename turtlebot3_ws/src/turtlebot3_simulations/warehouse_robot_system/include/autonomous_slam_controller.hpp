// MTRX3760 2025 Project 2: Autonomous SLAM Controller
// File: autonomous_slam_controller.hpp
// Author(s): Aryan Rai
//
// Header file for autonomous SLAM system with frontier exploration

#ifndef AUTONOMOUS_SLAM_CONTROLLER_HPP
#define AUTONOMOUS_SLAM_CONTROLLER_HPP

#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "slam_types.hpp"
#include "frontier_search.hpp"
#include "path_planner.hpp"

namespace slam {

/**
 * @brief High-level SLAM system states
 */
enum class SlamState {
    INITIALIZING,     ///< System starting up, waiting for map and pose
    MAPPING,          ///< Actively exploring and mapping environment
    RETURNING_HOME,   ///< Finished mapping, returning to origin
    OPERATIONAL,      ///< At origin, ready for warehouse operations
    ERROR             ///< Error state
};

/**
 * @brief Mapping sub-states for detailed control
 */
enum class MappingState {
    SEARCHING_FRONTIERS,  ///< Looking for unexplored areas
    PLANNING_PATH,        ///< Computing path to selected frontier
    NAVIGATING,           ///< Following path to frontier
    EXPLORING_AREA,       ///< Local exploration at frontier
    STUCK_RECOVERY        ///< Recovering from stuck situation
};

/**
 * @class AutonomousSlamController
 * @brief Main controller for autonomous SLAM with frontier exploration
 * 
 * Implements a high-level state machine that:
 * 1. Explores environment using frontier detection
 * 2. Plans optimal paths using A* algorithm
 * 3. Returns to origin when mapping is complete
 * 4. Transitions to operational mode for warehouse tasks
 */
class AutonomousSlamController : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     */
    AutonomousSlamController();

    /**
     * @brief Destructor
     */
    ~AutonomousSlamController();

    /**
     * @brief Main control loop
     */
    void run();

    /**
     * @brief Get current SLAM state
     */
    SlamState getCurrentState() const { return current_state_; }

    /**
     * @brief Check if mapping is complete
     */
    bool isMappingComplete() const { return current_state_ == SlamState::OPERATIONAL; }

    /**
     * @brief Force return to operational mode
     */
    void forceOperationalMode();

private:
    // Configuration parameters
    struct Config {
        double exploration_timeout_s = 300.0;        ///< Max time for exploration (5 min)
        double frontier_min_size = 8;                ///< Minimum frontier size to consider
        double goal_tolerance = 0.15;                ///< Distance tolerance for reaching goals
        double origin_tolerance = 0.2;               ///< Tolerance for returning to origin
        double max_linear_velocity = 0.15;           ///< Maximum forward speed
        double max_angular_velocity = 0.8;           ///< Maximum turn rate
        double stuck_timeout_s = 30.0;               ///< Time before considering robot stuck
        double frontier_search_rate_hz = 2.0;        ///< Rate for frontier detection
        int max_no_frontier_count = 15;              ///< Max consecutive no-frontier detections
        int max_no_path_count = 10;                  ///< Max consecutive path planning failures
        double a_star_cost_weight = 10.0;            ///< A* path cost weighting
        double frontier_size_weight = 1.0;           ///< Frontier size importance weighting
        int max_frontiers_to_check = 8;              ///< Limit frontiers evaluated per cycle
    } config_;

    // ROS2 interfaces
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    // State management
    SlamState current_state_;
    MappingState mapping_state_;
    rclcpp::Time state_start_time_;
    rclcpp::Time last_progress_time_;

    // SLAM data
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::Point origin_point_;
    nav_msgs::msg::Path current_path_;
    geometry_msgs::msg::Point current_goal_;
    
    // Frontier exploration
    std::unique_ptr<FrontierSearch> frontier_searcher_;
    std::unique_ptr<PathPlanner> path_planner_;
    std::vector<Frontier> detected_frontiers_;
    
    // Progress tracking
    int consecutive_no_frontiers_;
    int consecutive_no_paths_;
    double last_distance_to_goal_;
    rclcpp::Time last_movement_time_;
    
    // Statistics
    int total_frontiers_explored_;
    double total_distance_traveled_;
    rclcpp::Time exploration_start_time_;

    /**
     * @brief Main state machine execution
     */
    void executeStateMachine();

    /**
     * @brief State transition handlers
     */
    void handleInitializingState();
    void handleMappingState();
    void handleReturningHomeState();
    void handleOperationalState();
    void handleErrorState();

    /**
     * @brief Mapping sub-state handlers
     */
    void handleSearchingFrontiers();
    void handlePlanningPath();
    void handleNavigating();
    void handleExploringArea();
    void handleStuckRecovery();

    /**
     * @brief ROS2 callback functions
     */
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Utility functions
     */
    bool updateCurrentPose();
    void publishVelocityCommand(double linear, double angular);
    void publishPath(const std::vector<GridCell>& path);
    void stopRobot();
    
    /**
     * @brief Frontier exploration functions
     */
    bool searchForFrontiers();
    Frontier selectBestFrontier(const std::vector<Frontier>& frontiers);
    bool planPathToGoal(const geometry_msgs::msg::Point& goal);
    bool isGoalReached(const geometry_msgs::msg::Point& goal, double tolerance);
    
    /**
     * @brief Navigation functions
     */
    bool followCurrentPath();
    geometry_msgs::msg::Point calculateLookaheadPoint();
    void calculatePurePursuitControl(const geometry_msgs::msg::Point& lookahead);
    
    /**
     * @brief State transition functions
     */
    void transitionToState(SlamState new_state);
    void transitionToMappingState(MappingState new_mapping_state);
    
    /**
     * @brief Progress monitoring
     */
    bool isRobotStuck();
    bool hasExplorationTimedOut();
    void updateProgressTracking();
    
    /**
     * @brief Utility calculations
     */
    double calculateDistance(const geometry_msgs::msg::Point& p1, 
                           const geometry_msgs::msg::Point& p2);
    double calculateYawFromPose(const geometry_msgs::msg::PoseStamped& pose);
    geometry_msgs::msg::Point poseToPoint(const geometry_msgs::msg::PoseStamped& pose);
    
    /**
     * @brief Logging and diagnostics
     */
    void logStateTransition(SlamState old_state, SlamState new_state);
    void logMappingStateTransition(MappingState old_state, MappingState new_state);
    void printExplorationStatistics();
    std::string stateToString(SlamState state);
    std::string mappingStateToString(MappingState state);
};

} // namespace slam

#endif // AUTONOMOUS_SLAM_CONTROLLER_HPP