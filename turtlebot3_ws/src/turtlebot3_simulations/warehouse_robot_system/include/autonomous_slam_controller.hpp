// MTRX3760 2025 Project 2: Autonomous SLAM Controller (Refactored)
// File: autonomous_slam_controller.hpp
// Author(s): Aryan Rai
//
// Refactored header file - uses separate manager classes
// NOTE: This replaces the original autonomous_slam_controller.hpp completely

#ifndef AUTONOMOUS_SLAM_CONTROLLER_HPP
#define AUTONOMOUS_SLAM_CONTROLLER_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "slam_state_manager.hpp"
#include "navigation_controller.hpp"
#include "exploration_manager.hpp"

namespace slam {

/**
 * @class AutonomousSlamController
 * @brief Main controller for autonomous SLAM - orchestrates state management,
 *        navigation, and exploration through dedicated manager classes
 * 
 * REFACTORED ARCHITECTURE:
 * This class now delegates responsibilities to specialized managers:
 * - SlamStateManager: Handles state transitions and state logic
 * - NavigationController: Handles robot motion and path following
 * - ExplorationManager: Handles frontier detection and path planning
 * - AutonomousSlamController: Orchestrates high-level behavior
 * 
 * This design follows the Single Responsibility Principle and makes
 * the codebase more maintainable and testable.
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
    SlamState getCurrentState() const { return state_manager_->getCurrentState(); }

    /**
     * @brief Check if mapping is complete
     */
    bool isMappingComplete() const { return state_manager_->isMappingComplete(); }

    /**
     * @brief Force return to operational mode
     */
    void forceOperationalMode() { state_manager_->forceOperationalMode(); }

private:
    // Configuration
    struct Config {
        double exploration_timeout_s = 300.0;
        double control_rate_hz = 2.0;
    } config_;

    // Manager components (composition for separation of concerns)
    std::unique_ptr<SlamStateManager> state_manager_;
    std::unique_ptr<NavigationController> navigation_controller_;
    std::unique_ptr<ExplorationManager> exploration_manager_;

    ExplorationManager::Config exp_config_;

    // ROS2 interfaces
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;

    // SLAM data
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    geometry_msgs::msg::Point origin_point_;
    
    /**
     * @brief Main state machine execution
     */
    void executeStateMachine();

    /**
     * @brief High-level state handlers
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
    void publishGoalVisualization(const geometry_msgs::msg::Point& goal);
    void updateTravelDistance();
};

} // namespace slam

#endif // AUTONOMOUS_SLAM_CONTROLLER_HPP