// Autonomous Exploration Robot - SLAM-based autonomous mapping
// Integrates SlamController, ExplorationPlanner, and MotionController

#ifndef AUTONOMOUS_EXPLORATION_ROBOT_HPP
#define AUTONOMOUS_EXPLORATION_ROBOT_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "SlamController.hpp"
#include "ExplorationPlanner.hpp"
#include "MotionController.hpp"

class AutonomousExplorationRobot {
public:
    AutonomousExplorationRobot(rclcpp::Node::SharedPtr node);
    ~AutonomousExplorationRobot() = default;
    
    // Main control loop
    void update();
    
    // Control
    void startExploration();
    void stopExploration();
    void pauseExploration();
    void resumeExploration();
    
    // Status
    bool isExplorationComplete() const;
    bool isExploring() const;
    
    // Map management
    void saveMap(const std::string& map_name = "warehouse_map");
    
private:
    rclcpp::Node::SharedPtr node_;
    
    // SLAM components
    std::unique_ptr<SlamController> slam_controller_;
    std::unique_ptr<ExplorationPlanner> exploration_planner_;
    std::unique_ptr<MotionController> motion_controller_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr recovery_cmd_vel_pub_;
    
    // State
    bool is_exploring_;
    bool is_paused_;
    rclcpp::Time last_update_time_;
    rclcpp::Time last_replan_time_;
    geometry_msgs::msg::Point last_goal_position_;
    int consecutive_no_path_count_;
    rclcpp::Time recovery_start_time_;
    bool in_recovery_;
    int recovery_attempt_;  // Track which recovery attempt (0=forward, 1=backward, etc.)
    
    // Configuration
    static constexpr double UPDATE_RATE = 20.0;  // Hz
    static constexpr double MIN_REPLAN_INTERVAL = 2.0;  // seconds - minimum time between replans
    static constexpr int MAX_NO_PATH_BEFORE_RECOVERY = 5;  // attempts before recovery
    static constexpr double RECOVERY_DURATION = 3.0;  // seconds to rotate during recovery
    
    // Recovery behavior
    void performRecovery();
};

#endif // AUTONOMOUS_EXPLORATION_ROBOT_HPP
