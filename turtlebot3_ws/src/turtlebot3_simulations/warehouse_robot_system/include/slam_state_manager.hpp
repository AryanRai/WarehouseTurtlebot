// MTRX3760 2025 Project 2: SLAM State Manager
// File: slam_state_manager.hpp
// Author(s): Aryan Rai
//
// Manages high-level SLAM system states and transitions

#ifndef SLAM_STATE_MANAGER_HPP
#define SLAM_STATE_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <functional>

namespace slam {

/**
 * @brief High-level SLAM system states
 */
enum class SlamState {
    INITIALIZING,
    MAPPING,
    RETURNING_HOME,
    OPERATIONAL,
    ERROR
};

/**
 * @brief Mapping sub-states for detailed control
 */
enum class MappingState {
    SEARCHING_FRONTIERS,
    PLANNING_PATH,
    NAVIGATING,
    EXPLORING_AREA,
    STUCK_RECOVERY
};

/**
 * @class SlamStateManager
 * @brief Manages state transitions and state-related logic for SLAM system
 */
class SlamStateManager {
public:
    SlamStateManager(rclcpp::Node* node);

    // State queries
    SlamState getCurrentState() const { return current_state_; }
    MappingState getMappingState() const { return mapping_state_; }
    bool isMappingComplete() const { return current_state_ == SlamState::OPERATIONAL; }
    
    // State transitions
    void transitionToState(SlamState new_state);
    void transitionToMappingState(MappingState new_mapping_state);
    void forceOperationalMode();
    
    // Time tracking
    double getTimeSinceStateStart() const;
    double getTimeSinceLastProgress() const;
    void updateProgressTime();
    rclcpp::Time getExplorationStartTime() const { return exploration_start_time_; }
    
    // Timeout checking
    bool hasExplorationTimedOut(double timeout_s) const;
    
    // State string conversions
    std::string stateToString(SlamState state) const;
    std::string mappingStateToString(MappingState state) const;

private:
    rclcpp::Node* node_;
    
    SlamState current_state_;
    MappingState mapping_state_;
    
    rclcpp::Time state_start_time_;
    rclcpp::Time last_progress_time_;
    rclcpp::Time exploration_start_time_;
    
    void logStateTransition(SlamState old_state, SlamState new_state);
    void logMappingStateTransition(MappingState old_state, MappingState new_state);
};

} // namespace slam

#endif // SLAM_STATE_MANAGER_HPP