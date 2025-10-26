// MTRX3760 2025 Project 2: SLAM State Manager
// File: slam_state_manager.cpp
// Author(s): Aryan Rai
//
// Implementation of SLAM state management

#include "slam_state_manager.hpp"

namespace slam {

SlamStateManager::SlamStateManager(rclcpp::Node* node)
    : node_(node),
      current_state_(SlamState::INITIALIZING),
      mapping_state_(MappingState::SEARCHING_FRONTIERS)
{
    state_start_time_ = node_->now();
    last_progress_time_ = node_->now();
    exploration_start_time_ = node_->now();
}

void SlamStateManager::transitionToState(SlamState new_state) {
    SlamState old_state = current_state_;
    current_state_ = new_state;
    state_start_time_ = node_->now();
    
    logStateTransition(old_state, new_state);
}

void SlamStateManager::transitionToMappingState(MappingState new_mapping_state) {
    MappingState old_state = mapping_state_;
    mapping_state_ = new_mapping_state;
    
    logMappingStateTransition(old_state, new_mapping_state);
}

void SlamStateManager::forceOperationalMode() {
    RCLCPP_INFO(node_->get_logger(), "Forcing transition to operational mode");
    transitionToState(SlamState::OPERATIONAL);
}

double SlamStateManager::getTimeSinceStateStart() const {
    return (node_->now() - state_start_time_).seconds();
}

double SlamStateManager::getTimeSinceLastProgress() const {
    return (node_->now() - last_progress_time_).seconds();
}

void SlamStateManager::updateProgressTime() {
    last_progress_time_ = node_->now();
}

bool SlamStateManager::hasExplorationTimedOut(double timeout_s) const {
    return (node_->now() - exploration_start_time_).seconds() > timeout_s;
}

void SlamStateManager::logStateTransition(SlamState old_state, SlamState new_state) {
    RCLCPP_INFO(node_->get_logger(), "State transition: %s -> %s",
                stateToString(old_state).c_str(), stateToString(new_state).c_str());
}

void SlamStateManager::logMappingStateTransition(MappingState old_state, MappingState new_state) {
    RCLCPP_DEBUG(node_->get_logger(), "Mapping state transition: %s -> %s",
                 mappingStateToString(old_state).c_str(), mappingStateToString(new_state).c_str());
}

std::string SlamStateManager::stateToString(SlamState state) const {
    switch (state) {
        case SlamState::INITIALIZING: return "INITIALIZING";
        case SlamState::MAPPING: return "MAPPING";
        case SlamState::RETURNING_HOME: return "RETURNING_HOME";
        case SlamState::OPERATIONAL: return "OPERATIONAL";
        case SlamState::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

std::string SlamStateManager::mappingStateToString(MappingState state) const {
    switch (state) {
        case MappingState::SEARCHING_FRONTIERS: return "SEARCHING_FRONTIERS";
        case MappingState::PLANNING_PATH: return "PLANNING_PATH";
        case MappingState::NAVIGATING: return "NAVIGATING";
        case MappingState::EXPLORING_AREA: return "EXPLORING_AREA";
        case MappingState::STUCK_RECOVERY: return "STUCK_RECOVERY";
        default: return "UNKNOWN";
    }
}

} // namespace slam