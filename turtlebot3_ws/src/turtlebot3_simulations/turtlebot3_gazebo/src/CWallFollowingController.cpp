/*
 * File: CWallFollowingController.cpp
 * Description: Implements the CWallFollowingController class for 
   wall-following, obstacle avoidance, and goal detection in TurtleBot3 
   Gazebo simulation.
 * 
 * Date: 2025-10-10
 *
 * This file contains the state machine logic for wall-following, 
   PID control, loop detection, and red target goal finding.
 */

#include "turtlebot3_gazebo/CWallFollowingController.hpp"
#include "turtlebot3_gazebo/CTurtlebot3Sensor.hpp"
#include "turtlebot3_gazebo/CTurtlebot3Drive.hpp"
#include "turtlebot3_gazebo/CPIDController.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


// CWallFollowingController SConfiguration Constructor
CWallFollowingController::SConfiguration::SConfiguration()
  : mDesired_wall_distance(0.5),
    mHard_stop_distance(0.18),
    mSide_panic_distance(0.18),
    mSlow_down_distance(0.80),
    mStop_distance(0.25),
    mTight_corner_threshold(0.4),
    mNominal_velocity(0.20),
    mMax_angular_velocity(0.6),
    mBackup_speed(-0.12),
    mBackup_duration(0.6),
    mTurn_trigger_angle(2.0 * M_PI * 0.98),
    mForward_reset_distance(1.0),
    mRed_x_tolerance(0.1),
    mRed_stop_distance(0.5),
    mRed_lock_frames(2),
    mPID_kp(0.9),
    mPID_ki(0.1),
    mPID_kd(0.5)
{
  // Constructor initializes all configuration values
}

// CWallFollowingController Constructor
CWallFollowingController::CWallFollowingController(rclcpp::Node* aNode, CTurtlebot3Sensor* aSensor)
  : sConfig_(),
    node_(aNode),
    cSensor_(aSensor),
    state_(RIGHT_WALL_FOLLOW),
    ref_x_(0.0),
    ref_y_(0.0),
    spin_ref_yaw_(0.0),
    have_spin_ref_(false),
    first_update_(true),
    was_left_mode_(false),
    red_seen_(false),
    red_ahead_lock_(0),
    recovery_end_time_(0, 0, RCL_ROS_TIME),
    last_pose_record_time_(0, 0, RCL_ROS_TIME),
    last_loop_check_time_(0, 0, RCL_ROS_TIME)
{

  // Initialize PID controller
  cWall_follow_pid_ = std::make_unique<CPIDController>(
    sConfig_.mPID_kp,
    sConfig_.mPID_ki,
    sConfig_.mPID_kd,
    sConfig_.mMax_angular_velocity
  );

  // Subscribe to red detector topics
  red_seen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/red_detector/seen", 
      10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg != nullptr)
        {
          red_seen_ = msg->data;
          RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                "Red seen update: %s", red_seen_ ? "TRUE" : "FALSE");
        }
      });

  red_centroid_sub_ = node_->create_subscription<geometry_msgs::msg::Point>(
      "/red_detector/centroid", 10, [this](geometry_msgs::msg::Point::ConstSharedPtr msg) {
        if (msg != nullptr)
        {
          red_centroid_ = *msg;
          RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                "Red centroid update: x=%.3f, y=%.3f, z=%.3f", 
                                red_centroid_.x, red_centroid_.y, red_centroid_.z);
        }
      });

  RCLCPP_INFO(node_->get_logger(), "CWallFollowingController initialized");
}

// CWallFollowingController Destructor
CWallFollowingController::~CWallFollowingController()
{
  if (node_ != nullptr)
  {
    RCLCPP_INFO(node_->get_logger(), "CWallFollowingController closed");
  }
}

// Get current state
CWallFollowingController::eState CWallFollowingController::get_state() const
{
  return state_;
}

// Compute velocity command based on current state and sensor data
CWallFollowingController::SVelocityCommand CWallFollowingController::compute_velocity(double aDelta_time)
{
  // DISABLE WALL FOLLOWING FOR SLAM - Set to false to re-enable wall following
  static const bool WALL_FOLLOWING_ENABLED = false;
  
  if (!WALL_FOLLOWING_ENABLED) {
    SVelocityCommand sCmd;
    sCmd.mLinear = 0.0;
    sCmd.mAngular = 0.0;
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                          "Wall following disabled - robot ready for SLAM teleop control");
    return sCmd;
  }

  // Validate input
  if (aDelta_time <= 0.0)
  {
    SVelocityCommand sCmd;
    sCmd.mLinear = 0.0;
    sCmd.mAngular = 0.0;
    return sCmd;
  }

  // Initialize reference on first update
  if (first_update_) 
  {
    ref_x_ = cSensor_->get_pose_x();
    ref_y_ = cSensor_->get_pose_y();
    spin_ref_yaw_ = cSensor_->get_yaw_unwrapped();
    have_spin_ref_ = true;
    first_update_ = false;
  }

  // Check goal condition with debouncing
  if (check_goal()) 
  {
    red_ahead_lock_ = std::min(red_ahead_lock_ + 1, sConfig_.mRed_lock_frames);
  } 
  else 
  {
    red_ahead_lock_ = 0;
  }

  // Handle goal reached state
  if (red_ahead_lock_ >= sConfig_.mRed_lock_frames) 
  {
    state_ = GOAL_REACHED;
    const double front_dist = cSensor_->get_scan_data(CTurtlebot3Sensor::FORWARD);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                        "Goal reached: red wall detected at %.2fm", front_dist);
    
    SVelocityCommand sCmd;
    sCmd.mLinear = 0.0;
    sCmd.mAngular = 0.0;
    return sCmd;
  }

  // State machine execution
  SVelocityCommand sCmd;
  sCmd.mLinear = 0.0;
  sCmd.mAngular = 0.0;
  
  switch (state_) 
  {
    case RIGHT_WALL_FOLLOW:

      // Check for wall switch condition
      if (switch_wall_follow()) 
      {
        state_ = LEFT_WALL_FOLLOW;
        cWall_follow_pid_->reset();
        have_spin_ref_ = false;
      }
      sCmd = wall_following(false, aDelta_time);
      break;

    case LEFT_WALL_FOLLOW:
      sCmd = wall_following(true, aDelta_time);
      break;

    case GO_STRAIGHT:
      sCmd = go_straight(aDelta_time);
      break;

    case RECOVERY_BACKUP:
      sCmd = handle_recovery(aDelta_time);
      break;

    case GOAL_REACHED:
      sCmd.mLinear = 0.0;
      sCmd.mAngular = 0.0;
      break;
  }

  return sCmd;
}

// Get distance to wall on specified side
double CWallFollowingController::get_wall_distance(bool aUse_left_wall) const
{
  if (aUse_left_wall) 
  {
    return cSensor_->get_scan_data(CTurtlebot3Sensor::LEFT);
  } 
  else 
  {
    return cSensor_->get_scan_data(CTurtlebot3Sensor::RIGHT);
  }
}

// Compute speed scaling based on front obstacle distance
double CWallFollowingController::compute_speed_scaling(double mFront_distance) const
{
  // Validate input
  if (mFront_distance < 0.0)
  {
    return 0.0;
  }

  // No scaling needed if obstacle is far
  if (mFront_distance >= sConfig_.mSlow_down_distance) 
  {
    return 1.0;
  }
  
  // Full stop if obstacle is too close
  if (mFront_distance <= sConfig_.mStop_distance) 
  {
    return 0.0;
  }

  // Linear scaling between stop and slow distances
  const double range = sConfig_.mSlow_down_distance - sConfig_.mStop_distance;
  const double scaled = (mFront_distance - sConfig_.mStop_distance) / range;
  const double result = std::max(0.2, scaled);
  
  return result;
}

// Check if goal (red target) has been reached
bool CWallFollowingController::check_goal() const
{
  const double d_front = cSensor_->get_scan_data(CTurtlebot3Sensor::FORWARD);
  const bool red_ahead = red_seen_ && 
                        (std::abs(red_centroid_.x) <= sConfig_.mRed_x_tolerance);
  const bool result = red_ahead && (d_front < sConfig_.mRed_stop_distance);
  
  return result; 
}

// Check if robot should switch from right to left wall following
bool CWallFollowingController::switch_wall_follow()
{
  // Get current position
  const double pose_x = cSensor_->get_pose_x();
  const double pose_y = cSensor_->get_pose_y();
  
  // Check if robot has moved enough to reset spin reference
  const double dx = pose_x - ref_x_;
  const double dy = pose_y - ref_y_;
  const double dist_sq = dx * dx + dy * dy;
  const double threshold_sq = sConfig_.mForward_reset_distance * 
                             sConfig_.mForward_reset_distance;
  const bool moved_enough = dist_sq >= threshold_sq;

  if (moved_enough) 
  {
    ref_x_ = pose_x;
    ref_y_ = pose_y;
    spin_ref_yaw_ = cSensor_->get_yaw_unwrapped();
    have_spin_ref_ = true;
  }

  // Check if robot has completed a full clockwise rotation
  bool should_switch = false;
  
  if (have_spin_ref_ && state_ == RIGHT_WALL_FOLLOW) 
  {
    const double net_rotation = cSensor_->get_yaw_unwrapped() - spin_ref_yaw_;

    if (net_rotation <= -sConfig_.mTurn_trigger_angle) 
    {
      RCLCPP_INFO(node_->get_logger(), 
                  "360 deg clockwise rotation detected, switching to LEFT wall follow");
      should_switch = true;
    }
  }
  
  return should_switch;
}

// Trigger recovery backup behavior
void CWallFollowingController::trigger_recovery(const char* aReason)
{
  // Validate input
  if (aReason == nullptr)
  {
    aReason = "Unknown";
  }

  was_left_mode_ = (state_ == LEFT_WALL_FOLLOW);
  state_ = RECOVERY_BACKUP;
  recovery_end_time_ = node_->now() + 
                       rclcpp::Duration::from_seconds(sConfig_.mBackup_duration);
  cWall_follow_pid_->reset();
  
  RCLCPP_WARN(node_->get_logger(), "Recovery triggered: %s", aReason);
}

// Compute recovery behavior (backup and turn away from wall)
CWallFollowingController::SVelocityCommand CWallFollowingController::handle_recovery(double aDelta_time)
{
  SVelocityCommand sCmd;
  
  // Backup while turning away from wall
  const double turn_direction = was_left_mode_ ? -0.8 : 0.8;
  sCmd.mAngular = turn_direction * sConfig_.mMax_angular_velocity;
  sCmd.mLinear = sConfig_.mBackup_speed;

  // Check if recovery duration has elapsed
  if (node_->now() >= recovery_end_time_) 
  {
    // Return to appropriate wall-following state
    state_ = was_left_mode_ ? LEFT_WALL_FOLLOW : RIGHT_WALL_FOLLOW;
    cWall_follow_pid_->reset();
  }

  return sCmd;
}

bool CWallFollowingController::detect_loop_closure(const double aX, 
                                                   const double aY, 
                                                   const double aYaw, 
                                                   const rclcpp::Time& aNow)
{
  // Compute cutoff time, exclude recent samples (last 5.0 seconds)
  const rclcpp::Time cutoff_time = aNow - 
      rclcpp::Duration::from_seconds(loop_exclusion_period_s_);
  
  const double normalized_yaw = normalize_angle(aYaw);
  
  // Check each historical pose sample (excluding recent ones)
  for (const auto& sSample : pose_history_) 
  {
    // Skip recent samples to avoid false positives
    if (sSample.mStamp > cutoff_time) 
    {
      continue;
    }
    
    // Compute position distance
    const double dx = aX - sSample.mX;
    const double dy = aY - sSample.mY;
    const double position_distance = std::sqrt(dx * dx + dy * dy);
    
    // Compute angular difference
    const double yaw_diff = std::abs(smallest_yaw_diff(normalized_yaw, sSample.mYaw));
    
    // Check if both position and orientation are within tolerance
    if (position_distance <= position_tolerance_m_ && 
        yaw_diff <= yaw_tolerance_rad_) 
    {
      RCLCPP_DEBUG(node_->get_logger(), 
                  "Loop detected: pos_dist=%.3fm, yaw_diff=%.3frad, "
                  "sample_time=%.1fs ago", 
                  position_distance, 
                  yaw_diff,
                  (aNow - sSample.mStamp).seconds());
      
      // Update timestamp for next check
      last_loop_check_time_ = aNow;
      return true;
    }
  }
  
  // Update timestamp for next check
  last_loop_check_time_ = aNow;
  return false;
}

// Execute wall-following behavior
CWallFollowingController::SVelocityCommand CWallFollowingController::wall_following(bool aLeft_mode, double aDelta_time)
{
  // Validate input
  if (aDelta_time <= 0.0)
  {
    SVelocityCommand sCmd;
    sCmd.mLinear = 0.0;
    sCmd.mAngular = 0.0;
    return sCmd;
  }

  // Get sensor readings
  const double d_front = cSensor_->get_scan_data(CTurtlebot3Sensor::FORWARD);
  const double d_right = cSensor_->get_scan_data(CTurtlebot3Sensor::RIGHT);
  const double d_left = cSensor_->get_scan_data(CTurtlebot3Sensor::LEFT);
  const double d_frontright = cSensor_->get_scan_data(CTurtlebot3Sensor::FRONT_RIGHT);
  const double d_frontleft = cSensor_->get_scan_data(CTurtlebot3Sensor::FRONT_LEFT);

  const double wall_dist = get_wall_distance(aLeft_mode);
  
  // Check if red is directly ahead and switch to GO_STRAIGHT mode
  if (check_red_ahead()) 
  {
    RCLCPP_INFO(node_->get_logger(), "Red detected ahead! Switching to GO_STRAIGHT mode.");
    state_ = GO_STRAIGHT;
    
    SVelocityCommand sCmd;
    sCmd.mLinear = 0.0;
    sCmd.mAngular = 0.0;
    return sCmd;
  }
  
  // Loop detection: record pose and check for loop closure
  const double current_x = cSensor_->get_pose_x();
  const double current_y = cSensor_->get_pose_y();
  const double current_yaw = cSensor_->get_robot_pose();
  const rclcpp::Time now = node_->now();
  
  // Record pose at regular intervals
  if ((now - last_pose_record_time_).seconds() >= pose_record_period_s_) 
  {
    update_pose_history(current_x, current_y, current_yaw, now);
  }
  
  // Check for loop closure at regular intervals (only during right wall following)
  if (!aLeft_mode && 
      (now - last_loop_check_time_).seconds() >= loop_check_period_s_) 
  {
    if (detect_loop_closure(current_x, current_y, current_yaw, now)) 
    {
      RCLCPP_INFO(node_->get_logger(), 
                  "Loop closure detected! Switching to left wall following.");
      
      // Trigger state switch to left wall following
      state_ = LEFT_WALL_FOLLOW;
      cWall_follow_pid_->reset();
      have_spin_ref_ = false;
      
      // Continue with normal wall following logic in left mode
    }
  }
  
  // Safety check: front obstacle too close
  if (d_front < sConfig_.mHard_stop_distance) 
  {
    trigger_recovery("Front hard-stop");
    
    SVelocityCommand sCmd;
    sCmd.mLinear = 0.0;
    sCmd.mAngular = 0.0;
    return sCmd;
  }

  // Safety check: side obstacles too close
  if (!aLeft_mode) 
  {
    if (d_right < sConfig_.mSide_panic_distance || 
        d_frontright < sConfig_.mSide_panic_distance) 
    {
      trigger_recovery("Right side too close");
      
      SVelocityCommand sCmd;
      sCmd.mLinear = 0.0;
      sCmd.mAngular = 0.0;
      return sCmd;
    }
  } 
  else 
  {
    if (d_left < sConfig_.mSide_panic_distance || 
        d_frontleft < sConfig_.mSide_panic_distance) 
    {
      trigger_recovery("Left side too close");
      
      SVelocityCommand sCmd;
      sCmd.mLinear = 0.0;
      sCmd.mAngular = 0.0;
      return sCmd;
    }
  }

  // Corner detection
  const double front_sensor = aLeft_mode ? d_frontleft : d_frontright;
  const bool tight_corner = front_sensor < sConfig_.mTight_corner_threshold;
  const bool front_close = d_front < 0.6;
  const bool corner_detected = tight_corner || front_close;

  SVelocityCommand sCmd;
  
  if (corner_detected) 
  {
    // Turn away from the wall being followed
    const double turn_direction = aLeft_mode ? -0.7 : 0.7;
    sCmd.mAngular = turn_direction * sConfig_.mMax_angular_velocity;
    
    // Slow down for corners
    sCmd.mLinear = sConfig_.mNominal_velocity * 0.5;
    
    return sCmd;
  }

  // PID wall-following control
  const double side_error = sConfig_.mDesired_wall_distance - wall_dist;
  const double front_sensor_dist = aLeft_mode ? d_frontleft : d_frontright;
  const double front_error = sConfig_.mDesired_wall_distance - front_sensor_dist;

  // Weighted combination of side and front errors
  double error = 0.7 * side_error + 0.3 * front_error;

  // Invert error for left wall following
  if (aLeft_mode) 
  {
    error = -error;
  }
  
  // Compute angular velocity using PID
  double angular_vel = cWall_follow_pid_->compute(error, aDelta_time);

  // Clamp angular velocity
  if (angular_vel < -sConfig_.mMax_angular_velocity) 
  {
    angular_vel = -sConfig_.mMax_angular_velocity;
  }
  else if (angular_vel > sConfig_.mMax_angular_velocity) 
  {
    angular_vel = sConfig_.mMax_angular_velocity;
  }

  // Compute linear velocity with front distance scaling
  const double speed_scale = compute_speed_scaling(d_front);
  const double linear_vel = sConfig_.mNominal_velocity * speed_scale;

  sCmd.mLinear = linear_vel;
  sCmd.mAngular = angular_vel;
  
  return sCmd;
}

// Check if red target is directly ahead (for go straight mode)
bool CWallFollowingController::check_red_ahead() const
{
  // Check if red is seen and is approximately centered in the camera view
  const bool red_centered = red_seen_ && 
                           (std::abs(red_centroid_.x) <= go_straight_x_tolerance_);
  
  // Debug logging
  if (red_seen_) 
  {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Red detected: x=%.3f, centered=%s", 
                          red_centroid_.x, red_centered ? "YES" : "NO");
  }
  
  return red_centered;
}

// Execute go straight behavior towards red target
CWallFollowingController::SVelocityCommand CWallFollowingController::go_straight(double aDelta_time)
{
  // Validate input
  if (aDelta_time <= 0.0)
  {
    SVelocityCommand sCmd;
    sCmd.mLinear = 0.0;
    sCmd.mAngular = 0.0;
    return sCmd;
  }

  SVelocityCommand sCmd;
  
  // Check if we still see red ahead
  if (!check_red_ahead()) 
  {
    RCLCPP_INFO(node_->get_logger(), "Red no longer ahead. Returning to wall following.");
    // Return to the previous wall following state
    state_ = RIGHT_WALL_FOLLOW;  // Default back to right wall following
    sCmd.mLinear = 0.0;
    sCmd.mAngular = 0.0;
    return sCmd;
  }
  
  // Check if goal is reached (red is very close)
  if (check_goal()) 
  {
    RCLCPP_INFO(node_->get_logger(), "Goal reached during GO_STRAIGHT mode!");
    state_ = GOAL_REACHED;
    sCmd.mLinear = 0.0;
    sCmd.mAngular = 0.0;
    return sCmd;
  }
  
  // Check for front obstacle
  const double d_front = cSensor_->get_scan_data(CTurtlebot3Sensor::FORWARD);
  if (d_front < 0.3) 
  {
    RCLCPP_WARN(node_->get_logger(), "Obstacle ahead during GO_STRAIGHT mode! Stopping.");
    sCmd.mLinear = 0.0;
    sCmd.mAngular = 0.0;
    return sCmd;
  }
  
  // Simple proportional control to center the red target
  double angular_vel = 0.0;
  if (red_seen_) 
  {
    // Use centroid x position to steer towards target
    const double steering_gain = 0.8;
    angular_vel = -steering_gain * red_centroid_.x;  // Negative to correct towards center
    
    // Clamp angular velocity
    angular_vel = std::max(-0.5, std::min(0.5, angular_vel));
  }
  
  sCmd.mLinear = go_straight_velocity_;
  sCmd.mAngular = angular_vel;
  
  return sCmd;
}

// Update pose history with new sample
void CWallFollowingController::update_pose_history(const double aX, const double aY, const double aYaw, const rclcpp::Time& aNow)
{
  // Add new pose sample to history
  SPoseSample sSample;
  sSample.mX = aX;
  sSample.mY = aY;
  sSample.mYaw = normalize_angle(aYaw);
  sSample.mStamp = aNow;
  
  pose_history_.push_back(sSample);
  
  // Remove old samples to prevent unbounded growth
  clean_history(aNow);
  
  // Update timestamp for next recording
  last_pose_record_time_ = aNow;
}

// Remove old pose samples from history
void CWallFollowingController::clean_history(const rclcpp::Time& aNow)
{
  const rclcpp::Time cutoff_time = aNow - 
      rclcpp::Duration::from_seconds(history_horizon_s_);
  
  // Use iterator to efficiently remove from front of deque
  auto it = pose_history_.begin();
  while (it != pose_history_.end() && it->mStamp < cutoff_time) 
  {
    ++it;
  }
  
  if (it != pose_history_.begin()) 
  {
    pose_history_.erase(pose_history_.begin(), it);
  }
  
  const std::size_t max_samples = static_cast<std::size_t>(
      history_horizon_s_ / pose_record_period_s_) + 10;
  
  while (pose_history_.size() > max_samples) 
  {
    pose_history_.pop_front();
  }
}

// Normalize angle to (-pi, pi] range
double CWallFollowingController::normalize_angle(double aAngle)
{
  // Normalize angle to (-pi, pi] range
  while (aAngle > M_PI) 
  {
    aAngle -= 2.0 * M_PI;
  }
  while (aAngle <= -M_PI) 
  {
    aAngle += 2.0 * M_PI;
  }
  return aAngle;
}

// Compute smallest angular difference between two angles
double CWallFollowingController::smallest_yaw_diff(double aAngle_a, double aAngle_b)
{
  // Compute the smallest angular difference between two angles
  return normalize_angle(aAngle_a - aAngle_b);
}