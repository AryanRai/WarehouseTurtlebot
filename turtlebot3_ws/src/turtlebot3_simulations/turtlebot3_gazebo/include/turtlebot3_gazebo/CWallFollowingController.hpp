/*
 * File: CWallFollowingController.hpp
 * Description: Declares the CWallFollowingController class for wall-following, obstacle avoidance, and goal detection in TurtleBot3 Gazebo simulation.
 * 
 * Date: 2025-10-10
 *
 * This header defines the interface for the state machine logic for wall-following,
 * PID control, loop detection, and red target goal finding.
 */

#ifndef TURTLEBOT3_GAZEBO__CWALL_FOLLOWING_CONTROLLER_HPP_
#define TURTLEBOT3_GAZEBO__CWALL_FOLLOWING_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <deque>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "turtlebot3_gazebo/CTurtlebot3Sensor.hpp"

class CPIDController;

/**
 * @class CWallFollowingController
 * @brief Implements wall-following behavior with state machine
 * 
 * Controls robot to follow walls using PID control, detects corners,
 * handles obstacle recovery, and finds red target goal.
 */
class CWallFollowingController 
{
  public:
    /**
     * @enum eState
     * @brief Robot behavioral states
     */
  enum eState {
    RIGHT_WALL_FOLLOW,  ///< Following right wall
    LEFT_WALL_FOLLOW,   ///< Following left wall
    GO_STRAIGHT,        ///< Going straight towards red target
    RECOVERY_BACKUP,    ///< Backing up from obstacle
    GOAL_REACHED        ///< Red target found and reached
  };    /**
     * @struct SVelocityCommand
     * @brief Robot velocity command
     */
    struct SVelocityCommand {
      double mLinear;   ///< Linear velocity (m/s)
      double mAngular;  ///< Angular velocity (rad/s)
    };

    /**
     * @brief Constructor
     * @param aNode Pointer to parent ROS2 node (not owned)
     * @param aSensor Pointer to sensor manager (not owned)
     */
    CWallFollowingController(rclcpp::Node* aNode, CTurtlebot3Sensor* aSensor);
    
    /**
     * @brief Destructor
     */
    ~CWallFollowingController();

    /**
     * @brief Compute velocity command based on current state and sensors
     * @param aDelta_time Time since last computation (seconds)
     * @return Velocity command for robot
     */
    SVelocityCommand compute_velocity(double aDelta_time);
    
    /**
     * @brief Get current controller state
     * @return Current state
     */
    eState get_state() const;

  private:
    /**
     * @struct SConfiguration
     * @brief Controller configuration parameters
     */
    struct SConfiguration {
      // Wall following parameters
      double mDesired_wall_distance;
      double mHard_stop_distance;
      double mSide_panic_distance;
      double mSlow_down_distance;
      double mStop_distance;
      double mTight_corner_threshold;
      
      // Velocity limits
      double mNominal_velocity;
      double mMax_angular_velocity;
      
      // Recovery parameters
      double mBackup_speed;
      double mBackup_duration;
      
      // Wall switching parameters
      double mTurn_trigger_angle;
      double mForward_reset_distance;
      
      // Red target detection parameters
      double mRed_x_tolerance;
      double mRed_stop_distance;
      int mRed_lock_frames;
      
      // PID gains
      double mPID_kp;
      double mPID_ki;
      double mPID_kd;
      
      /**
       * @brief Constructor with default values
       */
      SConfiguration();
    };

    // Configuration
    SConfiguration sConfig_;
    
    // PID controller (owned by this class)
    std::unique_ptr<CPIDController> cWall_follow_pid_;

    // External dependencies (not owned)
    rclcpp::Node* node_;
    CTurtlebot3Sensor* cSensor_;

    // Red target detection subscriptions
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr red_seen_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr red_centroid_sub_;

    // Controller state
    eState state_;
    
    // Reference tracking for wall switching
    double ref_x_;
    double ref_y_;
    double spin_ref_yaw_;
    bool have_spin_ref_;
    bool first_update_;
    bool was_left_mode_;
    
    // Red target detection state
    bool red_seen_;
    geometry_msgs::msg::Point red_centroid_;
    int red_ahead_lock_;
    
    // Recovery state
    rclcpp::Time recovery_end_time_;

    // Loop detection members
    struct SPoseSample {
      double mX;
      double mY;
      double mYaw;
      rclcpp::Time mStamp;
    };
    
    std::deque<SPoseSample> pose_history_;
    rclcpp::Time last_pose_record_time_;
    rclcpp::Time last_loop_check_time_;
    
    // Loop detection configuration constants
    static constexpr double pose_record_period_s_ = 0.2;
    static constexpr double loop_check_period_s_ = 1.0;
    static constexpr double loop_exclusion_period_s_ = 5.0;  // Exclude last 5 seconds
    static constexpr double position_tolerance_m_ = 0.1;
    static constexpr double yaw_tolerance_rad_ = 0.1;
    static constexpr double history_horizon_s_ = 30.0;
    
    // Go straight configuration constants
    static constexpr double go_straight_x_tolerance_ = 0.3;  // Red target must be within this X range
    static constexpr double go_straight_velocity_ = 0.15;     // Velocity when going straight to target

    /**
     * @brief Execute wall-following control
     * @param aLeft_mode True for left wall, false for right wall
     * @param aDelta_time Time step (seconds)
     * @return Velocity command
     */
    SVelocityCommand wall_following(bool aLeft_mode, double aDelta_time);
    
    /**
     * @brief Execute recovery behavior
     * @param aDelta_time Time step (seconds)
     * @return Velocity command
     */
    SVelocityCommand handle_recovery(double aDelta_time);
    
    /**
     * @brief Execute go straight behavior towards red target
     * @param aDelta_time Time step (seconds)
     * @return Velocity command
     */
    SVelocityCommand go_straight(double aDelta_time);
    
    /**
     * @brief Get distance to wall on specified side
     * @param aUse_left_wall True for left wall, false for right wall
     * @return Distance to wall (meters)
     */
    double get_wall_distance(bool aUse_left_wall) const;
    
    /**
     * @brief Compute speed scaling based on front obstacle distance
     * @param mFront_distance Distance to front obstacle (meters)
     * @return Speed scaling factor [0.0, 1.0]
     */
    double compute_speed_scaling(double mFront_distance) const;
    
    /**
     * @brief Check if goal (red target) has been reached
     * @return True if goal reached
     */
    bool check_goal() const;
    
    /**
     * @brief Check if red target is directly ahead (for go straight mode)
     * @return True if red target is directly ahead
     */
    bool check_red_ahead() const;
    
    /**
     * @brief Check if robot should switch from right to left wall following
     * @return True if switch should occur
     */
    bool switch_wall_follow();
    
    /**
     * @brief Trigger recovery backup behavior
     * @param aReason Description of why recovery was triggered
     */
    void trigger_recovery(const char* aReason);
    
    /**
     * @brief Update pose history for loop detection
     * @param aX Current x position (meters)
     * @param aY Current y position (meters)
     * @param aYaw Current yaw angle (radians)
     * @param aNow Current timestamp
     */
    void update_pose_history(const double aX, 
                            const double aY, 
                            const double aYaw, 
                            const rclcpp::Time& aNow);
    
      /**
     * @brief Detect if robot has returned to a previous pose (loop closure)
     * @param aX Current x position (meters)
     * @param aY Current y position (meters)
     * @param aYaw Current yaw angle (radians)
     * @param aNow Current timestamp
     * @return True if loop closure detected
     */
    bool detect_loop_closure(const double aX, 
                            const double aY, 
                            const double aYaw, 
                            const rclcpp::Time& aNow);
    /**
     * @brief Remove old pose samples from history
     * @param aNow Current timestamp
     */
    void clean_history(const rclcpp::Time& aNow);
    
    /**
     * @brief Normalize angle to (-pi, pi] range
     * @param aAngle Input angle (radians)
     * @return Normalized angle (radians)
     */
    static double normalize_angle(double aAngle);
    
    /**
     * @brief Compute smallest angular difference between two angles
     * @param aAngle_a First angle (radians)
     * @param aAngle_b Second angle (radians)
     * @return Smallest angular difference (radians)
     */
    static double smallest_yaw_diff(double aAngle_a, double aAngle_b);
};

#endif  // TURTLEBOT3_GAZEBO__CWALL_FOLLOWING_CONTROLLER_HPP_