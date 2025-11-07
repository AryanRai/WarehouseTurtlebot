// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: AutonomousExplorationRobot.cpp
// Author(s): Aryan Rai
//
// Description: Implementation of CAutonomousExplorationRobot. Performs
// frontier-based
//              autonomous exploration to map unknown warehouse environments.

#include "Robot/AutonomousExplorationRobot.hpp"
#include <cmath>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

AutonomousExplorationRobot::AutonomousExplorationRobot(
    rclcpp::Node::SharedPtr node)
    : node_(node), is_exploring_(false), is_paused_(false),
      last_update_time_(node->now()), last_replan_time_(node->now()),
      consecutive_no_path_count_(0), recovery_start_time_(node->now()),
      in_recovery_(false), recovery_attempt_(0), returning_home_(false),
      at_home_(false), home_path_completed_(false),
      consecutive_no_frontiers_count_(0), return_home_failures_(0),
      last_return_home_progress_(node->now()),
      last_distance_to_home_(std::numeric_limits<double>::max()),
      in_docking_mode_(false), in_return_home_recovery_(false),
      return_home_recovery_step_(0), return_home_recovery_start_(node->now()),
      distance_before_recovery_(0.0), initial_yaw_(0.0),
      has_relocalized_(false), relocalization_start_time_(node->now())
{

    // Initialize SLAM components
    slam_controller_ = std::make_unique<SlamController>();
    motion_controller_ = std::make_unique<MotionController>();
    exploration_planner_ = std::make_unique<ExplorationPlanner>();

    // Set fast speeds for exploration
    motion_controller_->setExplorationSpeeds();

    // Create publishers (MotionController already publishes cmd_vel)
    path_pub_ =
        node_->create_publisher<nav_msgs::msg::Path>("/exploration/path", 10);
    recovery_cmd_vel_pub_ =
        node_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel",
                                                                  10);

    // Create subscribers for obstacle detection
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar/scan", 10, [this](sensor_msgs::msg::LaserScan::SharedPtr msg)
        { current_scan_ = msg; });

    // Initialize home position (0, 0)
    home_position_.x = 0.0;
    home_position_.y = 0.0;
    home_position_.z = 0.0;

    // Initialize last goal position
    last_goal_position_.x = 0.0;
    last_goal_position_.y = 0.0;
    last_goal_position_.z = 0.0;

    RCLCPP_INFO(node_->get_logger(),
                "Autonomous Exploration Robot initialized");
    RCLCPP_INFO(node_->get_logger(), "Home position set to (%.2f, %.2f)",
                home_position_.x, home_position_.y);
}

bool AutonomousExplorationRobot::isObstacleAhead(double min_distance)
{
    if (!current_scan_ || current_scan_->ranges.empty())
    {
        return false; // No scan data, assume clear
    }

    // Check front 60 degrees (30 degrees on each side)
    int num_ranges = current_scan_->ranges.size();
    int front_range = num_ranges / 6; // 60 degrees out of 360

    // Check front center ranges
    for (int i = -front_range / 2; i < front_range / 2; i++)
    {
        int idx = (num_ranges / 2 + i + num_ranges) % num_ranges;
        float range = current_scan_->ranges[idx];

        // Check if valid range and too close
        if (std::isfinite(range) && range > 0.0 && range < min_distance)
        {
            return true;
        }
    }

    return false;
}

bool AutonomousExplorationRobot::isObstacleBehind(double min_distance)
{
    if (!current_scan_ || current_scan_->ranges.empty())
    {
        return false; // No scan data, assume clear
    }

    // Check rear 60 degrees (30 degrees on each side of back)
    int num_ranges = current_scan_->ranges.size();
    int rear_range = num_ranges / 6; // 60 degrees out of 360

    // Check rear center ranges (opposite side from front)
    for (int i = -rear_range / 2; i < rear_range / 2; i++)
    {
        int idx = (i + num_ranges) % num_ranges; // Back is at index 0
        float range = current_scan_->ranges[idx];

        // Check if valid range and too close
        if (std::isfinite(range) && range > 0.0 && range < min_distance)
        {
            return true;
        }
    }

    return false;
}

void AutonomousExplorationRobot::performRecovery()
{
    // Recovery behavior: forward, backward, then forward+rotate pattern
    // This helps discover new frontiers and break oscillation loops
    // Now with comprehensive obstacle avoidance!

    if (!in_recovery_)
    {
        int pattern = recovery_attempt_ % 3;
        const char *action = (pattern == 0)   ? "forward"
                             : (pattern == 1) ? "backward"
                                              : "forward + rotate";
        RCLCPP_WARN(node_->get_logger(),
                    "Starting recovery behavior (attempt %d) - %s to find new "
                    "frontiers",
                    recovery_attempt_ + 1, action);
        in_recovery_ = true;
        recovery_start_time_ = node_->now();
        motion_controller_->clearPath(); // Clear any existing path
    }

    double elapsed = (node_->now() - recovery_start_time_).seconds();

    if (elapsed < RECOVERY_DURATION)
    {
        int pattern = recovery_attempt_ % 3;
        double linear_speed = 0.0;
        double angular_speed = 0.0;

        if (pattern == 0)
        {
            // Forward - but stop if obstacle ahead
            if (!isObstacleAhead(0.4))
            { // Check 40cm ahead
                linear_speed = 0.1;
                angular_speed = 0.0;
            }
            else
            {
                // Obstacle ahead, just rotate in place
                linear_speed = 0.0;
                angular_speed = 0.5;
                RCLCPP_DEBUG(node_->get_logger(),
                             "Obstacle detected during forward recovery, "
                             "rotating instead");
            }
        }
        else if (pattern == 1)
        {
            // Backward - check behind using map-based obstacle detection
            if (!isObstacleBehind(0.4))
            { // Check 40cm behind
                linear_speed = -0.1;
                angular_speed = 0.0;
            }
            else
            {
                // Obstacle behind, just rotate in place
                linear_speed = 0.0;
                angular_speed = 0.5;
                RCLCPP_DEBUG(node_->get_logger(),
                             "Obstacle detected during backward recovery, "
                             "rotating instead");
            }
        }
        else
        {
            // Forward + rotate (to break oscillation)
            if (!isObstacleAhead(0.4))
            { // Check 40cm ahead
                linear_speed = 0.1;
                angular_speed = 0.5;
            }
            else
            {
                // Obstacle ahead, just rotate in place
                linear_speed = 0.0;
                angular_speed = 0.5;
                RCLCPP_DEBUG(node_->get_logger(),
                             "Obstacle detected during forward+rotate "
                             "recovery, rotating only");
            }
        }

        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = node_->now();
        cmd_vel.header.frame_id = "base_footprint";
        cmd_vel.twist.linear.x = linear_speed;
        cmd_vel.twist.angular.z = angular_speed;
        recovery_cmd_vel_pub_->publish(cmd_vel);

        const char *action = (pattern == 0)   ? "forward"
                             : (pattern == 1) ? "backward"
                                              : "forward + rotating";
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Recovery: %s (%.1fs / %.1fs)", action, elapsed,
                             RECOVERY_DURATION);
    }
    else
    {
        // Stop movement
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = node_->now();
        cmd_vel.header.frame_id = "base_footprint";
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        recovery_cmd_vel_pub_->publish(cmd_vel);

        RCLCPP_INFO(node_->get_logger(),
                    "Recovery complete - resuming exploration");
        in_recovery_ = false;
        consecutive_no_path_count_ = 0;
        recovery_attempt_++; // Increment for next time
        last_replan_time_ =
            node_->now() - rclcpp::Duration::from_seconds(
                               MIN_REPLAN_INTERVAL); // Allow immediate replan
    }
}

void AutonomousExplorationRobot::performAdvancedReturnHomeRecovery(
    const geometry_msgs::msg::Pose &current_pose, double current_distance)
{

    if (!in_return_home_recovery_)
    {
        // Start recovery
        in_return_home_recovery_ = true;
        return_home_recovery_step_ = 0;
        return_home_recovery_start_ = node_->now();
        distance_before_recovery_ = current_distance;
        RCLCPP_WARN(node_->get_logger(),
                    "Starting advanced return-home recovery (distance: %.3fm)",
                    current_distance);
    }

    double elapsed = (node_->now() - return_home_recovery_start_).seconds();

    // Recovery sequence: forward, backward, forward+rotate left, forward+rotate
    // right Each step lasts 2 seconds, then we check if distance improved
    const double STEP_DURATION = 2.0;

    if (elapsed < STEP_DURATION)
    {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = node_->now();
        cmd_vel.header.frame_id = "base_footprint";

        switch (return_home_recovery_step_)
        {
        case 0: // Forward
            if (!isObstacleAhead(0.4))
            { // Check 40cm ahead
                cmd_vel.twist.linear.x = 0.15;
                cmd_vel.twist.angular.z = 0.0;
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(),
                                     1000, "Recovery step 1/4: Moving forward");
            }
            else
            {
                cmd_vel.twist.linear.x = 0.0;
                cmd_vel.twist.angular.z = 0.5;
                RCLCPP_INFO_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 1000,
                    "Recovery step 1/4: Obstacle ahead, rotating");
            }
            break;

        case 1: // Backward
            if (!isObstacleBehind(0.4))
            { // Check 40cm behind
                cmd_vel.twist.linear.x = -0.15;
                cmd_vel.twist.angular.z = 0.0;
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(),
                                     1000,
                                     "Recovery step 2/4: Moving backward");
            }
            else
            {
                cmd_vel.twist.linear.x = 0.0;
                cmd_vel.twist.angular.z = 0.5;
                RCLCPP_INFO_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 1000,
                    "Recovery step 2/4: Obstacle behind, rotating");
            }
            break;

        case 2: // Forward + rotate left
            if (!isObstacleAhead(0.4))
            { // Check 40cm ahead
                cmd_vel.twist.linear.x = 0.15;
                cmd_vel.twist.angular.z = 0.5;
                RCLCPP_INFO_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 1000,
                    "Recovery step 3/4: Forward + rotate left");
            }
            else
            {
                cmd_vel.twist.linear.x = 0.0;
                cmd_vel.twist.angular.z = 0.5;
                RCLCPP_INFO_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 1000,
                    "Recovery step 3/4: Obstacle ahead, rotating left");
            }
            break;

        case 3: // Forward + rotate right
            if (!isObstacleAhead(0.4))
            { // Check 40cm ahead
                cmd_vel.twist.linear.x = 0.15;
                cmd_vel.twist.angular.z = -0.5;
                RCLCPP_INFO_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 1000,
                    "Recovery step 4/4: Forward + rotate right");
            }
            else
            {
                cmd_vel.twist.linear.x = 0.0;
                cmd_vel.twist.angular.z = -0.5;
                RCLCPP_INFO_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 1000,
                    "Recovery step 4/4: Obstacle ahead, rotating right");
            }
            break;

        default:
            // Should not reach here
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.angular.z = 0.0;
            break;
        }

        recovery_cmd_vel_pub_->publish(cmd_vel);
    }
    else
    {
        // Step complete, stop and check if we improved
        geometry_msgs::msg::TwistStamped stop_cmd;
        stop_cmd.header.stamp = node_->now();
        stop_cmd.header.frame_id = "base_footprint";
        stop_cmd.twist.linear.x = 0.0;
        stop_cmd.twist.angular.z = 0.0;
        recovery_cmd_vel_pub_->publish(stop_cmd);

        // Check if distance improved
        double distance_improvement =
            distance_before_recovery_ - current_distance;

        if (distance_improvement > 0.05)
        {
            // Success! We got closer to home
            RCLCPP_INFO(
                node_->get_logger(),
                "Recovery successful! Reduced distance by %.3fm (%.3f -> %.3f)",
                distance_improvement, distance_before_recovery_,
                current_distance);
            in_return_home_recovery_ = false;
            return_home_failures_ = 0;
            motion_controller_->clearPath(); // Force replan with new position
        }
        else if (return_home_recovery_step_ < 3)
        {
            // Try next recovery step
            return_home_recovery_step_++;
            return_home_recovery_start_ = node_->now();
            RCLCPP_WARN(node_->get_logger(),
                        "Recovery step %d didn't help (distance: %.3fm), "
                        "trying step %d",
                        return_home_recovery_step_, current_distance,
                        return_home_recovery_step_ + 1);
        }
        else
        {
            // All steps tried, give up this recovery attempt
            RCLCPP_ERROR(
                node_->get_logger(),
                "All recovery steps failed. Distance: %.3fm (was %.3fm)",
                current_distance, distance_before_recovery_);
            in_return_home_recovery_ = false;
            return_home_failures_ = 0; // Reset to try path planning again
            motion_controller_->clearPath();
        }
    }
}

void AutonomousExplorationRobot::preciseDocking(
    const geometry_msgs::msg::Pose &current_pose, double distance_to_home)
{
    // SAFETY CHECK: Verify line of sight before every docking movement
    auto current_map = slam_controller_->getCurrentMap();
    if (!hasLineOfSight(current_pose.position, home_position_, *current_map))
    {
        RCLCPP_WARN(node_->get_logger(),
                    "Lost line of sight during docking! Exiting docking mode.");
        in_docking_mode_ = false;

        // Stop movement
        geometry_msgs::msg::TwistStamped stop_cmd;
        stop_cmd.header.stamp = node_->now();
        stop_cmd.header.frame_id = "base_footprint";
        stop_cmd.twist.linear.x = 0.0;
        stop_cmd.twist.angular.z = 0.0;
        recovery_cmd_vel_pub_->publish(stop_cmd);
        return;
    }

    // Calculate direction to home
    double dx = home_position_.x - current_pose.position.x;
    double dy = home_position_.y - current_pose.position.y;
    double angle_to_home = std::atan2(dy, dx);

    // Get current robot orientation
    tf2::Quaternion q(current_pose.orientation.x, current_pose.orientation.y,
                      current_pose.orientation.z, current_pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Calculate angle difference
    double angle_diff = angle_to_home - yaw;
    // Normalize to [-pi, pi]
    while (angle_diff > M_PI)
        angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI)
        angle_diff += 2.0 * M_PI;

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = node_->now();
    cmd_vel.header.frame_id = "base_footprint";

    // If angle is off by more than 10 degrees, rotate first
    if (std::abs(angle_diff) > 0.175)
    { // ~10 degrees
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z =
            (angle_diff > 0) ? DOCKING_ANGULAR_SPEED : -DOCKING_ANGULAR_SPEED;
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Docking: Aligning (angle error: %.1f°)",
                             angle_diff * 180.0 / M_PI);
    }
    else
    {
        // Move forward slowly toward home
        cmd_vel.twist.linear.x = DOCKING_LINEAR_SPEED;
        cmd_vel.twist.angular.z = angle_diff * 0.5; // Gentle correction
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Docking: Moving forward (%.3fm remaining)",
                             distance_to_home);
    }

    recovery_cmd_vel_pub_->publish(cmd_vel);
}

bool AutonomousExplorationRobot::hasLineOfSight(
    const geometry_msgs::msg::Point &from, const geometry_msgs::msg::Point &to,
    const nav_msgs::msg::OccupancyGrid &map)
{
    // Use Bresenham's line algorithm to check if path is clear
    // Also check cells around the line to account for robot width
    GridCell start = PathPlanner::worldToGrid(map, from);
    GridCell goal = PathPlanner::worldToGrid(map, to);

    int x0 = start.first;
    int y0 = start.second;
    int x1 = goal.first;
    int y1 = goal.second;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    // Robot radius in grid cells (TurtleBot3 is ~0.178m diameter, so ~0.09m
    // radius) With 0.05m resolution, that's ~2 cells radius
    const int robot_radius_cells = 2;

    while (true)
    {
        // Check if current cell and surrounding cells are walkable (robot
        // footprint)
        for (int dx_check = -robot_radius_cells; dx_check <= robot_radius_cells;
             ++dx_check)
        {
            for (int dy_check = -robot_radius_cells;
                 dy_check <= robot_radius_cells; ++dy_check)
            {
                // Only check cells within robot's circular footprint
                if (dx_check * dx_check + dy_check * dy_check <=
                    robot_radius_cells * robot_radius_cells)
                {
                    GridCell check_cell = {x0 + dx_check, y0 + dy_check};
                    if (!PathPlanner::isCellInBounds(map, check_cell) ||
                        !PathPlanner::isCellWalkable(map, check_cell))
                    {
                        return false; // Obstacle would hit robot body
                    }
                }
            }
        }

        if (x0 == x1 && y0 == y1)
        {
            break; // Reached goal
        }

        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y0 += sy;
        }
    }

    return true; // Clear path with robot footprint considered
}

void AutonomousExplorationRobot::returnToHome()
{
    // If already at home, do nothing
    if (at_home_)
    {
        return;
    }

    if (!slam_controller_->hasValidMap() || !slam_controller_->hasValidPose())
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "Waiting for valid map and pose to return home...");
        return;
    }

    auto current_map = slam_controller_->getCurrentMap();
    auto current_pose = slam_controller_->getCurrentPose();

    // Check if already at home
    double dx = home_position_.x - current_pose.position.x;
    double dy = home_position_.y - current_pose.position.y;
    double distance_to_home = std::sqrt(dx * dx + dy * dy);

    // Check if reached home with tighter tolerance
    if (distance_to_home < HOME_TOLERANCE)
    { // Within 5cm of home
        // Get current orientation
        tf2::Quaternion q(
            current_pose.orientation.x, current_pose.orientation.y,
            current_pose.orientation.z, current_pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, current_yaw;
        m.getRPY(roll, pitch, current_yaw);

        // Calculate angle difference from initial orientation
        double angle_diff = initial_yaw_ - current_yaw;
        while (angle_diff > M_PI)
            angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI)
            angle_diff += 2.0 * M_PI;

        // If not aligned with initial orientation, rotate to match
        if (std::abs(angle_diff) > 0.05)
        { // ~3 degrees tolerance
            geometry_msgs::msg::TwistStamped cmd_vel;
            cmd_vel.header.stamp = node_->now();
            cmd_vel.header.frame_id = "base_footprint";
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.angular.z = (angle_diff > 0) ? 0.2 : -0.2;

            recovery_cmd_vel_pub_->publish(cmd_vel);

            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 500,
                "Aligning to initial orientation (%.1f° remaining)",
                std::abs(angle_diff) * 180.0 / M_PI);
            return;
        }

        at_home_ = true;
        in_docking_mode_ = false;
        motion_controller_->clearPath();

        // Stop all motion completely
        geometry_msgs::msg::TwistStamped stop_cmd;
        stop_cmd.header.stamp = node_->now();
        stop_cmd.header.frame_id = "base_footprint";
        stop_cmd.twist.linear.x = 0.0;
        stop_cmd.twist.linear.y = 0.0;
        stop_cmd.twist.linear.z = 0.0;
        stop_cmd.twist.angular.x = 0.0;
        stop_cmd.twist.angular.y = 0.0;
        stop_cmd.twist.angular.z = 0.0;
        recovery_cmd_vel_pub_->publish(stop_cmd);

        // Give time for stop command to take effect
        rclcpp::sleep_for(std::chrono::milliseconds(200));

        RCLCPP_INFO(node_->get_logger(),
                    "✓ Successfully returned to home position! (distance: "
                    "%.3fm, aligned)",
                    distance_to_home);
        RCLCPP_INFO(node_->get_logger(),
                    "Exploration complete - saving final map");
        saveMap("warehouse_map_final");

        // Mark exploration as complete so the node can exit
        slam_controller_->setExplorationComplete(true);
        stopExploration();
        return;
    }

    // Enter precise docking mode when reasonably close to home (within 1m)
    // BUT only if there's a clear line of sight to home
    if (distance_to_home < 1.0)
    { // Increased from 0.5m to 1.0m for more aggressive docking
        // Check if there's a clear path to home
        bool has_clear_path =
            hasLineOfSight(current_pose.position, home_position_, *current_map);

        if (has_clear_path)
        {
            if (!in_docking_mode_)
            {
                RCLCPP_INFO(node_->get_logger(),
                            "Entering precise docking mode (%.3fm from home, "
                            "clear path)",
                            distance_to_home);
                in_docking_mode_ = true;
                motion_controller_->clearPath(); // Stop using path planner
            }

            // Use precise docking control
            preciseDocking(current_pose, distance_to_home);
            return;
        }
        else
        {
            // Close to home but obstacle in the way - keep using path planner
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "Close to home (%.2fm) but obstacle detected "
                                 "- continuing with path planner",
                                 distance_to_home);
            in_docking_mode_ = false; // Make sure we're not in docking mode
            // Fall through to path planning logic below
        }
    }

    // Reset docking mode if we're far from home
    if (in_docking_mode_ && distance_to_home >= 1.2)
    { // Increased threshold
        in_docking_mode_ = false;
    }

    // Plan path to home if we don't have one or if goal doesn't match home
    if (!motion_controller_->hasPath() ||
        std::abs(last_goal_position_.x - home_position_.x) > 0.1 ||
        std::abs(last_goal_position_.y - home_position_.y) > 0.1)
    {

        last_goal_position_ = home_position_;

        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 3000,
            "Planning path to home (%.2f, %.2f), distance: %.2fm",
            home_position_.x, home_position_.y, distance_to_home);

        // Use the path planner to plan a path to home
        GridCell start =
            PathPlanner::worldToGrid(*current_map, current_pose.position);
        GridCell goal_cell =
            PathPlanner::worldToGrid(*current_map, home_position_);

        auto [cspace, cspace_cells] =
            PathPlanner::calcCSpace(*current_map, false);
        cv::Mat cost_map = PathPlanner::calcCostMap(*current_map);

        // First try: plan to exact home position
        auto [path, cost, actual_start, actual_goal] =
            PathPlanner::aStar(cspace, cost_map, start, goal_cell);

        // If that fails, try to find nearest walkable cell to home
        if (path.empty())
        {
            static rclcpp::Time last_search_warn = node_->now();
            if ((node_->now() - last_search_warn).seconds() > 5.0)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "Cannot reach exact home position, searching for "
                            "nearest accessible point...");
                last_search_warn = node_->now();
            }

            // Search in expanding radius around home (check only 8 directions
            // per radius for speed)
            bool found_goal = false;
            std::vector<std::pair<int, int>> directions = {
                {1, 0}, {-1, 0}, {0, 1},  {0, -1}, // Cardinal
                {1, 1}, {1, -1}, {-1, 1}, {-1, -1} // Diagonal
            };

            for (int radius = 1; radius <= 30 && !found_goal; radius++)
            {
                for (const auto &[dx_unit, dy_unit] : directions)
                {
                    GridCell candidate = {goal_cell.first + dx_unit * radius,
                                          goal_cell.second + dy_unit * radius};

                    // Check if this cell is walkable
                    if (PathPlanner::isCellInBounds(*current_map, candidate) &&
                        PathPlanner::isCellWalkable(*current_map, candidate))
                    {

                        // Try to plan to this cell
                        auto [alt_path, alt_cost, alt_start, alt_goal] =
                            PathPlanner::aStar(cspace, cost_map, start,
                                               candidate);

                        if (!alt_path.empty())
                        {
                            path = alt_path;
                            found_goal = true;
                            geometry_msgs::msg::Point alt_home =
                                PathPlanner::gridToWorld(*current_map,
                                                         candidate);
                            RCLCPP_INFO(node_->get_logger(),
                                        "Found accessible point near home at "
                                        "(%.2f, %.2f), %.2fm from origin",
                                        alt_home.x, alt_home.y,
                                        std::sqrt(alt_home.x * alt_home.x +
                                                  alt_home.y * alt_home.y));
                            break;
                        }
                    }
                }
            }
        }

        if (!path.empty())
        {
            auto path_msg = PathPlanner::pathToMessage(*current_map, path);
            motion_controller_->setPath(path_msg);
            path_pub_->publish(path_msg);
            RCLCPP_INFO(node_->get_logger(),
                        "Path to home planned with %zu waypoints",
                        path_msg.poses.size());
        }
        else
        {
            RCLCPP_ERROR_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 5000,
                "Cannot find any path home! Robot may be trapped.");
            return;
        }
    }

    // If in recovery mode, perform recovery and return
    if (in_return_home_recovery_)
    {
        performAdvancedReturnHomeRecovery(current_pose, distance_to_home);
        return;
    }

    // Check if making progress toward home
    if (std::abs(distance_to_home - last_distance_to_home_) > 0.05)
    {
        // Made progress
        last_return_home_progress_ = node_->now();
        last_distance_to_home_ = distance_to_home;
        return_home_failures_ = 0;
    }
    else if ((node_->now() - last_return_home_progress_).seconds() > 5.0)
    {
        // Stuck for 5 seconds
        return_home_failures_++;
        last_return_home_progress_ = node_->now();

        if (return_home_failures_ >= 3)
        {
            // Start advanced recovery
            motion_controller_->clearPath();
            performAdvancedReturnHomeRecovery(current_pose, distance_to_home);
            return;
        }
    }

    // Follow path home
    if (motion_controller_->hasPath())
    {
        auto cmd_vel = motion_controller_->computeVelocityCommand(current_pose,
                                                                  *current_map);

        // Check if we've reached the path goal
        if (motion_controller_->isAtGoal() && !home_path_completed_)
        {
            home_path_completed_ = true;
            RCLCPP_INFO(node_->get_logger(),
                        "Path complete to home (%.3fm from exact position)",
                        distance_to_home);
        }

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                             "Returning home: %.2fm remaining",
                             distance_to_home);

        // If path following fails (returns zero velocity unexpectedly), clear
        // path to force replan
        if (cmd_vel.linear.x == 0.0 && cmd_vel.angular.z == 0.0 &&
            distance_to_home > 0.5)
        {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "Path following returned zero velocity, clearing path");
            motion_controller_->clearPath();
        }
        return; // Continue following path
    }

    // No path - check if we've already completed a path to home
    if (home_path_completed_)
    {
        // We've reached as close as we can get via path planning
        // Always try docking if there's line of sight, regardless of distance
        bool has_clear_path =
            hasLineOfSight(current_pose.position, home_position_, *current_map);

        if (has_clear_path && distance_to_home < 1.0)
        { // Within 1m and clear path
            RCLCPP_INFO(node_->get_logger(),
                        "Path complete, %.2fm from home with clear line of "
                        "sight - switching to docking mode",
                        distance_to_home);
            in_docking_mode_ = true;
            motion_controller_->clearPath();
            preciseDocking(current_pose, distance_to_home);
            return;
        }
        else if (!has_clear_path)
        {
            // Obstacle in way - accept this as close enough
            RCLCPP_INFO(node_->get_logger(),
                        "Path complete, %.2fm from home but obstacle detected "
                        "- accepting position",
                        distance_to_home);

            // Mark as home and complete
            at_home_ = true;
            home_path_completed_ = false;

            // Stop all motion
            geometry_msgs::msg::TwistStamped stop_cmd;
            stop_cmd.header.stamp = node_->now();
            stop_cmd.header.frame_id = "base_footprint";
            stop_cmd.twist.linear.x = 0.0;
            stop_cmd.twist.angular.z = 0.0;
            recovery_cmd_vel_pub_->publish(stop_cmd);

            RCLCPP_INFO(node_->get_logger(),
                        "✓ Reached nearest accessible point to home");
            RCLCPP_INFO(node_->get_logger(),
                        "Exploration complete - saving final map");
            saveMap("warehouse_map_final");

            slam_controller_->setExplorationComplete(true);
            stopExploration();
            return;
        }
        else
        {
            // Path complete but too far from home (>1m) - accept position
            RCLCPP_WARN(node_->get_logger(),
                        "Path complete but %.2fm from home (too far for "
                        "docking) - accepting position",
                        distance_to_home);

            at_home_ = true;
            home_path_completed_ = false;

            geometry_msgs::msg::TwistStamped stop_cmd;
            stop_cmd.header.stamp = node_->now();
            stop_cmd.header.frame_id = "base_footprint";
            stop_cmd.twist.linear.x = 0.0;
            stop_cmd.twist.angular.z = 0.0;
            recovery_cmd_vel_pub_->publish(stop_cmd);

            RCLCPP_INFO(node_->get_logger(),
                        "✓ Reached nearest accessible point to home");
            saveMap("warehouse_map_final");
            slam_controller_->setExplorationComplete(true);
            stopExploration();
            return;
        }
    }
}

void AutonomousExplorationRobot::startExploration()
{
    if (!is_exploring_)
    {
        is_exploring_ = true;
        is_paused_ = false;
        motion_controller_->setEnabled(true);
        RCLCPP_INFO(node_->get_logger(), "Starting autonomous exploration");
    }
}

void AutonomousExplorationRobot::stopExploration()
{
    if (is_exploring_)
    {
        is_exploring_ = false;
        is_paused_ = false;
        motion_controller_->setEnabled(false);
        motion_controller_->clearPath();

        RCLCPP_INFO(node_->get_logger(), "Stopped autonomous exploration");
    }
}

void AutonomousExplorationRobot::pauseExploration()
{
    if (is_exploring_ && !is_paused_)
    {
        is_paused_ = true;
        motion_controller_->setEnabled(false);

        RCLCPP_INFO(node_->get_logger(), "Paused autonomous exploration");
    }
}

void AutonomousExplorationRobot::resumeExploration()
{
    if (is_exploring_ && is_paused_)
    {
        is_paused_ = false;
        motion_controller_->setEnabled(true);
        RCLCPP_INFO(node_->get_logger(), "Resumed autonomous exploration");
    }
}

bool AutonomousExplorationRobot::isExplorationComplete() const
{
    return exploration_planner_->isExplorationComplete() ||
           slam_controller_->isExplorationComplete();
}

bool AutonomousExplorationRobot::isExploring() const
{
    return is_exploring_ && !is_paused_;
}

void AutonomousExplorationRobot::saveMap(const std::string &map_name)
{
    slam_controller_->saveMap(map_name);
}

void AutonomousExplorationRobot::update()
{
    // Check if we should update based on rate
    auto current_time = node_->now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 1.0 / UPDATE_RATE)
    {
        return;
    }
    last_update_time_ = current_time;

    // If not exploring or paused, do nothing
    if (!is_exploring_ || is_paused_)
    {
        return;
    }

    // Perform relocalization spin at the start
    if (!has_relocalized_ && slam_controller_->hasValidPose())
    {
        auto current_pose = slam_controller_->getCurrentPose();

        // Store initial orientation on first call
        if ((current_time - relocalization_start_time_).seconds() < 0.1)
        {
            tf2::Quaternion q(
                current_pose.orientation.x, current_pose.orientation.y,
                current_pose.orientation.z, current_pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, initial_yaw_);

            RCLCPP_INFO(node_->get_logger(),
                        "Starting relocalization spin (360°)...");
            relocalization_start_time_ = current_time;
        }

        double elapsed = (current_time - relocalization_start_time_).seconds();

        if (elapsed < RELOCALIZATION_DURATION)
        {
            // Spin in place for relocalization
            geometry_msgs::msg::TwistStamped cmd_vel;
            cmd_vel.header.stamp = current_time;
            cmd_vel.header.frame_id = "base_footprint";
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.angular.z = RELOCALIZATION_SPEED;

            recovery_cmd_vel_pub_->publish(cmd_vel);

            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "Relocalizing... %.1fs remaining",
                                 RELOCALIZATION_DURATION - elapsed);
            return;
        }
        else
        {
            // Stop spinning
            geometry_msgs::msg::TwistStamped stop_cmd;
            stop_cmd.header.stamp = current_time;
            stop_cmd.header.frame_id = "base_footprint";
            stop_cmd.twist.linear.x = 0.0;
            stop_cmd.twist.angular.z = 0.0;

            recovery_cmd_vel_pub_->publish(stop_cmd);

            has_relocalized_ = true;
            RCLCPP_INFO(node_->get_logger(),
                        "✓ Relocalization complete! Starting exploration...");
            return;
        }
    }

    // Check if exploration is complete
    if (isExplorationComplete())
    {
        RCLCPP_INFO(node_->get_logger(), "Exploration complete! Saving map...");
        saveMap("warehouse_map");
        slam_controller_->setExplorationComplete(true);
        stopExploration();
        return;
    }

    // Check if we have valid map and pose
    if (!slam_controller_->hasValidMap() || !slam_controller_->hasValidPose())
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "Waiting for valid map and pose from SLAM...");
        return;
    }

    // Get current state
    auto current_map = slam_controller_->getCurrentMap();
    auto current_pose = slam_controller_->getCurrentPose();

    // Check if returning home
    if (returning_home_)
    {
        returnToHome();
        return; // Skip normal exploration when returning home
    }

    // Check if in recovery mode
    if (in_recovery_)
    {
        performRecovery();
        return; // Skip normal exploration during recovery
    }

    // If no path or reached goal, plan new path (with minimum replan interval)
    bool should_replan =
        !motion_controller_->hasPath() || motion_controller_->isAtGoal();
    double time_since_last_replan =
        (current_time - last_replan_time_).seconds();

    if (should_replan && time_since_last_replan >= MIN_REPLAN_INTERVAL)
    {
        RCLCPP_INFO(node_->get_logger(), "Planning new exploration path...");

        auto new_path = exploration_planner_->planExplorationPath(*current_map,
                                                                  current_pose);

        if (!new_path.poses.empty())
        {
            // Store the new goal position
            last_goal_position_ = new_path.poses.back().pose.position;
            last_replan_time_ = current_time;
            consecutive_no_path_count_ = 0;      // Reset counter on success
            consecutive_no_frontiers_count_ = 0; // Reset no-frontiers counter

            motion_controller_->setPath(new_path);
            path_pub_->publish(new_path);
            RCLCPP_INFO(node_->get_logger(),
                        "New path planned with %zu waypoints",
                        new_path.poses.size());
        }
        else
        {
            // Check if dynamic lookahead is actively working
            bool is_reducing_lookahead =
                exploration_planner_->isReducingLookahead();

            // Track if this is due to no frontiers
            if (exploration_planner_->getNoFrontiersCounter() > 0)
            {
                consecutive_no_frontiers_count_++;
            }

            // Only count as failure if NOT actively reducing lookahead
            if (!is_reducing_lookahead)
            {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 2000,
                    "No valid exploration path found (%d/%d attempts, %d "
                    "no-frontiers)",
                    consecutive_no_path_count_ + 1, MAX_NO_PATH_BEFORE_RECOVERY,
                    consecutive_no_frontiers_count_);
                consecutive_no_path_count_++;
            }
            else
            {
                RCLCPP_DEBUG_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 2000,
                    "No path yet, but dynamic lookahead is reducing distance - "
                    "not counting as failure");
            }

            // If we've had many "no frontiers" results, skip recovery and go
            // home Recovery is risky when map is complete - robot might hit
            // walls
            if (consecutive_no_frontiers_count_ >= 8)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "No frontiers found for %d consecutive attempts - "
                            "map appears complete",
                            consecutive_no_frontiers_count_);
                RCLCPP_INFO(node_->get_logger(),
                            "Skipping recovery, initiating return to home");
                returning_home_ = true;
                recovery_attempt_ = 0;
                return;
            }

            // If we've had many "no frontiers" results, reduce recovery
            // attempts
            int max_recovery = MAX_RECOVERY_ATTEMPTS;
            if (consecutive_no_frontiers_count_ >= 5)
            {
                max_recovery =
                    3; // Reduce to 3 attempts when map seems nearly complete
                RCLCPP_DEBUG(node_->get_logger(),
                             "Map appears nearly complete, reducing recovery "
                             "attempts to %d",
                             max_recovery);
            }

            // If we've failed too many times, enter recovery mode
            if (consecutive_no_path_count_ >= MAX_NO_PATH_BEFORE_RECOVERY)
            {
                // Check if we should give up
                if (recovery_attempt_ >= max_recovery)
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "Max recovery attempts reached - exploration "
                                "appears complete");
                    RCLCPP_INFO(node_->get_logger(),
                                "Initiating return to home");
                    returning_home_ = true;
                    recovery_attempt_ = 0;
                    return;
                }
                performRecovery();
                return;
            }
        }
    }
    else if (should_replan)
    {
        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                              "Waiting %.1fs before replanning (%.1fs elapsed)",
                              MIN_REPLAN_INTERVAL, time_since_last_replan);
    }

    // Compute and publish velocity command (MotionController publishes
    // directly)
    if (motion_controller_->hasPath())
    {
        auto cmd_vel = motion_controller_->computeVelocityCommand(current_pose,
                                                                  *current_map);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Motion control: linear=%.3f, angular=%.3f",
                             cmd_vel.linear.x, cmd_vel.angular.z);
    }
    else
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "No path available for motion control");
    }
}
