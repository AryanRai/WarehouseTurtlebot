// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: MotionController.hpp
// Author(s): Inez Dumas, Tony Bechara, Aryan Rai, Filip Gusavac
//
// Description: slam controller for managing SLAM operations.

#ifndef SLAM_CONTROLLER_HPP
#define SLAM_CONTROLLER_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class SlamController : public rclcpp::Node
{
    public:
        SlamController();
        ~SlamController() = default;

        // Public methods for backwards compatibility (if needed)
        nav_msgs::msg::OccupancyGrid::SharedPtr getCurrentMap() const
        {
            return current_map_;
        }
        bool hasValidMap() const { return has_valid_map_; }
        geometry_msgs::msg::Pose getCurrentPose() const
        {
            return current_pose_;
        }
        bool hasValidPose() const { return has_valid_pose_; }
        bool isExplorationComplete() const { return exploration_complete_; }

        // Public methods for controlling SLAM
        void saveMap(const std::string &map_name = "warehouse_map");
        void setExplorationComplete(bool complete);

    private:
        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        // Publishers
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr map_ready_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
            exploration_status_pub_;

        // TF2
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // Timers
        rclcpp::TimerBase::SharedPtr pose_publish_timer_;
        rclcpp::TimerBase::SharedPtr map_republish_timer_;

        // State
        nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
        geometry_msgs::msg::Pose current_pose_;
        bool has_valid_map_;
        bool has_valid_pose_;
        bool exploration_complete_;

        // Parameters
        double pose_publish_rate_;  // Hz
        double map_republish_rate_; // Hz
        std::string map_frame_;
        std::string base_frame_;

        // Callbacks
        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void posePublishCallback();
        void mapRepublishCallback();

        // Helper functions
        void updatePoseFromTF();
        void declareParameters();
};

#endif // SLAM_CONTROLLER_HPP