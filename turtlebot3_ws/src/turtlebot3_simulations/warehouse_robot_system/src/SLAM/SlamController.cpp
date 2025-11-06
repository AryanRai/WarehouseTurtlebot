// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: SlamController.cpp
// Author(s): Inez Dumas, Tony Bechara, Aryan Rai
//
// Description: Implementation of slam controller for managing SLAM operations.

#include "SLAM/SlamController.hpp"
#include <fstream>
#include <sys/wait.h>

SlamController::SlamController()
    : Node("slam_controller"),
      has_valid_map_(false),
      has_valid_pose_(false),
      exploration_complete_(false) {
    
    // Declare parameters
    declareParameters();
    
    // Get parameters
    pose_publish_rate_ = this->get_parameter("pose_publish_rate").as_double();
    map_republish_rate_ = this->get_parameter("map_republish_rate").as_double();
    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    
    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&SlamController::mapCallback, this, std::placeholders::_1)
    );
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&SlamController::odomCallback, this, std::placeholders::_1)
    );
    
    // Publishers
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/slam/map", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/slam/pose", 10);
    map_ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("/slam/map_ready", 10);
    exploration_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/slam/exploration_complete", 10
    );
    
    // No services in simplified version - use public methods directly if needed
    
    // Timers
    pose_publish_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / pose_publish_rate_),
        std::bind(&SlamController::posePublishCallback, this)
    );
    
    map_republish_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / map_republish_rate_),
        std::bind(&SlamController::mapRepublishCallback, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "SlamController node initialized");
    RCLCPP_INFO(this->get_logger(), "  Pose publish rate: %.1f Hz", pose_publish_rate_);
    RCLCPP_INFO(this->get_logger(), "  Map republish rate: %.1f Hz", map_republish_rate_);
}

void SlamController::declareParameters() {
    this->declare_parameter("pose_publish_rate", 10.0);  // Hz
    this->declare_parameter("map_republish_rate", 1.0);  // Hz
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("base_frame", "base_footprint");
}

void SlamController::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = msg;
    has_valid_map_ = true;
    
    // Publish map ready status
    std_msgs::msg::Bool map_ready_msg;
    map_ready_msg.data = true;
    map_ready_pub_->publish(map_ready_msg);
}

void SlamController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Trigger pose update from TF
    updatePoseFromTF();
}

void SlamController::posePublishCallback() {
    updatePoseFromTF();
    
    if (has_valid_pose_) {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = map_frame_;
        pose_msg.pose = current_pose_;
        pose_pub_->publish(pose_msg);
    }
}

void SlamController::mapRepublishCallback() {
    if (has_valid_map_ && current_map_) {
        // Republish map with current timestamp
        auto map_msg = *current_map_;
        map_msg.header.stamp = this->now();
        map_pub_->publish(map_msg);
    }
    
    // Publish exploration status
    std_msgs::msg::Bool status_msg;
    status_msg.data = exploration_complete_;
    exploration_status_pub_->publish(status_msg);
}

void SlamController::updatePoseFromTF() {
    try {
        auto transform = tf_buffer_->lookupTransform(
            map_frame_, base_frame_, tf2::TimePointZero
        );
        
        current_pose_.position.x = transform.transform.translation.x;
        current_pose_.position.y = transform.transform.translation.y;
        current_pose_.position.z = transform.transform.translation.z;
        current_pose_.orientation = transform.transform.rotation;
        
        has_valid_pose_ = true;
    } catch (const tf2::TransformException& ex) {
        // TF not ready yet, will try again
        if (!has_valid_pose_) {
            RCLCPP_DEBUG(this->get_logger(), "Waiting for TF: %s", ex.what());
        }
    }
}

void SlamController::saveMap(const std::string& map_name) {
    RCLCPP_INFO(this->get_logger(), "Saving map to: %s", map_name.c_str());
    
    // Method 1: SLAM Toolbox's serialize_map service
    std::string slam_command = "ros2 service call /slam_toolbox/serialize_map "
                              "slam_toolbox/srv/SerializePoseGraph "
                              "\"{filename: '" + map_name + "'}\" "
                              "> /dev/null 2>&1 &";
    system(slam_command.c_str());
    
    // Method 2: Standard map_saver for .pgm/.yaml format
    std::string command = "(timeout 8s ros2 run nav2_map_server map_saver_cli "
                         "-f " + map_name + " "
                         "--ros-args -p timeout:=5000 "
                         "> /dev/null 2>&1) &";
    system(command.c_str());
    
    // Give it a moment to start
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // Save robot pose
    std::ofstream pose_file(map_name + "_pose.txt");
    if (pose_file.is_open()) {
        pose_file << current_pose_.position.x << " "
                 << current_pose_.position.y << " "
                 << current_pose_.position.z << " "
                 << current_pose_.orientation.x << " "
                 << current_pose_.orientation.y << " "
                 << current_pose_.orientation.z << " "
                 << current_pose_.orientation.w << std::endl;
        pose_file.close();
        RCLCPP_INFO(this->get_logger(), "Robot pose saved to: %s_pose.txt", map_name.c_str());
    }
    
    RCLCPP_INFO(this->get_logger(), "Map save initiated. Files will appear shortly.");
}

void SlamController::setExplorationComplete(bool complete) {
    exploration_complete_ = complete;
    RCLCPP_INFO(this->get_logger(), "Exploration marked as %s", complete ? "complete" : "incomplete");
}