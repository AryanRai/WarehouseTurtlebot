#include "SlamController.hpp"
#include <fstream>
#include <sys/wait.h>

SlamController::SlamController(rclcpp::Node::SharedPtr node)
    : node_(node),
      has_valid_map_(false),
      has_valid_pose_(false),
      exploration_complete_(false) {
    
    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Subscribe to map topic (from SLAM Toolbox)
    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&SlamController::mapCallback, this, std::placeholders::_1)
    );
    
    // Subscribe to odometry
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&SlamController::odomCallback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(node_->get_logger(), "SLAM Controller initialized");
}

void SlamController::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = msg;
    has_valid_map_ = true;
}

void SlamController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    updatePoseFromTF();
}

void SlamController::updatePoseFromTF() {
    try {
        auto transform = tf_buffer_->lookupTransform(
            "map", "base_footprint", tf2::TimePointZero
        );
        
        current_pose_.position.x = transform.transform.translation.x;
        current_pose_.position.y = transform.transform.translation.y;
        current_pose_.position.z = transform.transform.translation.z;
        current_pose_.orientation = transform.transform.rotation;
        
        has_valid_pose_ = true;
    } catch (const tf2::TransformException& ex) {
        // TF not ready yet, will try again
    }
}

nav_msgs::msg::OccupancyGrid::SharedPtr SlamController::getCurrentMap() const {
    return current_map_;
}

bool SlamController::hasValidMap() const {
    return has_valid_map_ && current_map_ != nullptr;
}

geometry_msgs::msg::Pose SlamController::getCurrentPose() const {
    return current_pose_;
}

bool SlamController::hasValidPose() const {
    return has_valid_pose_;
}

void SlamController::saveMap(const std::string& map_name) {
    if (!hasValidMap()) {
        RCLCPP_WARN(node_->get_logger(), "Cannot save map - no valid map available");
        return;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Saving map to: %s", map_name.c_str());
    
    // Method 1: Try SLAM Toolbox's serialize_map service first (most reliable)
    std::string slam_command = "ros2 service call /slam_toolbox/serialize_map "
                              "slam_toolbox/srv/SerializePoseGraph "
                              "\"{filename: '" + map_name + "'}\" "
                              "> /dev/null 2>&1 &";
    system(slam_command.c_str());
    
    // Method 2: Also save using map_saver for standard .pgm/.yaml format
    // Run in background with timeout to avoid blocking
    std::string command = "(timeout 8s ros2 run nav2_map_server map_saver_cli "
                         "-f " + map_name + " "
                         "--ros-args -p timeout:=5000 "
                         "> /dev/null 2>&1) &";
    
    system(command.c_str());
    
    // Give it a moment to start
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    RCLCPP_INFO(node_->get_logger(), "Map save commands issued (running in background)");
    
    // Save robot pose immediately
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
        RCLCPP_INFO(node_->get_logger(), "Robot pose saved to: %s_pose.txt", map_name.c_str());
    }
    
    // Note: Map files will be created in background, check filesystem later if needed
    RCLCPP_INFO(node_->get_logger(), "Map save initiated. Files will appear shortly.");
}

bool SlamController::isExplorationComplete() const {
    return exploration_complete_;
}

void SlamController::setExplorationComplete(bool complete) {
    exploration_complete_ = complete;
    if (complete) {
        RCLCPP_INFO(node_->get_logger(), "Exploration marked as complete");
    }
}
