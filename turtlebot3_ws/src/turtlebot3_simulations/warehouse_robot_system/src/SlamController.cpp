#include "SlamController.hpp"
#include <fstream>

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
    
    // Call map_saver service or use system command
    std::string command = "ros2 run nav2_map_server map_saver_cli -f " + map_name;
    int result = system(command.c_str());
    
    if (result == 0) {
        RCLCPP_INFO(node_->get_logger(), "Map saved successfully: %s", map_name.c_str());
        
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
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to save map");
    }
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
