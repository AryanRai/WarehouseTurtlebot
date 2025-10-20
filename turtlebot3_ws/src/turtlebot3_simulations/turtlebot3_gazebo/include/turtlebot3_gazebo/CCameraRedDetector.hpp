#ifndef TURTLEBOT3_GAZEBO__CCAMERA_RED_DETECTOR_HPP_
#define TURTLEBOT3_GAZEBO__CCAMERA_RED_DETECTOR_HPP_

/*
 * File: CCameraRedDetector.hpp
 * Description: Declares the CCameraRedDetector class for red object detection in camera images using OpenCV in ROS2.
 * 
 * Date: 2025-10-10
 *
 * This header is part of the TurtleBot3 Gazebo simulation package.
 * It defines the interface for subscribing to camera images, detecting red regions,
 * and publishing detection status and centroid location.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>          
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <limits>
#include <vector>

namespace turtlebot3_gazebo {

/**
 * @class CCameraRedDetector
 * @brief Camera-based red object detection node
 * 
 * Processes camera images to detect red objects and publishes detection
 * results including presence flag and centroid location.
 */
class CCameraRedDetector : public rclcpp::Node 
{
    public:
        /**
         * @brief Constructor
         * @param aOptions ROS2 node options
         */
        explicit CCameraRedDetector(const rclcpp::NodeOptions& aOptions = rclcpp::NodeOptions());

    private:
        /**
         * @brief Camera image callback for red detection processing
         * @param msg Camera image message
         */
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& aMsg);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr seen_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr center_pub_;

        std::string image_topic_;
        double min_area_frac_{0.02};
        bool publish_debug_{true};
};

} // namespace turtlebot3_gazebo

#endif  // TURTLEBOT3_GAZEBO__CCAMERA_RED_DETECTOR_HPP_
