/*
 * File: CCameraRedDetector.cpp
 * Description: Implements the CCameraRedDetector class 
   for detecting red objects in camera images using OpenCV in ROS2.
   Publishes detection results and debug images for TurtleBot3 Gazebo 
   simulation.
 *
 * Date: 2025-10-10
 *
 * This file is part of the TurtleBot3 Gazebo simulation package.
 * It subscribes to camera image topics, processes images to detect red regions,
 * and publishes detection status, centroid location, and debug images.
 */

// src/CCameraRedDetector.cpp
#include "turtlebot3_gazebo/CCameraRedDetector.hpp"

namespace turtlebot3_gazebo {

// CCameraRedDetector Constructor
CCameraRedDetector::CCameraRedDetector(const rclcpp::NodeOptions& aOptions)
  : rclcpp::Node("camera_red_detector", aOptions)
{
  // Params
  this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
  this->declare_parameter<double>("min_area_frac", 0.02);
  this->declare_parameter<bool>("publish_debug", true);

  image_topic_ = this->get_parameter("image_topic").as_string();
  min_area_frac_ = this->get_parameter("min_area_frac").as_double();
  publish_debug_ = this->get_parameter("publish_debug").as_bool();

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CCameraRedDetector::image_callback, this, std::placeholders::_1));

  seen_pub_ = this->create_publisher<std_msgs::msg::Bool>("/red_detector/seen", 10);
  center_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/red_detector/centroid", 10);

  if (publish_debug_) 
  {
    debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/red_detector/debug", 1);
  }

  RCLCPP_INFO(this->get_logger(), "CCameraRedDetector listening on %s", image_topic_.c_str());
}

// CCameraRedDetector image callback
void CCameraRedDetector::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& aMsg)
{
  cv::Mat bgr;
  try 
  {
    bgr = cv_bridge::toCvShare(aMsg, sensor_msgs::image_encodings::BGR8)->image;
  } 
  catch (const cv_bridge::Exception& e) 
  {
    RCLCPP_WARN(this->get_logger(), "cv_bridge error: %s", e.what());
    return;
  }
  if (bgr.empty()) return;

  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  // Two hue bands for red
  cv::Mat mask1, mask2, mask;
  cv::inRange(hsv, cv::Scalar(0, 80, 80), cv::Scalar(10, 255, 255), mask1);
  cv::inRange(hsv, cv::Scalar(170, 80, 80), cv::Scalar(180, 255, 255), mask2);
  cv::bitwise_or(mask1, mask2, mask);

  // Clean up with morphology
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5});
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

  const double red_pixels = static_cast<double>(cv::countNonZero(mask));
  const double area_frac = red_pixels / (mask.rows * mask.cols);

  std_msgs::msg::Bool seen;
  seen.data = (area_frac >= min_area_frac_);
  seen_pub_->publish(seen);

  geometry_msgs::msg::Point c;
  c.x = c.y = c.z = std::numeric_limits<double>::quiet_NaN();

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // Find largest contour
  if (!contours.empty()) 
  {
    std::size_t best_i = 0; 
    double best_a = 0.0;
    for (std::size_t i = 0; i < contours.size(); ++i) 
    {
      double a = cv::contourArea(contours[i]);
      if (a > best_a) 
      { 
        best_a = a; 
        best_i = i; 
      }
    }
    cv::Moments m = cv::moments(contours[best_i]);

    // Compute centroid in normalized coordinates
    if (m.m00 > 1e-6) 
    {
      double cx = m.m10 / m.m00;
      double cy = m.m01 / m.m00;
      c.x = 2.0 * (cx / (bgr.cols - 1.0)) - 1.0;
      c.y = 2.0 * (cy / (bgr.rows - 1.0)) - 1.0;
      c.z = area_frac;
      center_pub_->publish(c);
    }
  }

  // Publish debug image with marker
  if (publish_debug_ && debug_pub_) 
  {
    cv::Mat dbg;
    cv::cvtColor(mask, dbg, cv::COLOR_GRAY2BGR);
    if (!std::isnan(c.x)) 
    {
      int u = static_cast<int>((c.x * 0.5 + 0.5) * (bgr.cols - 1));
      int v = static_cast<int>((c.y * 0.5 + 0.5) * (bgr.rows - 1));
      cv::drawMarker(dbg, {u, v}, {255, 255, 255}, cv::MARKER_CROSS, 20, 2);
    }
    auto out = cv_bridge::CvImage(aMsg->header, sensor_msgs::image_encodings::BGR8, dbg).toImageMsg();
    debug_pub_->publish(*out);
  }
}

} // namespace turtlebot3_gazebo

int main(int argc, char** argv) 
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtlebot3_gazebo::CCameraRedDetector>());
  rclcpp::shutdown();
  return 0;
}
