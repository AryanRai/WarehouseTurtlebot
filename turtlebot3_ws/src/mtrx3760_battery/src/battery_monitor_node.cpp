// MTRX3760 2025 Project 2: Battery Monitor Node Implementation
// File: battery_monitor_node.cpp
//
// IMPLEMENTATION GUIDE:
//
// 1. BATTERY DATA SOURCES:
//    For Real TurtleBot3:
//    - Read from /sensor_state topic (SensorState message has battery voltage)
//    - Or use dynamixel_sdk to read directly from OpenCR board
//    
//    For Gazebo Simulation:
//    - Create simulated battery drain based on velocity commands
//    - Start at 100%, drain proportional to movement
//    - Example: -0.5% per meter traveled, -0.1% per rotation
//
// 2. BATTERY STATE CALCULATION:
//    - Voltage range: 11.0V (empty) to 12.6V (full) for 3S LiPo
//    - Percentage = (current_voltage - min_voltage) / (max_voltage - min_voltage) * 100
//    - Discharge rate = voltage_change / time_elapsed
//    - Time remaining = (current_voltage - cutoff_voltage) / discharge_rate
//
// Lightweight Battery Monitor node (ROS2 / rclcpp)
// Subscribes to a sensor_msgs::msg::BatteryState topic and prints a short status
// at a configurable interval. Designed to follow the project's class-based
// node style (parameters, timer, subscription, thread-safe cached message).

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <mutex>
#include <iomanip>

class BatteryMonitorNode : public rclcpp::Node {
public:
    BatteryMonitorNode()
    : Node("battery_monitor")
    {
    // Default to the project's main topic name '/battery_state'
    this->declare_parameter<std::string>("battery_topic", "/battery_state");
        this->declare_parameter<double>("print_interval", 1.0);

        this->get_parameter("battery_topic", battery_topic_);
        this->get_parameter("print_interval", print_interval_);

        RCLCPP_INFO(this->get_logger(), "BatteryMonitor: subscribing to %s, print interval %.2f s",
                                battery_topic_.c_str(), print_interval_);

        sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            battery_topic_, rclcpp::SensorDataQoS(),
            std::bind(&BatteryMonitorNode::batteryCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(print_interval_),
            std::bind(&BatteryMonitorNode::onTimer, this));
    }

private:
    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(m_);
        last_msg_ = *msg;
        have_msg_ = true;
    }

    void onTimer() {
        std::lock_guard<std::mutex> lk(m_);
        if (!have_msg_) {
            RCLCPP_INFO(this->get_logger(), "No battery message received yet on %s", battery_topic_.c_str());
            return;
        }
        // Print a compact, human-friendly line similar to movement diagnostics
        double perc = last_msg_.percentage;
        double volt = last_msg_.voltage;
        double curr = last_msg_.current;
        double charge = last_msg_.charge;
        double temp = last_msg_.temperature;

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(1);
        ss << "BATTERY: ";
        if (!std::isnan(perc)) ss << perc << "% "; else ss << "N/A% ";
        ss << "| V=" << std::setprecision(2) << volt << " V ";
        ss << "| I=" << std::setprecision(2) << curr << " A ";
        ss << "| Q=" << std::setprecision(2) << charge << " Ah ";
        ss << "| T=" << std::setprecision(1) << temp << " C";

        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    // params
    std::string battery_topic_;
    double print_interval_ = 1.0;

    // ROS primitives
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // cached message
    sensor_msgs::msg::BatteryState last_msg_;
    bool have_msg_ = false;
    std::mutex m_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryMonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
