// MTRX3760 2025 Project 2: Battery Terminal Display Implementation
// File: battery_terminal_display.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <ctime>

class BatteryTerminalDisplay : public rclcpp::Node {
public:
    BatteryTerminalDisplay() : Node("battery_terminal_display") {
        // Declare parameters
        this->declare_parameter<std::string>("battery_topic", "/battery_state");
        this->declare_parameter<double>("update_rate_hz", 1.0);
        this->declare_parameter<bool>("clear_screen", false);
        this->declare_parameter<double>("color_good_pct", 30.0);
        this->declare_parameter<double>("color_warning_pct", 15.0);
        
        // Get parameters
        std::string battery_topic;
        this->get_parameter("battery_topic", battery_topic);
        this->get_parameter("update_rate_hz", update_rate_);
        this->get_parameter("clear_screen", clear_screen_);
        this->get_parameter("color_good_pct", color_good_pct_);
        this->get_parameter("color_warning_pct", color_warning_pct_);
        
        // Subscribe to battery state
        battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            battery_topic, 10,
            std::bind(&BatteryTerminalDisplay::batteryCallback, this, std::placeholders::_1));
        
        // Create timer for display updates
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_));
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&BatteryTerminalDisplay::displayUpdate, this));
        
        RCLCPP_INFO(this->get_logger(), "Battery Terminal Display started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", battery_topic.c_str());
        
        // Print initial header
        printHeader();
    }

private:
    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        last_battery_state_ = msg;
        has_data_ = true;
    }
    
    void displayUpdate() {
        if (!has_data_) {
            return;
        }
        
        if (clear_screen_) {
            // Clear screen
            std::cout << "\033[2J\033[H";
            printHeader();
        }
        
        printBatteryStatus();
    }
    
    void printHeader() {
        std::cout << "\n";
        std::cout << "┌─────────────────────────────────────────────────────┐\n";
        std::cout << "│         TURTLEBOT3 BATTERY STATUS MONITOR          │\n";
        std::cout << "└─────────────────────────────────────────────────────┘\n";
        std::cout << std::endl;
    }
    
    void printBatteryStatus() {
        if (!last_battery_state_) return;
        
        auto battery = last_battery_state_;
        double percentage = battery->percentage;
        double voltage = battery->voltage;
        
        // Determine color based on percentage
        std::string color = getColorForPercentage(percentage);
        std::string status = getStatusText(percentage);
        
        // Get current time
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream time_ss;
        time_ss << std::put_time(std::localtime(&time_t), "%H:%M:%S");
        
        // Build battery bar
        std::string battery_bar = buildBatteryBar(percentage);
        
        // Print status (with color)
        std::cout << color;
        std::cout << "┌─────────────────────────────────────────────────────┐\n";
        std::cout << "│ Time:        " << std::setw(38) << std::left << time_ss.str() << "│\n";
        std::cout << "│ Voltage:     " << std::setw(38) << std::left 
                  << (std::to_string(voltage).substr(0, 5) + " V") << "│\n";
        std::cout << "│ Percentage:  " << std::setw(38) << std::left 
                  << (std::to_string(percentage).substr(0, 5) + " %  " + battery_bar) << "│\n";
        std::cout << "│ Temperature: " << std::setw(38) << std::left 
                  << (std::to_string(battery->temperature).substr(0, 4) + " °C") << "│\n";
        std::cout << "│ Status:      " << std::setw(38) << std::left << status << "│\n";
        std::cout << "└─────────────────────────────────────────────────────┘";
        std::cout << "\033[0m" << std::endl; // Reset color
        
        // Move cursor up if not clearing screen
        if (!clear_screen_) {
            std::cout << "\033[7A"; // Always 7 lines now
        }
        
        std::cout << std::flush;
    }
    
    std::string buildBatteryBar(double percentage) {
        int filled = static_cast<int>(percentage / 10.0);
        std::string bar;
        for (int i = 0; i < 10; ++i) {
            bar += (i < filled) ? "█" : "░";
        }
        return bar;
    }
    
    std::string getColorForPercentage(double percentage) {
        if (percentage >= color_good_pct_) {
            return "\033[32m"; // Green
        } else if (percentage >= color_warning_pct_) {
            return "\033[33m"; // Yellow
        } else {
            return "\033[31m"; // Red
        }
    }
    
    std::string getStatusText(double percentage) {
        if (percentage >= color_good_pct_) {
            return "HEALTHY ✓";
        } else if (percentage >= color_warning_pct_) {
            return "LOW BATTERY ⚠";
        } else {
            return "CRITICAL! ⚠⚠⚠";
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::BatteryState::SharedPtr last_battery_state_;
    
    bool has_data_ = false;
    double update_rate_ = 1.0;
    bool clear_screen_ = false;
    double color_good_pct_ = 30.0;
    double color_warning_pct_ = 15.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryTerminalDisplay>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
