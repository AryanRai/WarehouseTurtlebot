#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>

using std::chrono::milliseconds;

class BatterySimulator : public rclcpp::Node {
public:
  BatterySimulator()
  : Node("battery_simulator")
  {
    declare_parameter<double>("update_rate", 1.0);
    declare_parameter<double>("initial_percentage", 100.0);
    declare_parameter<double>("idle_drain_per_sec", 0.01); // percent per second
    declare_parameter<double>("move_drain_per_mps", 0.1); // percent per meter/sec per second
  // default to the project's main topic name
  declare_parameter<std::string>("battery_topic", "/battery_state");

    this->get_parameter("update_rate", update_rate_);
    this->get_parameter("initial_percentage", percentage_);
    this->get_parameter("idle_drain_per_sec", idle_drain_per_sec_);
    this->get_parameter("move_drain_per_mps", move_drain_per_mps_);
    this->get_parameter("battery_topic", battery_topic_);

    battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(battery_topic_, 10);

    // optional: listen to cmd_vel to simulate additional drain when moving
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&BatterySimulator::twistCb, this, std::placeholders::_1));

    auto period = std::chrono::duration<double>(1.0 / std::max(0.001, update_rate_));
    timer_ = this->create_wall_timer(period, std::bind(&BatterySimulator::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "Battery simulator started: publishing %s at %.2f Hz",
                battery_topic_.c_str(), update_rate_);
  }

private:
  void twistCb(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mu_);
    // approximate linear speed magnitude
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    last_speed_ = std::hypot(vx, vy);
  }

  void onTimer() {
    double dt = 1.0 / std::max(0.001, update_rate_);
    double extra_drain = 0.0;
    {
      std::lock_guard<std::mutex> lk(mu_);
      extra_drain = last_speed_ * move_drain_per_mps_ * dt; // percent
    }
    double drain = (idle_drain_per_sec_ * dt) + extra_drain;
    percentage_ -= drain;
    if (percentage_ < 0.0) percentage_ = 0.0;

    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "battery";
    // approximate voltage linearly 0-100% -> 18.0-24.0 V (example)
    msg.voltage = 18.0 + (percentage_ / 100.0) * (24.0 - 18.0);
    msg.current = 0.0; // unknown
    msg.charge = (percentage_ / 100.0) * 4.0; // assume 4Ah design capacity
    msg.percentage = percentage_;
    msg.temperature = 25.0;
    msg.present = true;

    battery_pub_->publish(msg);
  }

  // params
  double update_rate_ = 1.0;
  double percentage_ = 100.0;
  double idle_drain_per_sec_ = 0.01;
  double move_drain_per_mps_ = 0.1;
  std::string battery_topic_ = "/battery_state";

  // state
  double last_speed_ = 0.0;
  std::mutex mu_;

  // ROS primitives
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BatterySimulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
