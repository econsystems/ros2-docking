#include <memory>
#include <atomic>
#include <thread>
#include <chrono>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class BatteryDockNode : public rclcpp::Node {
public:
  BatteryDockNode()
  : Node("battery_monitor_and_docking"), battery_low_(false), docking_active_(false)
  {
    this->declare_parameter<double>("battery_threshold", 0.20);
    this->get_parameter("battery_threshold", battery_threshold_);

    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "battery_info", 10, std::bind(&BatteryDockNode::battery_cb, this, std::placeholders::_1));

    reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "reached_spawn", 10, std::bind(&BatteryDockNode::reached_cb, this, std::placeholders::_1));

    docking_charged_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "docking_charged", 10, std::bind(&BatteryDockNode::docking_charged_cb, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "BatteryDockNode started (threshold=%.2f)", battery_threshold_);
  }

private:
  void battery_cb(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    battery_low_.store(msg->percentage < battery_threshold_);
  }

  void reached_cb(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) return;
    if (!battery_low_.load()) {
      RCLCPP_INFO(this->get_logger(), "Reached true but battery not low - ignoring");
      return;
    }
    if (docking_active_.exchange(true)) {
      RCLCPP_INFO(this->get_logger(), "Docking already in progress - ignoring");
      return;
    }
    std::thread(&BatteryDockNode::docking_worker, this).detach();
  }

  void docking_charged_cb(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) return; // only act on true
    RCLCPP_INFO(this->get_logger(), "docking_charged == true received");
    // Only start nav if battery is NOT low
    if (battery_low_.load()) {
      RCLCPP_INFO(this->get_logger(), "Battery still low - not starting navigation");
      return;
    }

    // small settle
    std::this_thread::sleep_for(5s);

    // Stop docking service and start nav service
    int rc_stop = std::system("/usr/bin/systemctl stop econ-corridorrun_docking.service");
    RCLCPP_INFO(this->get_logger(), "systemctl stop econ-corridorrun_docking rc=%d", rc_stop);

    // small settle
    std::this_thread::sleep_for(5s);

    int rc_start = std::system("/usr/bin/systemctl start econ-corridorrun_nav.service");
    RCLCPP_INFO(this->get_logger(), "systemctl start nav rc=%d", rc_start);

    // optional: verify nav active (quick check)
    bool started = false;
    for (int i=0; i<5; ++i) {
      int rc = std::system("/usr/bin/systemctl is-active --quiet econ-corridorrun_nav.service");
      if (rc == 0) { started = true; break; }
      std::this_thread::sleep_for(1s);
    }
    if (started) {
      RCLCPP_INFO(this->get_logger(), "econ-corridorrun_nav.service active");
    } else {
      RCLCPP_ERROR(this->get_logger(), "econ-corridorrun_nav.service failed to start");
    }

    // clear docking flag if it was set
    docking_active_.store(false);
  }

  void docking_worker() {
    // Stop robot immediately
    stop_robot();

    // small settle
    std::this_thread::sleep_for(5s);
    
    // Stop nav and start docking service
    int rc1 = std::system("/usr/bin/systemctl stop econ-corridorrun_nav.service");
    RCLCPP_INFO(this->get_logger(), "systemctl stop nav rc=%d", rc1);

    std::this_thread::sleep_for(5s);

    int rc2 = std::system("/usr/bin/systemctl start econ-corridorrun_docking.service");
    RCLCPP_INFO(this->get_logger(), "systemctl start docking rc=%d", rc2);

    // verify docking service active
    bool active = false;
    for (int i=0; i<5; ++i) {
      int rc = std::system("/usr/bin/systemctl is-active --quiet econ-corridorrun_docking.service");
      if (rc == 0) { active = true; break; }
      std::this_thread::sleep_for(1s);
    }
    if (active) {
      RCLCPP_INFO(this->get_logger(), "econ-corridorrun_docking.service active");
    } else {
      RCLCPP_ERROR(this->get_logger(), "econ-corridorrun_docking.service failed to start");
    }

    // small settle
    std::this_thread::sleep_for(2s);
    docking_active_.store(false);
  }

  void stop_robot() {
    geometry_msgs::msg::Twist stop{};
    for (int i = 0; i < 5; ++i) {
      cmd_vel_pub_->publish(stop);
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(this->get_logger(), "Published stop commands");
  }

  // Members
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr docking_charged_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  double battery_threshold_;
  std::atomic<bool> battery_low_;
  std::atomic<bool> docking_active_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BatteryDockNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}