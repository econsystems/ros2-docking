#include <memory>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <yaml-cpp/yaml.h>
#include <cstdlib>
#include <std_msgs/msg/bool.hpp>

using std::placeholders::_1;

class BatteryNavNode : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  

  BatteryNavNode()
  : Node("battery_monitor_and_navigate_docking") {
    this->declare_parameter<std::string>("yaml_filename", "");
    this->declare_parameter<double>("battery_threshold", 0.3);

    // backup log source paths (provided from launch)
    this->declare_parameter<std::string>("memlog_src", "/sdcard/mmcblk2p1/soundarya/FILES_OF_ROVER/rover4_memlog.txt");
    this->declare_parameter<std::string>("log_src", "/sdcard/mmcblk2p1/soundarya/FILES_OF_ROVER/rover4_loggg.txt");

    this->get_parameter("memlog_src", memlog_src_);
    this->get_parameter("log_src", log_src_);

    // Get parameters
    this->get_parameter("yaml_filename", yaml_path_);
    this->get_parameter("battery_threshold", battery_threshold_);

    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "battery_info", 10, std::bind(&BatteryNavNode::battery_callback, this, _1));

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("reached_spawn", 10);

    // Wait for action server
    while (!nav_client_->wait_for_action_server(std::chrono::seconds(2)) && rclcpp::ok()) {
      RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_;
  std::string yaml_path_;
  double battery_threshold_; 
  bool goal_sent_ = false;
  bool navigating_to_dock_ = false;
  bool reached_published_ = false;
  int retry_count_ = 0;
  const int max_retries_ = 3;
  // backup path parameters
  std::string memlog_src_;
  std::string log_src_;

  std::string make_backup_path(const std::string &src)
  {
    // Insert "_bkp" before extension if any, otherwise append "_bkp"
    auto pos = src.find_last_of('.');
    if (pos == std::string::npos) return src + "_bkp";
    return src.substr(0, pos) + "_bkp" + src.substr(pos);
  }

  void backup_logs_now()
  {
    std::string memlog_bkp = make_backup_path(memlog_src_);
    std::string log_bkp = make_backup_path(log_src_);
    std::string cmd1 = "/bin/cp \"" + memlog_src_ + "\" \"" + memlog_bkp + "\"";
    std::string cmd2 = "/bin/cp \"" + log_src_ + "\" \"" + log_bkp + "\"";
    int cp1 = system(cmd1.c_str());
    int cp2 = system(cmd2.c_str());
    if (cp1 == 0 && cp2 == 0) {
      RCLCPP_INFO(this->get_logger(), "Backed up '%s' -> '%s' and '%s' -> '%s'",
                  memlog_src_.c_str(), memlog_bkp.c_str(), log_src_.c_str(), log_bkp.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to back up logs (cp1=%d cp2=%d)", cp1, cp2);
    }
  }

  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    if (msg->percentage < battery_threshold_ && !goal_sent_ && !navigating_to_dock_) {
        RCLCPP_WARN(this->get_logger(), "Battery low: %.2f%% — initiating waypoint navigation", msg->percentage * 100.0);
        navigating_to_dock_ = true;
        reached_published_ = false;
        // Kill waypoint follower process first
        int kill_ret = system("pkill -f 'cpp_waypoint_follower.launch.py'");
        if (kill_ret == 0) {
            RCLCPP_INFO(this->get_logger(), "Waypoint follower killed successfully.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Waypoint follower was not running or kill failed.");
        }

        rclcpp::sleep_for(std::chrono::seconds(1)); // allow cleanup

        // Turn LED off immediately
        int ret = system("/usr/bin/amba_debug -g 30 -d 0x0");
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute amba_debug command. Return code: %d", ret);
        } else {
            RCLCPP_INFO(this->get_logger(), "amba_debug led off executed successfully.");
        }

        // Cancel all previous goals before sending new one
        nav_client_->async_cancel_all_goals();

        // Give a short delay to ensure cancellation
        rclcpp::sleep_for(std::chrono::seconds(3));

        send_goal_from_yaml();
    }
  }

    void stop_robot() {
        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;

        for (int i = 0; i < 5; i++) {
            cmd_vel_pub_->publish(stop_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(this->get_logger(), "Sent stop command to robot.");
    }

  void send_goal_from_yaml() {
    YAML::Node yaml_file = YAML::LoadFile(yaml_path_);
    if (!yaml_file["waypoints"]) {
        RCLCPP_ERROR(this->get_logger(), "YAML file does not contain 'waypoints' field.");
        return;
    }

    YAML::Node waypoint = yaml_file["waypoints"].begin()->second;  // First waypoint

    nav2_msgs::action::NavigateToPose::Goal goal_msg;
    goal_msg.pose.pose.position.x = waypoint["pose"]["x"].as<double>();
    goal_msg.pose.pose.position.y = waypoint["pose"]["y"].as<double>();
    goal_msg.pose.pose.position.z = waypoint["pose"]["z"].as<double>();
    goal_msg.pose.pose.orientation.x = waypoint["orientation"]["x"].as<double>();
    goal_msg.pose.pose.orientation.y = waypoint["orientation"]["y"].as<double>();
    goal_msg.pose.pose.orientation.z = waypoint["orientation"]["z"].as<double>();
    goal_msg.pose.pose.orientation.w = waypoint["orientation"]["w"].as<double>();

    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->get_clock()->now();

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = [](auto) {};
    options.feedback_callback = [](auto, auto) {};

    options.result_callback = [this](const GoalHandleNav::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Goal reached successfully.");
            goal_sent_ = true;
            navigating_to_dock_ = false;

            
            // 1. Stop robot first
            stop_robot();
            rclcpp::sleep_for(std::chrono::seconds(3));  // wait for stop
            
            // Publish true once
            if (!reached_published_) {
                std_msgs::msg::Bool msg;
                msg.data = true;
                reached_pub_->publish(msg);
                reached_published_ = true;
                RCLCPP_INFO(this->get_logger(), "Published /reached_spawn = true");
                rclcpp::sleep_for(std::chrono::milliseconds(500)); // allow flush
            }

            // 2. Backup logs
            backup_logs_now();
            rclcpp::sleep_for(std::chrono::seconds(3));  // wait for backup
            
            // // 3. Finally kill ROS
            // system("pkill -f ros2");
            
        } else {
            if (++retry_count_ <= max_retries_) {
                RCLCPP_WARN(this->get_logger(), "Goal failed. Retrying (%d/%d)...", 
                           retry_count_, max_retries_);
                // stop_robot();
                rclcpp::sleep_for(std::chrono::seconds(1));
                send_goal_from_yaml();
            } else {
                RCLCPP_ERROR(this->get_logger(), 
                            "Max retries reached. Stopping robot and shutting down.");
                // 1. Stop robot
                stop_robot();
                rclcpp::sleep_for(std::chrono::seconds(3));
                
                // Publish false once (final failure)
                if (!reached_published_) {
                    std_msgs::msg::Bool msg;
                    msg.data = false;
                    reached_pub_->publish(msg);
                    reached_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Published /reached_spawn = false");
                    rclcpp::sleep_for(std::chrono::milliseconds(500)); // allow flush
                }

                // 2. Backup logs
                backup_logs_now();
                rclcpp::sleep_for(std::chrono::seconds(2));  // wait for backup
                
                // 3. Kill ROS (only once)
                // system("pkill -f ros2");
            }
        }
    };

    nav_client_->async_send_goal(goal_msg, options);
  }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BatteryNavNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

