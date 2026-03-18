/*
  Simplified Connect-to-Charging-Dock Node (ROS 2 Humble, C++)
  with Docking Station Protocol Support
  ---------------------------------------------------------------
  - Command Protocol:
    1 = Enable docking mode
    11 = Execute docking sequence
    10 = Execute undocking sequence
    0 = Undock (when already docked)
  - Detects charging via power_supply_status (1=charging, 2=not charging) and /dock/pin_status topic 
*/

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <atomic>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ros2_base_interfaces/srv/dock.hpp"

using namespace std::chrono_literals;

class ConnectToChargingDockNode : public rclcpp::Node
{
public:
  ConnectToChargingDockNode()
  : Node("connect_to_charging_dock_node"),
    linear_velocity_(0.09),
    angular_velocity_search_(0.33),  
    angular_velocity_align_(0.05),  
    goal_idx_(0),
    obstacle_tolerance_(0.14),
    center_offset_tolerance_(5),
    offset_search_tolerance_(50),
    undocking_distance_(0.40),
    approach_distance_(0.25),
    low_battery_min_threshold_(0.25),
    aruco_marker_detected_(false),
    aruco_center_offset_(0),
    obstacle_distance_front_(999999.9),
    pin_status_pressed_(false),
    docking_retry_count_(0),
    max_docking_retries_(5),
    cmd1_sent_(false),
    cmd1_send_distance_(0.25),
    cmd10_max_retries_(2),
    cmd0_max_retries_(2),
    cmd10_cmd0_delay_sec_(10)
  {
    // ---------------- Publishers / Subscribers ----------------
    publisher_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    docking_charged_pub_ = this->create_publisher<std_msgs::msg::Bool>("docking_charged", 10);

    subscription_battery_state_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "battery_info", 10,
      std::bind(&ConnectToChargingDockNode::battery_callback, this, std::placeholders::_1));

    subscription_aruco_detected_ = this->create_subscription<std_msgs::msg::Bool>(
      "aruco_marker_detected", 10,
      std::bind(&ConnectToChargingDockNode::aruco_detected_callback, this, std::placeholders::_1));

    subscription_center_offset_ = this->create_subscription<std_msgs::msg::Int32>(
      "aruco_marker_offset", 10,
      std::bind(&ConnectToChargingDockNode::center_offset_callback, this, std::placeholders::_1));

    subscription_laser_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_filtered", rclcpp::SensorDataQoS(),
      std::bind(&ConnectToChargingDockNode::scan_callback, this, std::placeholders::_1));

    // NEW: Subscribe to dock pin status
    subscription_pin_status_ = this->create_subscription<std_msgs::msg::Bool>(
      "dock/pin_status", 10,
      std::bind(&ConnectToChargingDockNode::pin_status_callback, this, std::placeholders::_1));

    // Create docking service client
    docking_client_ = this->create_client<ros2_base_interfaces::srv::Dock>("docking_control");

    status_timer_ = this->create_wall_timer(1s, [this]() {
      (void)this;
    });

    this->declare_parameter<double>("target_battery", 0.90);
    this->get_parameter("target_battery", target_battery_);
    RCLCPP_INFO(this->get_logger(), "Parameter 'target_battery' = %.0f%%", target_battery_ * 100);

    // Make low battery threshold configurable via ROS parameter (default set in initializer list)
    this->declare_parameter<double>("low_battery_min_threshold", low_battery_min_threshold_);
    this->get_parameter("low_battery_min_threshold", low_battery_min_threshold_);
    RCLCPP_INFO(this->get_logger(), "Parameter 'low_battery_min_threshold' = %.2f", low_battery_min_threshold_);

    this->declare_parameter<double>("approach_distance", approach_distance_);
    this->get_parameter("approach_distance", approach_distance_);
    RCLCPP_INFO(this->get_logger(), "Parameter 'approach_distance' = %.2f m", approach_distance_);

    this->declare_parameter<double>("undocking_distance", undocking_distance_);
    this->get_parameter("undocking_distance", undocking_distance_);
    RCLCPP_INFO(this->get_logger(), "Parameter 'undocking_distance' = %.2f m", undocking_distance_);

    this->declare_parameter<int>("offset_search_tolerance", offset_search_tolerance_);
    this->get_parameter("offset_search_tolerance", offset_search_tolerance_);
    RCLCPP_INFO(this->get_logger(), "Parameter 'offset_search_tolerance' = %d pixels", offset_search_tolerance_);

    this->declare_parameter<double>("angular_velocity_search", angular_velocity_search_);
    this->get_parameter("angular_velocity_search", angular_velocity_search_);
    RCLCPP_INFO(this->get_logger(), "Parameter 'angular_velocity_search' = %.2f rad/s", angular_velocity_search_);

    RCLCPP_INFO(this->get_logger(), "✅ connect_to_charging_dock_node started.");
  }
  
  ~ConnectToChargingDockNode()
  {
    shutdown_requested_.store(true);
    if (staging_thread_.joinable())
      staging_thread_.join();
  }

private:
  // ---------------- Docking Service Methods ----------------
  bool call_docking_service(int command, std::string & reason)
  {
    if (!docking_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(this->get_logger(), "Docking service not available");
      return false;
    }

    auto request = std::make_shared<ros2_base_interfaces::srv::Dock::Request>();
    request->command = command;

    auto future = docking_client_->async_send_request(request);
    
    // Wait for response without spinning (using busy-wait with timeout)
    auto start = std::chrono::steady_clock::now();
    auto timeout = 10s;
    
    while (std::chrono::steady_clock::now() - start < timeout) {
      if (future.wait_for(100ms) == std::future_status::ready) {
        auto response = future.get();
        reason = response->reason;
        
        RCLCPP_INFO(this->get_logger(), "Docking service (cmd=%d): response=%s, reason=%s",
                    command, response->response ? "SUCCESS" : "FAILED", reason.c_str());
        
        return response->response;
      }
      
      if (shutdown_requested_.load()) {
        RCLCPP_WARN(this->get_logger(), "Service call interrupted by shutdown");
        return false;
      }
    }

    RCLCPP_ERROR(this->get_logger(), "Docking service call timed out (cmd=%d)", command);
    return false;
  }

  bool enable_docking_mode()
  {
    std::string reason;
    RCLCPP_INFO(this->get_logger(), "Enabling docking mode (command=1)...");
    bool result = call_docking_service(1, reason);
    
    if (!result) {
      RCLCPP_WARN(this->get_logger(), "Enable docking mode failed: %s", reason.c_str());
    }
    
    return result;
  }

  bool execute_docking()
  {
    std::string reason;
    RCLCPP_INFO(this->get_logger(), "Executing docking sequence (command=11)...");
    bool result = call_docking_service(11, reason);
    
    if (!result) {
      RCLCPP_WARN(this->get_logger(), "Docking failed: %s", reason.c_str());
    }
    
    return result;
  }

  bool execute_undocking()
  {
    // Retry logic: cmd10 max 2 attempts, then cmd0 max 2 attempts (only if cmd10 succeeds)
    RCLCPP_INFO(this->get_logger(), "Executing undocking sequence...");
    
    std::string reason10;
    bool res10 = false;
    
    // Attempt command 10 up to cmd10_max_retries_ times
    for (int attempt = 1; attempt <= cmd10_max_retries_; attempt++) {
      RCLCPP_INFO(this->get_logger(), "Command 10 attempt %d/%d", attempt, cmd10_max_retries_);
      res10 = call_docking_service(10, reason10);
      
      if (res10) {
        RCLCPP_INFO(this->get_logger(), "Command 10 succeeded (attempt %d)", attempt);
        break;
      }
      
      RCLCPP_WARN(this->get_logger(), "Command 10 failed (attempt %d): %s", attempt, reason10.c_str());
      
      if (attempt < cmd10_max_retries_) {
        std::this_thread::sleep_for(1s);
      }
    }
    
    // If cmd10 failed all times, return failure
    if (!res10) {
      RCLCPP_ERROR(this->get_logger(), "Command 10 failed after %d attempts", cmd10_max_retries_);
      std_msgs::msg::Bool msg; msg.data = false;
      docking_charged_pub_->publish(msg);
      return false;
    }
    
    // Command 10 succeeded, wait cmd10_cmd0_delay_sec_ seconds before cmd0
    RCLCPP_INFO(this->get_logger(), "Command 10 success. Waiting %d seconds before Command 0...", cmd10_cmd0_delay_sec_);
    std::this_thread::sleep_for(std::chrono::seconds(cmd10_cmd0_delay_sec_));
    
    std::string reason0;
    bool res0 = false;
    
    // Attempt command 0 up to cmd0_max_retries_ times
    for (int attempt = 1; attempt <= cmd0_max_retries_; attempt++) {
      RCLCPP_INFO(this->get_logger(), "Command 0 attempt %d/%d", attempt, cmd0_max_retries_);
      res0 = call_docking_service(0, reason0);
      
      if (res0) {
        RCLCPP_INFO(this->get_logger(), "Command 0 succeeded (attempt %d)", attempt);
        break;
      }
      
      RCLCPP_WARN(this->get_logger(), "Command 0 failed (attempt %d): %s", attempt, reason0.c_str());
      
      if (attempt < cmd0_max_retries_) {
        std::this_thread::sleep_for(1s);
      }
    }

    // If cmd0 succeeded, back up undocking_distance_ meters
    if (res0) {
      RCLCPP_INFO(this->get_logger(), "Command 0 succeeded ....waiting %d to undock...",(cmd10_cmd0_delay_sec_+5));
      std::this_thread::sleep_for(std::chrono::seconds(cmd10_cmd0_delay_sec_));
      perform_backup(undocking_distance_);
      std::this_thread::sleep_for(std::chrono::seconds(cmd10_cmd0_delay_sec_));
      // Publish undock result to /docking_charged
      std_msgs::msg::Bool dock_msg;
      dock_msg.data = res0;  // true iff cmd0 succeeded (successful undock and backup)
      docking_charged_pub_->publish(dock_msg);
    }
    
    // Log final result
    RCLCPP_INFO(this->get_logger(),
                "Undock sequence complete: cmd10=%s  cmd0=%s",
                res10 ? "SUCCESS" : "FAILED",
                res0 ? "SUCCESS" : "FAILED");
    
    return res0;  // Return success only if cmd0 succeeded
  }

  bool undock_when_docked()
  {
    // force undock: send 10 then 0
    std::string reason10, reason0;
    RCLCPP_INFO(this->get_logger(), "Force undock: sending 10 then 0...");
    bool res10 = call_docking_service(10, reason10);
    std::this_thread::sleep_for(1s);
    bool res0 = call_docking_service(0, reason0);

    RCLCPP_INFO(this->get_logger(),
                "Force undock results: cmd10=%s (%s)  cmd0=%s (%s)",
                res10 ? "SUCCESS" : "FAILED", reason10.c_str(),
                res0 ? "SUCCESS" : "FAILED", reason0.c_str());

    return res0 || res10;
  }

  // ---------------- Battery Callback ----------------
  // Battery Callback with guard against multiple simultaneous docking threads
  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(batt_mutex_);
    this_battery_state_ = *msg;

    if (this_battery_state_.percentage < low_battery_min_threshold_) {
      // Only launch docking if:
      // 1. No docking is currently in progress
      // 2. We can successfully set the flag to true
      bool expected = false;
      if (staging_navigation_in_progress_.compare_exchange_strong(expected, true)) {
        RCLCPP_INFO(this->get_logger(), 
                    "Low battery (%.1f%%) detected → launching docking sequence",
                    this_battery_state_.percentage * 100);
        
        // Wait for previous thread to finish before starting new one
        if (staging_thread_.joinable()) {
          RCLCPP_DEBUG(this->get_logger(), "Waiting for previous staging thread to finish...");
          staging_thread_.join();
        }
        
        // Now launch new thread
        staging_thread_ = std::thread(std::bind(&ConnectToChargingDockNode::staging_and_docking_task, this));
      } else {
        RCLCPP_DEBUG(this->get_logger(), 
                     "Docking already in progress, ignoring low battery callback");
      }
    }
  }

    // ---------- Pin Status Callback (NEW) ----------
  void pin_status_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    pin_status_pressed_.store(msg->data);
    if (msg->data) {
      RCLCPP_INFO(this->get_logger(), "🔌 DOCK PIN PRESSED - Physical contact detected!");
    }
  }

  // ---------------- ArUco & Scan Callbacks ----------------
  void aruco_detected_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    aruco_marker_detected_.store(msg->data);
  }

  void center_offset_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    aruco_center_offset_.store(msg->data);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double val = 999999.9;
    if (!msg->ranges.empty()) {
      double angle_deg_front = 180.0;
      double angle_rad_front = angle_deg_front * M_PI / 180.0; 
      int index = static_cast<int>(
        std::round((angle_rad_front - msg->angle_min) / msg->angle_increment)
      );
      if (index >= 0 && index < static_cast<int>(msg->ranges.size()))
        val = msg->ranges[index];
    }

    std::lock_guard<std::mutex> lk(obs_mutex_);
    obstacle_distance_front_ = val;
  }

  // ---------------- Docking Thread Entry ----------------
  void staging_and_docking_task()
  {
    try {
      RCLCPP_INFO(this->get_logger(), "Starting navigation and docking task");
      connect_to_dock();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "Exception in docking thread: %s", ex.what());
    }

    staging_navigation_in_progress_.store(false);
    RCLCPP_INFO(this->get_logger(), "Docking sequence finished");
  }

  // MODIFIED: Main docking routine with reset ONLY at start
  void connect_to_dock()
  {
    // Reset state variables ONCE at the very beginning
    reset_docking_state();
    
    RCLCPP_INFO(this->get_logger(), "Beginning docking routine...");

    // Phase 1: Navigate to dock using ArUco markers
    while (!shutdown_requested_.load()) {
      // NEW: Check if pin was pressed during navigation
      if (pin_status_pressed_.load()) {
        RCLCPP_INFO(this->get_logger(), 
                    "🔌 Pin pressed detected during navigation → starting docking protocol");
        geometry_msgs::msg::Twist stop;
        publisher_cmd_vel_->publish(stop);
        goal_idx_ = 3;  // Skip to docking phase
        break;
      }

      if (goal_idx_ == 0) {
        search_for_aruco_marker();
      } else if (goal_idx_ == 1 && !search_phase_used_) {
        // Only run search pattern if it hasn't been used yet in this cycle
        int result = perform_search_pattern();
        if (result == 1) {
          goal_idx_ = 2;
        } else if (result == 2) {
          goal_idx_ = 1;  // Repeat pattern if not centered
        }
      } else if (goal_idx_ == 1 && search_phase_used_) {
        // Search phase already used; skip directly to alignment
        RCLCPP_INFO(this->get_logger(), "Search phase already used; skipping to navigation/alignment");
        goal_idx_ = 2;
      } else if (goal_idx_ == 2) {
        navigate_to_aruco_marker();
      } else {
        break;  // Reached docking position
      }
      std::this_thread::sleep_for(50ms);
    }

    if (shutdown_requested_.load()) return;

    // Phase 2: Enable docking mode and execute docking protocol
    RCLCPP_INFO(this->get_logger(), "Reached docking station, starting protocol sequence");
    
    // If cmd1 was already sent during approach, skip re-sending here.
    if (!cmd1_sent_) {
      if (!enable_docking_mode()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enable docking mode");
        return;
      }
      cmd1_sent_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Command 1 already sent during approach; skipping enable step");
    }

    std::this_thread::sleep_for(500ms);

    // Phase 3: Execute docking with retry logic and forward movement
    bool docking_success = false;
    while (docking_retry_count_ < max_docking_retries_ && !shutdown_requested_.load()) {
      RCLCPP_INFO(this->get_logger(), "Docking attempt %d/%d", 
                  docking_retry_count_ + 1, max_docking_retries_);
      
      // MODIFIED: Check pin status as early docking success indicator and send CMD11
      if (pin_status_pressed_.load()) {
        RCLCPP_INFO(this->get_logger(), "✅ Pin already pressed - attempting to run docking (cmd=11) now...");

        // Ensure docking mode enabled (cmd=1) before sending cmd=11
        if (!cmd1_sent_) {
          RCLCPP_INFO(this->get_logger(), "cmd1 not sent yet — trying to enable docking mode before cmd11");
          if (!enable_docking_mode()) {
            RCLCPP_WARN(this->get_logger(), "Failed to enable docking mode while handling pin press; will retry in loop");
            // fall through to retry loop
          } else {
            cmd1_sent_ = true;
          }
        }

        if (cmd1_sent_) {
          if (execute_docking()) {
            docking_success = true;
            break;
          } else {
            RCLCPP_WARN(this->get_logger(), "Command 11 failed even though pin pressed - will retry");
          }
        }
      }
      
      if (execute_docking()) {
        docking_success = true;
        break;
      }

      docking_retry_count_++;
      if (docking_retry_count_ < max_docking_retries_) {
        RCLCPP_WARN(this->get_logger(), "Docking failed, repositioning robot...");
        std::this_thread::sleep_for(500ms);
        
        // Move FORWARD to press switch/connector
        RCLCPP_INFO(this->get_logger(), "Moving forward to press dock connector...");
        geometry_msgs::msg::Twist forward;
        forward.linear.x = linear_velocity_ * 0.4;  // Move forward slowly
        for (int i = 0; i < 8; i++) {
          publisher_cmd_vel_->publish(forward);
          
          // MODIFIED: Check pin status during forward movement and attempt cmd11
          if (pin_status_pressed_.load()) {
            RCLCPP_INFO(this->get_logger(), 
                        "✅ Pin pressed during forward movement - attempting to run docking (cmd=11) now...");

            // Ensure cmd1 before cmd11
            if (!cmd1_sent_) {
              RCLCPP_INFO(this->get_logger(), "cmd1 not sent yet — trying to enable docking mode before cmd11");
              if (enable_docking_mode()) {
                cmd1_sent_ = true;
              } else {
                RCLCPP_WARN(this->get_logger(), "Failed to enable docking mode during forward movement; continuing");
                break;
              }
            }

            if (cmd1_sent_) {
              if (execute_docking()) {
                docking_success = true;
                break;
              } else {
                RCLCPP_WARN(this->get_logger(), "Command 11 failed when triggered by pin during forward movement");
              }
            }

            // if not successful, fall through to stop and retry repositioning
            break;
          }
          
          std::this_thread::sleep_for(150ms);
        }
        
        // Stop
        geometry_msgs::msg::Twist stop;
        publisher_cmd_vel_->publish(stop);
        std::this_thread::sleep_for(500ms);
        
        if (docking_success) break;  // Exit retry loop if pin detected and cmd11 succeeded
        
        // If dock mode got disabled, re-enable it
        RCLCPP_INFO(this->get_logger(), "Re-enabling docking mode after forward movement...");
        if (!enable_docking_mode()) {
          RCLCPP_WARN(this->get_logger(), "Failed to re-enable docking mode, sending undock first");
          
          // Send undock (0) to reset state
          std::string reason;
          call_docking_service(0, reason);
          std::this_thread::sleep_for(1s);
          
          // Try to enable again
          if (!enable_docking_mode()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to re-enable docking mode after undock");
            break;
          }
        }
        
        std::this_thread::sleep_for(1s);
      }
    }

    if (!docking_success) {
      RCLCPP_ERROR(this->get_logger(), "Docking failed after %d attempts", max_docking_retries_);
      RCLCPP_INFO(this->get_logger(), "Backing up %.2f m to clear dock before retry...", undocking_distance_);
      perform_backup(undocking_distance_);

      // After backup, do NOT restart the full search pattern — alignment only is sufficient.
      RCLCPP_INFO(this->get_logger(), "🔄 Restarting alignment after backup (no search pattern)...");
      // Important: don't call reset_docking_state() here because that clears ArUco detection/offset.
      docking_retry_count_ = 0;  // Reset retry counter for the upcoming attempts
      goal_idx_ = 2;  // Jump to navigate/align state so we use existing ArUco detection to align

      // Jump back to Phase 1 navigation loop but prefer alignment behavior (goal_idx_ == 2).
      // IMPORTANT: Do NOT restart search pattern after backup — alignment only (search_phase_used_ prevents it).
      while (!shutdown_requested_.load()) {
        if (pin_status_pressed_.load()) {
          RCLCPP_INFO(this->get_logger(), 
                      "🔌 Pin pressed detected during re-navigation → starting docking protocol");
          geometry_msgs::msg::Twist stop;
          publisher_cmd_vel_->publish(stop);
          goal_idx_ = 3;
          break;
        }

        if (goal_idx_ == 2) {
          navigate_to_aruco_marker();
        } else if (goal_idx_ == 0) {
          search_for_aruco_marker();
        } else if (goal_idx_ == 1 && !search_phase_used_) {
          // Only allow search pattern if it hasn't been used yet in this cycle
          int result = perform_search_pattern();
          if (result == 1) {
            goal_idx_ = 2;
          } else if (result == 2) {
            goal_idx_ = 1;  // Repeat pattern if not centered
          }
        } else if (goal_idx_ == 1 && search_phase_used_) {
          // Search phase already used; skip to alignment (goal_idx_ = 2)
          RCLCPP_INFO(this->get_logger(), "Search phase already used in this cycle; skipping to alignment (goal_idx_=2)");
          goal_idx_ = 2;
        } else {
          break;  // Reached docking position or other terminal state
        }

        std::this_thread::sleep_for(50ms);
      }
      
      // After re-navigation completes, retry docking protocol
      bool docking_success_retry = false;
      while (docking_retry_count_ < max_docking_retries_ && !shutdown_requested_.load()) {
        RCLCPP_INFO(this->get_logger(), "Docking retry attempt %d/%d", 
                    docking_retry_count_ + 1, max_docking_retries_);
        
        if (execute_docking()) {
          docking_success_retry = true;
          break;
        }
        
        docking_retry_count_++;
        if (docking_retry_count_ < max_docking_retries_) {
          std::this_thread::sleep_for(500ms);
          
          // Retry forward movement
          RCLCPP_INFO(this->get_logger(), "Moving forward for retry...");
          geometry_msgs::msg::Twist forward;
          forward.linear.x = linear_velocity_ * 0.4;
          for (int i = 0; i < 8; i++) {
            publisher_cmd_vel_->publish(forward);
            if (pin_status_pressed_.load()) {
              if (execute_docking()) {
                docking_success_retry = true;
                break;
              }
            }
            std::this_thread::sleep_for(150ms);
          }
          
          geometry_msgs::msg::Twist stop;
          publisher_cmd_vel_->publish(stop);
          std::this_thread::sleep_for(500ms);
          
          if (docking_success_retry) break;
        }
      }
      
      if (!docking_success_retry) {
        RCLCPP_ERROR(this->get_logger(), "Docking failed after backup retry");
        return;
      }
      
      docking_success = true;  // Mark as success to proceed to charging
    }

    RCLCPP_INFO(this->get_logger(), "✅ Docking successful!");
    geometry_msgs::msg::Twist stop_msg;
    publisher_cmd_vel_->publish(stop_msg);
    goal_idx_ = 0;

    // Phase 4: Wait for charging status (power_supply_status == 1)
    RCLCPP_INFO(this->get_logger(), "Waiting for charging to begin...");
    int charge_wait_timeout = 30;  // seconds
    int elapsed = 0;
    
    while (!shutdown_requested_.load() && elapsed < charge_wait_timeout) {
      {
        std::lock_guard<std::mutex> lk(batt_mutex_);
        if (this_battery_state_.power_supply_status == 1) {
          RCLCPP_INFO(this->get_logger(), "✅ Charging detected! (power_supply_status=1)");
          break;
        }
      }
      std::this_thread::sleep_for(500ms);
      elapsed++;
    }

    if (elapsed >= charge_wait_timeout) {
      RCLCPP_WARN(this->get_logger(), "Charging not detected within timeout");
    }

    // Phase 5: Wait for battery full (or sufficient charge)
    RCLCPP_INFO(this->get_logger(), "Waiting for battery to reach %.0f%%...", target_battery_ * 100);
    
    while (!shutdown_requested_.load()) {
      {
        std::lock_guard<std::mutex> lk(batt_mutex_);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Battery: %.1f%% (power_supply_status=%u)",
                             this_battery_state_.percentage * 100,
                             this_battery_state_.power_supply_status);
        
        // If charging stopped unexpectedly:
        if (debounce_charging_stopped(this_battery_state_.power_supply_status)) {
          RCLCPP_ERROR(this->get_logger(), "❌ Charging stopped unexpectedly (power_supply_status != 1) - initiating emergency undocking");
          // reset relevant flags
          cmd1_sent_ = false;
          pin_status_pressed_.store(false);
          docking_retry_count_ = 0;

          // Attempt proper undocking
          if (!execute_undocking()) {
            RCLCPP_WARN(this->get_logger(), "execute_undocking() failed — performing manual backup");
            perform_backup(undocking_distance_);
          }
          // Abort docking flow
          return;
        }

        if (this_battery_state_.percentage >= target_battery_) {
          RCLCPP_INFO(this->get_logger(), "🔋 Battery full! Beginning undocking...");
          break;
        }
      }
      std::this_thread::sleep_for(5s);
    }

    if (shutdown_requested_.load()) return;

    // Phase 6: Execute undocking protocol
    std::this_thread::sleep_for(500ms);
    
    if (!execute_undocking()) {
      RCLCPP_ERROR(this->get_logger(), "Undocking failed after all retry attempts");
      return;
    }

    std::this_thread::sleep_for(1s);
    
    // CRITICAL: Feed motor watchdog to prevent lockout after long docking period
    RCLCPP_INFO(this->get_logger(), "Feeding motor watchdog after docking...");
    for (int i = 0; i < 15; i++) {
      geometry_msgs::msg::Twist heartbeat;
      heartbeat.linear.x = 0.0;
      heartbeat.angular.z = 0.0;
      publisher_cmd_vel_->publish(heartbeat);
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(this->get_logger(), "✅ Motor watchdog fed successfully");
    
    RCLCPP_INFO(this->get_logger(), "✅ Undocking complete. Ready for next task.");
  }

  void reset_docking_state()
  {
    RCLCPP_INFO(this->get_logger(), "🔄 Resetting docking state variables...");
    cmd1_sent_ = false;
    pin_status_pressed_.store(false);
    docking_retry_count_ = 0;
    goal_idx_ = 0;
    search_phase_ = 0;
    search_phase_used_ = false;  // Reset search phase flag so it can be used again in next cycle
    aruco_marker_detected_.store(false);
    aruco_center_offset_.store(0);
    
    RCLCPP_DEBUG(this->get_logger(), 
                 "✅ State reset: cmd1_sent=%s | pin=%s | retry=%d | goal=%d | aruco=%s",
                 cmd1_sent_ ? "T" : "F",
                 pin_status_pressed_.load() ? "T" : "F",
                 docking_retry_count_,
                 goal_idx_,
                 aruco_marker_detected_.load() ? "T" : "F");
  }

  // ---------------- Search / Navigate Logic ----------------
  void search_for_aruco_marker()
  {
    if (!aruco_marker_detected_.load()) {
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = -angular_velocity_search_;
      publisher_cmd_vel_->publish(cmd);
    } else {
      goal_idx_ = 1;
      search_phase_ = 0;
      RCLCPP_INFO(this->get_logger(), "ArUco marker detected, starting search pattern");
    }
  }

  void navigate_to_aruco_marker()
  {
    double obs_dist;
    {
      std::lock_guard<std::mutex> lk(obs_mutex_);
      obs_dist = obstacle_distance_front_;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Front distance: %.2f m | ArUco offset: %d", obs_dist, aruco_center_offset_.load());

    // Final approach: within configured approach distance - STOP ArUco detection here
    if (obs_dist <= approach_distance_ && obs_dist > obstacle_tolerance_) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Close to dock (%.2f m) — straight approach (NO ArUco detection)", obs_dist);

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = linear_velocity_ * 0.5;
      publisher_cmd_vel_->publish(cmd);

      // Local retry counter for sending command 1 while in approach zone
      int cmd1_attempts = 0;
      const int cmd1_max_attempts = 3;

      while (!shutdown_requested_.load()) {
        {
          std::lock_guard<std::mutex> lk(obs_mutex_);
          obs_dist = obstacle_distance_front_;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "Approach loop - distance: %.2f m (target: %.2f m)", 
                             obs_dist, obstacle_tolerance_);

        // Send command 1 when close enough (configurable via cmd1_send_distance_) and not sent yet.
        if (!cmd1_sent_ && obs_dist <= cmd1_send_distance_) {
          if (cmd1_attempts < cmd1_max_attempts) {
            RCLCPP_INFO(this->get_logger(), "Attempting to enable docking mode (cmd=1) from approach loop (attempt %d/%d)...",
                        cmd1_attempts + 1, cmd1_max_attempts);
            std::string reason;
            if (call_docking_service(1, reason)) {
              cmd1_sent_ = true;
              RCLCPP_INFO(this->get_logger(), "Command 1 sent successfully from approach loop");
            } else {
              RCLCPP_WARN(this->get_logger(), "Command 1 failed from approach loop: %s", reason.c_str());
            }
            cmd1_attempts++;
            // small pause before retrying
            std::this_thread::sleep_for(300ms);
          } else {
            RCLCPP_WARN(this->get_logger(),
                        "Command 1 retries exhausted in approach loop — backing up %.2f m and restarting alignment",
                        undocking_distance_);
            perform_backup(undocking_distance_);
            // MODIFIED: Do NOT restart search pattern after backup — alignment only
            // Set goal_idx_ = 0 to force re-search detection, which will jump to alignment (goal_idx_ = 2)
            // because search_phase_used_ is already true
            goal_idx_ = 0;
            RCLCPP_INFO(this->get_logger(), "🔄 Exiting approach loop - restarting alignment only (search_phase_used_=%s)", search_phase_used_ ? "true" : "false");
            geometry_msgs::msg::Twist stop;
            publisher_cmd_vel_->publish(stop);
            return;  // Exit navigate_to_aruco_marker and return to main Phase 1 loop
          }
        }

        // If pin pressed becomes true after cmd1, consider docking ready immediately
        if (pin_status_pressed_.load() && cmd1_sent_) {
          RCLCPP_INFO(this->get_logger(), "Pin pressed detected during approach — docking ready");
          break;
        }

        if (obs_dist <= obstacle_tolerance_) {
          RCLCPP_INFO(this->get_logger(), "Obstacle reached (%.2f m) → docking ready", obs_dist);
          break;
        }

        geometry_msgs::msg::Twist forward;
        forward.linear.x = linear_velocity_ * 0.3;
        publisher_cmd_vel_->publish(forward);
        std::this_thread::sleep_for(100ms);
      }

      geometry_msgs::msg::Twist stop;
      publisher_cmd_vel_->publish(stop);
      goal_idx_ = 2;
      return;
    }

    // Beyond approach zone - CONTINUE ArUco detection
    if (aruco_marker_detected_.load() && (obs_dist > approach_distance_)) {
      adjust_heading();
    } else if (obs_dist <= obstacle_tolerance_) {
      goal_idx_ = 2;
    } else if (obs_dist > approach_distance_) {
      goal_idx_ = 0;  // Lost marker, search again
    }
  }

  void adjust_heading()
  {
    geometry_msgs::msg::Twist cmd;
    int32_t offset = aruco_center_offset_.load();

    if (offset < -center_offset_tolerance_)
      cmd.angular.z = angular_velocity_align_;
    else if (offset > center_offset_tolerance_)
      cmd.angular.z = -angular_velocity_align_;
    else
      cmd.linear.x = linear_velocity_;

    publisher_cmd_vel_->publish(cmd);
  }

  // ---------------- Member Variables ----------------
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr docking_charged_pub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_battery_state_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_aruco_detected_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_center_offset_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_laser_scan_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_pin_status_;
  rclcpp::Client<ros2_base_interfaces::srv::Dock>::SharedPtr docking_client_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  double linear_velocity_;
  double angular_velocity_search_;
  double angular_velocity_align_;
  int goal_idx_;
  double obstacle_tolerance_;
  int center_offset_tolerance_;
  int offset_search_tolerance_;
  double undocking_distance_;
  double approach_distance_;
  double low_battery_min_threshold_;
  double target_battery_;
  int docking_retry_count_;
  int max_docking_retries_;
  bool cmd1_sent_;
  double cmd1_send_distance_;
  int cmd10_max_retries_;
  int cmd0_max_retries_;
  int cmd10_cmd0_delay_sec_;
  sensor_msgs::msg::BatteryState this_battery_state_{};
  int search_phase_ = 0;
  std::mutex batt_mutex_;
  std::atomic<bool> aruco_marker_detected_;
  std::atomic<int32_t> aruco_center_offset_;
  std::atomic<bool> pin_status_pressed_;
  double obstacle_distance_front_;
  std::mutex obs_mutex_;

  std::atomic<bool> staging_navigation_in_progress_{false};
  std::atomic<bool> shutdown_requested_{false};
  std::thread staging_thread_;

  // Charging status debouncing: track consecutive readings of power_supply_status == 2
  int charging_stopped_count_{0};
  const int charging_stopped_debounce_threshold_{3};

  // Track whether search phase has been used in this docking cycle
  bool search_phase_used_{false};

  int perform_search_pattern()
  {
    static int sub_phase = 0;
    int offset = aruco_center_offset_.load();

    // Require a simple, continuous 3-sample stability check before using the offset.
    // This avoids acting on transient/mistaken values while the robot is still settling.
    if (sub_phase == 0) {
      if (!aruco_marker_detected_.load()) {
        return 0; // still searching
      }

      int prev = offset;
      int stable_count = 1;
      // Take two additional samples 100ms apart (total 3 samples)
      for (int i = 0; i < 2 && !shutdown_requested_.load(); ++i) {
        std::this_thread::sleep_for(100ms);
        int v = aruco_center_offset_.load();
        if (v == prev) {
          stable_count++;
        } else {
          prev = v;
          stable_count = 1;
        }
      }
      offset = prev;

      if (stable_count < 3) {
        RCLCPP_DEBUG(this->get_logger(), "ArUco offset not stable yet: %d (stable=%d) - continuing search", offset, stable_count);
        return 0; // keep searching/rotating
      }

      // Offset is stable — if it's already within tolerance, we're done
      if (std::abs(offset) <= offset_search_tolerance_) {
        sub_phase = 0;
        RCLCPP_INFO(this->get_logger(), "✅ Search pattern complete: final offset = %d pixels (within tolerance %d)", offset, offset_search_tolerance_);
        search_phase_used_ = true;  // Mark search phase as used
        return 1; // done
      }
      // else proceed into the search pattern using the stable offset value
    }
    bool marker_on_right = (offset < 0);  // Negative offset means marker on right side
    RCLCPP_INFO(this->get_logger(), "Search pattern: sub_phase=%d, offset=%d, marker_on_right=%s", sub_phase, offset, marker_on_right ? "true" : "false");
    if (sub_phase == 0) {
      // turn away 90 deg
      double turn_angle = M_PI / 2.0;
      RCLCPP_INFO(this->get_logger(), "Turning away 90 deg: %s", marker_on_right ? "right" : "left");
      if (marker_on_right) {
        turn_by_angle(turn_angle); 
      } else {
        turn_by_angle(-turn_angle); 
      }
      sub_phase = 1;
    } else if (sub_phase == 1) {
      // move straight - distance using exponential scaling (large offset -> max distance, small offset -> small distance)
      const double max_straight_distance = 0.15;  // meters
      // Controls curve steepness: tuned so
      // offset ~50 -> ~0.06m, 100 -> ~0.10m, 200 -> ~0.13m (max cap still 0.15m)
      const double exp_factor = 0.006;  // tuned factor
      double straight_distance = max_straight_distance * (1.0 - std::exp(-exp_factor * std::abs(offset)));
      RCLCPP_INFO(this->get_logger(), "Moving straight %.3f m (offset=%d, exponential-scaled)", straight_distance, offset);
      move_straight(straight_distance);
      sub_phase = 2;
    } else if (sub_phase == 2) {
      // turn back
      double turn_angle = M_PI / 2.0;
      RCLCPP_INFO(this->get_logger(), "Turning back 90 deg: %s", marker_on_right ? "left" : "right");
      if (marker_on_right) {
        turn_by_angle(turn_angle); 
      } else {
        turn_by_angle(-turn_angle); 
      }
      // Add delay to let offset update
      std::this_thread::sleep_for(3s);
      int new_offset = aruco_center_offset_.load();
      if (std::abs(new_offset) <= center_offset_tolerance_) {
        sub_phase = 0;
        RCLCPP_INFO(this->get_logger(), "✅ Search pattern complete: final offset = %d pixels (within tolerance %d)", new_offset, center_offset_tolerance_);
        search_phase_used_ = true;  // Mark search phase as used
        return 1; // done
      } else {
        // Not centered, repeat the pattern
        sub_phase = 0;
        return 2; // repeat
      }
    }
    return 0; // continue
  }

  void turn_by_angle(double angle_rad)
  {
    if (angle_rad == 0) return;
    double duration = std::abs(angle_rad) / angular_velocity_search_;  // Use faster search velocity
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = (angle_rad > 0) ? angular_velocity_search_ : -angular_velocity_search_;
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::duration<double>(duration) && !shutdown_requested_.load()) {
      publisher_cmd_vel_->publish(cmd);
      std::this_thread::sleep_for(50ms);
    }
    cmd.angular.z = 0;
    publisher_cmd_vel_->publish(cmd);
  }

  void move_straight(double distance)
  {
    if (distance <= 0) return;
    double duration = distance / linear_velocity_;
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_velocity_;
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::duration<double>(duration) && !shutdown_requested_.load()) {
      publisher_cmd_vel_->publish(cmd);
      std::this_thread::sleep_for(50ms);
    }
    cmd.linear.x = 0;
    publisher_cmd_vel_->publish(cmd);
  }

  void perform_backup(double distance_m)
  {
    if (distance_m <= 0.0 || linear_velocity_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "Invalid backup parameters (distance=%.2f, vel=%.2f). Skipping.", distance_m, linear_velocity_);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Backing up %.2f m...", distance_m);
    double duration_s = distance_m / linear_velocity_;
    auto start = std::chrono::steady_clock::now();
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = -linear_velocity_;

    while (std::chrono::steady_clock::now() - start < std::chrono::duration<double>(duration_s)
           && !shutdown_requested_.load()) {
      publisher_cmd_vel_->publish(cmd);
      std::this_thread::sleep_for(100ms);
    }
    geometry_msgs::msg::Twist stop;
    publisher_cmd_vel_->publish(stop);
    RCLCPP_INFO(this->get_logger(), "Backup finished (%.2f m)", distance_m);
    
    // CRITICAL: Send heartbeat velocity commands to prevent motor watchdog timeout
    // Many motor controllers disable motors if no cmd_vel is received for >1 second
    RCLCPP_INFO(this->get_logger(), "Sending motor watchdog heartbeat...");
    for (int i = 0; i < 10; i++) {
      geometry_msgs::msg::Twist zero_cmd;
      zero_cmd.linear.x = 0.0;
      zero_cmd.angular.z = 0.0;
      publisher_cmd_vel_->publish(zero_cmd);
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(this->get_logger(), "✅ Motor watchdog heartbeat sent");
  }

  bool debounce_charging_stopped(uint8_t power_supply_status)
  {
    if (power_supply_status != 1) {
      charging_stopped_count_++;
      if (charging_stopped_count_ >= charging_stopped_debounce_threshold_) {
        RCLCPP_WARN(this->get_logger(), 
                    "⚠️  Charging stopped detected after %d consecutive readings (power_supply_status=%u)",
                    charging_stopped_count_, power_supply_status);
        charging_stopped_count_ = 0;  // Reset for next cycle
        return true;  // Charging stopped confirmed
      }
    } else {
      // Back to charging (power_supply_status == 1) - reset counter
      if (charging_stopped_count_ > 0) {
        // RCLCPP_DEBUG(this->get_logger(), "Charging resumed - resetting debounce counter");
        charging_stopped_count_ = 0;
      }
    }
    return false;  // Not enough consecutive readings yet
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConnectToChargingDockNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
