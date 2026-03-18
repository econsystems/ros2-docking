#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <chrono>

// ROS 2 headers
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "cv_bridge/cv_bridge.h"

// OpenCV (no GUI headers needed)
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using std::placeholders::_1;

/**
 * ArUco dictionary lookup table
 */
static const std::unordered_map<std::string, int> ARUCO_DICT = {
  {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
  {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
  {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
  {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
  {"DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11}
};

class ArucoNode : public rclcpp::Node
{
public:
  ArucoNode()
  : Node("aruco_node"),
    offset_aruco_marker_(0),
    target_marker_id_(-1),
    running_(true),
    last_detected_(false),
    frame_count_(0),
    process_every_nth_frame_(2),
    smoothing_factor_(0.6)
  {
    // Parameters
    this->declare_parameter<std::string>("aruco_dictionary_name", "DICT_APRILTAG_36h11");
    this->declare_parameter<double>("aruco_marker_side_length", 0.09);
    this->declare_parameter<std::string>(
      "camera_calibration_parameters_filename",
      "/sdcard/mmcblk2p1/soundarya/ros2_humble/usr/share/econ_docking/config/calibration_chessboard.yaml"); 
    this->declare_parameter<std::string>("image_topic", "rgb/image_raw");
    this->declare_parameter<std::string>("aruco_marker_name", "aruco_marker");
    this->declare_parameter<int>("target_marker_id", -1);
    
    // New: CPU optimization parameters
    this->declare_parameter<int>("process_every_nth_frame", 2);  // 1=every frame, 2=every 2nd, etc
    this->declare_parameter<double>("smoothing_factor", 0.6);     // temporal smoothing for skipped frames

    // Read parameters
    std::string aruco_dictionary_name = this->get_parameter("aruco_dictionary_name").as_string();
    aruco_marker_side_length_ = this->get_parameter("aruco_marker_side_length").as_double();
    camera_calibration_parameters_filename_ =
      this->get_parameter("camera_calibration_parameters_filename").as_string();
    std::string image_topic = this->get_parameter("image_topic").as_string();
    aruco_marker_name_ = this->get_parameter("aruco_marker_name").as_string();
    target_marker_id_ = this->get_parameter("target_marker_id").as_int();
    process_every_nth_frame_ = this->get_parameter("process_every_nth_frame").as_int();
    smoothing_factor_ = this->get_parameter("smoothing_factor").as_double();
    
    if (process_every_nth_frame_ < 1) process_every_nth_frame_ = 1;
    if (smoothing_factor_ < 0.0 || smoothing_factor_ > 1.0) smoothing_factor_ = 0.6;

    // Validate dictionary
    if (ARUCO_DICT.find(aruco_dictionary_name) == ARUCO_DICT.end()) {
      RCLCPP_WARN(this->get_logger(), "Unsupported ArUco dictionary: %s", aruco_dictionary_name.c_str());
      aruco_dictionary_name = "DICT_APRILTAG_36h11";
    }

    // Load calibration
    cv::FileStorage fs(camera_calibration_parameters_filename_, cv::FileStorage::READ);
    if (fs.isOpened()) {
      fs["K"] >> camera_matrix_;
      fs["D"] >> dist_coeffs_;
      fs.release();
    } else {
      RCLCPP_WARN(this->get_logger(), "Camera calibration file not found: %s",
                  camera_calibration_parameters_filename_.c_str());
    }

    // Initialize dictionary and parameters
    this_aruco_dictionary_ = cv::aruco::getPredefinedDictionary(ARUCO_DICT.at(aruco_dictionary_name));
    this_aruco_parameters_ = cv::aruco::DetectorParameters::create();

    // CPU optimization: disable all refinements
    this_aruco_parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    
    // Faster edge detection for candidate filtering
    this_aruco_parameters_->minMarkerPerimeterRate = 0.03;
    this_aruco_parameters_->maxMarkerPerimeterRate = 4.0;

    // ROS2 interfaces - BestEffort + KeepLast(1) to minimize latency
    rclcpp::QoS qos_profile(5);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic, qos_profile, std::bind(&ArucoNode::image_callback, this, _1));

    rclcpp::QoS pub_qos(5);
    pub_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    publisher_aruco_marker_detected_ =
      this->create_publisher<std_msgs::msg::Bool>("aruco_marker_detected", pub_qos);
    publisher_offset_aruco_marker_ =
      this->create_publisher<std_msgs::msg::Int32>("aruco_marker_offset", pub_qos);

    RCLCPP_INFO(this->get_logger(), "ArUco detector: process_every_nth=%d, smoothing=%.2f, target_id=%d",
                process_every_nth_frame_, smoothing_factor_, target_marker_id_);

    // Start worker thread
    worker_thread_ = std::thread(&ArucoNode::detection_loop, this);
  }

  ~ArucoNode()
  {
    running_.store(false);
    cv_signal_.notify_all();
    if (worker_thread_.joinable()) worker_thread_.join();
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lk(latest_mutex_);
      latest_image_ = msg;
      latest_timestamp_ = std::chrono::high_resolution_clock::now();
    }
    cv_signal_.notify_one();
  }

  void detection_loop()
  {
    using namespace std::chrono;

    while (running_.load()) {
      sensor_msgs::msg::Image::SharedPtr local_msg;

      // Wait for a new frame
      {
        std::unique_lock<std::mutex> lk(wait_mutex_);
        cv_signal_.wait(lk, [&] {
          std::lock_guard<std::mutex> lk2(latest_mutex_);
          return !running_.load() || latest_image_ != nullptr;
        });
        if (!running_.load()) break;

        {
          std::lock_guard<std::mutex> lk2(latest_mutex_);
          local_msg = latest_image_;
          latest_image_.reset();
        }
      }

      if (!local_msg) continue;

      frame_count_++;
      bool should_detect = (frame_count_ % process_every_nth_frame_) == 0;

      // Convert to cv::Mat
      cv_bridge::CvImageConstPtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvShare(local_msg, "bgr8");
      } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        continue;
      }
      const cv::Mat &frame = cv_ptr->image;

      if (should_detect) {
        // Run full detection on grayscale (cheaper than BGR for detectMarkers)
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        member_marker_ids_.clear();
        member_corners_.clear();
        member_rejected_.clear();

        cv::aruco::detectMarkers(gray, this_aruco_dictionary_, member_corners_, 
                                 member_marker_ids_, this_aruco_parameters_, member_rejected_);

        if (!member_marker_ids_.empty()) {
          // Find requested id or use first
          int matched_index = -1;
          if (target_marker_id_ < 0) {
            matched_index = 0;
          } else {
            for (size_t i = 0; i < member_marker_ids_.size(); ++i) {
              if (member_marker_ids_[i] == target_marker_id_) {
                matched_index = static_cast<int>(i);
                break;
              }
            }
          }

          if (matched_index >= 0) {
            const auto &pts = member_corners_[matched_index];
            double cX = (pts[0].x + pts[1].x + pts[2].x + pts[3].x) * 0.25;
            int image_center = frame.cols / 2;
            
            // Update smoothed offset
            int new_offset = static_cast<int>(std::round(cX) - image_center);
            offset_aruco_marker_ = static_cast<int>(
              smoothing_factor_ * new_offset + (1.0 - smoothing_factor_) * offset_aruco_marker_
            );

            // Publish immediately
            std_msgs::msg::Bool detected_msg;
            detected_msg.data = true;
            std_msgs::msg::Int32 offset_msg;
            offset_msg.data = offset_aruco_marker_;

            publisher_aruco_marker_detected_->publish(detected_msg);
            publisher_offset_aruco_marker_->publish(offset_msg);

            last_detected_ = true;
            continue; // Go fetch next frame ASAP
          }
        }

        // No marker found in detection frame
        if (last_detected_) {
          std_msgs::msg::Bool detected_msg;
          detected_msg.data = false;
          publisher_aruco_marker_detected_->publish(detected_msg);
          last_detected_ = false;
        }
      } else {
        // Skipped frame: publish last known state if marker was detected
        if (last_detected_) {
          std_msgs::msg::Bool detected_msg;
          detected_msg.data = true;
          std_msgs::msg::Int32 offset_msg;
          offset_msg.data = offset_aruco_marker_;

          publisher_aruco_marker_detected_->publish(detected_msg);
          publisher_offset_aruco_marker_->publish(offset_msg);
        }
      }
    }
  }

  // Variables
  double aruco_marker_side_length_;
  std::string camera_calibration_parameters_filename_;
  std::string aruco_marker_name_;
  int offset_aruco_marker_;
  int target_marker_id_;

  // Camera calibration
  cv::Mat camera_matrix_, dist_coeffs_;
  cv::Ptr<cv::aruco::Dictionary> this_aruco_dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> this_aruco_parameters_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_aruco_marker_detected_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_offset_aruco_marker_;

  // Worker thread / frame buffer
  std::thread worker_thread_;
  std::atomic<bool> running_;
  std::mutex latest_mutex_;
  std::mutex wait_mutex_;
  std::condition_variable cv_signal_;
  sensor_msgs::msg::Image::SharedPtr latest_image_;
  std::chrono::high_resolution_clock::time_point latest_timestamp_;

  // State + optimization
  bool last_detected_;
  int frame_count_;
  int process_every_nth_frame_;
  double smoothing_factor_;
  std::vector<int> member_marker_ids_;
  std::vector<std::vector<cv::Point2f>> member_corners_;
  std::vector<std::vector<cv::Point2f>> member_rejected_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
