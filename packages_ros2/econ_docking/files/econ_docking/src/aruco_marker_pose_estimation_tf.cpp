#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using std::placeholders::_1;

class ArucoNode : public rclcpp::Node
{
public:
  ArucoNode() : Node("aruco_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("aruco_dictionary_name", "DICT_APRILTAG_36h11");
    this->declare_parameter<double>("aruco_marker_side_length", 0.09);
    this->declare_parameter<std::string>(
      "camera_calibration_parameters_filename",
      "/home/tsm02/Pictures/aruco_marker/econ_docking/src/econ_docking/config/calibration_chessboard.yaml");
    this->declare_parameter<std::string>("image_topic", "/rgb/image_raw");
    this->declare_parameter<std::string>("aruco_marker_name", "aruco_marker");
    // control whether to show GUI view (default: false -> no GUI, publish-only mode)
    this->declare_parameter<bool>("show_image", false);

    // Read parameters
    std::string aruco_dictionary_name =
      this->get_parameter("aruco_dictionary_name").as_string();
    aruco_marker_side_length_ =
      this->get_parameter("aruco_marker_side_length").as_double();
    camera_calibration_parameters_filename_ =
      this->get_parameter("camera_calibration_parameters_filename").as_string();
    std::string image_topic =
      this->get_parameter("image_topic").as_string();
    aruco_marker_name_ =
      this->get_parameter("aruco_marker_name").as_string();
    show_image_ = this->get_parameter("show_image").as_bool();

    // Map of dictionary names to OpenCV constants
    aruco_dict_map_ = {
      {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
      {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
      {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
      {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
      {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
      {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
      {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
      {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
      {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
      {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
      {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
      {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
      {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
      {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
      {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
      {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
      {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
      {"DICT_APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5},
      {"DICT_APRILTAG_25h9", cv::aruco::DICT_APRILTAG_25h9},
      {"DICT_APRILTAG_36h10", cv::aruco::DICT_APRILTAG_36h10},
      {"DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11}
    };

    // Check valid dictionary
    if (aruco_dict_map_.find(aruco_dictionary_name) == aruco_dict_map_.end()) {
      RCLCPP_ERROR(this->get_logger(),
        "[INFO] ArUCo tag of '%s' is not supported", aruco_dictionary_name.c_str());
      return;
    }

    // Load camera calibration
    cv::FileStorage fs(camera_calibration_parameters_filename_, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open camera calibration file: %s",
        camera_calibration_parameters_filename_.c_str());
      return;
    }
    fs["K"] >> camera_matrix_;
    fs["D"] >> dist_coeffs_;
    fs.release();

    // Load dictionary
    RCLCPP_INFO(this->get_logger(), "[INFO] detecting '%s' markers...",
      aruco_dictionary_name.c_str());
    this_aruco_dictionary_ = cv::aruco::getPredefinedDictionary(
      aruco_dict_map_[aruco_dictionary_name]);
    this_aruco_parameters_ = cv::aruco::DetectorParameters::create();

    // Initialize transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // create window only if user requested GUI
    if (show_image_) {
      cv::namedWindow("camera", cv::WINDOW_AUTOSIZE);
    }

    // Subscriber for camera images
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&ArucoNode::imageCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "ArucoNode initialized successfully.");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // convert image (use literal encoding to avoid extra headers)
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat current_frame = cv_ptr->image; // will be used only if show_image_ true or for detection

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;

    cv::aruco::detectMarkers(
      current_frame, this_aruco_dictionary_, marker_corners, marker_ids,
      this_aruco_parameters_, rejected_candidates, camera_matrix_, dist_coeffs_);

    if (!marker_ids.empty()) {
      // draw for debugging only when GUI is enabled
      if (show_image_) {
        cv::aruco::drawDetectedMarkers(current_frame, marker_corners, marker_ids);
      }

      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(
        marker_corners, aruco_marker_side_length_, camera_matrix_, dist_coeffs_,
        rvecs, tvecs);

      for (size_t i = 0; i < marker_ids.size(); ++i) {
        geometry_msgs::msg::TransformStamped t;
        // Use the camera image timestamp for the transform header
        t.header.stamp = msg->header.stamp;
        t.header.frame_id = "camera_depth_frame";
        t.child_frame_id = aruco_marker_name_;

        // Translation
        t.transform.translation.x = tvecs[i][0];
        t.transform.translation.y = tvecs[i][1];
        t.transform.translation.z = tvecs[i][2];

        // Rotation to quaternion
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvecs[i], rotation_matrix);

        // Copy cv::Mat -> tf2::Matrix3x3 and get quaternion (cv::Rodrigues yields CV_64F)
        tf2::Matrix3x3 mat(
          rotation_matrix.at<double>(0,0), rotation_matrix.at<double>(0,1), rotation_matrix.at<double>(0,2),
          rotation_matrix.at<double>(1,0), rotation_matrix.at<double>(1,1), rotation_matrix.at<double>(1,2),
          rotation_matrix.at<double>(2,0), rotation_matrix.at<double>(2,1), rotation_matrix.at<double>(2,2)
        );
        tf2::Quaternion q;
        mat.getRotation(q);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Broadcast transform
        tf_broadcaster_->sendTransform(t);

        // Draw axis only when GUI enabled
        if (show_image_) {
          cv::aruco::drawAxis(current_frame, camera_matrix_, dist_coeffs_,
                              rvecs[i], tvecs[i], 0.05);
        }
      }
    }

    // show image only when requested (avoids requiring X server in headless environments)
    if (show_image_) {
      cv::imshow("camera", current_frame);
      cv::waitKey(1);
    }
  }

  // Parameters and members
  double aruco_marker_side_length_;
  std::string camera_calibration_parameters_filename_;
  std::string aruco_marker_name_;
  bool show_image_;

  std::map<std::string, int> aruco_dict_map_;
  cv::Ptr<cv::aruco::Dictionary> this_aruco_dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> this_aruco_parameters_;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
