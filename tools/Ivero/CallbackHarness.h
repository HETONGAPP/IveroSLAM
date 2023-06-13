#pragma once

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <deque>
#include <optional>
#include <stdlib.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/core/core.hpp>

/// @brief Struct representing a single RGBD image
struct RGBDImage {
  cv::Mat rgb;
  cv::Mat depth;
  double timestamp;
};

class CallbackHarness {
public:
  /// @brief Gets the RGB and aligned Depth image from a frameset
  /// @param frameset to retrieve RGBD from
  /// @return <RGB, Depth>
  static std::optional<RGBDImage> GetRGBD(const rs2::frameset& frameset);

  /// @brief Main callback for realsense device it performs multiple functions:
  /// 1. Determine type of frame being read and will update the current measurements for SLAM
  /// 2. Process RGBD data and add to buffer
  /// @param frame to process
  static void Callback(const rs2::frame& frame);

  static std::string output_path;
  /// @brief
  static std::mutex imu_mutex;
  /// @brief
  static std::condition_variable cond_image_rec;
  /// @brief
  static std::vector<double> v_accel_timestamp;
  /// @brief
  static std::vector<rs2_vector> v_accel_data;
  /// @brief
  static std::vector<double> v_gyro_timestamp;
  /// @brief
  static std::vector<rs2_vector> v_gyro_data;
  /// @brief
  static double prev_accel_timestamp;
  /// @brief
  static rs2_vector prev_accel_data;
  /// @brief
  static double current_accel_timestamp;
  /// @brief
  static rs2_vector current_accel_data;
  /// @brief
  static std::vector<double> v_accel_timestamp_sync;
  /// @brief
  static std::vector<rs2_vector> v_accel_data_sync;
  /// @brief
  static std::deque<RGBDImage> rgbd_buffer;
  /// @brief
  static cv::Mat imCV, imRightCV;
  /// @brief
  static int width_img, height_img;
  /// @brief
  static double timestamp_image;
  /// @brief
  static bool image_ready;
  /// @brief
  static int count_im_buffer; // count dropped frames
  /// @brief
  static double prev_depthkf_timestamp;
  /// @brief
  static double offset;

private:
  // Disallow creating an instance of this object
  CallbackHarness() {}
};

#include "CallbackHarnessImp.hpp"