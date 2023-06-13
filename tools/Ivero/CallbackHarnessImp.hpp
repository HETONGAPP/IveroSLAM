#include "CallbackHarness.h"
#include "utils.h"

#include <opencv2/opencv.hpp>

std::string CallbackHarness::output_path;
std::mutex CallbackHarness::imu_mutex;
std::condition_variable CallbackHarness::cond_image_rec;
std::vector<double> CallbackHarness::v_accel_timestamp;
std::vector<rs2_vector> CallbackHarness::v_accel_data;
std::vector<double> CallbackHarness::v_gyro_timestamp;
std::vector<rs2_vector> CallbackHarness::v_gyro_data;
double CallbackHarness::prev_accel_timestamp;
rs2_vector CallbackHarness::prev_accel_data;
double CallbackHarness::current_accel_timestamp;
rs2_vector CallbackHarness::current_accel_data;
std::vector<double> CallbackHarness::v_accel_timestamp_sync;
std::vector<rs2_vector> CallbackHarness::v_accel_data_sync;
std::deque<RGBDImage> CallbackHarness::rgbd_buffer;
cv::Mat CallbackHarness::imCV, CallbackHarness::imRightCV;
int CallbackHarness::width_img, CallbackHarness::height_img;
double CallbackHarness::timestamp_image;
bool CallbackHarness::image_ready;
int CallbackHarness::count_im_buffer; 
double CallbackHarness::prev_depthkf_timestamp;
double CallbackHarness::offset;

std::optional<RGBDImage> CallbackHarness::GetRGBD(const rs2::frameset& frameset) {
  rs2::align align(RS2_STREAM_COLOR);
  // align frameset to color stream
  auto aligned_fs = align.process(frameset);
  auto timestamp = frameset.get_timestamp() * 1e-3;

  rs2::video_frame rgb_frame = frameset.get_color_frame();
  rs2::depth_frame depth_frame = aligned_fs.get_depth_frame();

  if (rgb_frame.get_data_size() <= 0 || depth_frame.get_data_size() <= 0) {
    std::cout << std::fixed << "No data in rgbd stream: " << timestamp << std::endl;
    return {};
  }

  RGBDImage rgbd_image;
  rgbd_image.timestamp = timestamp;

  cv::Mat depth_raw =
      cv::Mat(depth_frame.get_height(), depth_frame.get_width(), CV_16SC1, const_cast<void*>(depth_frame.get_data()));
  cv::Mat depth_metric_mm;
  depth_raw.convertTo(depth_metric_mm, CV_32FC1);
  depth_metric_mm *= depth_frame.get_units() * 1000.0;
  cv::Mat depth_out;
  depth_metric_mm.convertTo(depth_out, CV_16UC1);
  rgbd_image.depth = depth_out;

  cv::Mat rgb = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void*)(rgb_frame.get_data()), cv::Mat::AUTO_STEP);
  rgbd_image.rgb = rgb;

  return rgbd_image;
}

void CallbackHarness::Callback(const rs2::frame& frame) {
  std::unique_lock<std::mutex> lock(imu_mutex);

  if (rs2::frameset fs = frame.as<rs2::frameset>()) {
    assert(fs.supports_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_FRAME_EMITTER_MODE) &&
           "Metadata not supported, cannot perform laser emitter switching! Try reapplying patch to kernel for "
           "realsense SDK.");

    // metadata is delayed for this? i.e. if it says its off its bc the previous frame was off?
    auto emitter_on = fs.get_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_FRAME_EMITTER_MODE);
    if (emitter_on == 0) {
      auto cur_timestamp = fs.get_timestamp() / 1000.0;
      // output at 15hz
      if (cur_timestamp - prev_depthkf_timestamp < 0.067) { return; }
      prev_depthkf_timestamp = cur_timestamp;

      auto rgbd_image = GetRGBD(fs);

      if (!rgbd_image.has_value()) { return; }

      const auto filename = std::to_string(rgbd_image.value().timestamp) + ".png";
      if (output_path.empty()) { return; }
      cv::imwrite(output_path + "/depth/" + filename, rgbd_image.value().depth);
      cv::imwrite(output_path + "/rgb/" + filename, rgbd_image.value().rgb);

      return;
    }

    count_im_buffer++;

    double new_timestamp_image = fs.get_timestamp() * 1e-3;
    if (abs(timestamp_image - new_timestamp_image) < 0.001) {
      // cout << "Two frames with the same timeStamp!!!\n";
      count_im_buffer--;
      return;
    }

    rs2::video_frame ir_frameL = fs.get_infrared_frame(1);
    rs2::video_frame ir_frameR = fs.get_infrared_frame(2);

    imCV = cv::Mat(cv::Size(width_img, height_img), CV_8U, (void*)(ir_frameL.get_data()), cv::Mat::AUTO_STEP);
    imRightCV = cv::Mat(cv::Size(width_img, height_img), CV_8U, (void*)(ir_frameR.get_data()), cv::Mat::AUTO_STEP);

    timestamp_image = fs.get_timestamp() * 1e-3;
    image_ready = true;

    while (v_gyro_timestamp.size() > v_accel_timestamp_sync.size()) {
      int index = v_accel_timestamp_sync.size();
      double target_time = v_gyro_timestamp[index];

      v_accel_data_sync.push_back(current_accel_data);
      v_accel_timestamp_sync.push_back(target_time);
    }

    lock.unlock();
    cond_image_rec.notify_all();
  } else if (rs2::motion_frame m_frame = frame.as<rs2::motion_frame>()) {
    if (m_frame.get_profile().stream_name() == "Gyro") {
      // It runs at 200Hz
      v_gyro_data.push_back(m_frame.get_motion_data());
      v_gyro_timestamp.push_back((m_frame.get_timestamp() + offset) * 1e-3);
    } else if (m_frame.get_profile().stream_name() == "Accel") {
      // It runs at 60Hz
      prev_accel_timestamp = current_accel_timestamp;
      prev_accel_data = current_accel_data;

      current_accel_data = m_frame.get_motion_data();
      current_accel_timestamp = (m_frame.get_timestamp() + offset) * 1e-3;

      while (v_gyro_timestamp.size() > v_accel_timestamp_sync.size()) {
        int index = v_accel_timestamp_sync.size();
        double target_time = v_gyro_timestamp[index];

        rs2_vector interp_data = interpolate_measure(target_time, current_accel_data, current_accel_timestamp,
                                                     prev_accel_data, prev_accel_timestamp);

        v_accel_data_sync.push_back(interp_data);
        v_accel_timestamp_sync.push_back(target_time);
      }
    }
  }
}