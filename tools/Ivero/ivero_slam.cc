
#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <signal.h>
#include <sstream>
#include <stdlib.h>

#include <condition_variable>

#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>
#include <yaml-cpp/yaml.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <System.h>

#include "CallbackHarness.h"
#include "utils.h"

using namespace std;

bool b_continue_session;

void exit_loop_handler(int s) {
  cout << "Finishing session" << endl;
  b_continue_session = false;
}

int main(int argc, char** argv) {
  // sigint handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_loop_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
  b_continue_session = true;

  // look for realsense devices
  rs2::context ctx;
  rs2::device_list devices = ctx.query_devices();
  assert(devices.size() > 0 && "No device connected, please connect a RealSense device.");
  rs2::device selected_device = devices[0];

  // assert camera type
  auto camera_name = std::string(selected_device.get_info(RS2_CAMERA_INFO_NAME));
  assert(camera_name == "Intel RealSense D455" && "Device must be an Intel RealSense D455.");

  // create output folders
  auto date = getDateString();
  auto output_path = setupOutputFolders(date);
  std::string traj_file_name = output_path + "/trajectory.txt";

  // set sensor options
  setSensorOptions(selected_device.query_sensors());

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;

  // Create a configuration for configuring the pipeline with a non default
  // profile
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
  cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
  cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
  cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_ANY, 30);
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

  // setup callback variables
  CallbackHarness::output_path = output_path;
  CallbackHarness::prev_accel_timestamp = 0;
  CallbackHarness::current_accel_timestamp = 0;
  CallbackHarness::timestamp_image = -1.0;
  CallbackHarness::image_ready = false;
  CallbackHarness::count_im_buffer = 0;
  CallbackHarness::prev_depthkf_timestamp = 0;
  CallbackHarness::offset = 0;

  rs2::pipeline_profile pipe_profile = pipe.start(cfg, CallbackHarness::Callback);

  vector<ORB_SLAM3::IMU::Point> vImuMeas;
  rs2::stream_profile cam_left = pipe_profile.get_stream(RS2_STREAM_INFRARED, 1);
  rs2::stream_profile cam_right = pipe_profile.get_stream(RS2_STREAM_INFRARED, 2);
  rs2::stream_profile imu_stream = pipe_profile.get_stream(RS2_STREAM_GYRO);

  // get extrinsic from left camera to color camera
  Eigen::Matrix4f T_rgb_camleft = getExtrinsics(pipe_profile.get_stream(RS2_STREAM_COLOR), cam_left);

  // get config file, create if it doesnt exist
  auto serial_number = std::string(selected_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
  std::string config_file = getConfigPath(serial_number);
  if (!boost::filesystem::exists(config_file)) { createConfigFile(cam_left, cam_right, imu_stream, config_file); }

  rs2_intrinsics intrinsics_left = cam_left.as<rs2::video_stream_profile>().get_intrinsics();
  CallbackHarness::width_img = intrinsics_left.width;
  CallbackHarness::height_img = intrinsics_left.height;

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(get_vocab_path(), config_file, ORB_SLAM3::System::IMU_STEREO, true, 0, traj_file_name);
  float imageScale = SLAM.GetImageScale();

  double timestamp;
  cv::Mat im, imRight;

  // Clear IMU vectors
  CallbackHarness::v_gyro_data.clear();
  CallbackHarness::v_gyro_timestamp.clear();
  CallbackHarness::v_accel_data_sync.clear();
  CallbackHarness::v_accel_timestamp_sync.clear();

  double t_resize = 0.f;
  double t_track = 0.f;

  while (!SLAM.isShutDown() && b_continue_session) {
    std::vector<rs2_vector> vGyro;
    std::vector<double> vGyro_times;
    std::vector<rs2_vector> vAccel;
    std::vector<double> vAccel_times;

    {
      std::unique_lock<std::mutex> lk(CallbackHarness::imu_mutex);
      if (!CallbackHarness::image_ready) CallbackHarness::cond_image_rec.wait(lk);

      if (CallbackHarness::count_im_buffer > 1) cout << CallbackHarness::count_im_buffer - 1 << " dropped frs\n";
      CallbackHarness::count_im_buffer = 0;

      while (CallbackHarness::v_gyro_timestamp.size() > CallbackHarness::v_accel_timestamp_sync.size()) {
        int index = CallbackHarness::v_accel_timestamp_sync.size();
        double target_time = CallbackHarness::v_gyro_timestamp[index];

        rs2_vector interp_data = interpolate_measure(
            target_time, CallbackHarness::current_accel_data, CallbackHarness::current_accel_timestamp,
            CallbackHarness::prev_accel_data, CallbackHarness::prev_accel_timestamp);

        CallbackHarness::v_accel_data_sync.push_back(interp_data);
        CallbackHarness::v_accel_timestamp_sync.push_back(target_time);
      }

      // Copy the data
      vGyro = CallbackHarness::v_gyro_data;
      vGyro_times = CallbackHarness::v_gyro_timestamp;
      vAccel = CallbackHarness::v_accel_data_sync;
      vAccel_times = CallbackHarness::v_accel_timestamp_sync;
      timestamp = CallbackHarness::timestamp_image;
      im = CallbackHarness::imCV.clone();
      imRight = CallbackHarness::imRightCV.clone();

      // todo: stream im + mappoints with nhve

      // Clear IMU vectors
      CallbackHarness::v_gyro_data.clear();
      CallbackHarness::v_gyro_timestamp.clear();
      CallbackHarness::v_accel_data_sync.clear();
      CallbackHarness::v_accel_timestamp_sync.clear();
      CallbackHarness::image_ready = false;
    }

    for (int i = 0; i < vGyro.size(); ++i) {
      ORB_SLAM3::IMU::Point lastPoint(vAccel[i].x, vAccel[i].y, vAccel[i].z, vGyro[i].x, vGyro[i].y, vGyro[i].z,
                                      vGyro_times[i]);
      vImuMeas.push_back(lastPoint);
    }

    if (imageScale != 1.f) {
      int width = im.cols * imageScale;
      int height = im.rows * imageScale;
      cv::resize(im, im, cv::Size(width, height));
      cv::resize(imRight, imRight, cv::Size(width, height));
    }

    // Stereo images are already rectified.
    SLAM.TrackStereo(im, imRight, timestamp, vImuMeas);

    // Clear the previous IMU measurements to load the new ones
    vImuMeas.clear();
  }

  SLAM.Shutdown();
  SLAM.SaveTrajectoryTUMTransformed(traj_file_name, T_rgb_camleft);
}