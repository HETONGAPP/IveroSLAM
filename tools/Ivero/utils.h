#pragma once

#include <Eigen/Core>
#include <boost/filesystem.hpp>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <sstream>
#include <thread>
#include <yaml-cpp/yaml.h>

const std::string slam_config_folder = std::string(getenv("HOME")) + "/slam_configs/";

rs2_vector interpolate_measure(const double target_time, const rs2_vector current_data, const double current_time,
                               const rs2_vector prev_data, const double prev_time) {
  // If there are not previous information, the current data is propagated
  if (prev_time == 0) { return current_data; }

  rs2_vector increment;
  rs2_vector value_interp;

  if (target_time > current_time) {
    value_interp = current_data;
  } else if (target_time > prev_time) {
    increment.x = current_data.x - prev_data.x;
    increment.y = current_data.y - prev_data.y;
    increment.z = current_data.z - prev_data.z;

    double factor = (target_time - prev_time) / (current_time - prev_time);

    value_interp.x = prev_data.x + increment.x * factor;
    value_interp.y = prev_data.y + increment.y * factor;
    value_interp.z = prev_data.z + increment.z * factor;

    // zero interpolation
    value_interp = current_data;
  } else {
    value_interp = prev_data;
  }

  return value_interp;
}

rs2_option getSensorOptions(const rs2::sensor& sensor) {
  // Sensors usually have several options to control their properties
  //  such as Exposure, Brightness etc.

  std::cout << "Sensor supports the following options:\n" << std::endl;

  // The following loop shows how to iterate over all available options
  // Starting from 0 until RS2_OPTION_COUNT (exclusive)
  for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++) {
    rs2_option option_type = static_cast<rs2_option>(i);
    // SDK enum types can be streamed to get a string that represents them
    std::cout << "  " << i << ": " << option_type;

    // To control an option, use the following api:

    // First, verify that the sensor actually supports this option
    if (sensor.supports(option_type)) {
      std::cout << std::endl;

      // Get a human readable description of the option
      const char* description = sensor.get_option_description(option_type);
      std::cout << "       Description   : " << description << std::endl;

      // Get the current value of the option
      float current_value = sensor.get_option(option_type);
      std::cout << "       Current Value : " << current_value << std::endl;

      // To change the value of an option, please follow the change_sensor_option() function
    } else {
      std::cout << " is not supported" << std::endl;
    }
  }

  uint32_t selected_sensor_option = 0;
  return static_cast<rs2_option>(selected_sensor_option);
}

std::string getDateString() {
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");
  return oss.str();
}

std::string setupOutputFolders(const std::string& date) {
  auto root_path = std::string(getenv("HOME")) + "/ivero_results/";
  auto output_path = root_path + date;

  boost::filesystem::path root_folder = root_path;
  boost::filesystem::path output_folder = output_path;
  boost::filesystem::path depth_folder = output_path + "/depth/";
  boost::filesystem::path rgb_folder = output_path + "/rgb/";

  if (!boost::filesystem::is_directory(root_folder)) { boost::filesystem::create_directory(root_folder); }
  if (!boost::filesystem::is_directory(output_folder)) { boost::filesystem::create_directory(output_folder); }
  if (!boost::filesystem::is_directory(depth_folder)) { boost::filesystem::create_directory(depth_folder); }
  if (!boost::filesystem::is_directory(rgb_folder)) { boost::filesystem::create_directory(rgb_folder); }
  return output_path;
}

std::string get_vocab_path() {
  boost::filesystem::path cwd = boost::filesystem::current_path();
  std::string vocab_file = std::string(getenv("HOME")) + "/ORBvoc.txt";
  return vocab_file;
}

std::string getConfigPath(const std::string& serial_number) {
  std::string config_file = slam_config_folder + serial_number + ".yaml";
  return config_file;
}

bool configExists(const std::string& serial_number) {
  std::string config_file = getConfigPath(serial_number);
  return boost::filesystem::exists(config_file);
}

Eigen::Matrix4f getExtrinsics(const rs2::stream_profile& to_stream, const rs2::stream_profile& from_stream) {
  float* Rbc = from_stream.get_extrinsics_to(to_stream).rotation;
  float* tbc = from_stream.get_extrinsics_to(to_stream).translation;
  Eigen::Matrix<float, 3, 3, Eigen::RowMajor> R_to_from(Rbc);
  Eigen::Vector3f t_to_from(tbc);
  Eigen::Matrix4f T_to_from = Eigen::Matrix4f::Identity();
  T_to_from.block<3, 3>(0, 0) = R_to_from;
  T_to_from.block<3, 1>(0, 3) = t_to_from;
  return T_to_from;
}

bool createConfigFile(const rs2::stream_profile& cam_left, const rs2::stream_profile& cam_right,
                        const rs2::stream_profile& imu_stream, const std::string& config_file) {
  Eigen::Matrix4f T_b_c = getExtrinsics(imu_stream, cam_left);

  rs2_intrinsics intrinsics_left = cam_left.as<rs2::video_stream_profile>().get_intrinsics();
  rs2_intrinsics intrinsics_right = cam_left.as<rs2::video_stream_profile>().get_intrinsics();

  YAML::Node config;

  YAML::Emitter out;
  out << YAML::BeginMap;

  out << YAML::Key << "File.version" << YAML::Value << YAML::DoubleQuoted << "1.0";
  out << YAML::Key << "Camera.type" << YAML::Value << YAML::DoubleQuoted << "Rectified";

  out << YAML::Key << "Camera1.fx" << YAML::Value << intrinsics_left.fx;
  out << YAML::Key << "Camera1.fy" << YAML::Value << intrinsics_left.fy;
  out << YAML::Key << "Camera1.cx" << YAML::Value << intrinsics_left.ppx;
  out << YAML::Key << "Camera1.cy" << YAML::Value << intrinsics_left.ppy;

  out << YAML::Key << "Camera2.fx" << YAML::Value << intrinsics_right.fx;
  out << YAML::Key << "Camera2.fy" << YAML::Value << intrinsics_right.fy;
  out << YAML::Key << "Camera2.cx" << YAML::Value << intrinsics_right.ppx;
  out << YAML::Key << "Camera2.cy" << YAML::Value << intrinsics_right.ppy;

  out << YAML::Key << "Camera.fps" << YAML::Value << 30;
  out << YAML::Key << "Camera.RGB" << YAML::Value << 1;

  out << YAML::Key << "Stereo.b" << YAML::Value << cam_right.get_extrinsics_to(cam_left).translation[0];
  out << YAML::Key << "Stereo.ThDepth" << YAML::Value << std::to_string(40.0);

  out << YAML::Key << "Camera.width" << YAML::Value << intrinsics_left.width;
  out << YAML::Key << "Camera.height" << YAML::Value << intrinsics_left.height;

  std::vector<std::string> ext;
  for (int i = 0; i < T_b_c.rows(); i++) {
    for (int j = 0; j < T_b_c.cols(); j++) { ext.push_back(std::to_string(T_b_c(i, j))); }
  }
  out << YAML::Key << "IMU.T_b_c1";
  out << YAML::BeginMap;
  out << YAML::Key << "rows" << YAML::Value << 4;
  out << YAML::Key << "cols" << YAML::Value << 4;
  out << YAML::Key << "dt" << YAML::Value << "f";
  out << YAML::Key << "data" << YAML::Flow << YAML::Value << ext;
  out << YAML::EndMap;

  // todo: we need good generic values for these
  out << YAML::Key << "IMU.NoiseGyro" << YAML::Value << std::to_string(0.0002010317518288601);
  out << YAML::Key << "IMU.NoiseAcc" << YAML::Value << std::to_string(0.0031690290585787944);
  out << YAML::Key << "IMU.GyroWalk" << YAML::Value << std::to_string(1.6423932364699542e-06);
  out << YAML::Key << "IMU.AccWalk" << YAML::Value << std::to_string(0.0002876229293512109);
  out << YAML::Key << "IMU.Frequency" << YAML::Value << std::to_string(200.0);

  out << YAML::Key << "IMU.InsertKFsWhenLost" << YAML::Value << 0;

  out << YAML::Key << "ORBextractor.nFeatures" << YAML::Value << 1250;
  out << YAML::Key << "ORBextractor.scaleFactor" << YAML::Value << std::to_string(1.2);
  out << YAML::Key << "ORBextractor.nLevels" << YAML::Value << 8;
  out << YAML::Key << "ORBextractor.iniThFAST" << YAML::Value << 20;
  out << YAML::Key << "ORBextractor.minThFAST" << YAML::Value << 7;

  out << YAML::Key << "Viewer.KeyFrameSize" << YAML::Value << std::to_string(0.05);
  out << YAML::Key << "Viewer.KeyFrameLineWidth" << YAML::Value << std::to_string(1.0);
  out << YAML::Key << "Viewer.GraphLineWidth" << YAML::Value << std::to_string(0.9);
  out << YAML::Key << "Viewer.PointSize" << YAML::Value << std::to_string(2.0);
  out << YAML::Key << "Viewer.CameraSize" << YAML::Value << std::to_string(0.08);
  out << YAML::Key << "Viewer.CameraLineWidth" << YAML::Value << std::to_string(3.0);
  out << YAML::Key << "Viewer.ViewpointX" << YAML::Value << std::to_string(0.0);
  out << YAML::Key << "Viewer.ViewpointY" << YAML::Value << std::to_string(-0.7);
  out << YAML::Key << "Viewer.ViewpointZ" << YAML::Value << std::to_string(-3.5);
  out << YAML::Key << "Viewer.ViewpointF" << YAML::Value << std::to_string(500.0);
  out << YAML::EndMap;

  if (!boost::filesystem::is_directory(slam_config_folder)) { boost::filesystem::create_directory(slam_config_folder); }

  std::ofstream fout(config_file);
  fout << "%YAML:1.0\n";
  fout << out.c_str();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void setSensorOptions(const std::vector<rs2::sensor>& sensors) {
  int index = 0;
  for (rs2::sensor sensor : sensors) {
    if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
      ++index;
      if (index == 1) {
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
        sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.0);
        sensor.set_option(RS2_OPTION_EMITTER_ALWAYS_ON, 0.0);
        sensor.set_option(RS2_OPTION_EMITTER_ON_OFF, 1.0);
        sensor.set_option(RS2_OPTION_LASER_POWER, 360);
        sensor.set_option(RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
      }
      if (index == 2) { sensor.set_option(RS2_OPTION_EXPOSURE, 100.f); }

      if (index == 3) { sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION, 0); }
    }
  }
}
