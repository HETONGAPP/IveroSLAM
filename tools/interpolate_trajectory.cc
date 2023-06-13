
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <unsupported/Eigen/MatrixFunctions>

using Trajectory = std::map<double, Eigen::Isometry3d>;
using namespace boost::filesystem;

Eigen::Matrix4d InterpolateTransform(const Eigen::Matrix4d& m1, const double& t1, const Eigen::Matrix4d& m2,
                                     const double& t2, const double& t) {
  double w2 = 1.0 * (t - t1) / (t2 - t1);

  Eigen::Matrix4d T1 = m1;
  Eigen::Matrix4d T2 = m2;
  Eigen::Matrix4d T;

  Eigen::Matrix3d R1 = T1.block<3, 3>(0, 0);
  Eigen::Matrix3d R2 = T2.block<3, 3>(0, 0);
  Eigen::Matrix3d R = (R2 * R1.transpose()).pow(w2) * R1;

  Eigen::Vector4d tr1 = T1.rightCols<1>();
  Eigen::Vector4d tr2 = T2.rightCols<1>();
  Eigen::Vector4d tr = (1 - w2) * tr1 + w2 * tr2;

  T.setIdentity();
  T.block<3, 3>(0, 0) = R;
  T.rightCols<1>() = tr;

  return T;
}

int main(int argc, char** argv) {
  std::vector<std::string> argList(argv, argv + argc);

  if (argList.size() < 3) {
    std::cout << "Usage: ./interpolate_trajectory /path/to/trajectory.txt /path/to/rgb_folder" << std::endl;
    ;
    return -1;
  }

  path trajectory_file_path = argList[1];
  path rgb_folder_path = argList[2];

  if (!exists(trajectory_file_path)) {
    std::cout << "No such file: " << trajectory_file_path.string() << std::endl;
    return -1;
  }

  if (!is_directory(rgb_folder_path)) {
    std::cout << "No such directory: " << rgb_folder_path.string() << std::endl;
    return -1;
  }

  std::set<double> rgb_timestamps;
  // get all timestamps of rgb images in given folder
  for (auto& entry : boost::make_iterator_range(directory_iterator(rgb_folder_path), {})) {
    double timestamp = std::stod(entry.path().stem().string());
    rgb_timestamps.insert(timestamp);
  }

  // create trajectory object from file
  Trajectory traj;
  std::ifstream trajectory_file(trajectory_file_path.string());
  std::string s;
  std::string delim{" "};

  while (std::getline(trajectory_file, s)) {
    std::vector<std::string> words;
    boost::split(words, s, boost::is_any_of(" "), boost::token_compress_on);
    auto t = std::stod(words[0]);
    auto tx = std::stod(words[1]);
    auto ty = std::stod(words[2]);
    auto tz = std::stod(words[3]);
    auto qx = std::stod(words[4]);
    auto qy = std::stod(words[5]);
    auto qz = std::stod(words[6]);
    auto qw = std::stod(words[7]);

    Eigen::Vector3d translation(tx, ty, tz);
    Eigen::Quaterniond rotation(qw, qx, qy, qz);
    Eigen::Isometry3d transform;
    transform.linear() = rotation.toRotationMatrix();
    transform.translation() = translation;
    traj[t] = transform;
  }

  // for each image timestamp, interpolate its pose in the provided trajectory
  std::string out_file_path =
      trajectory_file_path.parent_path().string() + "/" + trajectory_file_path.stem().string() + "-interpolated.txt";
  std::cout << out_file_path << std::endl;
  std::ofstream outfile(out_file_path);
  auto start = traj.begin()->first;
  for (const auto& t : rgb_timestamps) {
    if (t < start) { continue; }
    auto ub = traj.upper_bound(t); // >t
    auto lb = traj.lower_bound(t); // >=t

    if (lb == traj.end() && ub == traj.end()) { continue; }
    lb = std::prev(lb);

    Eigen::Isometry3d T;
    T.matrix() = InterpolateTransform(lb->second.matrix(), lb->first, ub->second.matrix(), ub->first, t);

    Eigen::Vector4d q = Eigen::Quaterniond(T.linear()).coeffs();
    Eigen::Vector3d trans = T.translation();

    outfile << std::fixed << std::setprecision(9) << t << " ";
    outfile << std::fixed << std::setprecision(9) << trans.x() << " ";
    outfile << std::fixed << std::setprecision(9) << trans.y() << " ";
    outfile << std::fixed << std::setprecision(9) << trans.z() << " ";
    outfile << std::fixed << std::setprecision(9) << q.x() << " ";
    outfile << std::fixed << std::setprecision(9) << q.y() << " ";
    outfile << std::fixed << std::setprecision(9) << q.z() << " ";
    outfile << std::fixed << std::setprecision(9) << q.w() << std::endl;
  }
}