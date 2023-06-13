# IveroSLAM

This repository contains the SLAM software for the Ivero scene capturing device. It has been adapted from [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) with optimizations to ensure real time performance on the Jetson Nano Orin. The primary changes are:

1. Enabled IR emitter switching for simultaneous Stereo SLAM and accurate depth map output
2. (TODO) Upgraded DBoW2 to (DBoW3, FBoW, custom implementation?)
3. (TODO) CUDA enabled feature detection and matching
4. (TODO) Switching from the ORB descriptor to the newer BEBLID descriptor
5. Automatic config reading directly from realsense device
6. RGBD and trajectory output for 3D reconstruction
7. Start/Stop functions, exposed over network

# Prerequisites
We have tested the library in **Ubuntu 20.04** using **C++17**.

- [librealsense](https://github.com/IntelRealSense/librealsense/tree/b874e42685aed1269bc57a2fe5bf14946deb6ede)
- [Pangolin](https://github.com/stevenlovegrove/Pangolin/tree/0cccd5ab215ab2952eed8f4af07e0bd02e5fcb48)
- [OpenCV](https://github.com/opencv/opencv/releases/tag/4.2.0) **Required at least 4.2.0** - use built in version from ros
- [Eigen3](http://eigen.tuxfamily.org) **Required at least 3.1.0**
- [Zipper](https://github.com/sebastiandev/zipper)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- Boost
- DBoW2, Sophus and g2o (Included in Thirdparty folder)

# 3. Installing and Building IveroSLAM

## 3.1 Docker
```diff
- WIP (Do not use)
```

Build docker image:
`docker build -t="ivero-slam" --file Dockerfile .;`

Enter shell of image:
`docker exec -t -i ivero-slam /bin/bash`

## 3.2 Native (On Jetson Nano Orin)

1. `mkdir ~/projects`
2. `cd ~/projects`
3. `git clone https://github.com/Luxolis/IveroSLAM.git`
4. `sudo ./install.sh`

# 4. Running IveroSLAM

Make sure a realsense d455 is plugged in and the repository has been built then run:

`./ivero_slam`

Which will output results to ~/ivero_results/[date]/

# 5. Results folder

1. /rgb/ - color images
2. /depth/ - aligned depth images, as a 16UC1 image, with depth values in millimeters
3. /trajectory.txt - resulting trajectory from SLAM in the color frame

