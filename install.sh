cd
# update apt get
sudo apt-get update -y
apt-get upgrade -y
apt-get dist-upgrade -y

sudo apt-get install -y \
  # general dependencies
  build-essential \
  cmake \
  git \
  pkg-config \ 
  wget \
  software-properties-common \
  libeigen3-dev \
  libboost-all-dev

## tell system where cuda is
echo "export CUDA_HOME=/usr/local/cuda; export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64; export PATH=$PATH:$CUDA_HOME/bin" >> ~/.bashrc

## install librealsense with rsusb backend and cuda enabled
chmod +x ~/projects/IveroSLAM/libuvc_installation.sh
~/projects/IveroSLAM/libuvc_installation.sh

## install ros foxy
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-foxy-desktop python3-argcomplete ros-dev-tools
echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc 

mkdir software
cd software

## install yaml cpp
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp && mkdir build && cd build
cmake .. -DYAML_BUILD_SHARED_LIBS=ON
make -j$(nproc) && sudo make install
cd ~/software

## install pangolin
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
./scripts/install_prerequisites.sh recommended
cmake -B build
cmake --build build
cd build && sudo make install
cd ~/software

## install zipper
git clone --recursive https://github.com/sebastiandev/zipper.git
cd zipper && mkdir build && cd build
cmake ../
make -j$(nproc) && sudo make install

sudo ldconfig

cd ~/projects/IveroSLAM
chmod u+x build.sh
./build.sh

# cd ~
# mkdir -p ros2_ws/src
# git clone https://github.com/Luxolis/IveroSLAM.git ~/ros2_ws/src
# git clone https://github.com/Luxolis/IveroSLAM_interfaces.git ~/ros2_ws/src
# cd ros2_ws
# colcon build
