#!/bin/bash
cwd=$(pwd)

sudo apt-get update
sudo apt-get install -y python3-numpy
sudo ln -sf /usr/lib/python3/dist-packages/numpy/core/include/numpy /usr/include/numpy

sudo apt-get install -y cmake libncurses5-dev libglew-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev libdc1394-22-dev libraw1394-dev
sudo apt-get install -y ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libjpeg-dev libpng-dev libtiff5-dev libopenexr-dev
sudo apt-get install -y libboost-all-dev libopenblas-dev libbluetooth-dev libpcl-dev libxext-dev libxfixes-dev libxrender-dev libxcb1-dev libx11-xcb-dev libxcb-glx0-dev
sudo apt-get -y install libxkbcommon-dev libxcb-keysyms1-dev libxcb-image0-dev libxcb-shm0-dev libxcb-icccm4-dev libxcb-sync0-dev libxcb-xfixes0-dev libxcb-shape0-dev libxcb-randr0-dev libxcb-render-util0-dev
chmod +x opencv3.4.16Install.sh
bash opencv3.4.16Install.sh
cd "$cwd"

cd Thirdparty/
git clone https://gitlab.com/libeigen/eigen.git
cd eigen
git checkout 3.4.0
mkdir build
cd build
cmake ..
sudo make -j3 install
if [ ! -f "/usr/local/include/Eigen" ]; then
  sudo ln -sf /usr/local/include/eigen3/Eigen /usr/local/include/Eigen
fi
cd "$cwd"

cd Thirdparty/
git clone https://github.com/nlohmann/json.git
cd json
mkdir build
cd build
cmake ..
sudo make -j3 install

cd Thirdparty/
git clone https://github.com/lava/matplotlib-cpp.git
cd matplotlib-cpp
mkdir build
cd build
cmake ..
sudo make -j3 install

cd Thirdparty/
git clone https://github.com/gabime/spdlog.git
cd spdlog
mkdir build
cd build
cmake ..
sudo make -j3 install

cd Thirdparty/
git clone https://github.com/liamvanunu/ctello.git
cd ctello
mkdir build
cd build
cmake ..
sudo make -j3 install

cd "$cwd"
mkdir build
cd build
cmake ..
make -j3
