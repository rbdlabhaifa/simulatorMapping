#!/bin/bash
cwd=$(pwd)
echo "Configuring and building Thirdparty/DBoW2 ..."
sudo apt install -y libeigen3-dev
sudo ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen
sudo ln -sf /usr/include/eigen3/unsupported /usr/include/unsupported
sudo ln -sf /usr/local/include/eigen3/Eigen /usr/local/include/Eigen
sudo ln -sf /usr/local/include/eigen3/unsupported /usr/local/include/unsupported

cd Thirdparty/DBoW2
mkdir build
cd build

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
echo "Configuring and building Thirdparty/Pangolin ..."
sudo apt-get install -y libncurses5-dev
sudo apt-get install -y libglew-dev
sudo apt-get install -y libglu1-mesa-dev freeglut3-dev mesa-common-dev
sudo apt-get install -y ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev
sudo apt-get install -y libdc1394-22-dev libraw1394-dev
sudo apt-get install -y libjpeg-dev libpng-dev libtiff5-dev libopenexr-dev
sudo apt-get install -y libboost-all-dev libopenblas-dev
sudo apt-get install -y libbluetooth-dev
sudo apt install -y libpcl-dev

cd ~
git clone https://github.com/nlohmann/json.git
cd json
mkdir build
cd build
cmake ..
sudo make -j$(nproc) install
cd ~
git clone https://github.com/lava/matplotlib-cpp.git
cd matplotlib-cpp
mkdir build
cd build
cmake ..
sudo make -j$(nproc) install
cd ~
git clone https://github.com/gabime/spdlog.git
cd spdlog
mkdir build
cd build
cmake ..
sudo make -j$(nproc) install
cd ~
git clone https://github.com/tzukpolinsky/ctello.git
cd ctello
mkdir build
cd build
cmake ..
sudo make -j$(nproc) install
cd ~
git clone https://gitlab.com/libeigen/eigen.git
cd eigen
git checkout 3.4.0
mkdir build
cd build
cmake ..
sudo make -j$(nproc) install
cd "$cwd"
chmod +x build.sh
./build.sh
