sudo apt-get update
sudo apt-get install -y build-essential cmake git pkg-config libgtk-3-dev libprotobuf-dev protobuf-compiler \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libgoogle-glog-dev libgflags-dev libgphoto2-dev libeigen3-dev libhdf5-dev doxygen \
    libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev \
    gfortran openexr libatlas-base-dev python3-dev python3-numpy libxine2-dev libv4l-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev \
    libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev qt5-default libgtk2.0-dev libtbb-dev libatlas-base-dev libfaac-dev libmp3lame-dev libtheora-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libvorbis-dev libxvidcore-dev libopencore-amrnb-dev libopencore-amrwb-dev x264 v4l-utils

cd Thirdparty/
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
cd opencv
git checkout 3.4.16
cd ../opencv_contrib
git checkout 3.4.16
cd ../opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D WITH_V4L=ON \
    -D WITH_QT=ON \
    -D WITH_OPENGL=ON \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
    -D BUILD_EXAMPLES=ON ..
sudo make -j3 install

