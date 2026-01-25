FROM ubuntu:20.04

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release software-properties-common \
    build-essential git cmake \
    python3-pip \
    libceres-dev libeigen3-dev \
    libpcl-dev \
    nlohmann-json3-dev \
    tmux \
    wget \
    unzip \
    libusb-1.0-0-dev \
    libboost-all-dev \
    libpcl-dev \
    libtbb-dev \
    libmetis-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros1.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-catkin-tools \
    ros-noetic-geometry-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-message-generation \
    ros-noetic-message-runtime \
    ros-noetic-catkin \
    ros-noetic-tf \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt

RUN wget https://cmake.org/files/v3.20/cmake-3.20.5.tar.gz &&\
    tar -zxvf cmake-3.20.5.tar.gz && \ 
    cd cmake-3.20.5 && \
    ./bootstrap && \
    make && \ 
    make install

RUN wget -O /opt/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip && \
    unzip gtsam.zip && \
    cd gtsam-4.0.0-alpha2 && \
    mkdir build && cd build && \
    cmake .. && \
    make install 

RUN git clone https://github.com/Livox-SDK/Livox-SDK.git && \
    cd Livox-SDK && \
    rm -rf build && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

WORKDIR /ros_ws

COPY ./src ./src

WORKDIR /ros_ws/src


RUN PKG=/ros_ws/src/LeGO-LOAM/LeGO-LOAM && \
    sed -i 's|set(CMAKE_CXX_FLAGS.*-std=c++11.*)|set(CMAKE_CXX_STANDARD 14)\nset(CMAKE_CXX_STANDARD_REQUIRED ON)|' $PKG/CMakeLists.txt

RUN PKG=/ros_ws/src/LeGO-LOAM/LeGO-LOAM && \
    sed -i '/find_package(Boost/d' $PKG/CMakeLists.txt && \
    sed -i '/include_directories(${Boost_INCLUDE_DIRS})/d' $PKG/CMakeLists.txt && \
    sed -i '/find_package(GTSAM REQUIRED QUIET)/a \ 
    find_package(Boost REQUIRED COMPONENTS serialization thread timer chrono)' $PKG/CMakeLists.txt && \
    sed -i '/{GTSAM_INCLUDE_DIR}/a \ \ \ \ \ \ \ \(${Boost_INCLUDE_DIRS})' $PKG/CMakeLists.txt

RUN PKG=/ros_ws/src/LeGO-LOAM/LeGO-LOAM && \
    sed -i '/target_link_libraries(mapOptmization/ s/)/ ${Boost_LIBRARIES})/' $PKG/CMakeLists.txt

RUN PKG=/ros_ws/src/LeGO-LOAM/LeGO-LOAM && \
    sed -i 's|#include <opencv/cv.h>|#include <opencv2/opencv.hpp>|' \
    $PKG/include/utility.h

RUN PKG=/ros_ws/src/LeGO-LOAM/LeGO-LOAM && \
    grep -q "Eigen/Core" $PKG/include/utility.h || \
    (awk '/opencv2\/opencv.hpp/ { print; print "#include <eigen3/Eigen/Core>"; print "#include <eigen3/Eigen/Dense>"; print "#include <eigen3/Eigen/Geometry>"; next }1' \
        $PKG/include/utility.h > /tmp/utility.h && mv /tmp/utility.h $PKG/include/utility.h)

WORKDIR /ros_ws

RUN source /opt/ros/noetic/setup.bash && \
    catkin_make 

ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID ros && \
    useradd -m -u $UID -g $GID -s /bin/bash ros
    
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc

CMD ["bash"]
