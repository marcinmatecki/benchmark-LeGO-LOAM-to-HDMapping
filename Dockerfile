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
    libeigen3-dev \
    libpcl-dev \
    libtbb-dev \
    libmetis-dev \
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

RUN source /opt/ros/noetic/setup.bash && \
    catkin_make 

ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID ros && \
    useradd -m -u $UID -g $GID -s /bin/bash ros
    
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc

CMD ["bash"]
