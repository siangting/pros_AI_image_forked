FROM ros:humble-ros-core-jammy
ENV ROS2_WS /workspaces
ENV ROS_DOMAIN_ID=1
ENV ROS_DISTRO=humble
ARG THREADS=4
ARG TARGETPLATFORM

SHELL ["/bin/bash", "-c"] 

##### Copy Source Code #####
COPY . /tmp

##### Environment Settings #####
WORKDIR /tmp

# # Copy the script into the image to calculate threads of the host build machine
# COPY set_threads.sh /usr/local/bin/

# # Make the script executable
# RUN chmod +x /usr/local/bin/set_threads.sh

# # Run the script to determine and set the THREADS environment variable
# ENV THREADS $(/usr/local/bin/set_threads.sh)

# Copy the run command for rebuilding colcon. You can source it.
RUN mkdir ${ROS2_WS} && \
    mv /tmp/rebuild_colcon.rc ${ROS2_WS} && \

# Entrypoint
    mv /tmp/ros_entrypoint.bash /ros_entrypoint.bash && \
    chmod +x /ros_entrypoint.bash && \

# System Upgrade
    apt update && \
    apt upgrade -y && \

# Necessary System Package Installation
    apt install -y \
        axel \
        bash-completion \
        bat \
        bmon \
        build-essential \
        curl \
        git \
        git-flow \
        htop \
        iotop \
        iproute2 \
        libncurses5-dev \
        libncursesw5-dev \
        lsof \
        ncdu \
        net-tools \
        nvtop \
        python3-pip \
        python3-venv \
        screen \
        tig \
        tmux \
        tree \
        vim \
        wget \
        zsh && \

    pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir -r /tmp/requirements.txt && \

# Soft Link
    ln -s /usr/bin/python3 /usr/bin/python && \
    ln -s /usr/bin/batcat /usr/bin/bat && \

# Install oh-my-bash
    bash -c "$(curl -fsSL https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh)" && \

# Use our pre-defined bashrc
    mv /tmp/.bashrc /root && \
    ln -s /root/.bashrc /.bashrc && \

##### ROS2 Installation #####
# install ros2
    apt install -y \
        python3-colcon-common-extensions \
        python3-colcon-mixin \
        python3-rosdep \
        python3-vcstool \
        ros-${ROS_DISTRO}-ros-base \
        # install ros bridge
        ros-${ROS_DISTRO}-rosbridge-suite ccache && \

# install boost serial and json
    apt install -y \
        libboost-all-dev \
        libboost-program-options-dev \
        libboost-system-dev \
        libboost-thread-dev \
        libserial-dev \
        nlohmann-json3-dev && \

# bootstrap rosdep
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \

# setup colcon mixin and metadata
    colcon mixin add default \
        https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
        https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

WORKDIR ${ROS2_WS}

##### colcon Installation #####
### Copy Source Code
# Rplidar
RUN mv /tmp/rplidar_src ./src && \

# Astra Camera
    mv /tmp/camera_src ./src && \

### Installation ###
# Rplidar
    rosdep update && \

# Build your ROS packages
    rosdep install -q -y -r --from-paths src --ignore-src && \
    apt install ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup -y && \
    . /opt/ros/humble/setup.sh && \
    colcon build --packages-select rplidar_ros --symlink-install --parallel-workers ${THREADS} && \
    colcon build --packages-select csm --symlink-install --parallel-workers ${THREADS} && \
    colcon build --packages-select ros2_laser_scan_matcher --symlink-install --parallel-workers ${THREADS} && \
    . /opt/ros/humble/setup.sh && \
    colcon build --packages-select slam_toolbox --symlink-install --parallel-workers ${THREADS} && \

    apt install ros-humble-rplidar-ros && \

##### Astra Camera Installation #####
# install dependencies
    apt install -y libgflags-dev ros-${ROS_DISTRO}-image-geometry ros-${ROS_DISTRO}-camera-info-manager \
                   ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-image-publisher && \
    apt install -y libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev libopenni2-dev nlohmann-json3-dev && \
    apt install ros-${ROS_DISTRO}-image-transport-plugins -y && \
    git clone https://github.com/libuvc/libuvc.git /temp/libuvc && \
    mkdir -p /temp/libuvc/build
WORKDIR /temp/libuvc/build
RUN cmake .. && \
    make -j${THREADS} && \
    make install && \
    ldconfig

# Build
WORKDIR ${ROS2_WS}
RUN rosdep install -q -y -r --from-paths src --ignore-src && \
    . /opt/ros/humble/setup.sh && colcon build --packages-select pros_image --symlink-install --parallel-workers ${THREADS} && \
    . /opt/ros/humble/setup.sh && colcon build --packages-select astra_camera_msgs --symlink-install --parallel-workers ${THREADS} && \
    . /opt/ros/humble/setup.sh && colcon build --packages-select astra_camera --symlink-install --parallel-workers ${THREADS} && \

##### Sipeed A075v #####
    # https://wiki.sipeed.com/hardware/zh/maixsense/maixsense-a075v/maixsense-a075v.html?fbclid=IwZXh0bgNhZW0CMTAAAR0no57ZkSZQn1Vp0KB96VTxY7GkhBXH63Mz5LLvd-2o8IOXLnhKPf5IP9Y_aem_AUoqMDGoSwdGA0OwfJt78WNY0xl7XZ5pmuWfUfXxnfEzrEP-D-6yCmQ2ZnQ0-hieiYEBVvUv7tMQ978iflqkcb70
    mv /tmp/sipeed_camera_src/ros2 ${ROS2_WS}/src && \
    colcon build --packages-select ros2 --symlink-install --parallel-workers ${THREADS} && \

##### YDLidar #####
# SDK compile installation
    mkdir -p /tmp/ydlidar_src/YDLidar-SDK/build
WORKDIR /tmp/ydlidar_src/YDLidar-SDK/build
RUN cmake .. && \
    make -j && \
    make -j install && \

# colcon build
    mv /tmp/ydlidar_src/ydlidar_ros2_ws/src/ydlidar_ros2_driver ${ROS2_WS}/src
WORKDIR ${ROS2_WS}
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select ydlidar_ros2_driver --symlink-install --parallel-workers ${THREADS} && \

# ##### Build your ROS packages #####
# # We use mount instead of copy
# # COPY ./src ${ROS2_WS}/src
# RUN . /opt/ros/humble/setup.sh && colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

##### Post-Settings #####
# Clear tmp and cache
    rm -rf /tmp/* && \
    rm -rf /temp/* && \
    rm -rf /var/lib/apt/lists/*

# Add nvcc to PATH
ENV PATH="$PATH:/usr/local/cuda/bin"

ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD ["bash", "-l"]
