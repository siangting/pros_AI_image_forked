FROM public.ecr.aws/paia-tech/ros2-humble:dev
ENV ROS2_WS /workspaces
ENV ROS_DOMAIN_ID=1
ENV ROS_DISTRO humble
ARG THREADS 4
ARG TARGETPLATFORM

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

# Copy the python package requirements.txt.
    # mv /tmp/requirements.txt /tmp && \

# Use our pre-defined bashrc
    mv /tmp/.bashrc /root && \

# Remove the run command in ros2-humble image
    rm /.bashrc && rm /root/.bashrc && rm /ros_entrypoint.sh && \

# Entrypoint
    mv /tmp/ros_entrypoint.bash /ros_entrypoint.bash && \
    chmod +x /ros_entrypoint.bash && \

# System Upgrade
    apt update && \
    apt upgrade -y && \
    pip3 install --no-cache-dir --upgrade pip && \

# PyTorch and Others Installation
    pip3 install --no-cache-dir -r /tmp/requirements.txt && \
    apt install -y libncurses5-dev libncursesw5-dev tmux screen ncdu tree zsh axel python3-venv && \
    pip3 install --no-cache-dir torch torchvision torchaudio && \

# Soft Link
    ln -s /usr/bin/python3 /usr/bin/python && \

# cuda toolkit 12.4 Update 1
    if [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
        echo "Install cuda in amd64." >> log.txt && \
        axel -n 20 https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb && \
        dpkg -i cuda-keyring_1.1-1_all.deb && \
        apt update && \
        apt -y install cuda-toolkit-12-4; \
    elif [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
        echo "Install cuda in arm64." >> log.txt && \
        axel -n 20 https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/sbsa/cuda-keyring_1.1-1_all.deb && \
        dpkg -i cuda-keyring_1.1-1_all.deb && \
        apt update && \
        apt -y install cuda-toolkit-12-4; \
    else \
        echo "Error when installing cuda 12.4 update 1!!!" >> log.txt; \
    fi && \

# cudnn 9.1.0
    if [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
        echo "Install cudnn in amd64." >> log.txt && \
        axel -n 20 https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb && \
        dpkg -i cuda-keyring_1.1-1_all.deb && \
        apt update && \
        apt -y install cudnn-cuda-12; \
    elif [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
        echo "Install cudnn in arm64." >> log.txt && \
        axel -n 20 https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/sbsa/cuda-keyring_1.1-1_all.deb && \
        dpkg -i cuda-keyring_1.1-1_all.deb && \
        apt update && \
        apt -y install cudnn-cuda-12; \
    else \
        echo "Error when installing cudnn 9.1.0!!!" >> log.txt; \
    fi

WORKDIR ${ROS2_WS}

##### colcon Installation #####
### Copy Source Code
# Rplidar
RUN mv /tmp/rplidar_src ./src && \

# Astra Camera
    mv /tmp/camera_src ./src && \

### Installation ###
# Rplidar
# TODO install dependencies
# RUN apt install -y packages_to_install
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

# ##### Build your ROS packages #####
# # We use mount instead of copy
# # COPY ./src ${ROS2_WS}/src
# RUN . /opt/ros/humble/setup.sh && colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

##### Post-Settings #####
# Clear tmp
    rm -rf /tmp/* && \
    rm -rf /temp/*

# Add nvcc to PATH
ENV PATH="$PATH:/usr/local/cuda/bin"

ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD ["bash", "-l"]
