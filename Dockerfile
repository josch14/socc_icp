# Base image: ROS 2 Humble
FROM ros:humble

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Update and install dependencies
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    ros-humble-pcl-conversions \
    ros-humble-cv-bridge \
    libpcl-dev \
    python3-pip \
    python-is-python3 \
    wget \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Build and install CMake 4.0.3 manually
WORKDIR /home/temp
RUN wget https://cmake.org/files/v4.0/cmake-4.0.3.tar.gz && \
    tar -xzf cmake-4.0.3.tar.gz && \
    cd cmake-4.0.3 && \
    ./bootstrap && make -j"$(nproc)" && make install && \
    cd .. && rm -rf cmake-4.0.3*

# Build and install Sophus (specific (latest) commit)
RUN git clone --branch main --depth 1 https://github.com/strasdat/Sophus.git && \
    cd Sophus && \
    git fetch --depth 1 origin d0b7315a0d90fc6143defa54596a3a95d9fa10ec && \
    git checkout d0b7315a0d90fc6143defa54596a3a95d9fa10ec && \
    mkdir build && cd build && \
    cmake .. && make -j"$(nproc)" && make install && \
    cd /home && rm -rf temp

# Set working directory
WORKDIR /home

# create workspace
COPY . "/home/socc_icp_ws"

# radix_clients at commit 7966e71a626830a1491411d33d88aa1f5451beda
RUN git clone TODO socc_icp_ws/src/radix_clients && \
    cd socc_icp_ws/src/radix_clients && git reset --hard 7966e71a626830a1491411d33d88aa1f5451beda && cd ../../..
# radix_msgs_ros2_pkg at commit bd5eec03f401de86477f9eaf102980008a096c4e
RUN git clone TODO socc_icp_ws/src/radix_msgs_ros2_pkg && \
    cd socc_icp_ws/src/radix_msgs_ros2_pkg && git reset --hard bd5eec03f401de86477f9eaf102980008a096c4e && cd ../../..
# radix_ros2_pkg at commit e97c65b6b92b736160580d8e29528c234326e940
RUN git clone TODO socc_icp_ws/src/radix_ros2_pkg && \
    cd socc_icp_ws/src/radix_ros2_pkg && git reset --hard e97c65b6b92b736160580d8e29528c234326e940 && cd ../../..
# ros2_numpy at commit 74f826658a2e8b3d517487bfda26d0274d948310
RUN git clone TODO socc_icp_ws/src/ros2_numpy && \
    cd socc_icp_ws/src/ros2_numpy && git reset --hard 74f826658a2e8b3d517487bfda26d0274d948310 && cd ../../..


# Build Radix ROS packages
WORKDIR /home/socc_icp_ws
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select radix_ros radix_msgs
# Build deskewing_cpp module
WORKDIR /home/socc_icp_ws/src/socc_icp/socc_icp/deskewing_cpp
RUN mkdir build && cd build && \
    cmake .. && make -j"$(nproc)" && \
    mv sophus_deskewer.cpython-310-x86_64-linux-gnu.so ..

# Install Python dependencies
WORKDIR /home/socc_icp_ws/src/socc_icp
RUN pip install uv && \
    uv pip install -r pyproject.toml --system

# Source ROS 2 and workspace setup
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /home/socc_icp_ws/install/setup.bash" >> /root/.bashrc

# Default command
CMD ["bash"]
