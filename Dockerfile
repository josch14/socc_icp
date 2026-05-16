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
    git \
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
COPY . "/home/socc_icp"

# radix_clients at commit 604007a78e16b59e5eafbdbed0773406a5b764dc
RUN git clone https://github.com/ProjectVERUM/radix_clients.git socc_icp/src/radix_clients && \
    cd socc_icp/src/radix_clients && git reset --hard 604007a78e16b59e5eafbdbed0773406a5b764dc && cd ../../..
# radix_msgs_ros2_pkg at commit d5cd522109806354f2417146cce388292f63bcaa
RUN git clone https://github.com/ProjectVERUM/radix_msgs_ros2_pkg.git socc_icp/src/radix_msgs_ros2_pkg && \
    cd socc_icp/src/radix_msgs_ros2_pkg && git reset --hard d5cd522109806354f2417146cce388292f63bcaa && cd ../../..
# radix_ros2_pkg at commit 38c80c2a73644f4ec9a9d8140a1a7ad7f09ed974
RUN git clone https://github.com/ProjectVERUM/radix_ros2_pkg.git socc_icp/src/radix_ros2_pkg && \
    cd socc_icp/src/radix_ros2_pkg && git reset --hard 38c80c2a73644f4ec9a9d8140a1a7ad7f09ed974 && cd ../../..
# ros2_numpy at commit b69cb892f6609fbdb739afc8358226100fc5dda2
RUN git clone https://github.com/ProjectVERUM/ros2_numpy.git socc_icp/src/ros2_numpy && \
    cd socc_icp/src/ros2_numpy && git reset --hard b69cb892f6609fbdb739afc8358226100fc5dda2 && cd ../../..


# Build Radix ROS packages
WORKDIR /home/socc_icp
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select radix_ros radix_msgs
# Build deskewing_cpp module
WORKDIR /home/socc_icp/src/socc_icp/socc_icp/deskewing_cpp
RUN mkdir build && cd build && \
    cmake .. && make -j"$(nproc)" && \
    mv sophus_deskewer.cpython-310-x86_64-linux-gnu.so .. && \
    cd .. && rm -rf build

# Install Python dependencies
WORKDIR /home/socc_icp/src/socc_icp
RUN pip install uv && \
    uv pip install -r pyproject.toml --system

# Source ROS 2 and workspace setup
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /home/socc_icp/install/setup.bash" >> /root/.bashrc

# Default command
CMD ["bash"]
