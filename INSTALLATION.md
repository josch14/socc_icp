# Installation

The following instructions mirror the [Dockerfile](Dockerfile) and assume a **ROS 2 Humble** environment.

## [Optional] Docker Setup

Run a `ros:humble` container with a mounted dataset directory:
```bash
docker run -it -v /kitti_dataset:/kitti_dataset ros:humble bash
source /opt/ros/humble/setup.bash
```

## System Dependencies

```bash
sudo apt update && sudo apt install -y \
    python3-colcon-common-extensions \
    ros-humble-pcl-conversions \
    ros-humble-cv-bridge \
    libpcl-dev \
    python3-pip \
    python-is-python3 \
    wget \
    nano
```

## Clone and Build

Clone the repository with all submodules and build the Radix packages:
```bash
git clone https://github.com/josch14/socc_icp.git --recurse-submodules
cd socc_icp/
colcon build --packages-select radix_ros radix_msgs --symlink-install
source install/setup.bash
```

## [Optional] Update CMake

Required for the Sophus-pybind installation if your system CMake is too old:
```bash
mkdir temp && cd temp
wget https://cmake.org/files/v4.0/cmake-4.0.3.tar.gz
tar -xzf cmake-4.0.3.tar.gz
cd cmake-4.0.3
./bootstrap && make -j$(nproc) && sudo make install
cd ..
```

## Sophus (Deskewing Module)

```bash
git clone --branch main --depth 1 https://github.com/strasdat/Sophus.git
cd Sophus
git fetch --depth 1 origin d0b7315a0d90fc6143defa54596a3a95d9fa10ec
git checkout d0b7315a0d90fc6143defa54596a3a95d9fa10ec
mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
cd ../../../  # socc_icp
rm -rf temp
```

## Python Extension Module (Deskewing)

```bash
cd src/socc_icp/socc_icp/deskewing_cpp/
mkdir build && cd build
cmake .. && make -j$(nproc)
mv sophus_deskewer.cpython-310-x86_64-linux-gnu.so ..
cd .. && rm -rf build
cd ../../
```

## Python Dependencies

```bash
pip install uv  # if uv is not installed yet
sudo uv pip install -r pyproject.toml --system
```
