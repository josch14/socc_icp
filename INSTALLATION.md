# Local Installation Guide

Tested on **Ubuntu 22.04** with **ROS 2 Humble**. Python 3.10(.12) is required (exactly).

---

## 1. Install ROS 2 Humble

Follow the official guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

---

## 2. Install system dependencies

```bash
sudo apt update && sudo apt install -y \
    python3-colcon-common-extensions \
    ros-humble-pcl-conversions \
    ros-humble-cv-bridge \
    libpcl-dev \
    python3-pip \
    python-is-python3 \
    libeigen3-dev \
    libtbb-dev \
    pybind11-dev \
    git \
    wget
```

---

## 3. Build and install CMake 4.0.3 (or higher) 

The Sophus main branch requires a recent CMake. Ubuntu 22.04 ships 3.22, which may not suffice — build 4.0.3 from source:

```bash
cd /tmp
wget https://cmake.org/files/v4.0/cmake-4.0.3.tar.gz
tar -xzf cmake-4.0.3.tar.gz
cd cmake-4.0.3
./bootstrap && make -j$(nproc) && sudo make install
cd /tmp && rm -rf cmake-4.0.3*
```

Verify: `cmake --version` should report 4.0.3.

---

## 4. Build and install Sophus (C++ library)

The deskewing C++ extension requires Sophus headers installed system-wide at `/usr/local/include`.

```bash
cd /tmp
git clone --branch main --depth 1 https://github.com/strasdat/Sophus.git
cd Sophus
git fetch --depth 1 origin d0b7315a0d90fc6143defa54596a3a95d9fa10ec
git checkout d0b7315a0d90fc6143defa54596a3a95d9fa10ec
mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
cd /tmp && rm -rf Sophus
```

---

## 5. Clone the repository with submodules

```bash
git clone --recurse-submodules https://github.com/josch14/socc-icp.git
cd socc-icp
```

If you already cloned without `--recurse-submodules`:

```bash
git submodule update --init --recursive
```

---

## 6. Build the Radix ROS packages

Run from the workspace root (the cloned `socc-icp/` directory):

```bash
colcon build --packages-select radix_ros radix_msgs --symlink-install
source install/setup.bash
```

---

## 7. Build the deskewing C++ extension

```bash
cd src/socc_icp/socc_icp/deskewing_cpp
mkdir build && cd build
cmake .. && make -j$(nproc)
mv sophus_deskewer.cpython-310-x86_64-linux-gnu.so ..
cd .. && rm -rf build
```

---

## 8. Install Python dependencies

Install `uv`, then install all Python packages system-wide (Python 3.10 is required):

```bash
pip install uv

cd src/socc_icp   # must be run from here — pyproject.toml references ../radix_clients
sudo uv pip install -r pyproject.toml --system
```

---

## Running

All run commands must be executed from `src/socc_icp/`:

```bash
cd src/socc_icp

python -m run.run_kitti                  # KITTI with semantics
python -m run.run_kitti --no-semantics   # KITTI without semantics
python -m run.run_mulran
python -m run.run_newer_college
python -m run.run_ground_challenge
python -m run.run_subt_mrs
```

Results are written to `log/<timestamp>_sequ_<id>/` containing poses, config, and metrics.

See [DATASETS.md](DATASETS.md) for instructions on obtaining and setting up each dataset.
