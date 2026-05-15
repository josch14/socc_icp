**NOTE** Info/data structure might not be up to date. 

The long experiment (02) of Newer College dataset is only provided in a rosbag. Therefore, we need to extract the point cloud data from the rosbag.

For this, ROS Noetic needs to be installed. Used a docker container for this purpose:

```bash
docker pull ros:noetic
```

Run container and mount data into container (rosbag/dataset is on `/mnt/d`)
```bash
docker run -it --name ros_noetic \
  --net=host \
  -v ~/ROS/socc_icp_ws:/root/socc_icp_ws \
  -v /mnt/d:/mnt/d \
  ros:noetic bash
```

Required installation steps for script
```bash
source /opt/ros/noetic/setup.bash
cd socc_icp_ws/socc_icp/
cd socc_icp/
apt update
apt install -y python3-pip python3-dev python3-numpyapt install -y python3-pip python3-dev python3-numpy
apt install -y python3-pip
pip3 install natsort
pip3 install pyquaternion
apt install -y libgl1-mesa-glx libglib2.0-0
pip3 install open3d
pip3 install --upgrade "numpy>=1.21,<1.27" scipy scikit-learn
```


Finally prepare data
```bash
python3 -m data_prep.newer_college.prep_newer_college_long_experiment
```