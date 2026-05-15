**NOTE** Info/data structure might not be up to date. 

The long experiment (02) of Newer College dataset is only provided in a rosbag. Therefore, we need to extract the point cloud data from the rosbag.

For this, ROS Noetic needs to be installed. Used a docker container for this purpose:

```bash
docker pull ros:noetic
```

Run container and mount data into container (rosbag/dataset is on `/mnt/e`)
```bash
docker run -it --name ros_noetic_ground_challenge \
  --net=host \
  -v ~/ROS/socc_icp_ws:/root/socc_icp_ws \
  -v /mnt/e:/mnt/e \
  ros:noetic bash
```

Required installation steps for script
```bash
cd ~/socc_icp_ws/src/socc_icp
source /opt/ros/noetic/setup.bash
apt update
apt install -y python3-pip
apt install -y libgl1-mesa-glx libglib2.0-0
pip3 install open3d
pip3 install --upgrade "numpy>=1.21,<1.27" scipy scikit-learn
```


Finally prepare data
```bash
python3 -m data_prep.ground_challenge.prep_ground_challenge
```

Afterwards, removed the first 3 clouds of both sequences (those do not have a GT pose)

