# Dataset Preparation

Instructions for downloading and preparing the datasets used in the paper. The default configurations assume the datasets to be located at `/home` after data preparation (e.g. `/home/kitti_dataset`, see `configs/config_kitti_no_semantics.yaml`). The current implementation does not support rosbags, each dataset must be provided as individual point cloud frames stored in separate files, with ground truth poses stored alongside. Datasets distributed as rosbags therefore need to be extracted into this format first.

## KITTI

Download from the following sources:
- [Velodyne laser data, calibration files, ground truth poses](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)
- To run SOCC-ICP with ground truth labels: [SemanticKITTI label data](https://semantic-kitti.org/dataset.html#download) (179 MB file)
- To run SOCC-ICP with labels predicted by [LSK3DNet](https://arxiv.org/abs/2403.15173), download labels at [josch14/LSK3DNet_kitti_preds](https://github.com/josch14/LSK3DNet_kitti_preds)

SOCC-ICP expects the KITTI data to be structured as follows:
```
/home/kitti_dataset/
├── poses/
│   ├── 00.txt
│   ├── 01.txt
│   └── ...
└── sequences/
    ├── 00/
    │   ├── velodyne/
    │   │   ├── 000000.bin
    │   │   └── ...
    │   ├── labels/
    │   │   ├── 000000.label
    │   │   └── ...
    │   ├── labels_lsk3dnet/
    │   │   ├── 000000.label
    │   │   └── ...
    │   └── calib.txt
    └── ...
```

## MulRan

To download the data, follow the instructions at the [MulRan dataset website](https://sites.google.com/view/mulran-pr/download). Download the `Ouster` point clouds and `global_pose.csv` for the `DCC01–03`, `KAIST01–03`, `Riverside01–03`, and `Sejong01–03` sequences.

SOCC-ICP expects the MulRan data to be structured as follows (only the listed files are required):
```
/home/MulRan/
├── dcc/
│   ├── DCC01/
│   │   ├── Ouster/
│   │   │   ├── 1564718964199537916.bin
│   │   │   └── ...
│   │   └── global_pose.csv
│   ├── DCC02/
│   │   └── ...
│   └── DCC03/
│       └── ...
├── kaist/
│   ├── KAIST01/
│   │   └── ...
│   └── ...
├── riverside/
│   ├── Riverside01/
│   │   └── ...
│   └── ...
└── sejong/
    ├── Sejong01/
    │   └── ...
    └── ...
```

## Ground-Challenge

Download from the [Ground-Challenge Dataset GitHub](https://github.com/sjtuyinjie/Ground-Challenge) page. Two sequences are used: `corridor1` and `corridor2`. Place the rosbags as follows before extraction:
```
/home/ground-challenge/
├── corridor1/
│   ├── corridor1.bag
│   └── poses_gt.txt      ← psudo_gt/corridor1.txt, renamed
└── corridor2/
    ├── corridor2.bag
    └── poses_gt.txt      ← psudo_gt/corridor2.txt, renamed
```

The point clouds are extracted from the `/velodyne_points` topic and saved as `.npy` files (xyz + per-point timestamps) using `scripts/prepare_ground_challenge.py`. This script requires ROS Noetic. A Docker container is the easiest way to get it:

Run the following from the repository root (`socc-icp/`):
```bash
docker pull ros:noetic

docker run -it --name ros_noetic_ground_challenge \
  --net=host \
  -v $(pwd):/workspace \
  -v /home/ground-challenge:/home/ground-challenge \
  ros:noetic bash
```

Inside the container, install dependencies:
```bash
source /opt/ros/noetic/setup.bash
apt update && apt install -y python3-pip
pip3 install tqdm
cd /workspace/src/socc_icp
```

Then run (processes both sequences):
```bash
python3 -m scripts.prepare_ground_challenge
```

Once done, exit the container:
```bash
exit
```

The script creates the following structure, ready to be used by SOCC-ICP:
```
/home/ground-challenge/
├── corridor1/
│   ├── clouds_with_times/
│   │   ├── cloud_<secs>_<nsecs>.npy
│   │   └── ...
│   └── poses_gt.txt
└── corridor2/
    ├── clouds_with_times/
    │   └── ...
    └── poses_gt.txt
```

Note: the first 3 clouds of each sequence do not have a matching GT pose and must be deleted after extraction.

## SubT MRS

Download from the [SubT-MRS dataset website](https://superodometry.com/iccv23_challenge_LiI). The single sequence used is `long_corridor`.

SOCC-ICP expects the following structure (only the listed files are required):
```
/home/subt-mrs/
└── long_corridor/
    ├── clouds/
    │   ├── 0.las
    │   └── ...
    ├── ground_truth_path.csv
    └── cloud_timestamps.txt
```


## Newer College Dataset

Download from the [Newer College Dataset website](https://ori-drs.github.io/newer-college-dataset/). Two sequences are used: `01_short_experiment` (available as pre-extracted PCD files) and `02_long_experiment` (distributed as rosbags, requires extraction).

For `02_long_experiment`, place the downloaded rosbags and ground truth poses as follows before running the extraction script:
```
/home/newer_college/
├── 01_short_experiment/        ← pre-extracted, no further steps needed
│   ├── raw_format/
│   │   └── ouster_scan/
│   │       ├── cloud_<secs>_<nsecs>.pcd
│   │       └── ...
│   └── ground_truth/
│       └── registered_poses.csv
└── 02_long_experiment/
    ├── ground_truth/
    │   └── registered_poses.csv
    └── rosbags/
        └── *.bag
```

The PCD files for `02_long_experiment` must be extracted from the rosbags using `scripts/prepare_newer_college.py`. This script uses `rosbag` and therefore requires ROS Noetic. A Docker container is the easiest way to get it:

Run the following from the repository root (`socc-icp/`):
```bash
docker pull ros:noetic

docker run -it --name ros_noetic_newer_college \
  --net=host \
  -v $(pwd):/workspace \
  -v /home/newer_college:/home/newer_college \
  ros:noetic bash
```

Inside the container, install dependencies and run the script (no script edits needed):
```bash
source /opt/ros/noetic/setup.bash
apt update && apt install -y python3-pip libgl1-mesa-glx libglib2.0-0
pip3 install natsort pyquaternion open3d tqdm
cd /workspace/src/socc_icp
python3 -m scripts.prepare_newer_college
```

Once done, exit the container:
```bash
exit
```

The script creates `02_long_experiment/raw_format/ouster_scan/`, resulting in the following structure ready to be used by SOCC-ICP:
```
/home/newer_college/
├── 01_short_experiment/
│   ├── raw_format/
│   │   └── ouster_scan/
│   │       ├── cloud_<secs>_<nsecs>.pcd
│   │       └── ...
│   └── ground_truth/
│       └── registered_poses.csv
└── 02_long_experiment/
    ├── raw_format/
    │   └── ouster_scan/
    │       ├── cloud_<secs>_<nsecs>.pcd
    │       └── ...
    ├── ground_truth/
    │   └── registered_poses.csv
    └── rosbags/
        └── *.bag
```