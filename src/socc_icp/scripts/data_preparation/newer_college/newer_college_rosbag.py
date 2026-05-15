import os
from pathlib import Path

import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2
from natsort import natsorted
from pyquaternion import Quaternion


class NewerCollegeRosbag:
    def __init__(self, data_dir: Path):
        self.data_source = os.path.join(data_dir, "")
        self.pose_file = os.path.join(
            self.data_source, "ground_truth/registered_poses.csv"
        )
        self.gt_poses = self.load_gt_poses(self.pose_file)
        self.sequence_id = "02_long_experiment"

        # process rosbag-files
        print("Processing rosbag files (this will take some time)")
        self.topic = "/os1_cloud_node/points"
        self.rosbag_dir = os.path.join(data_dir, "rosbags")
        self.rosbag_files = natsorted(
            [
                os.path.join(self.rosbag_dir, f)
                for f in os.listdir(self.rosbag_dir)
                if f.endswith(".bag")
            ]
        )
        self.bags = []
        self.n_scans = 0
        for rosbag_file in self.rosbag_files:
            bag = rosbag.Bag(rosbag_file, mode="r")
            self.bags.append(bag)
            self.n_scans += bag.get_message_count(topic_filters=self.topic)
        self.bag = self.bags.pop(0)
        self.msgs = self.bag.read_messages(topics=[self.topic])
        print(f"Now processing {self.bag.filename}")

    def __len__(self):
        return self.n_scans

    def __getitem__(self, idx):
        try:
            _, msg, _ = next(self.msgs)
        except StopIteration:
            self.bag.close()
            # new bagile
            self.bag = self.bags.pop(0)
            self.msgs = self.bag.read_messages(topics=[self.topic])
            print(f"Now processing {self.bag.filename}")
            _, msg, _ = next(self.msgs)

        points = np.array(list(pc2.read_points(msg, field_names=["x", "y", "z"])))
        return points.astype(np.float64)

    @staticmethod
    def load_gt_poses(file_path: str):
        """Taken from pyLiDAR-SLAM/blob/master/slam/dataset/nhcd_dataset.py"""
        ground_truth_df = np.genfromtxt(str(file_path), delimiter=",", dtype=np.float64)
        xyz = ground_truth_df[:, 2:5]
        rotations = np.array(
            [
                Quaternion(x=x, y=y, z=z, w=w).rotation_matrix
                for x, y, z, w in ground_truth_df[:, 5:]
            ]
        )

        num_poses = rotations.shape[0]
        poses = np.eye(4, dtype=np.float64).reshape(1, 4, 4).repeat(num_poses, axis=0)
        poses[:, :3, :3] = rotations
        poses[:, :3, 3] = xyz

        T_CL = np.eye(4, dtype=np.float32)
        T_CL[:3, :3] = Quaternion(x=0, y=0, z=0.924, w=0.383).rotation_matrix
        T_CL[:3, 3] = np.array([-0.084, -0.025, 0.050], dtype=np.float32)
        poses = np.einsum("nij,jk->nik", poses, T_CL)
        poses = np.einsum("ij,njk->nik", np.linalg.inv(poses[0]), poses)
        return poses
