# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
# Copyright (c) 2025 Johannes Scherer.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import importlib
import os

import numpy as np
from pyquaternion import Quaternion

from socc_icp.dataset.dataset import Dataset
import laspy


class SubTMRSDataset(Dataset):
    def __init__(self, data_dir: str, *_, **__):
        try:
            self.PyntCloud = importlib.import_module("pyntcloud").PyntCloud
        except ModuleNotFoundError:
            print('Ground Challenge requires pnytccloud: "pip install pyntcloud"')

        self.scan_folder = os.path.join(data_dir, "clouds")
        self.pose_file = os.path.join(data_dir, "ground_truth_path.csv")
        self.scan_stamps_file = os.path.join(data_dir, "cloud_timestamps.txt")
        self.sequence_id = os.path.basename(data_dir)

        # Load scan files and poses
        self.scan_files = self.get_pcd_filenames(self.scan_folder)
        self.scan_files_timestamps = self.load_scan_timestamps(self.scan_stamps_file)
        self.gt_poses, self.gt_poses_timestamps = self.load_gt_poses(self.pose_file)

        # stamps look like this
        # (1517157221, 4338980)
        # (1517157221, 105199099)
        # (1517157221, 206058979)
        # self.gt_poses, self.gt_poses_timestamps contains only half of the data compared to scans
        # therefore,

        # make sure that for each scan and scan_stamp we have a gt pose
        assert len(self.scan_files) == len(self.scan_files_timestamps)

        gt_poses_new = []
        gt_poses_timestamps_new = []
        for scan_sec, scan_nanosec in self.scan_files_timestamps:
            # Find the closest (sec, nanosec) tuple in self.gt_poses_timestamps
            min_diff = float("inf")
            min_idx = -1
            for i, (gt_sec, gt_nanosec) in enumerate(self.gt_poses_timestamps):
                diff = abs((scan_sec - gt_sec) * 1e9 + (scan_nanosec - gt_nanosec))
                if diff < min_diff:
                    min_diff = diff
                    min_idx = i

            # If the closest timestamp is within 0.01 seconds, use it; otherwise, append (None, None)
            if min_diff < 1e3:  # max. 1,000 nanoseconds difference
                gt_poses_new.append(self.gt_poses[min_idx])
                gt_poses_timestamps_new.append(self.gt_poses_timestamps[min_idx])
            else:
                gt_poses_new.append(np.zeros((4, 4), dtype=np.float64))
                gt_poses_timestamps_new.append((None, None))

        n_pc_stamp_corr = len([e for e, n in gt_poses_timestamps_new if e is not None])
        print(f"# gt_poses with LiDAR correspondence: {n_pc_stamp_corr}")
        print(f"n_scans: {len(self.scan_files_timestamps)}")
        print(f"n_gt_poses: {len(self.gt_poses)}")
        assert n_pc_stamp_corr == len(self.gt_poses), (
            f"Expected {len(self.gt_poses)} gt_poses stamps to match lidar stamps, but only matched {n_pc_stamp_corr}"
        )
        self.gt_poses = np.array(gt_poses_new)
        self.gt_poses_timestamps = gt_poses_timestamps_new

    def __len__(self):
        return len(self.scan_files)

    def __getitem__(self, idx) -> tuple[np.ndarray, np.ndarray, None]:
        file_path = os.path.join(self.scan_folder, self.scan_files[idx])
        points = self._read_point_cloud(file_path)
        labels = np.zeros((points.shape[0],), dtype=np.uint32)  # no labels
        timestamps = None
        return points, labels, timestamps

    def _read_point_cloud(self, scan_file: str) -> np.ndarray:
        with laspy.open(scan_file) as lasf:
            las = lasf.read()
            points = np.vstack((las.x, las.y, las.z)).T

        return points.astype(np.float32)

    @staticmethod
    def load_scan_timestamps(scan_stamps_file):
        with open(scan_stamps_file, "r") as f:
            timestamps = [int(line.strip()) for line in f if line.strip()]
        secs_nanosecs = [(ts // 10**9, ts % 10**9) for ts in timestamps]
        return secs_nanosecs

    @staticmethod
    def get_pcd_filenames(scans_folder):
        # Return all .las files sorted numerically by filename
        files = [f for f in os.listdir(scans_folder) if f.endswith(".las")]
        return sorted(files, key=lambda x: int(os.path.splitext(x)[0]))

    @staticmethod
    def load_gt_poses(file_path: str):
        """
        Load ground truth poses from a CSV file with columns:
        timestamp, p_w_b_x, p_w_b_y, p_w_b_z, q_w_b_x, q_w_b_y, q_w_b_z, q_w_b_w
        Returns:
            poses: (N, 4, 4) numpy array of SE(3) matrices
            timestamps: list of int
        """
        data = np.genfromtxt(str(file_path), delimiter=",", skip_header=1)
        # Split timestamps into (secs, nsecs) tuples
        raw_timestamps = data[:, 0].astype(np.int64)
        secs_nanosecs = [(ts // 10**9, ts % 10**9) for ts in raw_timestamps]

        # poses
        xyz = data[:, 1:4]
        rotations = np.array(
            [
                Quaternion(x=row[4], y=row[5], z=row[6], w=row[7]).rotation_matrix
                for row in data
            ]
        )
        num_poses = rotations.shape[0]
        poses = np.eye(4, dtype=np.float64).reshape(1, 4, 4).repeat(num_poses, axis=0)
        poses[:, :3, :3] = rotations
        poses[:, :3, 3] = xyz

        return poses, secs_nanosecs
