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
import re

import numpy as np
from pyquaternion import Quaternion

from socc_icp.dataset.dataset import Dataset


class GroundChallengeDataset(Dataset):
    def __init__(self, data_dir: str, *_, **__):
        try:
            self.PyntCloud = importlib.import_module("pyntcloud").PyntCloud
        except ModuleNotFoundError:
            print('Ground Challenge requires pnytccloud: "pip install pyntcloud"')

        self.scan_folder = os.path.join(data_dir, "clouds_with_times")
        self.pose_file = os.path.join(data_dir, "poses_gt.txt")
        self.sequence_id = os.path.basename(data_dir)

        # Load scan files and poses
        self.scan_files = self.get_pcd_filenames(self.scan_folder)
        self.gt_poses = self.load_gt_poses(self.pose_file)

    def __len__(self):
        return len(self.scan_files)

    def __getitem__(self, idx) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        file_path = os.path.join(self.scan_folder, self.scan_files[idx])
        points, timestamps = self._read_point_cloud(file_path)
        labels = np.zeros((points.shape[0],), dtype=np.uint32)  # unlabeled = 0

        return points, labels, timestamps

    def _read_point_cloud(self, scan_file: str) -> tuple[np.ndarray, np.ndarray]:
        combined = np.load(scan_file)
        points = combined[:, :3]
        times = combined[:, 3]

        return points.astype(np.float32), times.astype(np.float32)

    @staticmethod
    def get_pcd_filenames(scans_folder):
        # cloud_1583836591_182590976.pcd
        regex = re.compile("^cloud_(\d*_\d*)")

        def get_cloud_timestamp(pcd_filename):
            m = regex.search(pcd_filename)
            secs, nsecs = m.groups()[0].split("_")
            return int(secs) * int(1e9) + int(nsecs)

        return sorted(os.listdir(scans_folder), key=get_cloud_timestamp)

    @staticmethod
    def load_gt_poses(file_path: str):
        """
        Load ground truth poses from a text file where each line contains:
        timestamp x y z qx qy qz qw
        """
        ground_truth = np.genfromtxt(str(file_path), delimiter=" ", dtype=np.float64)
        xyz = ground_truth[:, 1:4]
        rotations = np.array(
            [
                Quaternion(x=qx, y=qy, z=qz, w=qw).rotation_matrix
                for qw, qx, qy, qz in ground_truth[:, 4:8]
            ]
        )

        num_poses = rotations.shape[0]
        poses = np.eye(4, dtype=np.float64).reshape(1, 4, 4).repeat(num_poses, axis=0)
        poses[:, :3, :3] = rotations
        poses[:, :3, 3] = xyz

        return poses
