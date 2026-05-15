# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
# Copyright (c) 2025 Johannes Scherer.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to mse, copy, modify, merge, publish, distribute, sublicense, and/or sell
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
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE msE OR OTHER DEALINGS IN THE
# SOFTWARE.
import glob
import os

import numpy as np

from socc_icp.dataset.dataset import Dataset


class SemanticKITTIDataset(Dataset):
    def __init__(
        self, data_dir: str, use_predicted_labels: bool, sequence: int, *_, **__
    ):
        if not isinstance(sequence, int) or not (0 <= sequence <= 10):
            raise ValueError("sequence must be an integer between 0 and 10")

        self.sequence_id = str(sequence).zfill(2)
        self.kitti_sequence_dir = os.path.join(data_dir, "sequences", self.sequence_id)
        self.velodyne_dir = os.path.join(self.kitti_sequence_dir, "velodyne/")
        if use_predicted_labels:
            self.label_dir = os.path.join(self.kitti_sequence_dir, "labels_lsk3dnet/")
        else:
            self.label_dir = os.path.join(self.kitti_sequence_dir, "labels/")
        self.calibration = self._read_calib_file(
            os.path.join(self.kitti_sequence_dir, "calib.txt")
        )
        self.scan_files = sorted(glob.glob(self.velodyne_dir + "*.bin"))
        self.label_files = sorted(glob.glob(self.label_dir + "*.label"))

        assert len(self.scan_files) == len(self.label_files), (
            f"Number of scans ({len(self.scan_files)}) and labels ({len(self.label_files)}) do not match."
        )
        assert len(self.scan_files) > 0, "No scans found in the specified directory."

        # Load GT Poses (if available)
        if int(sequence) < 11:
            self.poses_fn = os.path.join(data_dir, f"poses/{self.sequence_id}.txt")
            self.gt_poses = self._load_poses(self.poses_fn)
        else:
            raise NotImplementedError("sequence number > 10 not implemented yet")

        # Add correction for KITTI datasets, can be easilty removed if unwanted
        from kiss_icp.pybind import kiss_icp_pybind

        self.correct_kitti_scan = lambda frame: np.asarray(
            kiss_icp_pybind._correct_kitti_scan(kiss_icp_pybind._Vector3dVector(frame))
        )

    def __getitem__(self, idx) -> tuple[np.ndarray, np.ndarray, None]:
        points, labels = self._scans(idx)
        timestamps = None
        return points, labels, timestamps

    def __len__(self):
        return len(self.scan_files)

    def _scans(self, idx):
        pc = self._read_point_cloud(self.scan_files[idx])  # np.float32
        labels = self._read_labels(self.label_files[idx])  # np.uint8
        assert pc.shape[0] == labels.shape[0], (
            f"Point cloud ({pc.shape[0]}) and labels ({labels.shape[0]}) do not match."
        )
        return pc, labels

    def _apply_calibration(self, poses: np.ndarray) -> np.ndarray:
        """Converts from Velodyne to Camera Frame"""
        Tr = np.eye(4, dtype=np.float64)
        Tr[:3, :4] = self.calibration["Tr"].reshape(3, 4)
        return Tr @ poses @ np.linalg.inv(Tr)

    def _read_point_cloud(self, scan_file: str):
        points = np.fromfile(scan_file, dtype=np.float32)
        points = points.reshape((-1, 4))[:, :3]
        points = self.correct_kitti_scan(points)  # converts to np.float64
        points = points.astype(np.float32)
        return points

    def _read_labels(self, label_file: str):
        labels = np.fromfile(label_file, dtype=np.uint32)
        labels = labels & 0xFFFF  # extract semantic label (lower 16 bits)
        labels = labels.astype(np.uint32)  # convert to uint32
        return labels

    def _load_poses(self, poses_file):
        def _lidar_pose_gt(poses_gt):
            _tr = self.calibration["Tr"].reshape(3, 4)
            tr = np.eye(4, dtype=np.float64)
            tr[:3, :4] = _tr
            left = np.einsum("...ij,...jk->...ik", np.linalg.inv(tr), poses_gt)
            right = np.einsum("...ij,...jk->...ik", left, tr)
            return right

        poses = np.loadtxt(poses_file, delimiter=" ")
        n = poses.shape[0]
        poses = np.concatenate(
            (
                poses,
                np.zeros((n, 3), dtype=np.float32),
                np.ones((n, 1), dtype=np.float32),
            ),
            axis=1,
        )
        poses = poses.reshape((n, 4, 4))  # [N, 4, 4]
        return _lidar_pose_gt(poses)

    def _get_frames_timestamps(self) -> np.ndarray:
        timestamps = np.loadtxt(
            os.path.join(self.kitti_sequence_dir, "times.txt")
        ).reshape(-1, 1)
        return timestamps

    @staticmethod
    def _read_calib_file(file_path: str) -> dict:
        calib_dict = {}
        with open(file_path, "r") as calib_file:
            for line in calib_file.readlines():
                tokens = line.split(" ")
                if tokens[0] == "calib_time:":
                    continue
                # Only read with float data
                if len(tokens) > 0:
                    values = [float(token) for token in tokens[1:]]
                    values = np.array(values, dtype=np.float32)

                    # The format in KITTI's file is <key>: <f1> <f2> <f3> ...\n -> Remove the ':'
                    key = tokens[0][:-1]
                    calib_dict[key] = values
        return calib_dict
