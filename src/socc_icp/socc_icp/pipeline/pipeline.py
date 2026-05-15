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
import contextlib
import os
import time
from datetime import datetime

import numpy as np
import rclpy
import yaml
from kiss_icp.metrics import absolute_trajectory_error, sequence_error
from kiss_icp.tools.pipeline_results import PipelineResults
from kiss_icp.tools.progress_bar import get_progress_bar
from pyquaternion import Quaternion
from run.eval import run_evaluation
from sophus_pybind import SE3

from radix_clients import ChunkClientGaussian
from radix_clients.publish_client import PublishClient
from socc_icp.config.config import SoccIcpConfig
from socc_icp.dataset.kitti import Dataset
from socc_icp.deskewing_cpp import sophus_deskewer
from socc_icp.pipeline.socc_icp import SoccIcp
from socc_icp.util.radix import (
    kill_radix_servers,
    request_chunk,
    start_radix_server,
)
from socc_icp.util.statistics import write_and_log
from socc_icp.util.transform_msg import se3_to_transform

from ..metrics.metrics import rotation_error, translation_error


class MapPoints:
    def __init__(self, points: np.ndarray):
        self.points = points

    def point_cloud(self) -> np.ndarray:
        """Returns the point cloud as a numpy array."""
        return self.points


class OdometryPipeline:
    def __init__(
        self,
        dataset: Dataset,
        config_dict: dict,
        config: SoccIcpConfig,
        log_dir: str = "log/",
    ):
        # seed
        np.random.seed(config.data_params.seed)

        # dataset
        self._dataset = dataset
        self._jump = config.data_params.jump
        self._first = config.data_params.jump
        self._n_scans = (
            len(self._dataset) - self._jump
            if config.data_params.n_scans == -1
            else min(len(self._dataset) - self._jump, config.data_params.n_scans)
        )
        self._last = self._jump + self._n_scans

        # config and output dir
        self.config_dict = config_dict
        self.config = config
        self.log_dir = log_dir
        self.results_dir: str = ""

        # Pipeline
        self.odometry = SoccIcp(config=self.config)
        self.results = PipelineResults()
        self.times_total = np.zeros(self._n_scans)
        self.poses = np.zeros((self._n_scans, 4, 4))
        self.has_gt = hasattr(self._dataset, "gt_poses")
        self.gt_poses = (
            self._dataset.gt_poses[self._first : self._last] if self.has_gt else None
        )
        self.dataset_name = self._dataset.__class__.__name__
        assert hasattr(self._dataset, "sequence_id")
        self.dataset_sequence = self._dataset.sequence_id

    # Public interface  ------
    def run(self):
        self._create_output_dir()
        self._run_pipeline()
        self._run_evaluation()
        self._write_result_poses()
        self._write_gt_poses()
        self._write_cfg()
        self._write_log()
        # custom evaluation
        run_evaluation(self.results_dir, alignment=None)  # "scale_7dof"
        return self.results

    # Private interface  ------
    def _run_pipeline(self):
        # ----- stop Radix servers that might be running -----
        kill_radix_servers(sleep_time=1.0)

        # ----- get Radix server running -----
        start_radix_server(sleep_time=1.0)

        # ----- create SLAM client node -----
        rclpy.init()
        publish_client = PublishClient(
            pc_topic=self.config.slam_client_params.radix_topic,
            log=False,
        )
        chunk_client = ChunkClientGaussian(log=False)
        time.sleep(2.0)

        # ----- run pipeline -----
        T_pred = SE3()
        deskewer_c = sophus_deskewer.Deskewer(
            mode=self.config.general_params.deskewing_mode
        )

        for idx in get_progress_bar(self._first, self._last):
            # pc_raw ((N, 3) of type float32) and frame_labels ((N,) of type uint32) are guaranteed to be not None
            # timestamps ((N,) of type float32) can be None
            frame_pc_raw, frame_labels, frame_stamps = self._dataset[idx]
            # if we explicetly do not want to make use of label information (e.g. KITTI without semantics)
            # set labels to 0
            if not self.config.general_params.use_labels:
                frame_labels.fill(np.uint32(0))

            # empty frame issue for some MulRan sequences towards the end, simply skip
            if len(frame_pc_raw) == 0:
                msg = f"Encountered empty point cloud at index {idx}, skipping frame and stopping pipeline."
                print(msg)
                break

            # perform de-skewing
            time_start = time.perf_counter_ns()
            if (
                frame_stamps is not None
                and len(self.odometry.poses) >= 2
                and self.config.general_params.deskewing_mode != "none"
            ):
                if frame_stamps.shape[0] == 0:
                    # final frame of MulRan may be corrupted, simply skip de-skewing in this case
                    msg = f"Encountered empty (but not None) timestamps for de-skewing at index {idx}, skipping de-skewing."
                    print(msg)
                else:
                    frame_pc_raw = deskewer_c.deskew_scan(
                        points=frame_pc_raw,
                        timestamps=frame_stamps,
                        pose=self.odometry.get_prediction_model().to_matrix(),
                    ).astype(np.float32)
            time_deskewing = time.perf_counter_ns() - time_start

            # map labels
            label_mapping: dict[int, int] | None = self.config.label_mapping
            if label_mapping is not None:
                frame_labels = np.vectorize(
                    lambda x: label_mapping.get(x, x), otypes=[np.uint32]
                )(frame_labels)

            # [Radix] get chunk
            time_start = time.perf_counter_ns()
            grid_chunk = request_chunk(
                T=T_pred,
                chunk_client=chunk_client,
                chunk_range=self.config.slam_client_params.chunk_range,
                chunk_delete_rest=self.config.slam_client_params.chunk_delete_rest,
                max_distance=self.config.registration_params.max_distance,
            )
            time_radix_chunk = time.perf_counter_ns() - time_start

            # [Localization] find transformation
            time_start = time.perf_counter_ns()
            (
                T_pred,
                pc_filtered,
                labels_filtered,
                statistics,
            ) = self.odometry.register_frame(grid_chunk, frame_pc_raw, frame_labels)
            self.poses[idx - self._first] = self.odometry.poses[-1].to_matrix()
            time_registration = time.perf_counter_ns() - time_start

            # [Radix] publish tf and point cloud
            time_start = time.perf_counter_ns()
            ack_received = publish_client.publish_tf_pc(
                points=pc_filtered,
                labels=labels_filtered,
                frame_id_header="map",
                frame_id_child="pub_client_frame",
                transform=se3_to_transform(T_pred),
                batch_size=30_000,
                batch_timeout=5.0,
                batch_by_angle=True,
            )
            if not ack_received:
                raise TimeoutError(
                    "Timeout while waiting for ACK from Radix after 3 seconds"
                )
            time_radix_insert = time.perf_counter_ns() - time_start

            # [Timing]
            self.times_total[idx - self._first] = (
                time_registration
                + time_radix_chunk
                + time_radix_insert
                + time_deskewing
            )

            # [Logging]
            statistics["time_deskewing"] = time_deskewing * 1e-9
            statistics["time_registration"] = time_registration * 1e-9
            statistics["time_radix_chunk"] = time_radix_chunk * 1e-9
            statistics["time_radix_insert"] = time_radix_insert * 1e-9

            statistics["chunk_size"] = grid_chunk.n
            # pred
            T_pred = SE3.from_matrix(self.poses[idx - self._first])
            statistics["T_pred"] = T_pred
            # gt and errors
            if self.has_gt and self.gt_poses is not None:
                if (idx - self._first) >= self.gt_poses.shape[0]:
                    # break if we run out of GT poses
                    print("Ran out of GT poses, stopping evaluation")
                    break

                T_true = SE3.from_matrix(self.gt_poses[idx - self._first])
                statistics["T_true"] = T_true
                statistics["rotation_error_deg"] = rotation_error(T_pred, T_true)
                statistics["translation_error"] = translation_error(T_pred, T_true)

            write_and_log(statistics, self.results_dir, idx)

        # ----- stop SLAM client node -----
        rclpy.shutdown()
        time.sleep(2.0)

        # ----- stop Radix server -----
        kill_radix_servers(sleep_time=2.5)

    @staticmethod
    def save_poses_kitti_format(filename: str, poses: np.ndarray):
        def _to_kitti_format(poses: np.ndarray) -> np.ndarray:
            return poses[:, :3].reshape(-1, 12)

        np.savetxt(fname=f"{filename}_kitti.txt", X=_to_kitti_format(poses))

    @staticmethod
    def save_poses_tum_format(filename, poses, timestamps):
        def _to_tum_format(poses, timestamps):
            tum_data = np.zeros((len(poses), 8))
            with contextlib.suppress(ValueError):
                for idx in range(len(poses)):
                    tx, ty, tz = poses[idx, :3, -1].flatten()
                    qw, qx, qy, qz = Quaternion(matrix=poses[idx], atol=0.01).elements
                    tum_data[idx] = np.r_[
                        float(timestamps[idx]), tx, ty, tz, qx, qy, qz, qw
                    ]
                tum_data.flatten()
                return tum_data.astype(np.float64)

        np.savetxt(
            fname=f"{filename}_tum.txt",
            X=_to_tum_format(poses, timestamps),  # type: ignore
            fmt="%.4f",
        )

    def _calibrate_poses(self, poses):
        return (
            self._dataset._apply_calibration(poses)
            if hasattr(self._dataset, "apply_calibration")
            else poses
        )

    def _get_frames_timestamps(self):
        return (
            self._dataset._get_frames_timestamps()
            if hasattr(self._dataset, "get_frames_timestamps")
            else np.arange(0, self._n_scans, 1.0)
        )

    def _save_poses(self, filename: str, poses, timestamps):
        np.save(filename, poses)
        self.save_poses_kitti_format(filename, poses)
        # self.save_poses_tum_format(filename, poses, timestamps)

    def _write_result_poses(self):
        self._save_poses(
            filename=f"{self.results_dir}/{self.dataset_sequence}_poses",
            poses=self._calibrate_poses(self.poses),
            timestamps=self._get_frames_timestamps(),
        )

    def _write_gt_poses(self):
        if not self.has_gt:
            return
        self._save_poses(
            filename=f"{self.results_dir}/{self._dataset.sequence_id}_gt",
            poses=self._calibrate_poses(self.gt_poses),
            timestamps=self._get_frames_timestamps(),
        )

    def _get_fps(self):
        times_nozero = self.times_total[self.times_total != 0]
        total_time_s = np.sum(times_nozero) * 1e-9
        return float(times_nozero.shape[0] / total_time_s) if total_time_s > 0 else 0

    def _run_evaluation(self):
        # Run estimation metrics evaluation, only when GT data was provided
        if self.has_gt and self.gt_poses is not None:
            avg_tra, avg_rot = sequence_error(self.gt_poses, self.poses)
            ate_rot, ate_trans = absolute_trajectory_error(self.gt_poses, self.poses)
            self.results.append(
                desc="Average Translation Error", units="%", value=avg_tra
            )
            self.results.append(
                desc="Average Rotational Error", units="deg/m", value=avg_rot
            )
            self.results.append(
                desc="Absolute Trajectory Error (ATE)", units="m", value=ate_trans
            )
            self.results.append(
                desc="Absolute Rotational Error (ARE)", units="rad", value=ate_rot
            )

        # Run timing metrics evaluation, always
        fps = self._get_fps()
        avg_fps = round(fps, 3)
        avg_ms = int(np.ceil(1e3 / fps)) if fps > 0 else 0
        if avg_fps > 0:
            self.results.append(
                desc="Average Frequency", units="Hz", value=avg_fps, trunc=False
            )
            self.results.append(
                desc="Average Runtime", units="ms", value=avg_ms, trunc=True
            )

    def _write_log(self):
        if not self.results.empty():
            self.results.log_to_file(
                f"{self.results_dir}/result_metrics.log",
                f"Results for {self.dataset_name} Sequence {self.dataset_sequence}",
            )

    def _write_cfg(self):
        filename = os.path.join(self.results_dir, "config.yaml")
        with open(filename, "w") as f:
            yaml.dump(self.config_dict, f)

    def _create_output_dir(self):
        run_id = (
            datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            + f"_sequ_{self._dataset.sequence_id}"
        )
        self.results_dir = os.path.join(self.log_dir, run_id)
        os.makedirs(self.results_dir, exist_ok=True)
