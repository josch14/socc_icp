# Copyright (C) Huangying Zhan 2019. All rights reserved.
# Copyright (C) 2025 Johannes Scherer.

# Implementation based on https://github.com/Huangying-Zhan/kitti-odom-eval.

import copy
import os
from glob import glob
from typing import Any, Dict, List, Tuple, Optional

import numpy as np
from matplotlib import pyplot as plt


def scale_lse_solver(X: np.ndarray, Y: np.ndarray) -> float:
    """Compute optimal scaling factor minimizing least-squares error between scaled X and Y."""
    return np.sum(X * Y) / np.sum(X**2)


def umeyama_alignment(
    x: np.ndarray, y: np.ndarray, with_scale: bool = False
) -> Tuple[np.ndarray, np.ndarray, float]:
    """Least squares Sim(m) matrix minimizing distance between two point sets."""
    if x.shape != y.shape:
        raise ValueError("Input shapes of x and y must match.")
    m, n = x.shape
    mean_x, mean_y = x.mean(axis=1), y.mean(axis=1)
    sigma_x = np.sum((x - mean_x[:, None]) ** 2) / n
    cov_xy = (y - mean_y[:, None]) @ (x - mean_x[:, None]).T / n
    u, d, v = np.linalg.svd(cov_xy)
    s = np.eye(m)
    if np.linalg.det(u) * np.linalg.det(v) < 0:
        s[-1, -1] = -1
    r = u @ s @ v
    c = np.trace(np.diag(d) @ s) / sigma_x if with_scale else 1.0
    t = mean_y - c * r @ mean_x
    return r, t, c


class KittiEvalOdom:
    def __init__(self):
        self.lengths = [100 * i for i in range(1, 9)]

    @staticmethod
    def load_poses_from_txt(file_name: str) -> Dict[int, np.ndarray]:
        """Load poses from KITTI format txt file."""
        poses = {}
        with open(file_name, "r") as f:
            for cnt, line in enumerate(f):
                vals = [float(i) for i in line.split() if i]
                with_idx = len(vals) == 13
                idx = int(vals[0]) if with_idx else cnt
                pose = np.eye(4)
                pose[:3, :4] = np.array(vals[with_idx:]).reshape(3, 4)
                poses[idx] = pose
        return poses

    def trajectory_distances(self, poses: Dict[int, np.ndarray]) -> List[float]:
        """Compute cumulative distances for each pose relative to the first frame."""
        dist = [0]
        idxs = sorted(poses.keys())
        for i in range(len(idxs) - 1):
            P1, P2 = poses[idxs[i]], poses[idxs[i + 1]]
            dist.append(dist[-1] + np.linalg.norm(P1[:3, 3] - P2[:3, 3]))
        return dist

    @staticmethod
    def rotation_error(pose_error: np.ndarray) -> float:
        """Compute rotation error from relative pose error matrix."""
        trace = np.trace(pose_error[:3, :3])
        d = 0.5 * (trace - 1.0)
        return np.arccos(np.clip(d, -1.0, 1.0))

    @staticmethod
    def translation_error(pose_error: np.ndarray) -> float:
        """Compute translation error from relative pose error matrix."""
        return np.linalg.norm(pose_error[:3, 3])

    def last_frame_from_segment_length(
        self, dist: List[float], first_frame: int, length: float
    ) -> int:
        """Find index of last frame at least specified distance from first frame."""
        target = dist[first_frame] + length
        for i in range(first_frame, len(dist)):
            if dist[i] > target:
                return i
        return -1

    def calc_sequence_errors(
        self, poses_gt: Dict[int, np.ndarray], poses_result: Dict[int, np.ndarray]
    ) -> List[List[float]]:
        """Calculate sequence errors for trajectory evaluation."""
        errors = []
        distances = self.trajectory_distances(poses_gt)
        step_size = 10
        idxs = sorted(poses_gt.keys())
        for first_frame in range(0, len(idxs), step_size):
            for length in self.lengths:
                last_frame = self.last_frame_from_segment_length(
                    distances, first_frame, length
                )
                if (
                    last_frame == -1
                    or last_frame not in poses_result
                    or first_frame not in poses_result
                ):
                    continue
                pose_delta_gt = (
                    np.linalg.inv(poses_gt[first_frame]) @ poses_gt[last_frame]
                )
                pose_delta_result = (
                    np.linalg.inv(poses_result[first_frame]) @ poses_result[last_frame]
                )
                pose_error = np.linalg.inv(pose_delta_result) @ pose_delta_gt
                rot_err = self.rotation_error(pose_error) / length
                trans_err = self.translation_error(pose_error) / length
                num_frames = last_frame - first_frame + 1.0
                speed = length / (0.1 * num_frames)
                errors.append([first_frame, rot_err, trans_err, length, speed])
        return errors

    @staticmethod
    def save_sequence_errors(errors: List[List[float]], file_name: str):
        """Save sequence errors to a file."""
        with open(file_name, "w") as fp:
            for error in errors:
                fp.write(" ".join(map(str, error)) + "\n")

    @staticmethod
    def compute_overall_err(seq_errors: List[List[float]]) -> Tuple[float, float]:
        """Compute average translation and rotation errors."""
        if not seq_errors:
            return 0.0, 0.0
        total_t_err = sum(error[2] for error in seq_errors)
        total_r_err = sum(error[1] for error in seq_errors)
        seq_len = len(seq_errors)
        return total_t_err / seq_len, total_r_err / seq_len

    def plot_trajectory(
        self,
        poses_gt: Dict[int, np.ndarray],
        poses_result: Dict[int, np.ndarray],
        seq: int,
        dir_out: str,
    ):
        """Plot trajectory for ground truth and predictions."""
        fig, ax = plt.subplots()
        for label, poses in zip(["Ground Truth", "Ours"], [poses_gt, poses_result]):
            positions = np.array([[pose[0, 3], pose[2, 3]] for pose in poses.values()])
            ax.plot(positions[:, 0], positions[:, 1], label=label)
        fontsize = 20
        ax.legend(loc="upper right", prop={"size": fontsize})
        ax.set_xlabel("x (m)", fontsize=fontsize)
        ax.set_ylabel("z (m)", fontsize=fontsize)
        ax.tick_params(axis="both", labelsize=fontsize)
        fig.set_size_inches(10, 10)
        os.makedirs(dir_out, exist_ok=True)
        fig.savefig(
            os.path.join(dir_out, f"sequence_{seq:02}.pdf"),
            bbox_inches="tight",
            pad_inches=0,
        )
        plt.close(fig)

    def plot_trajectory_mod(
        self,
        poses_pred: List[np.ndarray],
        poses_gt: List[np.ndarray],
        ax1: str,
        ax2: str,
        dir_out: str,
    ):
        """Plot trajectory for ground truth and predicted poses."""
        axis_map = {"x": 0, "y": 1, "z": 2}
        ax1_idx, ax2_idx = axis_map.get(ax1), axis_map.get(ax2)
        if ax1_idx is None or ax2_idx is None:
            raise ValueError("Invalid axis. Choose from 'x', 'y', or 'z'.")
        fig, ax = plt.subplots()
        xlim, ylim = [None, None], [None, None]
        for label, poses in zip(["Ground Truth", "Predicted"], [poses_gt, poses_pred]):
            positions = np.array(
                [[pose[ax1_idx, 3], pose[ax2_idx, 3]] for pose in poses]
            )
            ax.plot(positions[:, 0], positions[:, 1], label=label)
            xlim = [
                min(
                    xlim[0] if xlim[0] is not None else positions[:, 0].min(),
                    positions[:, 0].min(),
                ),
                max(
                    xlim[1] if xlim[1] is not None else positions[:, 0].max(),
                    positions[:, 0].max(),
                ),
            ]
            ylim = [
                min(
                    ylim[0] if ylim[0] is not None else positions[:, 1].min(),
                    positions[:, 1].min(),
                ),
                max(
                    ylim[1] if ylim[1] is not None else positions[:, 1].max(),
                    positions[:, 1].max(),
                ),
            ]
        xlim = [xlim[0] - 20, xlim[1] + 20]
        ylim = [ylim[0] - 20, ylim[1] + 20]
        ax.axhline(0, color="gray", linestyle="--", linewidth=0.8)
        ax.axvline(0, color="gray", linestyle="--", linewidth=0.8)
        fontsize = 20
        ax.legend(loc="upper right", prop={"size": fontsize})
        ax.set_xlabel(f"{ax1} (m)", fontsize=fontsize)
        ax.set_ylabel(f"{ax2} (m)", fontsize=fontsize)
        ax.tick_params(axis="both", labelsize=fontsize)
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        fig.set_size_inches(10, 10)
        os.makedirs(dir_out, exist_ok=True)
        fig.savefig(
            os.path.join(dir_out, f"trajectory_{ax1}_{ax2}.png"),
            bbox_inches="tight",
            pad_inches=0,
        )
        plt.close(fig)

    def plot_error(
        self,
        avg_segment_errs: Dict[int, List[float]],
        dir_out: str,
        seq: Optional[int] = None,
    ):
        """Plot per-length error for translation and rotation."""

        def plot_metric(metric_values, ylabel, filename_suffix, label):
            plot_x = self.lengths
            plot_y = [
                metric_values[len_][0] * 100 if len(metric_values[len_]) > 0 else 0
                for len_ in self.lengths
            ]
            fontsize = 10
            fig = plt.figure()
            plt.plot(plot_x, plot_y, "bs-", label=label)
            plt.ylabel(ylabel, fontsize=fontsize)
            plt.xlabel("Path Length (m)", fontsize=fontsize)
            plt.legend(loc="upper right", prop={"size": fontsize})
            fig.set_size_inches(5, 5)
            filename = (
                f"{dir_out}/{filename_suffix}_{seq:02}.pdf"
                if seq is not None
                else f"{dir_out}/{filename_suffix}.pdf"
            )
            plt.savefig(filename, bbox_inches="tight", pad_inches=0)
            plt.close(fig)

        plot_metric(
            avg_segment_errs, "Translation Error (%)", "trans_err", "Translation Error"
        )
        plot_metric(
            avg_segment_errs, "Rotation Error (deg/100m)", "rot_err", "Rotation Error"
        )

    def compute_segment_error(
        self, seq_errs: List[List[float]]
    ) -> Dict[int, List[float]]:
        """Calculate average errors for different segments."""
        segment_errs = {length: [] for length in self.lengths}
        for _, r_err, t_err, length, _ in seq_errs:
            segment_errs[length].append([t_err, r_err])
        avg_segment_errs = {
            length: [np.mean(errors, axis=0)[0], np.mean(errors, axis=0)[1]]
            if errors
            else []
            for length, errors in segment_errs.items()
        }
        return avg_segment_errs

    @staticmethod
    def compute_ATE(gt: Dict[int, np.ndarray], pred: Dict[int, np.ndarray]) -> float:
        """Compute RMSE of Absolute Trajectory Error (ATE)."""
        errors = [np.linalg.norm(gt[i][:3, 3] - pred[i][:3, 3]) for i in pred]
        return np.sqrt(np.mean(np.square(errors)))

    def compute_RPE(
        self, gt: Dict[int, np.ndarray], pred: Dict[int, np.ndarray]
    ) -> Tuple[float, float]:
        """Compute Relative Pose Error (RPE) for translation and rotation."""
        trans_errors, rot_errors = [], []
        idxs = list(pred.keys())
        for i in idxs[:-1]:
            gt_rel = np.linalg.inv(gt[i]) @ gt[i + 1]
            pred_rel = np.linalg.inv(pred[i]) @ pred[i + 1]
            rel_err = np.linalg.inv(gt_rel) @ pred_rel
            trans_errors.append(self.translation_error(rel_err))
            rot_errors.append(self.rotation_error(rel_err))
        return np.mean(trans_errors), np.mean(rot_errors)

    @staticmethod
    def scale_optimization(
        gt: Dict[int, np.ndarray], pred: Dict[int, np.ndarray]
    ) -> Dict[int, np.ndarray]:
        """Optimize scaling factor for predicted poses."""
        xyz_pred = np.array([pred[i][:3, 3] for i in pred])
        xyz_ref = np.array([gt[i][:3, 3] for i in gt])
        scale = scale_lse_solver(xyz_pred, xyz_ref)
        pred_updated = copy.deepcopy(pred)
        for i in pred_updated:
            pred_updated[i][:3, 3] *= scale
        return pred_updated

    @staticmethod
    def write_result(f, seq: int, errs: List[float]):
        """Write evaluation results to a file."""
        ave_t_err, ave_r_err, ate, rpe_trans, rpe_rot = errs
        f.writelines(
            [
                f"Sequence: \t {seq} \n",
                f"Trans. err. (%): \t {ave_t_err * 100:.3f} \n",
                f"Rot. err. (deg/100m): \t {ave_r_err / np.pi * 180 * 100:.3f} \n",
                f"ATE (m): \t {ate:.3f} \n",
                f"RPE (m): \t {rpe_trans:.3f} \n",
                f"RPE (deg): \t {rpe_rot * 180 / np.pi:.3f} \n\n",
            ]
        )

    def eval_full(
        self,
        dir_true: str,
        dir_pred: str,
        alignment: Optional[str] = None,
        seqs: Optional[List[int]] = None,
    ):
        """Evaluate required/available sequences."""
        seq_list = [f"{i:02}" for i in range(11)]
        results = {
            k: []
            for k in [
                "ave_t_errs",
                "ave_r_errs",
                "seq_ate",
                "seq_rpe_trans",
                "seq_rpe_rot",
            ]
        }
        dir_evaluation = os.path.join(dir_pred, "evaluation")
        dir_errors = os.path.join(dir_evaluation, "errors")
        dir_plot_trajectory = os.path.join(dir_evaluation, "plot_trajectory")
        dir_plot_error = os.path.join(dir_evaluation, "plot_error")
        for d in [dir_errors, dir_plot_trajectory, dir_plot_error]:
            os.makedirs(d, exist_ok=True)
        result_txt = os.path.join(dir_evaluation, "result.txt")
        with open(result_txt, "w") as f:
            if seqs is None:
                available_seqs = sorted(glob(os.path.join(dir_pred, "*.txt")))
                eval_seqs = [
                    int(os.path.basename(seq)[-6:-4])
                    for seq in available_seqs
                    if os.path.basename(seq)[-6:-4] in seq_list
                ]
            else:
                eval_seqs = seqs
            for seq in eval_seqs:
                seq_str = f"{seq:02}"
                file_name = f"{seq_str}.txt"
                poses_result = self.load_poses_from_txt(
                    os.path.join(dir_pred, file_name)
                )
                poses_gt = self.load_poses_from_txt(os.path.join(dir_true, file_name))
                idx_0 = sorted(poses_result.keys())[0]
                pred_0, gt_0 = poses_result[idx_0], poses_gt[idx_0]
                for cnt in poses_result:
                    poses_result[cnt] = np.linalg.inv(pred_0) @ poses_result[cnt]
                    poses_gt[cnt] = np.linalg.inv(gt_0) @ poses_gt[cnt]
                if alignment:
                    poses_result = KittiEvalOdom.apply_alignment(
                        poses_gt, poses_result, alignment
                    )
                seq_err = self.calc_sequence_errors(poses_gt, poses_result)
                self.save_sequence_errors(seq_err, os.path.join(dir_errors, file_name))
                avg_segment_errs = self.compute_segment_error(seq_err)
                ave_t_err, ave_r_err = self.compute_overall_err(seq_err)
                results["ave_t_errs"].append(ave_t_err)
                results["ave_r_errs"].append(ave_r_err)
                ate = self.compute_ATE(poses_gt, poses_result)
                results["seq_ate"].append(ate)
                rpe_trans, rpe_rot = self.compute_RPE(poses_gt, poses_result)
                results["seq_rpe_trans"].append(rpe_trans)
                results["seq_rpe_rot"].append(rpe_rot)
                print(f"Sequence: {seq}")
                print(f"Translational error (%): {ave_t_err * 100:.3f}")
                print(
                    f"Rotational error (deg/100m): {ave_r_err / np.pi * 180 * 100:.3f}"
                )
                print(f"ATE (m): {ate:.3f}")
                print(f"RPE (m): {rpe_trans:.3f}")
                print(f"RPE (deg): {rpe_rot * 180 / np.pi:.3f}")
                self.plot_trajectory(poses_gt, poses_result, seq, dir_plot_trajectory)
                self.plot_error(avg_segment_errs, dir_plot_error, seq)
                self.write_result(
                    f, seq, [ave_t_err, ave_r_err, ate, rpe_trans, rpe_rot]
                )

    def eval_sequence(self, log_dir: str, alignment: Optional[str] = None):
        """Evaluate a single sequence and save results."""
        dir_evaluation = os.path.join(log_dir, "evaluation")
        dir_errors = os.path.join(dir_evaluation, "errors")
        dir_plot_trajectory = os.path.join(dir_evaluation, "plot_trajectory")
        dir_plot_error = os.path.join(dir_evaluation, "plot_error")
        file_results_txt = os.path.join(dir_evaluation, "result.txt")
        for d in [dir_errors, dir_plot_trajectory, dir_plot_error]:
            os.makedirs(d, exist_ok=True)
        file_pred = os.path.join(log_dir, "statistics/T_pred.txt")
        file_true = os.path.join(log_dir, "statistics/T_true.txt")
        if not os.path.exists(file_pred) or not os.path.exists(file_true):
            print(f"File not found: {file_pred} or {file_true}")
            return
        poses_pred = self.load_poses_from_txt(file_pred)
        poses_true = self.load_poses_from_txt(file_true)
        assert len(poses_pred) == len(poses_true), (
            "Predicted and ground truth poses must have the same length."
        )
        idx_0 = sorted(poses_pred.keys())[0]
        pred_0, gt_0 = poses_pred[idx_0], poses_true[idx_0]
        for cnt in poses_pred:
            poses_pred[cnt] = np.linalg.inv(pred_0) @ poses_pred[cnt]
            poses_true[cnt] = np.linalg.inv(gt_0) @ poses_true[cnt]
        if alignment:
            poses_pred = KittiEvalOdom.apply_alignment(
                poses_true, poses_pred, alignment
            )
        seq_err = self.calc_sequence_errors(poses_true, poses_pred)
        self.save_sequence_errors(seq_err, os.path.join(dir_errors, "seq_err.txt"))
        avg_segment_errs = self.compute_segment_error(seq_err)
        # ave_t_err, ave_r_err = self.compute_overall_err(seq_err)
        # ate = self.compute_ATE(poses_true, poses_pred)
        # rpe_trans, rpe_rot = self.compute_RPE(poses_true, poses_pred)
        # print(f"Translational error (%): {ave_t_err * 100:.3f}")
        # print(f"Rotational error (deg/100m): {ave_r_err / np.pi * 180 * 100:.3f}")
        # print(f"ATE (m): {ate:.3f}")
        # print(f"RPE (m): {rpe_trans:.3f}")
        # print(f"RPE (deg): {rpe_rot * 180 / np.pi:.3f}")
        for ax1, ax2 in [("x", "y"), ("x", "z"), ("y", "z")]:
            self.plot_trajectory_mod(
                list(poses_pred.values()),
                list(poses_true.values()),
                ax1,
                ax2,
                dir_plot_trajectory,
            )
        self.plot_error(avg_segment_errs, dir_out=dir_plot_error)
        # with open(file_results_txt, "w") as f:
        #     self.write_result(f, 0, [ave_t_err, ave_r_err, ate, rpe_trans, rpe_rot])

    @staticmethod
    def apply_alignment(
        poses_gt: Dict[Any, np.ndarray],
        poses_pred: Dict[Any, np.ndarray],
        alignment: str,
    ) -> Dict[Any, np.ndarray]:
        """Apply alignment to predicted poses."""
        if alignment == "scale":
            return KittiEvalOdom.scale_optimization(poses_gt, poses_pred)
        if alignment in {"scale_7dof", "7dof", "6dof"}:
            xyz_gt = np.array([pose[:3, 3] for pose in poses_gt.values()]).T
            xyz_pred = np.array([pose[:3, 3] for pose in poses_pred.values()]).T
            with_scale = alignment != "6dof"
            r, t, scale = umeyama_alignment(xyz_pred, xyz_gt, with_scale)
            align_trans = np.eye(4)
            align_trans[:3, :3] = r
            align_trans[:3, 3] = t
            for cnt, pose in poses_pred.items():
                if with_scale:
                    pose[:3, 3] *= scale
                if alignment in {"7dof", "6dof"}:
                    poses_pred[cnt] = align_trans @ pose
        return poses_pred
