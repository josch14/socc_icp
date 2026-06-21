"""
Reproduce SOCC-ICP paper metrics from the provided trajectory logs.

Usage:
    python calculate_metrics.py kitti
    python calculate_metrics.py mulran
    python calculate_metrics.py newer_college
    python calculate_metrics.py ground_challenge
    python calculate_metrics.py subt_mrs

KITTI, MulRan, and Newer College are evaluated with the KISS-ICP metrics
(sequence_error = KITTI-benchmark-style translation/rotation error,
absolute_trajectory_error = ATE/ARE after SE3 alignment).

Ground-Challenge and SubT-MRS are evaluated with evo_ape.
"""

import argparse
import os
import subprocess

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

KISS_ICP_DATASETS = {"kitti", "mulran", "newer_college"}
EVO_DATASETS = {"ground_challenge", "subt_mrs"}
ALL_DATASETS = KISS_ICP_DATASETS | EVO_DATASETS


def load_kitti_poses(path):
    poses = []
    with open(path) as f:
        for line in f:
            vals = list(map(float, line.strip().split()))
            T = np.eye(4)
            T[:3, :] = np.array(vals).reshape(3, 4)
            poses.append(T)
    return np.array(poses)


def eval_kiss_icp(seq_dir, seq_id):
    from kiss_icp.metrics import absolute_trajectory_error, sequence_error

    gt_file = os.path.join(seq_dir, f"{seq_id}_gt_kitti.txt")
    pred_file = os.path.join(seq_dir, f"{seq_id}_poses_kitti.txt")

    if not os.path.exists(gt_file) or not os.path.exists(pred_file):
        print(f"  [{seq_id}] missing trajectory files, skipping")
        return None

    poses_gt = load_kitti_poses(gt_file)
    poses_pred = load_kitti_poses(pred_file)

    avg_tra, avg_rot = sequence_error(poses_gt, poses_pred)
    ate_rot, ate_trans = absolute_trajectory_error(poses_gt, poses_pred)

    print(
        f"  [{seq_id:>30}]  "
        f"Trans: {avg_tra:7.4f}%   "
        f"Rot: {100 * avg_rot:7.4f} deg/100m   "
        f"ATE: {ate_trans:7.4f} m   "
        f"ARE: {ate_rot:.4f} rad"
    )
    return avg_tra, avg_rot, ate_trans, ate_rot


def eval_evo(seq_dir, seq_id):
    gt_file = os.path.join(seq_dir, f"{seq_id}_gt_kitti.txt")
    pred_file = os.path.join(seq_dir, f"{seq_id}_poses_kitti.txt")

    if not os.path.exists(gt_file) or not os.path.exists(pred_file):
        print(f"  [{seq_id}] missing trajectory files, skipping")
        return

    print(f"\n  [{seq_id}] — APE")
    subprocess.run(["evo_ape", "kitti", gt_file, pred_file, "-a"])
    print(f"\n  [{seq_id}] — RPE")
    subprocess.run(["evo_rpe", "kitti", gt_file, pred_file])


def evaluate_dataset(dataset):
    dataset_dir = os.path.join(SCRIPT_DIR, dataset)
    if not os.path.isdir(dataset_dir):
        print(f"Directory not found: {dataset_dir}")
        return

    variants = sorted(
        d
        for d in os.listdir(dataset_dir)
        if os.path.isdir(os.path.join(dataset_dir, d))
    )
    if not variants:
        print(f"No variants found for {dataset}")
        return

    for variant in variants:
        var_dir = os.path.join(dataset_dir, variant)
        sequences = sorted(
            d for d in os.listdir(var_dir) if os.path.isdir(os.path.join(var_dir, d))
        )
        if not sequences:
            continue

        print(f"\n{'=' * 70}")
        print(f"  {dataset} / {variant}")
        print(f"{'=' * 70}")

        results = []
        seq_results = {}
        for seq_id in sequences:
            seq_dir = os.path.join(var_dir, seq_id)
            if dataset in KISS_ICP_DATASETS:
                r = eval_kiss_icp(seq_dir, seq_id)
                if r is not None:
                    results.append(r)
                    seq_results[seq_id] = r
            else:
                eval_evo(seq_dir, seq_id)

        if results:
            if dataset == "mulran":
                grouped = {}
                for seq_id, r in seq_results.items():
                    loc = seq_id.rstrip("0123456789")
                    grouped.setdefault(loc, []).append(r)
                print()
                for loc, loc_results in sorted(grouped.items()):
                    n_loc = len(loc_results)
                    print(
                        f"  [{'Avg ' + loc:>30} ({n_loc} seqs)]  "
                        f"Trans: {np.mean([r[0] for r in loc_results]):7.4f}%   "
                        f"Rot: {100 * np.mean([r[1] for r in loc_results]):7.4f} deg/100m   "
                        f"ATE: {np.mean([r[2] for r in loc_results]):7.4f} m   "
                        f"ARE: {np.mean([r[3] for r in loc_results]):.4f} rad"
                    )

            n = len(results)
            print(
                f"\n  [{'Average':>30} ({n} seqs)]  "
                f"Trans: {np.mean([r[0] for r in results]):7.4f}%   "
                f"Rot: {100 * np.mean([r[1] for r in results]):7.4f} deg/100m   "
                f"ATE: {np.mean([r[2] for r in results]):7.4f} m   "
                f"ARE: {np.mean([r[3] for r in results]):.4f} rad"
            )


def main():
    parser = argparse.ArgumentParser(
        description="Reproduce SOCC-ICP paper metrics from provided logs."
    )
    parser.add_argument("dataset", choices=sorted(ALL_DATASETS))
    args = parser.parse_args()
    evaluate_dataset(args.dataset)


if __name__ == "__main__":
    main()
