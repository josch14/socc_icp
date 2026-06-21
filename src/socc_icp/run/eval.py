import glob
import os

import numpy as np
from kiss_icp.metrics import absolute_trajectory_error, sequence_error

from socc_icp.metrics.kitti_eval_odom import KittiEvalOdom


def load_kitti_poses(path):
    poses = []
    with open(path) as f:
        for line in f:
            vals = list(map(float, line.strip().split()))
            T = np.eye(4)
            T[:3, :] = np.array(vals).reshape(3, 4)
            poses.append(T)
    return np.array(poses)


TAGS = [
    ("adaptive_voxel_size", float),
    ("adaptive_threshold", float),
    ("alpha", float),
    ("chunk_size", int),
    ("dx_norm", float),
    ("n_iterations", int),
    ("n_downsampled", int),
    ("n_non_planar", int),
    ("n_planar", int),
    ("rotation_error_deg", float),
    ("time_radix_chunk", float),
    ("time_radix_insert", float),
    ("time_registration", float),
    ("time_deskewing", float),
    ("translation_error", float),
]


def run_evaluation(dir_out: str, alignment: str | None = None):
    statistics_dir = os.path.join(dir_out, "statistics")

    # Plot per-frame statistics (timing, registration stats, etc.)
    for tag, dtype in TAGS:
        file_path = os.path.join(statistics_dir, f"{tag}.txt")
        if not os.path.exists(file_path):
            continue
        with open(file_path, "r") as f:
            result_list = []
            for line in f:
                parts = line.strip().split()
                if len(parts) != 2:
                    raise ValueError(f"Expected 2 elements, got {parts}")
                result_list.append((int(parts[0]), dtype(parts[1])))
        result_list.sort(key=lambda x: x[0])
        save_statistics_plot(result_list, tag, dir_out)

    # KITTI-benchmark-style trajectory and error plots
    eval_tool = KittiEvalOdom()
    eval_tool.eval_sequence(log_dir=dir_out, alignment=alignment)

    # KISS-ICP metrics (sequence error + ATE) from stored KITTI trajectory files
    pred_files = glob.glob(os.path.join(dir_out, "*_poses_kitti.txt"))
    gt_files = glob.glob(os.path.join(dir_out, "*_gt_kitti.txt"))
    if not pred_files or not gt_files:
        print("No KITTI trajectory files found, skipping KISS-ICP evaluation.")
        return
    poses_pred = load_kitti_poses(pred_files[0])
    poses_gt = load_kitti_poses(gt_files[0])

    avg_tra, avg_rot = sequence_error(poses_gt, poses_pred)
    ate_rot, ate_trans = absolute_trajectory_error(poses_gt, poses_pred)

    # log and write to file
    s = "\n"
    s += "Evaluation using KISS-ICP eval code:\n"
    s += "-------------\n"
    s += f"Average Translation Error:       {round(avg_tra, 3)} %\n"
    s += f"Average Rotation Error:          {round(100 * avg_rot, 4)} deg/100m\n"
    s += f"Absolute Trajectory Error (ATE): {round(ate_trans, 3)} m\n"
    s += f"Absolute Trajectory Error (ARE): {round(ate_rot, 4)} rad\n"

    # Write evaluation results to a text file in dir_out
    eval_file_path = os.path.join(dir_out, "kiss_icp_eval.txt")
    with open(eval_file_path, "w") as f:
        f.write(s)

    # log
    print(s)
    print()


def save_statistics_plot(result_list, tag, dir_out):
    import matplotlib.pyplot as plt

    # Extract idx and value for plotting
    idx, values = zip(*result_list)

    # Create the plot
    plt.figure()
    plt.plot(idx, values, label="Values")
    plt.xlabel("Frame ID")
    plt.ylabel(tag)
    plt.title(tag)
    plt.grid(True)

    # Plot the mean of values as a red line
    mean_value = sum(values) / len(values)
    plt.axhline(mean_value, color="red", linestyle="--", label="Mean")

    # Set y-axis range from 0 to max value
    upper = max(values) + 0.05 * max(values)
    if upper > 0:
        plt.ylim(0, upper)
    else:
        plt.ylim(-1, 1)

    # Add legend
    plt.legend()

    # Save the plot to the evaluation directory
    statistics_plots_dir = os.path.join(dir_out, "statistics_plots")
    os.makedirs(statistics_plots_dir, exist_ok=True)
    plot_path = os.path.join(statistics_plots_dir, f"{tag}.png")
    plt.savefig(plot_path)
    plt.close()


if __name__ == "__main__":
    """
    evaluation on single sequence/config/run saved to log
    python -m socc_icp.eval
    """
    dir_out = "log/sequence_00/"
    run_evaluation(
        dir_out,
        alignment=None,  # ["scale", "scale_7dof", "7dof", "6dof"]
    )
