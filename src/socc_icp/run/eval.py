import os

import numpy as np
from kiss_icp.metrics import absolute_trajectory_error, sequence_error

from socc_icp.metrics.kitti_eval_odom import KittiEvalOdom
from socc_icp.util.tf_to_str import str_to_tf

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

    # (1) visualize recorded statistics
    for tag, dtype in TAGS:
        file_path = os.path.join(statistics_dir, f"{tag}.txt")
        if not os.path.exists(file_path):
            continue

        # Read and process data
        with open(file_path, "r") as f:
            result_list = []
            for line in f:
                parts = line.strip().split()
                if len(parts) != 2:
                    raise ValueError(f"Expected 2 elements, got {parts}")
                idx = int(parts[0])
                value = dtype(parts[1])
                result_list.append((idx, value))

        result_list.sort(key=lambda x: x[0])

        # save results as plots
        save_statistics_plot(result_list, tag, dir_out)

    # (2) pose evaluation
    eval_tool = KittiEvalOdom()
    eval_tool.eval_sequence(
        log_dir=dir_out,
        alignment=alignment,
    )
    print("-------------\n")

    # (3) pose evaluation using KISS-ICP eval code
    # pred
    poses_pred = []
    file_path = os.path.join(statistics_dir, "T_pred.txt")
    with open(file_path, "r") as f:
        for line in f:
            T, idx = str_to_tf(line)
            poses_pred.append(T.to_matrix())
    # gt
    poses_gt = []
    file_path = os.path.join(statistics_dir, "T_true.txt")
    with open(file_path, "r") as f:
        for line in f:
            T, idx = str_to_tf(line)
            poses_gt.append(T.to_matrix())
    # convert to numpy
    poses_gt = np.array(poses_gt)
    poses_pred = np.array(poses_pred)

    avg_tra, avg_rot = sequence_error(poses_gt, poses_pred)
    ate_rot, ate_trans = absolute_trajectory_error(poses_gt, poses_pred)

    # log and write to file
    s = ""
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
    dir_out = "log/res_0.2_min_range_1.0_delete_rest_False_fixed_limited"
    run_evaluation(
        dir_out,
        alignment=None,  # ["scale", "scale_7dof", "7dof", "6dof"]
    )
