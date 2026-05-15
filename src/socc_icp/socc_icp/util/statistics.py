import os

import numpy as np

from socc_icp.util.tf_to_str import tf_to_str
from sophus_pybind import SE3


def write_and_log(statistics: dict, dir_out: str, idx: int, log: bool = False):
    statistics_dir = os.path.join(dir_out, "statistics")
    if not os.path.exists(statistics_dir):
        os.makedirs(statistics_dir)

    # tfs
    write_tfs(
        T_pred=statistics["T_pred"],
        T_true=statistics["T_true"] if "T_true" in statistics else None,
        dir_out=statistics_dir,
        idx=idx,
    )

    # statistics
    for tag in statistics.keys():
        if tag in ["T_pred", "T_true"]:
            continue
        s = f"{idx} {statistics[tag]}"
        f = os.path.join(statistics_dir, f"{tag}.txt")
        with open(f, "a") as f:
            f.write(f"{s}\n")

    if log:
        precision = 4
        np.set_printoptions(precision=precision, suppress=True)
        rotation_err = statistics["rotation_error_deg"]
        translation_err = statistics["translation_error"]
        print(f"[Error] Rotation (deg): {rotation_err:.{precision}f}")
        print(f"[Error] Translation:    {translation_err:.{precision}f}")


def write_tfs(T_pred: SE3, T_true: SE3 | None, dir_out, idx: int):
    # T_pred
    s = tf_to_str(T_pred, idx)
    f = os.path.join(dir_out, "T_pred.txt")
    with open(f, "a") as f:
        f.write(f"{s}\n")

    # T_true
    if T_true is not None:
        s = tf_to_str(T_true, idx)
        f = os.path.join(dir_out, "T_true.txt")
        with open(f, "a") as f:
            f.write(f"{s}\n")
