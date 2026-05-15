import os
import shutil
from run.eval import run_evaluation


def fix_t_pred_t_true():
    DIR_PTH_FIXED = DIR_PTH + "_fixed"
    if not os.path.exists(DIR_PTH_FIXED):
        shutil.copytree(DIR_PTH, DIR_PTH_FIXED)
    else:
        raise RuntimeError(f"Directory {DIR_PTH_FIXED} already exists, delete it first")

    # get file paths
    PATH_GT_FILE = os.path.join(DIR_PTH_FIXED, "statistics/T_true.txt")
    PATH_PRED_FILE = os.path.join(DIR_PTH_FIXED, "statistics/T_pred.txt")

    with open(PATH_GT_FILE, "r") as f_gt, open(PATH_PRED_FILE, "r") as f_pred:
        gt_lines = f_gt.readlines()
        pred_lines = f_pred.readlines()

    assert len(gt_lines) == len(pred_lines), "Files have different number of lines"

    filtered_gt = []
    filtered_pred = []

    IGNORE_LINE_STR = "1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0"

    for gt_line, pred_line in zip(gt_lines, pred_lines):
        if (
            IGNORE_LINE_STR not in gt_line.strip()
            and IGNORE_LINE_STR not in pred_line.strip()
        ):
            gt_parts = gt_line.strip().split()
            pred_parts = pred_line.strip().split()
            gt_parts[0] = str(len(filtered_gt))
            pred_parts[0] = str(len(filtered_pred))
            gt_line = " ".join(gt_parts) + "\n"
            pred_line = " ".join(pred_parts) + "\n"

            filtered_gt.append(gt_line)
            filtered_pred.append(pred_line)

    with (
        open(PATH_GT_FILE, "w") as f_gt,
        open(PATH_PRED_FILE, "w") as f_pred,
    ):
        f_gt.writelines(filtered_gt)
        f_pred.writelines(filtered_pred)

    return DIR_PTH_FIXED


def fix_gt_kitti_poses_kitti(dir_path_fixed):
    # get file paths
    FILE_GT_KITTI = os.path.join(dir_path_fixed, f"{SEQ_NAME}_gt_kitti.txt")
    FILE_POSE_KITTI = os.path.join(dir_path_fixed, f"{SEQ_NAME}_poses_kitti.txt")
    IGNORE_LINE_STR = "0.000000000000000000e+00 0.000000000000000000e+00 0.000000000000000000e+00 0.000000000000000000e+00 0.000000000000000000e+00 0.000000000000000000e+00 0.000000000000000000e+00 0.000000000000000000e+00 0.000000000000000000e+00 0.000000000000000000e+00 0.000000000000000000e+00 0.000000000000000000e+00"

    with open(FILE_GT_KITTI, "r") as f_gt, open(FILE_POSE_KITTI, "r") as f_pred:
        gt_lines = f_gt.readlines()
        pred_lines = f_pred.readlines()

    assert len(gt_lines) == len(pred_lines), "Files have different number of lines"

    filtered_gt = []
    filtered_pred = []

    for gt_line, pred_line in zip(gt_lines, pred_lines):
        if gt_line.strip() != IGNORE_LINE_STR and pred_line.strip() != IGNORE_LINE_STR:
            filtered_gt.append(gt_line)
            filtered_pred.append(pred_line)

    with (
        open(FILE_GT_KITTI, "w") as f_gt,
        open(FILE_POSE_KITTI, "w") as f_pred,
    ):
        f_gt.writelines(filtered_gt)
        f_pred.writelines(filtered_pred)

    return FILE_GT_KITTI, FILE_POSE_KITTI


# dir must contain {}_gt_kitti.txt and {}_poses_kitti.txt
DIR_PTH = "log/subt_mrs"
SEQ_NAME = "long_corridor"

if __name__ == "__main__":
    """
    python -m scripts.fix_subt_mrs
    """
    DIR_PTH_FIXED = fix_t_pred_t_true()
    FILE_GT_KITTI, FILE_POSE_KITTI = fix_gt_kitti_poses_kitti(DIR_PTH_FIXED)

    # run eval using kitti eval code
    run_evaluation(DIR_PTH_FIXED, alignment="7dof")

    # run eval using EVO
    os.system(f"evo_ape kitti {FILE_GT_KITTI} {FILE_POSE_KITTI} -a")
    os.system(f"evo_rpe kitti {FILE_GT_KITTI} {FILE_POSE_KITTI} -a")
