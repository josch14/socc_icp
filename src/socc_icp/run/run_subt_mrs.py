import os
from pathlib import Path

from socc_icp.config.config import SoccIcpConfig
from socc_icp.dataset.subt_mrs import SubTMRSDataset
from socc_icp.pipeline.pipeline import OdometryPipeline
from socc_icp.config.load import load_config_dict
from socc_icp.util.run_sequence import run_sequence


def main():
    CONFIG_FILE = "configs/config_subt_mrs.yaml"

    # read config
    config_dict = load_config_dict(path_override=CONFIG_FILE)
    config = SoccIcpConfig.from_dict(config_dict)

    # create pipeline object
    pipeline = OdometryPipeline(
        dataset=SubTMRSDataset(
            data_dir=os.path.join(Path(config.data_params.base_path), "long_corridor"),  # type: ignore
        ),
        config_dict=config_dict,
        config=config,
    )
    results = {}
    run_sequence(pipeline, results=results)


if __name__ == "__main__":
    main()


"""
python -m run.run_subt_mrs

Afterwards:
python -m scripts.fix_subt_mrs_pose_files
evo_ape kitti long_corridor_gt_kitti_fixed.txt long_corridor_poses_kitti_fixed.txt
evo_rpe kitti long_corridor_gt_kitti_fixed.txt long_corridor_poses_kitti_fixed.txt
"""
