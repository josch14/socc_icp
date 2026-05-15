import argparse
from pathlib import Path

from socc_icp.config.config import SoccIcpConfig
from socc_icp.dataset.kitti import SemanticKITTIDataset
from socc_icp.pipeline.pipeline import OdometryPipeline
from socc_icp.config.load import load_config_dict
from socc_icp.util.run_sequence import run_sequence


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--no-semantics", action="store_true", help="Disable semantic labels"
    )
    args = parser.parse_args()

    if args.no_semantics:
        CONFIG_FILE = "configs/config_kitti_no_semantics.yaml"
    else:
        CONFIG_FILE = "configs/config_kitti_semantics.yaml"

    # read config
    config_dict = load_config_dict(path_override=CONFIG_FILE)
    config = SoccIcpConfig.from_dict(config_dict)

    for sequence in [1, 3, 4, 6, 7, 9, 10, 0, 2, 5, 8]:
        # create pipeline object
        pipeline = OdometryPipeline(
            dataset=SemanticKITTIDataset(
                data_dir=Path(config.data_params.base_path),  # type: ignore
                use_predicted_labels=config.general_params.use_predicted_labels,
                sequence=sequence,
            ),
            config_dict=config_dict,
            config=config,
        )
        results = {}
        run_sequence(pipeline, results=results)


if __name__ == "__main__":
    main()
