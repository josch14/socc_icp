import os
from pathlib import Path

from socc_icp.config.config import SoccIcpConfig
from socc_icp.dataset.ground_challenge import GroundChallengeDataset
from socc_icp.pipeline.pipeline import OdometryPipeline
from socc_icp.config.load import load_config_dict
from socc_icp.util.run_sequence import run_sequence


def main():
    CONFIG_FILE = "configs/config_ground_challenge.yaml"

    # read config
    config_dict = load_config_dict(path_override=CONFIG_FILE)
    config = SoccIcpConfig.from_dict(config_dict)

    for seq in ["corridor1", "corridor2"]:
        print(f"Processing sequence {seq}")
        # create pipeline object
        pipeline = OdometryPipeline(
            dataset=GroundChallengeDataset(
                data_dir=os.path.join(Path(config.data_params.base_path), seq),  # type: ignore
            ),
            config_dict=config_dict,
            config=config,
        )
        results = {}
        run_sequence(pipeline, results=results)


if __name__ == "__main__":
    main()
