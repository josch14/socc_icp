from pathlib import Path

from socc_icp.config.config import SoccIcpConfig
from socc_icp.dataset.mulran import MulranDataset
from socc_icp.pipeline.pipeline import OdometryPipeline
from socc_icp.config.load import load_config_dict
from socc_icp.util.run_sequence import run_sequence


def main():
    CONFIG_FILE = "configs/config_mulran.yaml"

    # read config
    config_dict = load_config_dict(path_override=CONFIG_FILE)
    config = SoccIcpConfig.from_dict(config_dict)

    for sequence_name in ["dcc", "kaist", "riverside", "sejong"]:
        for sequence_idx in [1, 2, 3]:
            print(f"Processing sequence: {sequence_name} {sequence_idx}")

            # create pipeline object
            pipeline = OdometryPipeline(
                dataset=MulranDataset(
                    data_dir=Path(config.data_params.base_path),  # type: ignore
                    sequence_name=sequence_name,
                    sequence_idx=sequence_idx,
                ),
                config_dict=config_dict,
                config=config,
            )
            results = {}
            run_sequence(pipeline, results=results)


if __name__ == "__main__":
    main()
