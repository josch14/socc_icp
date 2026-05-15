from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass
class SoccIcpConfig:
    data_params: DataParams | None
    slam_client_params: SlamClientParams
    downsampling_params: DownsamplingParams
    adaptive_threshold_params: AdaptiveThresholdParams
    registration_params: RegistrationParams
    general_params: GeneralParams
    semantic_downsampling_factors: dict[int, float]
    label_mapping: dict[int, int] | None = None

    @classmethod
    def from_dict(cls, config: dict) -> "SoccIcpConfig":
        """Factory constructor that builds a SoccIcpConfig from a config dictionary."""

        # required config params
        slam_client_params = SlamClientParams(**config["slam_client_params"])
        downsampling_params = DownsamplingParams(**config["downsampling_params"])
        adaptive_threshold_params = AdaptiveThresholdParams(
            **config["adaptive_threshold_params"]
        )
        registration_params = RegistrationParams(**config["registration_params"])
        general_params = GeneralParams(**config["general_params"])

        # optional data params (not required for server)
        data_params = (
            DataParams(**config["data_params"]) if "data_params" in config else None
        )
        semantic_downsampling_factors = config.get(
            "semantic_downsampling_factors",
            {0: 1.0},  # default (w.o. semantics): factor 1.0 for unlabeled
        )
        label_mapping = config.get("label_mapping", None)

        return cls(
            data_params=data_params,
            slam_client_params=slam_client_params,
            downsampling_params=downsampling_params,
            adaptive_threshold_params=adaptive_threshold_params,
            registration_params=registration_params,
            general_params=general_params,
            semantic_downsampling_factors=semantic_downsampling_factors,
            label_mapping=label_mapping,
        )


@dataclass
class DataParams:
    base_path: Path
    jump: int
    n_scans: int
    seed: int

    def __post_init__(self):
        # base_path
        if isinstance(self.base_path, str):
            self.base_path = Path(self.base_path)

        # jump
        if self.jump < 0:
            raise ValueError(f"jump must be >= 0, got {self.jump}")

        # n_scans
        if self.n_scans != -1 and self.n_scans <= 0:
            raise ValueError(f"n_scans must be -1 or > 0, got {self.n_scans}")

        # base_path exists
        if not self.base_path.exists():
            raise FileNotFoundError(f"Base path does not exist: {self.base_path}")


@dataclass
class SlamClientParams:
    radix_topic: str
    chunk_range: float
    chunk_delete_rest: bool

    def __post_init__(self):
        # radix_topic
        if not isinstance(self.radix_topic, str):
            raise TypeError(
                f"radix_topic must be str, got {type(self.radix_topic).__name__}"
            )

        # chunk_range
        if not isinstance(self.chunk_range, float) or self.chunk_range <= 0:
            raise ValueError(
                f"chunk_range must be float > 0, got {self.chunk_range} ({type(self.chunk_range).__name__})"
            )

        # chunk_delete_rest
        if not isinstance(self.chunk_delete_rest, bool):
            raise TypeError(
                f"chunk_delete_rest must be bool, got {type(self.chunk_delete_rest).__name__}"
            )


@dataclass
class DownsamplingParams:
    desired_num_points: int
    voxel_size_init: float
    adaptive_voxel_size_min: float
    adaptive_voxel_size_max: float
    use_centroid: bool

    def __post_init__(self):
        # desired_num_points
        if not isinstance(self.desired_num_points, int) or self.desired_num_points <= 0:
            raise ValueError(
                f"desired_num_points must be int > 0, got {self.desired_num_points}"
            )

        # voxel_size_init
        if not isinstance(self.voxel_size_init, float) or self.voxel_size_init <= 0:
            raise ValueError(
                f"voxel_size_init must be float > 0, got {self.voxel_size_init}"
            )

        # adaptive_voxel_size_min
        if (
            not isinstance(self.adaptive_voxel_size_min, float)
            or self.adaptive_voxel_size_min <= 0
        ):
            raise ValueError(
                f"adaptive_voxel_size_min must be float > 0, got {self.adaptive_voxel_size_min}"
            )

        # adaptive_voxel_size_max
        if (
            not isinstance(self.adaptive_voxel_size_max, float)
            or self.adaptive_voxel_size_max <= 0
        ):
            raise ValueError(
                f"adaptive_voxel_size_max must be float > 0, got {self.adaptive_voxel_size_max}"
            )

        # adaptive_voxel_size_min <= adaptive_voxel_size_max
        if self.adaptive_voxel_size_min > self.adaptive_voxel_size_max:
            raise ValueError(
                "adaptive_voxel_size_min must be <= adaptive_voxel_size_max"
            )

        # use_centroid
        if not isinstance(self.use_centroid, bool):
            raise TypeError(
                f"use_centroid must be bool, got {type(self.use_centroid).__name__}"
            )


@dataclass
class AdaptiveThresholdParams:
    adaptive_threshold_init: float
    is_adaptive: bool
    min_motion_threshold: float
    max_distance: float

    def __post_init__(self):
        # adaptive_threshold_init
        if (
            not isinstance(self.adaptive_threshold_init, float)
            or self.adaptive_threshold_init <= 0
        ):
            raise ValueError(
                f"adaptive_threshold_init must be float > 0, got {self.adaptive_threshold_init}"
            )

        # is_adaptive
        if not isinstance(self.is_adaptive, bool):
            raise TypeError(
                f"is_adaptive must be bool, got {type(self.is_adaptive).__name__}"
            )

        # min_motion_threshold
        if (
            not isinstance(self.min_motion_threshold, float)
            or self.min_motion_threshold < 0
        ):
            raise ValueError(
                f"min_motion_threshold must be float >= 0, got {self.min_motion_threshold}"
            )

        # max_distance
        if not isinstance(self.max_distance, float) or self.max_distance <= 0:
            raise ValueError(f"max_distance must be float > 0, got {self.max_distance}")


@dataclass
class RegistrationParams:
    min_distance: float
    max_distance: float
    max_num_iterations: int
    convergence_criterion: float
    planarity_threshold: float
    n_points_for_normal: int
    sigma_factor_correspondence: float
    sigma_factor_kernel: float
    weight_occ_prob_gamma: float
    weight_semantic_lower: float
    voxel_reference_point: str

    def __post_init__(self):
        # min_distance
        if not isinstance(self.min_distance, float) or self.min_distance < 0:
            raise ValueError(
                f"min_distance must be float >= 0, got {self.min_distance}"
            )

        # max_distance
        if not isinstance(self.max_distance, float) or self.max_distance <= 0:
            raise ValueError(f"max_distance must be float > 0, got {self.max_distance}")

        # min_distance < max_distance
        if self.min_distance >= self.max_distance:
            raise ValueError("min_distance must be < max_distance")

        # max_num_iterations
        if not isinstance(self.max_num_iterations, int) or self.max_num_iterations <= 0:
            raise ValueError(
                f"max_num_iterations must be int > 0, got {self.max_num_iterations}"
            )

        # convergence_criterion
        if (
            not isinstance(self.convergence_criterion, float)
            or self.convergence_criterion < 0
        ):
            raise ValueError(
                f"convergence_criterion must be float >= 0, got {self.convergence_criterion}"
            )

        # planarity_threshold
        if (
            not isinstance(self.planarity_threshold, float)
            or self.planarity_threshold < 0
        ):
            raise ValueError(
                f"planarity_threshold must be float >= 0, got {self.planarity_threshold}"
            )

        # n_points_for_normal
        if (
            not isinstance(self.n_points_for_normal, int)
            or self.n_points_for_normal <= 2
        ):
            raise ValueError(
                f"n_points_for_normal must be int > 2, got {self.n_points_for_normal}"
            )

        # sigma_factor_correspondence
        if (
            not isinstance(self.sigma_factor_correspondence, float)
            or self.sigma_factor_correspondence <= 0
        ):
            raise ValueError(
                f"sigma_factor_correspondence must be float > 0, got {self.sigma_factor_correspondence}"
            )

        # sigma_factor_kernel
        if (
            not isinstance(self.sigma_factor_kernel, float)
            or self.sigma_factor_kernel <= 0
        ):
            raise ValueError(
                f"sigma_factor_kernel must be float > 0, got {self.sigma_factor_kernel}"
            )

        # weight_occ_prob_gamma
        if (
            not isinstance(self.weight_occ_prob_gamma, float)
            or self.weight_occ_prob_gamma < 0
        ):
            raise ValueError(
                f"weight_occ_prob_gamma must be float >= 0, got {self.weight_occ_prob_gamma}"
            )

        # weight_semantic_lower
        if not isinstance(self.weight_semantic_lower, float) or not (
            0.0 <= self.weight_semantic_lower <= 1.0
        ):
            raise ValueError(
                f"weight_semantic_lower must be float in [0.0, 1.0], got {self.weight_semantic_lower}"
            )

        # voxel_reference_point
        if self.voxel_reference_point not in ["center", "mean", "first_point"]:
            raise ValueError(
                f"voxel_reference_point must be one of ['center', 'mean', 'first_point'], got {self.voxel_reference_point}"
            )


@dataclass
class GeneralParams:
    deskewing_mode: str
    use_labels: bool
    use_predicted_labels: bool

    def __post_init__(self):
        # deskewing_mode
        if self.deskewing_mode not in ["mid", "end", "none"]:
            raise ValueError(
                f"deskewing_mode must be one of ['mid', 'end', 'none'], got {self.deskewing_mode}"
            )

        # use_labels
        if not isinstance(self.use_labels, bool):
            raise TypeError(
                f"use_labels must be bool, got {type(self.use_labels).__name__}"
            )

        # use_predicted_labels
        if not isinstance(self.use_predicted_labels, bool):
            raise TypeError(
                f"use_predicted_labels must be bool, got {type(self.use_predicted_labels).__name__}"
            )
