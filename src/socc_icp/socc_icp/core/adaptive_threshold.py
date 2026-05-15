import numpy as np
from sophus_pybind import SE3

from ..config.config import AdaptiveThresholdParams


class AdaptiveThreshold:
    def __init__(self, params: AdaptiveThresholdParams):
        # configurable parameters
        self.adaptive_threshold_init = params.adaptive_threshold_init
        self.is_adaptive = params.is_adaptive
        self.min_motion_threshold = params.min_motion_threshold
        self.max_distance = params.max_distance

        # computation parameters
        self.model_error_sse2 = 0.0
        self.num_samples = 0
        self.T_model_deviation = None

    def get_threshold(self, poses: list[SE3]) -> float:
        if not self.is_adaptive:
            sigma = self.adaptive_threshold_init

        else:
            if self.has_moved(poses):
                sigma = self.compute_threshold()
            else:
                sigma = self.adaptive_threshold_init

        # print("[AdaptiveThreshold] sigma=", sigma)
        return sigma

    def has_moved(self, poses: list[SE3]) -> bool:
        if poses is None or len(poses) == 0:
            return False

        # calculate the relative motion between first and last pose
        T_motion = poses[0].inverse() @ poses[-1]
        motion = np.linalg.norm(T_motion.translation())
        has_moved = bool(motion > 5.0 * self.min_motion_threshold)
        return has_moved

    def compute_threshold(self) -> float:
        model_error = self.compute_model_error()
        if model_error > self.min_motion_threshold:
            self.model_error_sse2 += model_error**2
            self.num_samples += 1

        if self.num_samples < 1:
            sigma = self.adaptive_threshold_init
        else:
            sigma = float(np.sqrt(self.model_error_sse2 / self.num_samples))

        return sigma

    def update_model_deviation(self, T_initial: SE3, T_pred: SE3) -> None:
        self.T_model_deviation = T_initial.inverse() @ T_pred

    def compute_model_error(self):
        if self.T_model_deviation is None:
            raise ValueError(
                "Model deviation is not set. Call update_model_deviation first."
            )

        R = self.T_model_deviation.rotation().to_matrix()
        t = self.T_model_deviation.translation()

        angle_axis = np.linalg.norm(np.arccos((np.trace(R) - 1) / 2))
        theta = angle_axis

        delta_rot = 2.0 * self.max_distance * np.sin(theta / 2.0)
        delta_trans = np.linalg.norm(t)
        model_error = delta_trans + delta_rot

        return model_error
