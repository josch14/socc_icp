import numpy as np
from sophus_pybind import SE3


def rotation_error(T_pred: SE3, T_true: SE3) -> float:
    """Compute the rotation error (in degrees) between two SE3 poses."""
    R_pred = T_pred.rotation().to_matrix()
    R_true = T_true.rotation().to_matrix()
    R_diff = R_true.T @ R_pred
    cos_theta = (np.trace(R_diff) - 1) / 2
    rotation_error_rad = np.arccos(np.clip(cos_theta, -1.0, 1.0))
    return np.degrees(rotation_error_rad)


def translation_error(T_pred: SE3, T_true: SE3) -> float:
    """Compute the translation error (Euclidean distance) between two SE3 poses."""
    return np.linalg.norm(T_pred.translation() - T_true.translation())
