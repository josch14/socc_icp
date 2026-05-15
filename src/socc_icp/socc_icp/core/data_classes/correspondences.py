from dataclasses import dataclass
import numpy as np


@dataclass
class CorrespondencesPlanar:
    n: int
    points_src: np.ndarray
    points_tar: np.ndarray
    normals: np.ndarray
    weights_semantic: np.ndarray
    weights_occ_prob: np.ndarray


@dataclass
class CorrespondencesNonPlanar:
    n: int
    points_src: np.ndarray
    points_tar: np.ndarray
    weights_semantic: np.ndarray
    weights_occ_prob: np.ndarray
