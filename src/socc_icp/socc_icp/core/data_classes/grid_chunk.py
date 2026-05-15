from dataclasses import dataclass
import numpy as np
from scipy.spatial import cKDTree


@dataclass
class GridChunk:
    n: int
    centers: np.ndarray
    labels: np.ndarray
    label_probs: np.ndarray
    means: np.ndarray
    cov_matrices: np.ndarray
    n_points: np.ndarray
    points: np.ndarray
    occ_probs: np.ndarray

    # after processing
    mask_planar: np.ndarray | None = None
    normals: np.ndarray | None = None
    map_reference_points: cKDTree | None = None

    def prepare_for_correspondence_search(
        self,
        planarity_threshold: float,
        n_points_for_normal: int,
        voxel_reference_point: str,
    ):
        # (1) depending on voxel_reference parameter, prepare the map points for correspondence search
        map_reference_points = None
        if voxel_reference_point == "center":
            map_reference_points = self.centers
        elif voxel_reference_point == "mean":
            map_reference_points = self.means
        elif voxel_reference_point == "first_point":
            map_reference_points = self.points
        assert isinstance(map_reference_points, np.ndarray)
        # create cKDTree of map points for fast nearest neighbor search
        self.map_reference_points = cKDTree(map_reference_points)

        # (2) calculate mask_planar and normals
        # returns eigenvalues and eigenvectors in ascending order
        eigenvalues, eigenvectors = np.linalg.eigh(self.cov_matrices)

        # calculate local surface variation
        sum_eigenvalues = np.sum(eigenvalues, axis=1)
        # avoid division by zero and mark invalid as non-planar
        # may happen due to covariance matrix being very close but not equal to zero
        sum_eigenvalues = np.where(sum_eigenvalues <= 0, 1e-6, sum_eigenvalues)
        local_surface_variation = eigenvalues[:, 0] / sum_eigenvalues
        local_surface_variation[sum_eigenvalues == 1e-6] = np.inf

        # process planar points using mask
        mask_planar = (local_surface_variation < planarity_threshold) & (
            self.n_points >= n_points_for_normal
        )
        normals = np.full_like(eigenvectors[:, :, 0], np.nan)
        normals[mask_planar] = eigenvectors[mask_planar, :, 0]

        self.normals = normals
        self.mask_planar = mask_planar
