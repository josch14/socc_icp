import numpy as np
from scipy.linalg import lstsq
from scipy.spatial import cKDTree
from sophus_pybind import SE3

from socc_icp.core.data_classes.grid_chunk import GridChunk
from socc_icp.core.data_classes.correspondences import (
    CorrespondencesNonPlanar,
    CorrespondencesPlanar,
)

from ..config.config import RegistrationParams


class Registration:
    def __init__(
        self,
        params: RegistrationParams,
    ):
        self.max_num_iterations = params.max_num_iterations
        self.convergence_criterion = params.convergence_criterion
        self.planarity_threshold = params.planarity_threshold
        self.n_points_for_normal = params.n_points_for_normal
        self.sigma_factor_correspondence = params.sigma_factor_correspondence
        self.sigma_factor_kernel = params.sigma_factor_kernel
        self.weight_occ_prob_gamma = params.weight_occ_prob_gamma
        self.weight_semantic_lower = params.weight_semantic_lower
        self.voxel_reference_point = params.voxel_reference_point

    def register_frame(
        self,
        grid_chunk: GridChunk,
        pc: np.ndarray,
        labels: np.ndarray,
        sigma: float,
        T_initial_guess: SE3,
    ) -> tuple[SE3, dict]:
        if grid_chunk.n == 0:
            statistics = {
                "n_no_correspondence": 0,
                "n_planar": 0,
                "n_non_planar": 0,
                "alpha": 0.0,
                "dx_norm": 0.0,
                "n_iterations": 0,
            }

            return T_initial_guess, statistics

        # use sigma
        max_correspondence_distance = self.sigma_factor_correspondence * sigma
        kernel = self.sigma_factor_kernel * sigma

        # apply initial transformation
        pc = (T_initial_guess @ pc.T).T
        T_icp = SE3()

        # process grid chunk for correspondence search
        # create cKDTree for fast nearest neighbor searchassert isinstance(grid_chunk.points, np.ndarray)
        grid_chunk.prepare_for_correspondence_search(
            planarity_threshold=self.planarity_threshold,
            n_points_for_normal=self.n_points_for_normal,
            voxel_reference_point=self.voxel_reference_point,
        )

        # GenZ-ICP loop
        for j in range(self.max_num_iterations):
            correspondences_planar, correspondences_non_planar = (
                self.find_correspondences(
                    grid_chunk=grid_chunk,
                    pc=pc,
                    labels=labels,
                    max_correspondence_distance=max_correspondence_distance,
                )
            )

            alpha = correspondences_planar.n / (
                correspondences_planar.n + correspondences_non_planar.n
            )

            # build and solve linear system
            A, b = build_linear_system(
                correspondences_planar=correspondences_planar,
                correspondences_non_planar=correspondences_non_planar,
                kernel=kernel,
                alpha=alpha,
            )
            dx, _, _, _ = lstsq(A, -b)  # type: ignore

            # exponentiate to get the SE3 transformation
            T_estimation = SE3.exp(dx[:3], dx[3:])

            # apply new transformation to already transformed point cloud
            pc = (T_estimation @ pc.T).T

            # update overall transformation
            T_icp = T_estimation @ T_icp

            # check termination criteria
            dx_norm = np.linalg.norm(dx)
            if dx_norm < self.convergence_criterion:
                break

        # final transformation
        T_pred = T_icp @ T_initial_guess

        statistics = {
            "n_planar": correspondences_planar.n,  # type: ignore
            "n_non_planar": correspondences_non_planar.n,  # type: ignore
            "alpha": alpha,  # type: ignore
            "dx_norm": dx_norm,  # type: ignore
            "n_iterations": j + 1,  # type: ignore
        }
        return T_pred, statistics

    def find_correspondences(
        self,
        grid_chunk: GridChunk,
        pc: np.ndarray,
        labels: np.ndarray,
        max_correspondence_distance: float,
    ) -> tuple[CorrespondencesPlanar, CorrespondencesNonPlanar]:
        if not isinstance(grid_chunk.map_reference_points, cKDTree):
            raise TypeError("grid_chunk.map_reference_points must be a cKDTree")

        # query nearest grid cell for each point
        distances, grid_idx = grid_chunk.map_reference_points.query(pc)
        mask = distances <= max_correspondence_distance

        # keep track of indices in input pc
        pc_idx = np.nonzero(mask)[0]  # indices in point cloud
        grid_idx = grid_idx[mask]  # matched grid indices

        # process planar points using mask
        mask_planar = grid_chunk.mask_planar[grid_idx]  # type: ignore
        pc_idx_planar = np.array(pc_idx)[mask_planar]
        grid_idx_planar = grid_idx[mask_planar]
        planar_normals = grid_chunk.normals[grid_idx_planar]  # type: ignore

        # process non-planar points
        pc_idx_non_planar = pc_idx[~mask_planar]
        grid_idx_non_planar = grid_idx[~mask_planar]

        # compute residual weights
        # (1) Geman-McClure kernel is applied during linear system construction

        # (2) weights based on occuppied probability
        weights_occ_prob_planar = compute_occ_prob_weights(
            grid_chunk.occ_probs[grid_idx_planar], self.weight_occ_prob_gamma
        )
        weights_occ_prob_non_planar = compute_occ_prob_weights(
            grid_chunk.occ_probs[grid_idx_non_planar],
            self.weight_occ_prob_gamma,
        )

        # (3) weights based on semantic consistency
        weights_semantic_planar = compute_semantic_weights(
            point_labels=labels[pc_idx_planar],
            grid_labels=grid_chunk.labels[grid_idx_planar],
            grid_label_probs=grid_chunk.label_probs[grid_idx_planar],
            weight_semantic_lower=self.weight_semantic_lower,
        )
        weights_semantic_non_planar = compute_semantic_weights(
            point_labels=labels[pc_idx_non_planar],
            grid_labels=grid_chunk.labels[grid_idx_non_planar],
            grid_label_probs=grid_chunk.label_probs[grid_idx_non_planar],
            weight_semantic_lower=self.weight_semantic_lower,
        )

        # create correspondence instances
        correspondences_planar = CorrespondencesPlanar(
            n=len(pc_idx_planar),
            points_src=pc[pc_idx_planar],
            points_tar=grid_chunk.map_reference_points.data[grid_idx_planar],
            normals=planar_normals,
            weights_semantic=weights_semantic_planar,
            weights_occ_prob=weights_occ_prob_planar,
        )
        correspondences_non_planar = CorrespondencesNonPlanar(
            n=len(pc_idx_non_planar),
            points_src=pc[pc_idx_non_planar],
            points_tar=grid_chunk.map_reference_points.data[grid_idx_non_planar],
            weights_semantic=weights_semantic_non_planar,
            weights_occ_prob=weights_occ_prob_non_planar,
        )

        return correspondences_planar, correspondences_non_planar


def compute_kernel_weights(residual: np.ndarray, kernel: float) -> np.ndarray:
    weight = (kernel**2) / ((kernel + residual) ** 2)
    return weight


def compute_occ_prob_weights(
    occ_probs: np.ndarray,
    gamma: float,
) -> np.ndarray:
    # check whether all probabilities are between 0.5 and 1
    if np.any(occ_probs < 0.5) or np.any(occ_probs > 1.0):
        raise ValueError("All occupancy probabilities must be between 0.5 and 1.0.")

    # w = p^gamma
    weights = np.ones_like(occ_probs) if gamma == 0.0 else occ_probs**gamma

    return weights


def compute_semantic_weights(
    point_labels, grid_labels, grid_label_probs, weight_semantic_lower: float = 0.25
) -> np.ndarray:
    # semantic weighting of correspondences
    # only perform this if any of the point or voxel labels are non-zero
    if np.all(grid_labels == 0) and np.all(point_labels == 0):
        return np.ones(shape=(len(point_labels),), dtype=float)

    # calculate semantic weights based on label matches and maximizing probabilities
    mask_match = determine_matches(point_labels, grid_labels)
    weight_semantic = (
        weight_semantic_lower + (1.0 - weight_semantic_lower) * grid_label_probs
    )
    return np.where(mask_match, weight_semantic, weight_semantic_lower)


def compute_j_r_w_planar_vectorized(
    points_src: np.ndarray,
    points_tar: np.ndarray,
    normals: np.ndarray,
    kernel: float,
):
    """
    Compute the Jacobian, residual and weight for all planar points in a vectorized manner.
    """
    # jacobians
    n_points = points_src.shape[0]
    jacobians = np.zeros((n_points, 6))
    jacobians[:, :3] = normals
    jacobians[:, 3:] = np.cross(points_src, normals)

    # residuals
    residuals = np.sum((points_src - points_tar) * normals, axis=1, keepdims=True)

    # Weight
    weights = compute_kernel_weights(residuals**2, kernel)

    return jacobians, residuals, weights


def compute_j_r_w_non_planar_vectorized(
    points_src: np.ndarray, points_tar: np.ndarray, kernel: float
):
    """
    Compute the Jacobian, residual and weight for given non-planar points using vectorized operations.
    """

    # Vectorized Jacobian construction
    batch_size = points_src.shape[0]
    jacobian = np.zeros((batch_size, 3, 6))
    jacobian[:, :3, :3] = np.eye(3)[None, :, :]  # Broadcast identity to all batches
    jacobian[:, :3, 3:] = -vec_to_skew_symmetric(points_src)

    # Residual
    residual = points_src - points_tar
    residual = residual.reshape(batch_size, 3, 1)

    # Weight
    weight = compute_kernel_weights(np.sum(residual**2, axis=1), kernel)
    weight = weight.reshape(batch_size, 1, 1)

    return jacobian, residual, weight


def build_linear_system(
    correspondences_planar: CorrespondencesPlanar,
    correspondences_non_planar: CorrespondencesNonPlanar,
    kernel: float,
    alpha: float,
):
    # build linear system
    A = np.zeros(shape=(6, 6))
    b = np.zeros(shape=(6, 1))

    # (1) planar points
    if correspondences_planar.n > 0:
        jacobians, residuals, weights_kernel = compute_j_r_w_planar_vectorized(
            points_src=correspondences_planar.points_src,
            points_tar=correspondences_planar.points_tar,
            normals=correspondences_planar.normals,
            kernel=kernel,
        )
        weights_combined = (
            weights_kernel
            * correspondences_planar.weights_occ_prob.reshape(-1, 1)
            * correspondences_planar.weights_semantic.reshape(-1, 1)
        )

        # add planar component to linear system
        A += (alpha * weights_combined * jacobians).T @ jacobians
        b += (alpha * weights_combined * jacobians).T @ residuals

    # (2) non-planar points
    if correspondences_non_planar.n > 0:
        jacobians, residuals, weights_kernel = compute_j_r_w_non_planar_vectorized(
            points_src=correspondences_non_planar.points_src,
            points_tar=correspondences_non_planar.points_tar,
            kernel=kernel,
        )
        weights_combined = (
            weights_kernel
            * correspondences_non_planar.weights_occ_prob.reshape(-1, 1, 1)
            * correspondences_non_planar.weights_semantic.reshape(-1, 1, 1)
        )

        weights_combined_sqrt = np.sqrt(weights_combined)
        weighted_jacobians = weights_combined_sqrt * jacobians
        weighted_residuals = weights_combined_sqrt * residuals

        # add non-planar component to linear system
        A += np.sum(
            (1 - alpha)
            * np.matmul(weighted_jacobians.transpose(0, 2, 1), weighted_jacobians),
            axis=0,
        )
        b += np.sum(
            (1 - alpha)
            * np.matmul(weighted_jacobians.transpose(0, 2, 1), weighted_residuals),
            axis=0,
        )

    return A, b


def determine_matches(
    labels_a: np.ndarray,
    labels_b: np.ndarray,
) -> np.ndarray:
    return (labels_a == labels_b) | (labels_a == 0) | (labels_b == 0)


def vec_to_skew_symmetric(vectors: np.ndarray):
    """
    Convert a batch of vectors to skew-symmetric matrices.
    vectors: (N, 3)
    Returns: (N, 3, 3)
    """
    N = vectors.shape[0]
    vx, vy, vz = vectors[:, 0], vectors[:, 1], vectors[:, 2]

    skew = np.zeros((N, 3, 3))
    skew[:, 0, 1] = -vz
    skew[:, 0, 2] = vy
    skew[:, 1, 0] = vz
    skew[:, 1, 2] = -vx
    skew[:, 2, 0] = -vy
    skew[:, 2, 1] = vx

    return skew
