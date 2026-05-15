from typing import Any

import numpy as np
from sophus_pybind import SE3

from socc_icp.config.config import SoccIcpConfig
from socc_icp.core.adaptive_threshold import AdaptiveThreshold
from socc_icp.core.data_classes.grid_chunk import GridChunk
from socc_icp.core.downsampler import Downsampling
from socc_icp.core.registration import Registration


class SoccIcp:
    def __init__(self, config: SoccIcpConfig):
        self.config = config
        self.data_params = config.data_params
        self.downsampling_params = config.downsampling_params
        self.adaptive_threshold_params = config.adaptive_threshold_params
        self.registration_params = config.registration_params
        self.general_params = config.general_params
        self.semantic_downsampling_factors = config.semantic_downsampling_factors

        # params used in class methods
        self.poses: list[SE3] = []

        # important modules
        self.downsampling = Downsampling(
            params=self.downsampling_params,
            semantic_downsampling_factors=self.semantic_downsampling_factors,
        )
        self.adaptive_threshold = AdaptiveThreshold(
            params=self.adaptive_threshold_params
        )
        self.registration = Registration(params=self.registration_params)

    def register_frame(
        self,
        grid_chunk: GridChunk,
        pc_in: np.ndarray,
        labels_in: np.ndarray,
        T_initial: SE3 = SE3(),
    ) -> tuple[SE3, np.ndarray, np.ndarray, dict[str, Any]]:
        """
        Registers a new frame to the current map using ICP and adaptive thresholding.

        This method processes the input point cloud and optional label information, applies range filtering,
        downsamples the point cloud, and performs registration against the provided map chunk. It updates
        the internal pose history and adaptive threshold model.

        Args:
            grid_chunk (dict): Dictionary containing a semantic occupancy grid map chunk information, including:
                - "label": np.ndarray of map point labels
                - "label_prob": np.ndarray of label probabilities
                - "cov_matrix": np.ndarray of covariance matrices
                - "n_points": np.ndarray of number of points per map entry
                - "point_xyz": np.ndarray of map point coordinates
                - "prob": np.ndarray of occupancy probabilities
            pc_in (np.ndarray): Input point cloud of shape (N, 3), dtype np.float32.
            labels_in (np.ndarray | None): Optional input labels of shape (N,), dtype np.uint32, or None.
            T_initial (SE3, optional): Initial transformation guess for registration. Defaults to identity.

        Returns:
            tuple: (
                T_pred (SE3): Estimated transformation after registration,
                radix_map_points (np.ndarray): Map points used for registration,
                pc_filtered (np.ndarray): Filtered input point cloud,
                labels_filtered (np.ndarray | None): Filtered labels or None,
                pc_downsampled (np.ndarray): Downsampled input point cloud,
                statistics (dict[str, Any]): Dictionary of registration statistics and parameters
        """

        # remove point outside target distance
        dist_squared = np.sum(pc_in * pc_in, axis=1)
        min_dist_squared = self.registration_params.min_distance**2
        max_dist_squared = self.registration_params.max_distance**2
        ind = (dist_squared >= min_dist_squared) & (dist_squared <= max_dist_squared)
        # filter
        pc_filtered = pc_in[ind]
        labels_filtered = labels_in[ind]

        # downsample input point cloud
        pc_downsampled, labels_downsampled = self.downsampling.downsample(
            pc=pc_filtered, labels=labels_filtered
        )

        # Get adaptive_threshold
        sigma = self.adaptive_threshold.get_threshold(self.poses)

        # initial guess for ICP
        T_last_delta = self.get_prediction_model()
        T_last = self.poses[-1] if len(self.poses) > 0 else T_initial
        T_initial_guess = T_last @ T_last_delta

        # run ICP
        T_pred, statistics = self.registration.register_frame(
            grid_chunk=grid_chunk,
            pc=pc_downsampled,
            labels=labels_downsampled,
            sigma=sigma,
            T_initial_guess=T_initial_guess,
        )

        # update info for adaptive threshold
        self.adaptive_threshold.update_model_deviation(T_initial_guess, T_pred)

        # register calculated pose
        self.poses.append(T_pred)

        # collect remaining statistics
        statistics["adaptive_threshold"] = sigma
        statistics["n_downsampled"] = len(pc_downsampled)
        statistics["adaptive_voxel_size"] = self.downsampling.adaptive_voxel_size

        return (
            T_pred,
            pc_filtered,
            labels_filtered,
            statistics,
        )

    def get_prediction_model(self) -> SE3:
        """
        Returns a baseline pose prediction by calculating the
        transformation between the last two poses.

        Returns
        -------
        np.ndarray
            A 4x4 transformation matrix representing the relative transformation
            between the last two poses. If there are fewer than two poses,
            returns the identity matrix.
        """
        if len(self.poses) < 2:
            return SE3()

        # predicted transformation is the relative transformation between the last two poses
        T_last_delta = self.poses[-2].inverse() @ self.poses[-1]

        return T_last_delta
