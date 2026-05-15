import numpy as np

from socc_icp.config.config import DownsamplingParams


class Downsampling:
    def __init__(
        self,
        params: DownsamplingParams,
        semantic_downsampling_factors: dict[int, float],
    ):
        # config: downsampling params
        self.desired_num_points = params.desired_num_points
        self.voxel_size_init = params.voxel_size_init
        self.adaptive_voxel_size_min = params.adaptive_voxel_size_min
        self.adaptive_voxel_size_max = params.adaptive_voxel_size_max
        self.use_centroid = params.use_centroid

        # config: semantic downsampling factors
        self.semantic_downsampling_factors = semantic_downsampling_factors

        # current adaptive voxel size
        self.adaptive_voxel_size = params.voxel_size_init

    def downsample(
        self, pc: np.ndarray, labels: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Downsamples a point cloud using the specified method and validates input shapes and data types.

        Parameters
        ----------
        pc : np.ndarray
            Input point cloud of shape (N, 3) and dtype float32.
        labels : np.ndarray or None, optional
            Optional semantic labels for each point, shape (N,) and dtype uint32. Required for 'voxel_adaptive_semantic' method.

        Returns
        -------
        tuple[np.ndarray, np.ndarray or None]
            Downsampled point cloud and corresponding labels (if applicable).

        Raises
        ------
        ValueError
            If the input point cloud is empty, not Nx3, or has incorrect dtype.
            If labels are provided but do not match the number of points or have incorrect dtype.
            If an unsupported downsampling method is specified.
            If 'voxel_adaptive_semantic' is selected but labels are not provided.

        Downsampling Methods
        --------------------
        - "voxel_adaptive_semantic": Performs semantic-aware voxel downsampling. Requires labels.
        - "voxel_adaptive": Performs adaptive voxel downsampling without labels.
        - Other: Raises ValueError for unsupported methods.

        Notes
        -----
        - Input validation is performed for both shape and dtype of point cloud and labels.
        - Output labels are set to None for methods that do not use semantic information.
        """

        # validation
        # self._check_shape(pc, labels)
        # dont do dtype check for performance reasons
        # assumptions: pc is float32 Nx3, labels is uint32 N, N > 0

        # determine adaptive voxel size
        pc_tmp, _ = semantic_voxel_downsampling(
            pc=pc,
            labels=labels,
            semantic_downsampling_factors=self.semantic_downsampling_factors,
            voxel_size=self.adaptive_voxel_size,
            use_centroid=self.use_centroid,
        )
        n_voxelized_points = len(pc_tmp)
        adaptive_voxel_size = self._clamp(
            value=(
                self.adaptive_voxel_size
                * (n_voxelized_points / self.desired_num_points)
            ),
            min_value=self.adaptive_voxel_size_min,
            max_value=self.adaptive_voxel_size_max,
        )

        # downsample pointcloud
        pc, labels = semantic_voxel_downsampling(
            pc=pc,
            labels=labels,
            semantic_downsampling_factors=self.semantic_downsampling_factors,
            voxel_size=adaptive_voxel_size,
            use_centroid=self.use_centroid,
        )

        # update adaptive voxel size for next frame
        self.adaptive_voxel_size = adaptive_voxel_size

        return pc, labels

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float):
        return max(min(value, max_value), min_value)


def semantic_voxel_downsampling(
    pc: np.ndarray,
    labels: np.ndarray,
    semantic_downsampling_factors: dict[int, float],
    voxel_size: float = 1.0,
    use_centroid: bool = False,
) -> tuple[np.ndarray, np.ndarray]:
    # make sure the dictionary contains all labels
    labels_not_in_factors = set(np.unique(labels)) - set(
        semantic_downsampling_factors.keys()
    )
    if not len(labels_not_in_factors) == 0:
        msg = (
            f"semantic_downsampling_factors must contain all labels present in the point cloud, "
            f"but got labels {sorted(labels_not_in_factors)} not in {sorted(semantic_downsampling_factors.keys())}"
        )
        raise ValueError(msg)

    pc_downsampled, labels_downsampled = [], []

    for label, factor in semantic_downsampling_factors.items():
        # ignore/remove points/labels with None or 0.0 as downsampling factor
        if factor == 0.0:
            continue

        # extract points with the current label
        mask = labels == label
        if not np.any(mask):
            continue
        pc_grp = pc[mask]

        # downsample points
        pc_grp_down = voxel_downsampling(
            pc=pc_grp,
            voxel_size=factor * voxel_size,
            use_centroid=use_centroid,
        )

        # store downsampled points and labels
        pc_downsampled.append(pc_grp_down)
        labels_downsampled.append(
            np.full(pc_grp_down.shape[0], label, dtype=labels.dtype)
        )

    # concatenate downsampled points and labels
    if len(pc_downsampled) > 0:
        pc_downsampled = np.concatenate(pc_downsampled, axis=0)
        labels_downsampled = np.concatenate(labels_downsampled, axis=0)
    else:
        pc_downsampled = np.empty((0, pc.shape[1]), dtype=pc.dtype)
        labels_downsampled = np.empty((0,), dtype=labels.dtype)

    return pc_downsampled, labels_downsampled


def voxel_downsampling(
    pc: np.ndarray, voxel_size: float, use_centroid: bool = True
) -> np.ndarray:
    """
    Implementation based on PointCloud::VoxelDownSample of Open3D.
    See: https://github.com/isl-org/Open3D/blob/fb7088ceebef38d54c575b47935e568979b50954/cpp/open3d/geometry/PointCloud.cpp
    """

    # compute voxel indices
    min_bound = pc.min(axis=0) - voxel_size * 0.5
    ref_coords = (pc - min_bound) / voxel_size
    voxel_indices = np.floor(ref_coords).astype(np.int32)

    # hash voxel indices to 1D flat IDs
    min_idx = voxel_indices.min(axis=0)
    voxel_indices_adj = voxel_indices - min_idx  # ensure nonnegative
    grid_shape = voxel_indices_adj.max(axis=0) + 1
    flat_idx = np.ravel_multi_index(voxel_indices_adj.T, grid_shape)

    if use_centroid:
        # default behavior: compute centroids of points within each voxel

        # accumulate sums per voxel using bincount
        n_voxels = flat_idx.max() + 1
        sums = [
            np.bincount(flat_idx, weights=pc[:, i], minlength=n_voxels)
            for i in range(pc.shape[1])
        ]
        counts = np.bincount(flat_idx, minlength=n_voxels)

        # form centroids, exclude empty voxels
        mask = counts > 0
        pc_downsampled = np.stack(
            [s[mask] / counts[mask] for s in sums], axis=1
        ).astype(np.float32)

    else:
        # pick one representative point per voxel (first occurrence)
        _, unique_indices = np.unique(flat_idx, return_index=True)
        pc_downsampled = pc[unique_indices].astype(np.float32)

    return pc_downsampled
