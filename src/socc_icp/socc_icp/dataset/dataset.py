from abc import ABC, abstractmethod
import numpy as np


class Dataset(ABC):
    def __init__(self):
        self.sequence_id: str
        self.gt_poses: np.ndarray

    @abstractmethod
    def __getitem__(self, idx: int) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
        """Returns a tuple of (points, labels, timestamps).

        - points: np.ndarray of shape (N, 3) containing x, y, z coordinates in float32.
        - labels: np.ndarray of shape (N,) with labels per point in uint32.
        - timestamps: np.ndarray of shape (N,) with relative timestamps per point (float32), or None if unavailable.
        """
        pass

    @abstractmethod
    def __len__(self) -> int:
        pass
