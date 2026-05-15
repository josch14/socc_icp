import numpy as np

from sophus_pybind import SE3


def tf_to_str(T: SE3, idx: int) -> str:
    """
    FRAME_NUMBER:
    T00 T01 T02 T03
    T10 T11 T12 T13
    T20 T21 T22 T23
    0   0   0   1

    to

    FRAME_NUMBER T00 T01 T02 T03 T10 T11 T12 T13 T20 T21 T22 T23
    """
    T = T.to_matrix()

    s_line = f"{idx} "
    s_line += f"{T[0, 0]} {T[0, 1]} {T[0, 2]} {T[0, 3]} "
    s_line += f"{T[1, 0]} {T[1, 1]} {T[1, 2]} {T[1, 3]} "
    s_line += f"{T[2, 0]} {T[2, 1]} {T[2, 2]} {T[2, 3]}"
    return s_line


def str_to_tf(s_line: str) -> SE3:
    """
    FRAME_NUMBER T00 T01 T02 T03 T10 T11 T12 T13 T20 T21 T22 T23

    to

    FRAME_NUMBER:
    T00 T01 T02 T03
    T10 T11 T12 T13
    T20 T21 T22 T23
    0   0   0   1
    """
    parts = s_line.strip().split()
    if len(parts) != 13:
        raise ValueError(f"Expected 13 elements, got {len(parts)}")

    idx = int(parts[0])  # FRAME_NUMBER
    T = np.eye(4)
    T[0, :] = [float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])]
    T[1, :] = [float(parts[5]), float(parts[6]), float(parts[7]), float(parts[8])]
    T[2, :] = [float(parts[9]), float(parts[10]), float(parts[11]), float(parts[12])]

    T = SE3.from_matrix(T)
    return T, idx
