import time
import psutil
import os
import subprocess
from sophus_pybind import SE3

from radix_clients import ChunkClientGaussian

from socc_icp.core.data_classes.grid_chunk import GridChunk


def start_radix_server(sleep_time: float = 1.0):
    current_wd = os.getcwd()
    os.chdir(os.path.abspath(os.path.join(current_wd, "../..")))
    command = "source install/setup.bash && ros2 launch radix_ros radix_icp.launch.py"
    # surpress logging output
    with open("/dev/null", "w") as devnull:
        _ = subprocess.Popen(
            command,
            shell=True,
            executable="/bin/bash",
            stdout=devnull,
            stderr=devnull,
        )
    # return to the original working directory
    os.chdir(current_wd)
    time.sleep(sleep_time)


def kill_radix_servers(sleep_time: float = 1.0):
    process_name = "radix_server_node"
    found = False
    for proc in psutil.process_iter(["name"]):
        try:
            if proc.info["name"] == process_name:
                print(f"killing process {proc.pid}")
                proc.kill()
                proc.wait()
                found = True
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue

    if not found:
        print("no running Radix server to kill found")

    time.sleep(sleep_time)


def request_chunk(
    T: SE3,
    chunk_client: ChunkClientGaussian,
    chunk_range: float,
    chunk_delete_rest: bool,
    max_distance: float,
) -> GridChunk:
    if not isinstance(T, SE3):
        raise TypeError("T must be an instance of sophus_pybind.SE3")

    translation = T.translation().squeeze()

    x = translation[0]
    y = translation[1]
    z = translation[2]

    params = {
        "x_min": x - chunk_range,
        "x_max": x + chunk_range,
        "y_min": y - chunk_range,
        "y_max": y + chunk_range,
        "z_min": z - chunk_range,
        "z_max": z + chunk_range,
        "free": False,
        "delete_rest": chunk_delete_rest,
        "publish": False,
        "occupancy_threshold": 0.5,
        "level": "cell",
    }

    # send request and receive result
    radix_chunk = chunk_client.send_request(**params)

    # chunk is squared area, filter for circle around sensor location to rule out voxels that are not going to be relevant anyway
    centers = radix_chunk["center_xyz"]
    dist_squared = (centers[:, 0] - x) ** 2 + (centers[:, 1] - y) ** 2
    ind = dist_squared <= (max_distance + 1.0) ** 2  # add 1m margin

    # process into data class
    grid_chunk = GridChunk(
        n=len(radix_chunk["center_xyz"][ind]),
        centers=radix_chunk["center_xyz"][ind],
        labels=radix_chunk["label"][ind],
        label_probs=radix_chunk["label_prob"][ind],
        means=radix_chunk["mean_xyz"][ind],
        cov_matrices=radix_chunk["cov_matrix"][ind],
        n_points=radix_chunk["n_points"][ind],
        points=radix_chunk["point_xyz"][ind],
        occ_probs=radix_chunk["prob"][ind],
    )

    return grid_chunk
