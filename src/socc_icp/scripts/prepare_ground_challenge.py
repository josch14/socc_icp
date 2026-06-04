import os

import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2  # type: ignore
from tqdm import tqdm

BASE_DIR = "/home/ground-challenge"
SEQUENCES = ["corridor1", "corridor2"]


def save_point_cloud_and_times(points: np.ndarray, times: np.ndarray, filename):
    combined = np.hstack((points, times))  # shape (N, 4)
    np.save(filename, combined)


def process_sequence(sequence: str):
    path_rosbag = os.path.join(BASE_DIR, sequence, f"{sequence}.bag")
    path_data_save = os.path.join(BASE_DIR, sequence, "clouds_with_times")

    if not os.path.exists(path_rosbag):
        raise FileNotFoundError(f"Data path {path_rosbag} does not exist")
    os.makedirs(path_data_save, exist_ok=True)

    bag = rosbag.Bag(path_rosbag, mode="r")
    n_scans = bag.get_message_count(topic_filters="/velodyne_points")
    print(f"Processing bag file {bag.filename} with {n_scans} scans")

    msgs = bag.read_messages(topics=["/velodyne_points"])
    for i in tqdm(range(n_scans)):
        _, msg, _ = next(msgs)  # type: ignore

        secs = msg.header.stamp.secs
        nsecs = msg.header.stamp.nsecs

        points_times = np.array(
            list(pc2.read_points(msg, field_names=["x", "y", "z", "time"]))
        ).astype(np.float32)
        points = points_times[:, :3]
        times = points_times[:, 3:]

        if points.shape[0] > 0:
            min_time = times.min()
            max_time = times.max()
            if max_time > min_time:
                times = (times - min_time) / (max_time - min_time)
            else:
                raise ValueError("max_time should be greater than min_time")

        name_cloud = f"cloud_{secs:10d}_{nsecs:09d}.npy"
        save_point_cloud_and_times(
            points, times, os.path.join(path_data_save, name_cloud)
        )


if __name__ == "__main__":
    for sequence in SEQUENCES:
        process_sequence(sequence)
