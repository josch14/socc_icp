import os

import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2  # type: ignore
from tqdm import tqdm

sequence = "corridor1"  # "corridor1" or "corridor2"
PATH_ROSBAG = f"/mnt/e/ground-challenge/{sequence}/{sequence}.bag"
PATH_DATA_SAVE = f"/mnt/e/ground-challenge/{sequence}/clouds_with_times/"


def save_point_cloud_and_times(points: np.ndarray, times: np.ndarray, filename):
    combined = np.hstack((points, times))  # shape (N, 4)
    np.save(filename, combined)


def main():
    bag = rosbag.Bag(PATH_ROSBAG, mode="r")
    n_scans = bag.get_message_count(topic_filters="/velodyne_points")
    print(f"Processing bag file {bag.filename} with {n_scans} scans")

    msgs = bag.read_messages(topics=["/velodyne_points"])
    for i in tqdm(range(n_scans)):
        _, msg, _ = next(msgs)  # type: ignore

        # stamp info
        secs = msg.header.stamp.secs
        nsecs = msg.header.stamp.nsecs

        # pc
        points_times = np.array(
            list(pc2.read_points(msg, field_names=["x", "y", "z", "time"]))
        ).astype(np.float32)
        points = points_times[:, :3]
        times = points_times[:, 3:]

        # normalize time between 0 and 1
        if points.shape[0] > 0:
            min_time = times.min()
            max_time = times.max()
            if max_time > min_time:
                times = (times - min_time) / (max_time - min_time)
            else:
                raise ValueError("max_time should be greater than min_time")

        # save pc
        name_cloud = f"cloud_{secs:10d}_{nsecs:09d}.npy"
        save_point_cloud_and_times(
            points, times, os.path.join(PATH_DATA_SAVE, name_cloud)
        )


if __name__ == "__main__":
    if not os.path.exists(PATH_ROSBAG):
        raise FileNotFoundError(f"Data path {PATH_ROSBAG} does not exist")
    if not os.path.exists(PATH_DATA_SAVE):
        os.makedirs(PATH_DATA_SAVE)

    main()
