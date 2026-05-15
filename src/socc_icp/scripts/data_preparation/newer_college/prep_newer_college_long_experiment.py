import os

from tqdm import tqdm

from .newer_college_rosbag import NewerCollegeRosbag

PATH_DATA = "/mnt/d/newer_college/02_long_experiment"
PATH_DATA_SAVE = "/mnt/d/newer_college/02_long_experiment/raw_format/ouster_scan/"


def save_point_cloud_to_pcd(points, filename):
    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # write binary PCD (faster than ASCII)
    o3d.io.write_point_cloud(filename, pcd, write_ascii=False)


def main():
    dataset = NewerCollegeRosbag(data_dir=PATH_DATA)

    for i in tqdm(range(len(dataset))):
        points = dataset[i]

        name_cloud = f"cloud_0000000000_{i:09d}.pcd"
        save_point_cloud_to_pcd(points, os.path.join(PATH_DATA_SAVE, name_cloud))


if __name__ == "__main__":
    if not os.path.exists(PATH_DATA):
        raise FileNotFoundError(f"Data path {PATH_DATA} does not exist")
    if not os.path.exists(PATH_DATA_SAVE):
        os.makedirs(PATH_DATA_SAVE)

    main()
