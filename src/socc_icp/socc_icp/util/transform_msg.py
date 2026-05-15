import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import (
    PoseStamped,
    Transform,
)
from scipy.spatial.transform import Rotation as R
from sophus_pybind import SE3


def get_pose_stamped(transform: Transform, stamp: Time) -> PoseStamped:
    translation = transform.translation
    rotation = transform.rotation

    # Create PoseStamped from tf
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = "map"
    pose.pose.position.x = translation.x
    pose.pose.position.y = translation.y
    pose.pose.position.z = translation.z
    pose.pose.orientation.x = rotation.x
    pose.pose.orientation.y = rotation.y
    pose.pose.orientation.z = rotation.z
    pose.pose.orientation.w = rotation.w

    return pose


def transform_to_se3(transform: Transform) -> SE3:
    """
    Convert a ROS Transform message to an SE3 transformation.
    """
    # Scipy rotation format is (x, y, z, w)
    quat = [
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    ]
    rot_matrix = R.from_quat(quat).as_matrix()
    trans = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ]
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = rot_matrix
    T[:3, 3] = trans
    T = SE3.from_matrix(T)

    return T


def pose_stamped_to_se3(pose: PoseStamped) -> SE3:
    """
    Convert a ROS PoseStamped message to an SE3 transformation.
    """
    trans = np.array(
        [
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ],
        dtype=np.float32,
    )
    quat = np.array(
        [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ],
        dtype=np.float32,
    )
    rot_matrix = R.from_quat(quat).as_matrix()
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = rot_matrix
    T[:3, 3] = trans
    return SE3.from_matrix(T)


def se3_to_transform(T: SE3) -> Transform:
    """
    Convert an SE3 transformation to a ROS Transform message.
    """
    translation = T.translation().squeeze()
    q = T.rotation().to_quat().squeeze()  # (w, x, y, z)

    transform = Transform()

    transform.translation.x = translation[0]
    transform.translation.y = translation[1]
    transform.translation.z = translation[2]

    transform.rotation.x = q[1]
    transform.rotation.y = q[2]
    transform.rotation.z = q[3]
    transform.rotation.w = q[0]

    return transform
