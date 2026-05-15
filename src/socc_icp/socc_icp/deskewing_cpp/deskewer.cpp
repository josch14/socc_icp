#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <tbb/parallel_for.h> // include TBB

#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;
using SE3d = Sophus::SE3d;

class Deskewer
{
public:
    explicit Deskewer(const std::string &mode = "mid") : mode_(mode)
    {
        if (mode_ != "mid" && mode_ != "end" && mode_ != "none")
        {
            throw std::invalid_argument("Invalid mode '" + mode_ +
                                        "', must be 'mid', 'end' or 'none'.");
        }
    }

    py::array_t<double> deskew_scan(
        const py::array_t<double> &points_in,
        const py::array_t<double> &timestamps_in,
        const py::array_t<double> &pose_np) const
    {
        // --- Convert numpy 4x4 to Sophus::SE3d ---
        if (pose_np.ndim() != 2 || pose_np.shape(0) != 4 || pose_np.shape(1) != 4)
            throw std::invalid_argument("Pose must be a 4x4 numpy array.");
        auto pose_buf = pose_np.unchecked<2>();
        Eigen::Matrix4d pose_mat;
        for (ssize_t i = 0; i < 4; ++i)
            for (ssize_t j = 0; j < 4; ++j)
                pose_mat(i, j) = pose_buf(i, j);
        SE3d T_delta_last(pose_mat);

        if (mode_ == "none")
            return points_in;

        auto pts = points_in.unchecked<2>();
        auto ts = timestamps_in.unchecked<1>();
        if (pts.shape(0) != ts.shape(0))
            throw std::invalid_argument("Number of points does not match number of timestamps.");
        if (pts.shape(1) != 3)
            throw std::invalid_argument("Points must have shape (N,3).");

        Eigen::Matrix<double, 6, 1> omega = T_delta_last.log();
        if (omega.isZero(1e-12))
            throw std::invalid_argument("Delta pose is too small for de-skewing.");

        std::vector<double> stamps(ts.shape(0));
        if (mode_ == "mid")
        {
            for (ssize_t i = 0; i < ts.shape(0); ++i)
            {
                double t = ts(i);
                if (t < 0.0 || t > 1.0)
                    throw std::invalid_argument("For 'mid' mode, timestamps must be normalized to [0,1].");
                stamps[i] = t - 0.5;
            }
        }
        else if (mode_ == "end")
        {
            double t_min = ts(0), t_max = ts(0);
            for (ssize_t i = 1; i < ts.shape(0); ++i)
            {
                if (ts(i) < t_min)
                    t_min = ts(i);
                if (ts(i) > t_max)
                    t_max = ts(i);
            }
            double denom = t_max - t_min;
            if (denom <= 0)
                throw std::invalid_argument("Invalid timestamps: max <= min.");
            for (ssize_t i = 0; i < ts.shape(0); ++i)
            {
                stamps[i] = ((ts(i) - t_min) / denom) - 1.0;
            }
        }

        py::array_t<double> result(std::vector<py::ssize_t>{pts.shape(0), 3});
        auto res = result.mutable_unchecked<2>();

        tbb::parallel_for(ssize_t(0), pts.shape(0), [&](ssize_t i)
                          {
            Eigen::Matrix<double, 6, 1> scaled_omega = stamps[i] * omega;
            SE3d pose = SE3d::exp(scaled_omega);

            Eigen::Vector3d point(pts(i, 0), pts(i, 1), pts(i, 2));
            Eigen::Vector3d p_new = pose * point;

            res(i, 0) = p_new(0);
            res(i, 1) = p_new(1);
            res(i, 2) = p_new(2); });

        return result;
    }

private:
    std::string mode_;
};

// --- Pybind wrapper ---
PYBIND11_MODULE(sophus_deskewer, m)
{
    py::class_<Deskewer>(m, "Deskewer")
        .def(py::init<const std::string &>(), py::arg("mode") = "mid")
        .def("deskew_scan", &Deskewer::deskew_scan,
             py::arg("points"), py::arg("timestamps"), py::arg("pose"));
}

/* Corresponding Python code for reference:

import numpy as np
from sophus_pybind import SE3


class Deskewer:
    def __init__(self, mode: str = "mid") -> None:
        if mode not in ("mid", "end", "none"):
            raise ValueError(f"Invalid mode '{mode}', must be 'mid', 'end' or 'none'.")
        self.mode = mode

    def deskew_scan(
        self,
        points: np.ndarray,
        timestamps: np.ndarray,
        T_delta_last: SE3,
    ) -> np.ndarray:
        """
        Deskews a LiDAR scan by compensating for motion distortion using the provided delta pose.

        Args:
            points (np.ndarray): Array of 3D points from the scan, shape (N, 3).
            timestamps (np.ndarray): Array of timestamps for each point, shape (N,).
            T_delta_last (SE3): SE3 transformation representing the pose change during the scan.
            mode (str): Reference pose choice:
                        - "mid": mid-scan reference (GENZICP-style, assumes normalized timestamps in [0,1]).
                        - "end": end-of-scan reference (KISSICP-style, normalizes timestamps automatically).

        Returns:
            np.ndarray: Deskewed array of 3D points, shape (N, 3).
        """
        if self.mode == "none":
            return points

        # validation
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(
                f"Points array must have shape (N, 3), got {points.shape}."
            )
        if timestamps.ndim != 1:
            raise ValueError(
                f"Timestamps array must be 1D, got shape {timestamps.shape}."
            )
        if points.shape[0] != timestamps.shape[0]:
            # MulRan points may be broken, in that case timestamsp will be of shape (0,)
            # in that case, do not perform de-skewing and do not raise error
            if timestamps.shape[0] > 0:
                raise ValueError(
                    f"Number of points ({points.shape[0]}) does not match number of timestamps ({timestamps.shape[0]})."
                )
            else:
                return points

        # log
        omega = T_delta_last.log().ravel()
        if np.allclose(omega, 0):
            raise ValueError("Delta pose is too small for de-skewing.")

        if self.mode == "mid":
            # make sure timestamps already normalized to [0,1]
            if not np.all((timestamps >= 0) & (timestamps <= 1)):
                raise ValueError(
                    "For 'mid' mode, timestamps must be normalized to [0, 1]."
                )

            stamps = (timestamps - 0.5).reshape(-1, 1)  # MID_POSE_TIMESTAMP = 0.5

        elif self.mode == "end":
            # normalizes timestamps based on min/max
            t_min, t_max = np.min(timestamps), np.max(timestamps)
            stamps = ((timestamps - t_min) / (t_max - t_min) - 1.0).reshape(-1, 1)

        else:
            raise ValueError(f"Invalid mode '{self.mode}', must be 'mid' or 'end'.")

        # Compute motion for each point
        omega_points = stamps * omega  # (N,6)
        poses = SE3.exp(omega_points[:, :3], omega_points[:, 3:])

        deskewed_frame = np.array(
            [(poses[i] @ points[i]).ravel() for i in range(points.shape[0])],
            dtype=np.float32,
        )

        return deskewed_frame
*/