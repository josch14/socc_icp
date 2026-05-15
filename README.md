# SOCC-ICP: Semantics-Assisted Odometry based on Occupancy Grids and ICP

**[RA-L 2026]** &nbsp;|&nbsp; [Paper (arXiv)](https://arxiv.org/abs/2605.15074)

**Abstract:** Reliable pose estimation in previously unseen environments is a fundamental capability of autonomous systems. Existing LiDAR odometry methods typically employ point-, surfel-, or NDT-based map representations, which are distinct from the semantic occupancy grids commonly used for downstream tasks such as motion planning. We introduce SOCC-ICP, a semantics-assisted odometry framework that jointly performs Semantic OCCupancy grid mapping and LiDAR scan alignment. Each map voxel encodes geometric and semantic statistics, enabling adaptive point-to-point or point-to-plane ICP based on local planarity. Further, the occupancy grid naturally filters dynamic objects through raycasting-based free-space updates. Across diverse evaluation scenarios, SOCC-ICP achieves performance competitive with state-of-the-art LiDAR odometry and remains robust in geometrically degenerate environments, even in the absence of semantic cues. When semantic labels are available, integrating them into map construction, downsampling, and correspondence weighting yields further accuracy gains. By unifying odometry and semantic occupancy grid mapping within a single representation, SOCC-ICP eliminates redundant map structures and directly provides a map suitable for downstream robotic applications.

<img src="assets/socc-icp.png" style="max-width: 100%; height: auto;">
<img src="assets/cleaning-ray.png" style="max-width: 100%; height: auto;">


## 📖 Overview

SOCC-ICP is implemented on ROS2 Humble, with [Radix](https://github.com/ProjectVERUM/radix_ros2_pkg) handling semantic occupancy grid mapping in a separate node and scan registration currently performed in Python. The current system is a proof of concept and is not optimized for efficiency. This repository is intended for reproducing the results reported in the paper, an odometry server or a highly optimized implementation is currently not provided 🤖

This repository contains the scan registration module as described in the paper. The semantic occupancy grid mapping side is handled by the [radix_ros2_pkg](https://github.com/ProjectVERUM/radix_ros2_pkg) submodule and its companions, included here as git submodules under `src/`.


## 🚀 Getting Started

See [INSTALLATION.md](INSTALLATION.md) for full setup instructions.


## 📦 Dataset Preparation

### KITTI

Download from the following sources:
- [Velodyne laser data, calibration files, ground truth poses](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)
- [SemanticKITTI label data](https://semantic-kitti.org/dataset.html#download) (179 MB)
- Predicted labels from [LSK3DNet](https://arxiv.org/abs/2403.15173) — available at [josch14/LSK3DNet_kitti_preds](https://github.com/josch14/LSK3DNet_kitti_preds)

Expected directory structure:
```
kitti_dataset/
├── poses/
│   ├── 00.txt
│   ├── 01.txt
│   └── ...
└── sequences/
    ├── 00/
    │   ├── velodyne/
    │   │   ├── 000000.bin
    │   │   └── ...
    │   ├── labels/
    │   │   ├── 000000.label
    │   │   └── ...
    │   ├── labels_lsk3dnet/
    │   │   ├── 000000.label
    │   │   └── ...
    │   ├── calib.txt
    │   └── times.txt
    └── ...
```

TODO add info for other datasets


## ▶️ Usage

From `src/socc_icp/`:
```bash
cd src/socc_icp/
```

**KITTI** (with / without semantics):
```bash
python -m run.run_kitti
python -m run.run_kitti --no-semantics
```

**MulRan:**
```bash
python -m run.run_mulran
```

**Newer College:**
```bash
python -m run.run_newer_college
```

**Ground Challenge:**
```bash
python -m run.run_ground_challenge
```

**SubT-MRS:**
```bash
python -m run.run_subt_mrs
```

TODO add info for evaluation


## 🙏 Acknowledgements

This work builds on [KISS-ICP](https://github.com/PRBonn/kiss-icp) and [GenZ-ICP](https://github.com/cocel-postech/genz-icp), as well as [Bonxai](https://github.com/facontidavide/Bonxai). Without their foundational contributions SOCC-ICP would not have been possible!


## 📬 Contact

For questions or feedback, feel free to [open an issue](https://github.com/josch14/socc_icp/issues) or reach out via johannes.scherer.ivi@outlook.com


## 📝 Citation

If you find this work useful, please cite:

```bibtex
@misc{socc-icp,
      title={SOCC-ICP: Semantics-Assisted Odometry based on Occupancy Grids and ICP},
      author={Johannes Scherer and Sebastian Hirt and Henri Meeß},
      year={2026},
      eprint={2605.15074},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2605.15074},
}
```
