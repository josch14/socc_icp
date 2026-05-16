# Dataset Preparation

Instructions for downloading and preparing the datasets used in the paper.

## KITTI

Download from the following sources:
- [Velodyne laser data, calibration files, ground truth poses](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)
- To run SOCC-ICP with ground truth labels: [SemanticKITTI label data](https://semantic-kitti.org/dataset.html#download) (179 MB file)
- To run SOCC-ICP with labels predicted by [LSK3DNet](https://arxiv.org/abs/2403.15173), download labels at [josch14/LSK3DNet_kitti_preds](https://github.com/josch14/LSK3DNet_kitti_preds)

SOCC-ICP expects the KITTI data to be structured as follows:
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

## MulRan

TODO

## Newer College

TODO

## Ground Challenge

TODO

## SubT MRS

TODO
