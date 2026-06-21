# SOCC-ICP — Paper Logs

This directory provides the infrastructure to reproduce the trajectory evaluation metrics reported in the paper (`calculate_metrics.py`, this README, and per-dataset notes). The actual log files (trajectories and configs) are not tracked in git, they are provided as a release asset and must be downloaded separately.

## Downloading the Log Files

Download `logs_paper.zip` from the [v0.1.0 release assets](https://github.com/josch14/socc_icp/releases/tag/v0.1.0) and extract the logs into the `logs_paper/` directory.

## Directory Structure

```
logs_paper/
├── kitti/
│   ├── socc_icp/               ← with semantic labels
│   └── socc_icp_no_semantics/  ← without semantic labels
├── mulran/
│   └── socc_icp_no_semantics/
├── newer_college/
│   └── socc_icp_no_semantics/
├── ground_challenge/
│   └── socc_icp_no_semantics/
└── subt_mrs/
    └── socc_icp_no_semantics/
```

Each leaf directory holds:

- `{sequence_id}_poses_kitti.txt` — predicted trajectory in KITTI format (one $3 \times 4$ matrix per line)
- `{sequence_id}_gt_kitti.txt` — ground-truth trajectory in KITTI format
- `config.yaml` — exact scan-registration config used for this run

## Reproducing the Paper Metrics

### Prerequisites

```bash
pip install kiss-icp==1.2.3 evo==1.31.1
```

### Running the evaluation script

From this directory:

```bash
python calculate_metrics.py kitti
python calculate_metrics.py mulran
python calculate_metrics.py newer_college
python calculate_metrics.py ground_challenge
python calculate_metrics.py subt_mrs
```

### Evaluation method per dataset

| Dataset | Tool | Metrics |
|---|---|---|
| KITTI | `kiss_icp.metrics` | Translation error (%), Rotation error (deg/100m), ATE (m), ARE (rad) |
| MulRan | `kiss_icp.metrics` | same; additionally averaged per location type (DCC, KAIST, Riverside, Sejong) |
| Newer College | `kiss_icp.metrics` | same |
| Ground-Challenge | `evo_ape kitti -a` | ATE after SE(3) alignment |
| SubT-MRS | `evo_ape kitti -a` | ATE after SE(3) alignment |

The KITTI-style translation/rotation errors follow the KITTI odometry benchmark protocol (averaged over sub-sequences of 100–800 m). ATE/ARE are computed after SE(3) alignment.

## Notes on Non-Default Radix Parameters

The scan-registration `config.yaml` files reflect only the parameters of the Python registration module. Several Radix (occupancy grid mapping) parameters differ from the default across datasets and are **not** captured in those configs.

The Radix default config is: voxel size $= 0.5\,\text{m}$, $p_{\text{hit}} = 0.55$, $p_{\text{miss}} = 0.49$.

| Dataset | Radix parameters used |
|---|---|
| KITTI | default |
| Newer College | default |
| MulRan | default except $p_{\text{miss}} = 0.475$ |
| Ground-Challenge | default except voxel size $= 0.2\,\text{m}$, $p_{\text{miss}} = 0.485$ |
| SubT-MRS | default except voxel size $= 0.2\,\text{m}$, $p_{\text{miss}} = 0.485$ |
