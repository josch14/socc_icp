# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
# Copyright (c) 2025 Johannes Scherer.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from dataclasses import dataclass

import numpy as np

from socc_icp.pipeline.pipeline import OdometryPipeline


@dataclass
class Metric:
    units: str
    values: list


def run_sequence(pipeline: OdometryPipeline, results: dict):
    # New entry to the results dictionary
    results.setdefault("dataset_name", pipeline.dataset_name)

    # Run pipeline
    print(f"Now evaluating sequence {pipeline.dataset_sequence}")
    seq_res = pipeline.run()
    seq_res.print()

    # Update the metrics dictionary
    for result in seq_res:
        results.setdefault("metrics", {}).setdefault(
            result.desc, Metric(result.units, [])
        ).values.append(result.value)

    # Update the trajectories results
    results.setdefault("trajectories", {}).update(
        {
            pipeline.dataset_sequence: {
                "gt_poses": pipeline.gt_poses,
                "poses": np.asarray(pipeline.poses).reshape(len(pipeline.poses), 4, 4),
            }
        }
    )
