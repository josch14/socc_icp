import json
import os
import re
from collections import defaultdict

RESULTS_DIRECTORY = "log"


def read_all_result_metrics(dir_results):
    result_strings = []
    # List all directories in log_best_dir
    for entry in os.listdir(dir_results):
        dir_path = os.path.join(dir_results, entry)
        if os.path.isdir(dir_path):
            result_file = os.path.join(dir_path, "result_metrics.log")
            assert os.path.isfile(result_file), (
                f"Missing result_metrics.log in {dir_path}"
            )
            with open(result_file, "r") as f:
                result_strings.append(f.read())
    return result_strings


if __name__ == "__main__":
    results = read_all_result_metrics(RESULTS_DIRECTORY)
    print(f"Read {len(results)} result_metrics.log files.")

    def parse_result(result_str):
        # Extract sequence number
        seq_match = re.search(r"Sequence (\d+)", result_str)
        if not seq_match:
            return None, None
        seq_num = seq_match.group(1)

        # Extract metrics
        metrics = {}
        # Find all metric rows
        metric_pattern = re.compile(
            r"\|\s*([^\|]+?)\s*\|\s*([^\|]+?)\s*\|\s*([^\|]+?)\s*\|"
        )
        rows = metric_pattern.findall(result_str)
        # Skip the header row
        for row in rows[1:]:
            metric, value, _ = row
            metric = metric.strip()
            value = value.strip()
            metrics[metric] = value
        return seq_num, metrics

    results_dict = {}
    for r in results:
        seq_num, metrics = parse_result(r)
        if seq_num and metrics:
            results_dict[seq_num] = metrics

    # Sort the dictionary by sequence number (as integer)
    results_dict = dict(sorted(results_dict.items(), key=lambda x: int(x[0])))

    # Collect metrics per metric name
    metric_values = defaultdict(dict)
    for seq_num, metrics in results_dict.items():
        for metric, value in metrics.items():
            try:
                val = float(value)
            except ValueError:
                continue
            metric_values[metric][seq_num] = val

    for metric, seq_vals in metric_values.items():
        avg = sum(seq_vals.values()) / len(seq_vals)
        metric_values[metric]["mean"] = avg

    # print results and save to file
    print(json.dumps(metric_values, indent=4))
    with open(os.path.join(RESULTS_DIRECTORY, "metrics_summary.json"), "w") as f:
        json.dump(metric_values, f, indent=4)
