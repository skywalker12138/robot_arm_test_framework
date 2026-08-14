"""Generate target-versus-actual joint plots from a test run CSV."""

import argparse
from collections import defaultdict
import csv
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


def generate_plots(output_directory):
    """Create one six-joint PNG for every test that has samples."""
    output_directory = Path(output_directory)
    csv_path = output_directory / 'trajectory_samples.csv'
    if not csv_path.exists():
        return []

    grouped_rows = defaultdict(lambda: defaultdict(list))
    with csv_path.open(encoding='utf-8', newline='') as csv_file:
        for row in csv.DictReader(csv_file):
            test_number = int(row['test_number'])
            joint_name = row['joint_name']
            grouped_rows[test_number][joint_name].append({
                'time': float(row['elapsed_seconds']),
                'target': float(row['target_position']),
                'actual': float(row['actual_position']),
            })

    created_files = []
    for test_number, joint_data in sorted(grouped_rows.items()):
        figure, axes = plt.subplots(2, 3, figsize=(14, 8), sharex=True)
        for axis, (joint_name, rows) in zip(
            axes.flat,
            sorted(joint_data.items()),
        ):
            times = [row['time'] for row in rows]
            targets = [row['target'] for row in rows]
            actuals = [row['actual'] for row in rows]
            axis.plot(times, targets, '--', label='Target', linewidth=1.5)
            axis.plot(times, actuals, label='Actual', linewidth=1.5)
            axis.set_title(joint_name)
            axis.set_xlabel('Time (s)')
            axis.set_ylabel('Position (rad)')
            axis.grid(True, alpha=0.3)
            axis.legend()

        figure.suptitle(f'Joint Trajectory Test {test_number}')
        figure.tight_layout()
        file_path = output_directory / f'trajectory_test_{test_number:02d}.png'
        figure.savefig(file_path, dpi=150)
        plt.close(figure)
        created_files.append(file_path)

    return created_files


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result_directory', type=Path)
    args = parser.parse_args()
    created_files = generate_plots(args.result_directory)
    print(f'Created {len(created_files)} plot(s) in {args.result_directory}')


if __name__ == '__main__':
    main()
