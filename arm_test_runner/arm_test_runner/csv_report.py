"""CSV report helpers for the arm trajectory tests."""

import csv
from datetime import datetime
from pathlib import Path


def create_output_directory():
    """Create one timestamped output directory for the current run."""
    run_name = datetime.now().strftime('run_%Y%m%d_%H%M%S')
    output_directory = Path.home() / 'ros2_ws' / 'test_results' / run_name
    output_directory.mkdir(parents=True, exist_ok=True)
    return output_directory


def save_reports(
    output_directory,
    test_results,
    joint_result_rows,
    trajectory_rows,
):
    """Write summary, final-position, and time-series CSV files."""
    _write_csv(
        output_directory / 'test_summary.csv',
        ['test_number', 'passed', 'average_error', 'maximum_error', 'reason'],
        test_results,
    )
    _write_csv(
        output_directory / 'joint_results.csv',
        [
            'test_number', 'joint_name', 'target_position', 'actual_position',
            'absolute_error', 'tolerance', 'joint_passed',
        ],
        joint_result_rows,
    )
    _write_csv(
        output_directory / 'trajectory_samples.csv',
        [
            'test_number', 'elapsed_seconds', 'joint_name',
            'target_position', 'actual_position', 'absolute_error',
        ],
        trajectory_rows,
    )


def _write_csv(file_path, field_names, rows):
    with file_path.open(mode='w', newline='', encoding='utf-8') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=field_names)
        writer.writeheader()
        writer.writerows(rows)
