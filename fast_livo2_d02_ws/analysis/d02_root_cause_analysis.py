#!/usr/bin/env python3
"""Recompute the D02 causal experiment metrics from archived CSV logs."""

import csv
import math
from pathlib import Path

import numpy as np


RUN_ROOT = Path(__file__).resolve().parents[1] / "experiment_runs" / "20260715_correct_mid360"
VARIANTS = (
    ("A_free_gravity", "A_free_gravity"),
    ("B_fixed_gravity", "B_fixed_gravity"),
    ("C_fixed_acc_bias", "C_fixed_acc_bias"),
    ("D_fixed_inertial_pair", "D_fixed_inertial_pair"),
    ("E_fixed_inertial_calib", "E_fixed_inertial_calib"),
    ("F_fitted_lidar_imu_rotation", "F_fitted_lidar_imu_rotation"),
)


def read_complete_csv(path):
    with path.open(newline="") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        rows = [row for row in reader if len(row) == len(header)]
    return header, rows


def find_column(header, prefix):
    return next(i for i, name in enumerate(header) if name.startswith(prefix))


def number(row, index):
    try:
        return float(row[index])
    except (TypeError, ValueError):
        return math.nan


def first_event(rows, time_index, predicate, start_time=80.0, consecutive=1):
    for index in range(len(rows) - consecutive + 1):
        if number(rows[index], time_index) < start_time:
            continue
        if all(predicate(rows[index + offset]) for offset in range(consecutive)):
            return number(rows[index], time_index)
    return math.nan


def gravity_angle_deg(row, indices, initial):
    current = np.array([number(row, i) for i in indices])
    denominator = np.linalg.norm(current) * np.linalg.norm(initial)
    if denominator <= 0.0:
        return math.nan
    cosine = np.clip(current.dot(initial) / denominator, -1.0, 1.0)
    return math.degrees(math.acos(cosine))


def quaternion_rotation(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    x, y, z, w = qx / norm, qy / norm, qz / norm, qw / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def excitation_ratio(pose_path, start_time, end_time, first_pose_rel_time=0.3):
    poses = []
    with pose_path.open() as stream:
        for line in stream:
            values = [float(value) for value in line.split()]
            if len(values) == 8:
                poses.append(values)
    if not poses:
        return math.nan
    first_stamp = poses[0][0]
    blocks = []
    for pose in poses:
        relative_time = pose[0] - first_stamp + first_pose_rel_time
        if start_time <= relative_time < end_time:
            rotation = quaternion_rotation(*pose[4:8])
            blocks.append(np.hstack((-rotation, np.eye(3))))
    if not blocks:
        return math.nan
    sensitivity = np.vstack(blocks)
    eigenvalues = np.linalg.eigvalsh(sensitivity.T @ sensitivity / len(blocks))
    return float(eigenvalues[0] / eigenvalues[-1]) if eigenvalues[-1] > 0.0 else math.nan


def variant_metrics(run_dir, tag):
    artifact_dir = run_dir / "artifacts"
    turn_header, turn_rows = read_complete_csv(artifact_dir / f"d02_turn_risk_diag_{tag}.csv")
    structure_header, structure_rows = read_complete_csv(artifact_dir / f"d02_structure_diag_{tag}.csv")
    inertial_header, all_inertial_rows = read_complete_csv(
        artifact_dir / f"d02_inertial_observability_diag_{tag}.csv"
    )
    inertial_rows = [row for row in all_inertial_rows if row[1] == "LIO_UPDATE"]

    turn_time = find_column(turn_header, "time")
    speed = find_column(turn_header, "vel_norm")
    bias_norm = find_column(turn_header, "bias_a_norm")
    gravity_indices = tuple(find_column(turn_header, f"gravity_{axis}") for axis in "xyz")
    initial_gravity = np.array([number(turn_rows[0], i) for i in gravity_indices])

    structure_time = find_column(structure_header, "time")
    effective_ratio = find_column(structure_header, "effective_ratio")
    residual = find_column(structure_header, "avg_residual")
    visual_points = find_column(structure_header, "visual_tracked_points")
    inertial_time = find_column(inertial_header, "time")
    gyro_bias_norm = find_column(inertial_header, "bias_g_norm")

    metrics = {
        "variant": tag,
        "last_time_s": number(turn_rows[-1], turn_time),
        "first_ba_gt_0p1_s": first_event(
            turn_rows, turn_time, lambda row: number(row, bias_norm) > 0.1
        ),
        "first_bg_gt_0p03_s": first_event(
            inertial_rows, inertial_time, lambda row: number(row, gyro_bias_norm) > 0.03
        ),
        "first_gravity_angle_gt_1deg_s": first_event(
            turn_rows,
            turn_time,
            lambda row: gravity_angle_deg(row, gravity_indices, initial_gravity) > 1.0,
        ),
        "first_speed_gt_3_s": first_event(
            turn_rows, turn_time, lambda row: number(row, speed) > 3.0
        ),
        "first_effective_lt_0p5_5frames_s": first_event(
            structure_rows,
            structure_time,
            lambda row: number(row, effective_ratio) < 0.5,
            consecutive=5,
        ),
        "first_residual_gt_0p06_5frames_s": first_event(
            structure_rows,
            structure_time,
            lambda row: number(row, residual) > 0.06,
            consecutive=5,
        ),
        "first_visual_lt_20_5frames_s": first_event(
            structure_rows,
            structure_time,
            lambda row: number(row, visual_points) < 20.0,
            consecutive=5,
        ),
        "max_bias_norm": max(number(row, bias_norm) for row in turn_rows),
        "max_gyro_bias_norm": max(number(row, gyro_bias_norm) for row in inertial_rows),
        "max_gravity_angle_deg": max(
            gravity_angle_deg(row, gravity_indices, initial_gravity) for row in turn_rows
        ),
        "max_speed": max(number(row, speed) for row in turn_rows),
    }

    pose_path = artifact_dir / f"lili_2_out_first500s_fastlivo_D02_{tag}.txt"
    metrics["bias_gravity_excitation_ratio_95_105"] = excitation_ratio(pose_path, 95.0, 105.0)
    return metrics


def write_metrics(metrics, output_dir):
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "causal_variant_metrics.csv"
    fieldnames = list(metrics[0])
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(metrics)

    markdown_path = output_dir / "causal_variant_metrics.md"
    with markdown_path.open("w") as stream:
        stream.write("| " + " | ".join(fieldnames) + " |\n")
        stream.write("| " + " | ".join("---" for _ in fieldnames) + " |\n")
        for metric in metrics:
            values = []
            for field in fieldnames:
                value = metric[field]
                values.append(f"{value:.6g}" if isinstance(value, float) else str(value))
            stream.write("| " + " | ".join(values) + " |\n")
    return csv_path, markdown_path


def main():
    metrics = []
    for directory_name, tag in VARIANTS:
        run_dir = RUN_ROOT / directory_name
        artifact_dir = run_dir / "artifacts"
        if not artifact_dir.exists():
            print(f"skip incomplete run: {tag}")
            continue
        metrics.append(variant_metrics(run_dir, tag))
    if not metrics:
        raise SystemExit("no archived variants found")
    csv_path, markdown_path = write_metrics(metrics, RUN_ROOT / "analysis")
    print(csv_path)
    print(markdown_path)


if __name__ == "__main__":
    main()
