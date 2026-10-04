#!/usr/bin/env python3

import bisect
import copy
import hashlib
import json
import math
import sys
from collections import Counter
from pathlib import Path

import rosbag
import numpy as np
import yaml


EXPECTED_MODES = ("stock_baseline", "all_off", "legacy_3x3", "coupled_6x6", "full")
CURRENT_MODES = set(EXPECTED_MODES) - {"stock_baseline"}
EXPECTED_LIO_WIDTH = 104
EXPECTED_VIO_WIDTH = 69
EXPECTED_RUNTIME_ARTIFACTS = {
    "fastlivo_mapping", "liblaser_mapping.so", "liblio.so", "libvio.so",
    "libpre.so", "libimu_proc.so",
}
COVERAGE_END_TOLERANCE_SECONDS = 0.75
VOLATILE_PARAMETER_KEYS = {"rosdistro", "roslaunch", "rosversion", "run_id"}
ADAPTIVE_SWITCHES = (
    "degeneracy_aware_en", "coupled_degeneracy_en", "normal_anisotropy_weight_en",
    "normal_balance_en", "incidence_weight_en", "robust_kernel_en",
    "scan_distortion_weight_en", "lidar_return_quality_en", "lidar_range_adaptive_en",
    "lidar_range_candidate_source_en", "map_write_gate_en", "dynamic_object_filter_en",
    "plane_quality_weight_en", "vio_quality_en", "vio_cov_fusion_en",
    "visual_lifecycle_en", "visual_view_weight_en", "cross_modal_gate_en",
    "temporal_degrade_en", "ref_patch_adaptive_en", "motion_degrade_en",
    "vibration_degrade_en", "preprocess_adaptive_en", "preprocess_intensity_quality_en",
    "lio_hard_outlier_reject_en", "map_write_recovery_en",
)


def write_json(path, value):
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False),
        encoding="utf-8",
    )


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_digest(value):
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def nonfinite_parameter_paths(value, prefix=""):
    paths = []
    if isinstance(value, dict):
        for key, child in value.items():
            child_prefix = f"{prefix}/{key}" if prefix else str(key)
            paths.extend(nonfinite_parameter_paths(child, child_prefix))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            paths.extend(nonfinite_parameter_paths(child, f"{prefix}[{index}]"))
    elif isinstance(value, float) and not math.isfinite(value):
        paths.append(prefix)
    return paths


def percentile(values, probability):
    if not values:
        return None
    if not all(math.isfinite(value) for value in values):
        raise ValueError("non-finite value passed to percentile")
    ordered = sorted(values)
    position = probability * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def stats(values):
    if not values:
        return {"count": 0, "mean": None, "p95": None, "p99": None, "max": None, "min": None}
    if not all(math.isfinite(value) for value in values):
        raise ValueError("non-finite value passed to stats")
    return {
        "count": len(values),
        "mean": sum(values) / len(values),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
        "max": max(values),
        "min": min(values),
    }


def parse_key_value_file(path):
    result = {}
    errors = []
    if not path.exists():
        return result, [f"missing {path.name}"]
    for line_number, line in enumerate(
            path.read_text(encoding="utf-8", errors="replace").splitlines(), start=1):
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            errors.append(f"{path.name}:{line_number}: expected key=value")
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        if not key:
            errors.append(f"{path.name}:{line_number}: empty key")
            continue
        if key in result:
            errors.append(f"{path.name}:{line_number}: duplicate key {key}")
        result[key] = value.strip()
    return result, errors


def validate_status(status, mode):
    errors = []
    required = (
        "mode", "stage", "success", "script_exit", "player_exit", "node_exit",
        "record_exit", "player_timed_out", "node_timed_out", "record_timed_out",
        "health_failure", "compressed_image", "republisher_alive_through_run",
        "contract_version", "use_sim_time", "bag_realpath", "bag_bytes", "bag_sha256",
        "source_config_sha256", "camera_config_sha256", "mapping_binary_sha256",
        "runtime_bundle_sha256", "parameters_sha256", "duration", "rate",
    )
    for key in required:
        if key not in status:
            errors.append(f"status missing {key}")
    if status.get("mode") != mode:
        errors.append(f"status mode {status.get('mode')!r} does not match directory {mode!r}")
    if status.get("stage") != "complete":
        errors.append(f"run stopped in stage {status.get('stage')!r}")
    if status.get("success") != "true":
        errors.append("status success is not true")
    for key in ("script_exit", "player_exit", "node_exit", "record_exit"):
        if status.get(key) != "0":
            errors.append(f"{key} is {status.get(key)!r}, expected '0'")
    for key in ("player_timed_out", "node_timed_out", "record_timed_out"):
        if status.get(key) != "false":
            errors.append(f"{key} is {status.get(key)!r}, expected 'false'")
    if status.get("health_failure") != "none":
        errors.append(f"health_failure is {status.get('health_failure')!r}")
    if status.get("compressed_image") == "true" and status.get("republisher_alive_through_run") != "true":
        errors.append("compressed-image republisher did not remain alive through replay")
    if status.get("contract_version") != "3":
        errors.append(f"unsupported experiment contract {status.get('contract_version')!r}; expected '3'")
    if status.get("use_sim_time") != "true":
        errors.append("run did not use simulated replay time")
    for key in ("bag_sha256", "source_config_sha256", "camera_config_sha256",
                "mapping_binary_sha256", "runtime_bundle_sha256", "parameters_sha256"):
        value = status.get(key, "")
        if len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
            errors.append(f"status {key} is not a lowercase SHA-256 digest")
    try:
        if int(status.get("bag_bytes", "0")) <= 0:
            errors.append("status bag_bytes is not positive")
    except ValueError:
        errors.append(f"status bag_bytes is not an integer: {status.get('bag_bytes')!r}")
    for key in ("duration", "rate"):
        try:
            value = float(status.get(key, "nan"))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError
        except ValueError:
            errors.append(f"status {key} is not a positive finite number")
    return errors


def expected_switch_values(mode):
    default = mode == "full"
    expected = {key: default for key in ADAPTIVE_SWITCHES}
    if mode == "legacy_3x3":
        expected["degeneracy_aware_en"] = True
        expected["coupled_degeneracy_en"] = False
    elif mode == "coupled_6x6":
        expected["degeneracy_aware_en"] = True
        expected["coupled_degeneracy_en"] = True
    return expected


def validate_parameters(path, mode):
    if not path.exists():
        return {}, {}, ["missing parameters.yaml"]
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8", errors="strict")) or {}
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        return {}, {}, [f"cannot parse parameters.yaml: {error}"]
    if not isinstance(document, dict):
        return {}, {}, ["parameters.yaml root is not a mapping"]
    errors = []
    bad_paths = nonfinite_parameter_paths(document)
    if bad_paths:
        errors.append("parameters.yaml contains non-finite values at " + ", ".join(bad_paths[:10]))
    adaptive = document.get("adaptive")
    if not isinstance(adaptive, dict):
        return {}, {}, errors + ["parameters.yaml has no adaptive mapping"]
    expected = expected_switch_values(mode)
    actual = {}
    for key, expected_value in expected.items():
        value = adaptive.get(key)
        actual[key] = value
        if not isinstance(value, bool):
            errors.append(f"adaptive/{key} is not a boolean: {value!r}")
        elif value != expected_value:
            errors.append(f"adaptive/{key}={value}, expected {expected_value} for {mode}")

    non_ablation = copy.deepcopy(document)
    for key in VOLATILE_PARAMETER_KEYS:
        non_ablation.pop(key, None)
    normalized_adaptive = non_ablation.get("adaptive")
    if isinstance(normalized_adaptive, dict):
        for key in ADAPTIVE_SWITCHES:
            normalized_adaptive.pop(key, None)
    return actual, non_ablation, errors


def validate_runtime_manifest(path, expected_digest):
    errors = []
    entries = {}
    if not path.exists():
        return errors + ["missing runtime_manifest.txt"]
    try:
        lines = path.read_text(encoding="utf-8", errors="strict").splitlines()
    except (OSError, UnicodeError) as error:
        return errors + [f"cannot read runtime_manifest.txt: {error}"]
    for line_number, line in enumerate(lines, start=1):
        fields = line.split()
        if len(fields) != 2:
            errors.append(f"runtime_manifest.txt:{line_number}: expected name sha256")
            continue
        name, digest = fields
        if name in entries:
            errors.append(f"runtime_manifest.txt:{line_number}: duplicate artifact {name}")
        if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
            errors.append(f"runtime_manifest.txt:{line_number}: invalid SHA-256 for {name}")
        entries[name] = digest
    if set(entries) != EXPECTED_RUNTIME_ARTIFACTS:
        errors.append(
            "runtime artifact set mismatch: "
            f"missing={sorted(EXPECTED_RUNTIME_ARTIFACTS - set(entries))}, "
            f"extra={sorted(set(entries) - EXPECTED_RUNTIME_ARTIFACTS)}")
    actual_digest = sha256_file(path)
    if actual_digest != expected_digest:
        errors.append("runtime_manifest.txt SHA-256 does not match status contract")
    return errors


def parse_time_file(path):
    result = {}
    if not path.exists():
        return result
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if ": " not in line:
            continue
        # GNU time keys such as "Elapsed ... (h:mm:ss or m:ss)" contain
        # colons themselves; the final colon-space separates the value.
        key, value = line.rsplit(": ", 1)
        result[key.strip()] = value.strip()
    return result


def diagnostic_integrity(rows, expected_width, strict_schema, label, errors):
    widths = Counter(len(row) for row in rows)
    nonfinite_rows = sum(not all(math.isfinite(value) for value in row) for row in rows)
    summary = {
        "count": len(rows),
        "widths": {str(width): count for width, count in sorted(widths.items())},
        "nonfinite_rows": nonfinite_rows,
    }
    if strict_schema and not rows:
        errors.append(f"missing {label} diagnostics")
    if strict_schema and set(widths) != {expected_width}:
        errors.append(f"{label} schema widths {sorted(widths)} do not equal [{expected_width}]")
    if nonfinite_rows:
        errors.append(f"{label} contains {nonfinite_rows} non-finite rows")
    return summary


def path_metrics(samples, total_messages, invalid_messages, monotonic):
    result = {
        "count": total_messages,
        "valid_count": len(samples),
        "invalid_count": invalid_messages,
        "finite": invalid_messages == 0,
        "timestamp_monotonic": monotonic,
        "path_length": None,
        "final_displacement": None,
        "start_time": None,
        "end_time": None,
        "duration": None,
    }
    if not samples or invalid_messages or not monotonic:
        return result
    points = [point for _, point in samples]
    length = 0.0
    for first, second in zip(points, points[1:]):
        length += math.sqrt(sum((second[index] - first[index]) ** 2 for index in range(3)))
    origin = points[0]
    endpoint = points[-1]
    result.update({
        "path_length": length,
        "final_displacement": math.sqrt(
            sum((endpoint[index] - origin[index]) ** 2 for index in range(3))),
        "start_time": samples[0][0],
        "end_time": samples[-1][0],
        "duration": samples[-1][0] - samples[0][0],
    })
    return result


def add_lio_metrics(summary, rows):
    if not rows or any(len(row) < EXPECTED_LIO_WIDTH for row in rows) or any(
            not all(math.isfinite(value) for value in row) for row in rows):
        return
    recovery_rows = [row for row in rows if row[98] > 0.5]
    recovery_state_counts = {
        str(state): sum(int(round(row[88])) == state for row in rows)
        for state in range(5)
    }
    summary.update({
        "frame_time_seconds": stats([row[6] for row in rows]),
        "effective_points": stats([row[3] for row in rows]),
        "residual_m": stats([row[4] for row in rows if row[4] >= 0.0]),
        "converged_ratio": sum(row[5] > 0.5 for row in rows) / len(rows),
        "pose_cov_trace": stats([row[7] for row in rows]),
        "degeneracy_factor": stats([row[8] for row in rows]),
        "map_write_accept_ratio": stats([row[28] for row in rows]),
        "effective_weight": stats([row[38] for row in rows]),
        "preprocess_filter_num": stats([row[40] for row in rows]),
        "preprocess_quality": stats([row[46] for row in rows]),
        "coupled_active_ratio": sum(row[55] > 0.5 for row in rows) / len(rows),
        "numerical_fallback_frames": sum(row[61] > 0.5 for row in rows),
        "covariance_reset_angle": stats([row[67] for row in rows]),
        "covariance_min_eigenvalue": stats([row[68] for row in rows]),
        "covariance_negative_count": sum(row[68] < -1e-12 for row in rows),
        "covariance_symmetry_error": stats([row[69] for row in rows]),
        "covariance_relinearization_angle": stats([row[70] for row in rows]),
        "map_write_quality_risk": stats([row[71] for row in rows]),
        "map_write_starvation_frames": stats([row[72] for row in rows]),
        "map_write_recovery_active_ratio": sum(row[73] > 0.5 for row in rows) / len(rows),
        "map_write_recovery_points": stats([row[74] for row in rows]),
        "hard_outlier_reject_points": stats([row[75] for row in rows]),
        "residual_consistency_weight": stats([row[76] for row in rows]),
        "candidate_map_voxels": stats([row[77] for row in rows]),
        "candidate_map_points": stats([row[78] for row in rows]),
        "candidate_promoted_voxels": stats([row[79] for row in rows]),
        "candidate_expired_voxels": stats([row[80] for row in rows]),
        "formal_map_frozen_ratio": sum(row[81] > 0.5 for row in rows) / len(rows),
        "direction_reliability": [stats([row[82 + direction] for row in rows])
                                  for direction in range(6)],
        "estimator_recovery_state_counts": recovery_state_counts,
        "estimator_recovery_active_ratio": sum(row[88] > 0.5 for row in rows) / len(rows),
        "estimator_unrecoverable_frames": sum(row[88] > 3.5 for row in rows),
        "healthy_checkpoint_ratio": sum(row[89] > 0.5 for row in rows) / len(rows),
        "recovery_degraded_frames": stats([row[90] for row in rows]),
        "recovery_attempts": stats([row[91] for row in rows]),
        "recovery_validation_streak": stats([row[92] for row in rows]),
        "recovery_rollbacks_max": max(row[93] for row in rows),
        "recovery_successes_max": max(row[94] for row in rows),
        "recovery_failures_max": max(row[95] for row in rows),
        "external_formal_map_freeze_ratio": sum(row[96] > 0.5 for row in rows) / len(rows),
        "healthy_checkpoint_cache": stats([row[97] for row in rows]),
        "scan_recovery_frames": len(recovery_rows),
        "scan_recovery_fitness": stats([row[99] for row in recovery_rows]),
        "scan_recovery_rmse_m": stats([row[100] for row in recovery_rows]),
        "scan_recovery_correspondences": stats([row[101] for row in recovery_rows]),
        "scan_recovery_weakest_direction": stats([row[102] for row in recovery_rows]),
        "scan_recovery_max_successes_since_reset": max(row[103] for row in rows),
        "usable_constraint_ratio": sum(row[3] > 0.0 or row[98] > 0.5 for row in rows) / len(rows),
    })


def add_vio_metrics(summary, rows):
    if not rows or any(len(row) < EXPECTED_VIO_WIDTH for row in rows) or any(
            not all(math.isfinite(value) for value in row) for row in rows):
        return
    summary.update({
        "frame_time_seconds": stats([row[5] for row in rows]),
        "visual_quality": stats([row[7] for row in rows]),
        "tracked_points": stats([row[1] for row in rows]),
        "visual_covariance": stats([row[8] for row in rows]),
        "covariance_reset_angle": stats([row[40] for row in rows]),
        "covariance_min_eigenvalue": stats([row[41] for row in rows]),
        "covariance_negative_count": sum(row[41] < -1e-12 for row in rows),
        "covariance_symmetry_error": stats([row[42] for row in rows]),
        "covariance_numerical_failure_frames": sum(row[43] > 0.5 for row in rows),
        "accepted_update_ratio": sum(row[44] > 0.5 for row in rows) / len(rows)
        if all(len(row) > 44 for row in rows) else None,
        "recovery_active_frames": sum(row[54] > 0.5 for row in rows),
        "recovery_success_count": max(row[51] for row in rows),
        "recovery_failure_count": max(row[52] for row in rows),
        "recovery_seeded_points": sum(row[53] for row in rows),
        "recovery_candidate_voxels": stats([row[49] if row[54] > 0.5 else 0.0 for row in rows]),
        "recovery_frozen_voxels": stats([row[50] for row in rows]),
    })


def time_axis_summary(values):
    finite = all(math.isfinite(value) for value in values)
    monotonic = finite and all(second >= first - 1e-9 for first, second in zip(values, values[1:]))
    return {
        "count": len(values),
        "finite": finite,
        "monotonic": monotonic,
        "start_time": values[0] if values and finite else None,
        "end_time": values[-1] if values and finite else None,
        "duration": values[-1] - values[0] if values and finite and monotonic else None,
    }


_INPUT_BAG_INFO = {}


def input_bag_info(path):
    resolved = str(path.resolve())
    if resolved not in _INPUT_BAG_INFO:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        with rosbag.Bag(resolved, "r") as bag:
            _INPUT_BAG_INFO[resolved] = {
                "path": resolved,
                "bytes": path.stat().st_size,
                "sha256": digest.hexdigest(),
                "start_time": float(bag.get_start_time()),
                "end_time": float(bag.get_end_time()),
            }
    return _INPUT_BAG_INFO[resolved]


def validate_replay_coverage(status, mode, lio, vio, trajectory, errors):
    coverage = {"valid": False}
    coverage_errors = []
    bag_value = status.get("bag_realpath", "")
    try:
        source_bag = Path(bag_value)
        if not source_bag.is_file():
            raise FileNotFoundError(bag_value)
        source = input_bag_info(source_bag)
        declared_bytes = int(status.get("bag_bytes", "0"))
        if source["bytes"] != declared_bytes:
            coverage_errors.append(
                f"input bag size changed: status={declared_bytes}, current={source['bytes']}")
        declared_sha256 = status.get("bag_sha256", "").lower()
        if source["sha256"] != declared_sha256:
            coverage_errors.append(
                f"input bag SHA-256 changed: status={declared_sha256}, current={source['sha256']}")
        duration = float(status["duration"])
        target_end = min(source["start_time"] + duration, source["end_time"])
    except (OSError, KeyError, ValueError, rosbag.bag.ROSBagException) as error:
        errors.append(f"cannot validate input bag coverage: {error}")
        return coverage

    axes = {
        "trajectory": time_axis_summary([stamp for stamp, _ in trajectory]),
    }
    if mode in CURRENT_MODES:
        axes["lio_sensor"] = time_axis_summary([row[0] for row in lio if row])
        axes["vio_sensor"] = time_axis_summary([row[0] for row in vio if row])

    span = max(target_end - source["start_time"], 1e-9)
    for name, axis in axes.items():
        if not axis["count"]:
            coverage_errors.append(f"missing {name} time coverage")
            continue
        if not axis["finite"] or not axis["monotonic"]:
            coverage_errors.append(f"{name} time axis is non-finite or non-monotonic")
            continue
        if name == "trajectory":
            if axis["start_time"] > source["start_time"] + COVERAGE_END_TOLERANCE_SECONDS:
                coverage_errors.append(
                    f"{name} starts {axis['start_time'] - source['start_time']:.3f}s after input bag")
            if axis["start_time"] < source["start_time"] - COVERAGE_END_TOLERANCE_SECONDS:
                coverage_errors.append(f"{name} starts before input bag time range")
            if axis["end_time"] < target_end - COVERAGE_END_TOLERANCE_SECONDS:
                coverage_errors.append(
                    f"{name} ends {target_end - axis['end_time']:.3f}s before replay target")
            if axis["end_time"] > source["end_time"] + COVERAGE_END_TOLERANCE_SECONDS:
                coverage_errors.append(f"{name} extends beyond input bag time range")
            minimum_duration = max(span - 2.0 * COVERAGE_END_TOLERANCE_SECONDS, 0.0)
            if axis["duration"] < minimum_duration:
                coverage_errors.append(
                    f"{name} covers only {axis['duration']:.3f}s of {span:.3f}s replay target")
        elif axis["duration"] < span - COVERAGE_END_TOLERANCE_SECONDS:
            coverage_errors.append(
                f"{name} covers only {axis['duration']:.3f}s of {span:.3f}s replay target")

    coverage = {
        "valid": not coverage_errors,
        "input": source,
        "target_end_time": target_end,
        "target_duration": span,
        "end_tolerance_seconds": COVERAGE_END_TOLERANCE_SECONDS,
        "axes": axes,
    }
    for name, axis in coverage["axes"].items():
        axis["end_coverage_ratio"] = (
            ((axis["end_time"] - source["start_time"]) / span
             if name == "trajectory" else axis["duration"] / span)
            if axis["end_time"] is not None and axis["duration"] is not None else None)
    errors.extend(coverage_errors)
    return coverage


def read_run(run_dir):
    mode = run_dir.name
    errors = []
    status, status_errors = parse_key_value_file(run_dir / "status.txt")
    errors.extend(status_errors)
    errors.extend(validate_status(status, mode))
    parameter_path = run_dir / "parameters.yaml"
    switches, non_ablation_parameters, parameter_errors = validate_parameters(parameter_path, mode)
    errors.extend(parameter_errors)
    if parameter_path.exists() and status.get("parameters_sha256"):
        actual_parameters_sha256 = sha256_file(parameter_path)
        if actual_parameters_sha256 != status.get("parameters_sha256"):
            errors.append("parameters.yaml SHA-256 does not match status contract")
    errors.extend(validate_runtime_manifest(
        run_dir / "runtime_manifest.txt", status.get("runtime_bundle_sha256", "")))

    bag_path = run_dir / "output.bag"
    lio = []
    vio = []
    odom = []
    odom_total = 0
    invalid_odom = 0
    odom_monotonic = True
    last_odom_time = None
    if not bag_path.exists():
        active_bag = run_dir / "output.bag.active"
        errors.append("missing output.bag" + ("; recorder left output.bag.active" if active_bag.exists() else ""))
    elif bag_path.stat().st_size == 0:
        errors.append("output.bag is empty")
    else:
        try:
            with rosbag.Bag(str(bag_path), "r") as bag:
                for topic, message, bag_stamp in bag.read_messages(
                        topics=["/fast_livo2/lio_diag", "/fast_livo2/vio_diag", "/aft_mapped_to_init"]):
                    if topic == "/fast_livo2/lio_diag":
                        lio.append([float(value) for value in message.data])
                    elif topic == "/fast_livo2/vio_diag":
                        vio.append([float(value) for value in message.data])
                    else:
                        odom_total += 1
                        stamp = float(message.header.stamp.to_sec())
                        position = message.pose.pose.position
                        orientation = message.pose.pose.orientation
                        values = (
                            stamp, float(position.x), float(position.y), float(position.z),
                            float(orientation.x), float(orientation.y), float(orientation.z),
                            float(orientation.w), float(bag_stamp.to_sec()),
                        )
                        if not all(math.isfinite(value) for value in values):
                            invalid_odom += 1
                            continue
                        if last_odom_time is not None and stamp < last_odom_time - 1e-9:
                            odom_monotonic = False
                        last_odom_time = stamp
                        odom.append((stamp, (values[1], values[2], values[3])))
        except Exception as error:  # rosbag raises several backend-specific exception classes
            errors.append(f"cannot read output.bag: {error}")

    strict_schema = mode in CURRENT_MODES
    lio_summary = diagnostic_integrity(lio, EXPECTED_LIO_WIDTH, strict_schema, "LIO", errors)
    vio_summary = diagnostic_integrity(vio, EXPECTED_VIO_WIDTH, strict_schema, "VIO", errors)
    add_lio_metrics(lio_summary, lio)
    add_vio_metrics(vio_summary, vio)
    trajectory_summary = path_metrics(odom, odom_total, invalid_odom, odom_monotonic)
    if odom_total == 0:
        errors.append("missing /aft_mapped_to_init trajectory")
    if invalid_odom:
        errors.append(f"trajectory contains {invalid_odom} non-finite poses")
    if not odom_monotonic:
        errors.append("trajectory header timestamps are not monotonic")

    replay_coverage = validate_replay_coverage(status, mode, lio, vio, odom, errors)
    try:
        parameter_contract_digest = canonical_digest(non_ablation_parameters)
    except (TypeError, ValueError):
        parameter_contract_digest = None
        errors.append("non-ablation parameter tree cannot be canonicalized")

    summary = {
        "mode": mode,
        "valid": not errors,
        "errors": errors,
        "bag_bytes": bag_path.stat().st_size if bag_path.exists() else 0,
        "status": status,
        "adaptive_switches": switches,
        "non_ablation_parameter_digest": parameter_contract_digest,
        "node_resource": parse_time_file(run_dir / "node_time.txt"),
        "player_resource": parse_time_file(run_dir / "player_time.txt"),
        "trajectory": trajectory_summary,
        "replay_coverage": replay_coverage,
        "lio": lio_summary,
        "vio": vio_summary,
    }
    summary["valid"] = not errors
    return summary, odom, non_ablation_parameters


def collapse_duplicate_times(samples):
    collapsed = []
    for stamp, point in samples:
        if collapsed and abs(stamp - collapsed[-1][0]) <= 1e-12:
            collapsed[-1] = (stamp, point)
        else:
            collapsed.append((stamp, point))
    return collapsed


def interpolate_at(samples, stamps, target):
    if target <= stamps[0]:
        return samples[0][1]
    if target >= stamps[-1]:
        return samples[-1][1]
    upper = bisect.bisect_right(stamps, target)
    lower = upper - 1
    first_time, first = samples[lower]
    second_time, second = samples[upper]
    alpha = (target - first_time) / (second_time - first_time)
    return tuple(first[index] * (1.0 - alpha) + second[index] * alpha for index in range(3))


def trajectory_disagreement(reference, candidate, sample_count=200):
    reference = collapse_duplicate_times(reference)
    candidate = collapse_duplicate_times(candidate)
    if len(reference) < 2 or len(candidate) < 2:
        return None
    # Contract v2 runs use /use_sim_time and rosbag --clock, so header stamps share
    # the input bag's sensor timeline and can be compared without per-run shifting.
    start = max(reference[0][0], candidate[0][0])
    end = min(reference[-1][0], candidate[-1][0])
    if not math.isfinite(start) or not math.isfinite(end) or end <= start:
        return None
    reference_stamps = [stamp for stamp, _ in reference]
    candidate_stamps = [stamp for stamp, _ in candidate]
    reference_origin = interpolate_at(reference, reference_stamps, start)
    candidate_origin = interpolate_at(candidate, candidate_stamps, start)
    squared = []
    endpoint_delta = None
    sampled_reference = []
    sampled_candidate = []
    for index in range(sample_count):
        stamp = start + (end - start) * index / (sample_count - 1)
        ref = interpolate_at(reference, reference_stamps, stamp)
        other = interpolate_at(candidate, candidate_stamps, stamp)
        sampled_reference.append(ref)
        sampled_candidate.append(other)
        delta = tuple(
            (other[axis] - candidate_origin[axis]) - (ref[axis] - reference_origin[axis])
            for axis in range(3)
        )
        squared.append(sum(value * value for value in delta))
        endpoint_delta = delta

    reference_array = np.asarray(sampled_reference, dtype=float)
    candidate_array = np.asarray(sampled_candidate, dtype=float)
    reference_center = reference_array.mean(axis=0)
    candidate_center = candidate_array.mean(axis=0)
    reference_centered = reference_array - reference_center
    candidate_centered = candidate_array - candidate_center
    covariance = candidate_centered.T @ reference_centered
    left, _, right_transpose = np.linalg.svd(covariance)
    rotation = left @ right_transpose
    if np.linalg.det(rotation) < 0.0:
        right_transpose[-1, :] *= -1.0
        rotation = left @ right_transpose
    aligned_candidate = candidate_centered @ rotation + reference_center
    aligned_delta = aligned_candidate - reference_array
    aligned_squared = np.sum(aligned_delta * aligned_delta, axis=1)

    def sample_by_progress(samples):
        points = [point for _, point in samples]
        output = []
        for index in range(sample_count):
            position = index * (len(points) - 1) / (sample_count - 1)
            lower = int(math.floor(position))
            upper = min(lower + 1, len(points) - 1)
            alpha = position - lower
            output.append(tuple(
                points[lower][axis] * (1.0 - alpha) + points[upper][axis] * alpha
                for axis in range(3)))
        return np.asarray(output, dtype=float)

    progress_reference = sample_by_progress(reference)
    progress_candidate = sample_by_progress(candidate)
    progress_raw_delta = ((progress_candidate - progress_candidate[0]) -
                          (progress_reference - progress_reference[0]))
    progress_reference_centered = progress_reference - progress_reference.mean(axis=0)
    progress_candidate_centered = progress_candidate - progress_candidate.mean(axis=0)
    progress_covariance = progress_candidate_centered.T @ progress_reference_centered
    progress_left, _, progress_right_transpose = np.linalg.svd(progress_covariance)
    progress_rotation = progress_left @ progress_right_transpose
    if np.linalg.det(progress_rotation) < 0.0:
        progress_right_transpose[-1, :] *= -1.0
        progress_rotation = progress_left @ progress_right_transpose
    progress_aligned = (progress_candidate_centered @ progress_rotation +
                        progress_reference.mean(axis=0))
    progress_aligned_delta = progress_aligned - progress_reference
    return {
        "common_start_time": start,
        "common_end_time": end,
        "common_duration": end - start,
        "sample_count": sample_count,
        "elapsed_time_aligned_translation_rmse": math.sqrt(sum(squared) / len(squared)),
        "endpoint_difference": math.sqrt(sum(value * value for value in endpoint_delta)),
        "rigid_aligned_translation_rmse": float(math.sqrt(float(np.mean(aligned_squared)))),
        "rigid_aligned_endpoint_difference": float(np.linalg.norm(aligned_delta[-1])),
        "progress_aligned_translation_rmse": float(math.sqrt(float(np.mean(
            np.sum(progress_raw_delta * progress_raw_delta, axis=1))))),
        "progress_rigid_aligned_translation_rmse": float(math.sqrt(float(np.mean(
            np.sum(progress_aligned_delta * progress_aligned_delta, axis=1))))),
    }


def main():
    if len(sys.argv) < 2:
        raise SystemExit(f"Usage: {sys.argv[0]} RESULT_ROOT [MODE ...]")
    root = Path(sys.argv[1])
    if not root.is_dir():
        raise SystemExit(f"Result root is not a directory: {root}")
    requested_modes = tuple(sys.argv[2:]) if len(sys.argv) > 2 else EXPECTED_MODES
    if not requested_modes or len(set(requested_modes)) != len(requested_modes):
        raise SystemExit("Requested modes must be a non-empty unique list")
    unknown_modes = sorted(set(requested_modes) - set(EXPECTED_MODES))
    if unknown_modes:
        raise SystemExit(f"Unknown requested modes: {', '.join(unknown_modes)}")

    candidate_dirs = [root / mode for mode in requested_modes if (root / mode).is_dir()]
    ignored_mode_dirs = sorted(
        path.name for path in root.iterdir()
        if path.is_dir() and path.name in EXPECTED_MODES and path.name not in requested_modes
    )
    found_names = {path.name for path in candidate_dirs}
    missing_modes = sorted(set(requested_modes) - found_names)
    summaries = {}
    trajectories = {}
    parameter_contracts = {}
    for run_dir in candidate_dirs:
        summary, trajectory, non_ablation_parameters = read_run(run_dir)
        summaries[run_dir.name] = summary
        trajectories[run_dir.name] = trajectory
        parameter_contracts[run_dir.name] = non_ablation_parameters
        write_json(run_dir / "summary.json", summary)

    aggregate_errors = [f"missing run directory for {mode}" for mode in missing_modes]
    for name, summary in summaries.items():
        if not summary["valid"]:
            aggregate_errors.append(f"invalid run {name}")

    present_modes = [mode for mode in requested_modes if mode in summaries]
    if present_modes:
        contract_reference = present_modes[0]
        reference_status = summaries[contract_reference]["status"]
        exact_status_keys = (
            "contract_version", "use_sim_time", "bag_realpath", "bag_bytes", "bag_sha256",
            "duration", "rate", "compressed_image", "source_config_sha256",
            "camera_config_sha256",
        )
        for mode in present_modes[1:]:
            status = summaries[mode]["status"]
            for key in exact_status_keys:
                if status.get(key) != reference_status.get(key):
                    aggregate_errors.append(
                        f"input contract mismatch for {key}: {mode}={status.get(key)!r}, "
                        f"{contract_reference}={reference_status.get(key)!r}")
            if parameter_contracts[mode] != parameter_contracts[contract_reference]:
                aggregate_errors.append(
                    f"non-ablation parameter tree differs: {mode} vs {contract_reference}")
            reference_axis = summaries[contract_reference].get("replay_coverage", {}).get("axes", {}).get("trajectory", {})
            mode_axis = summaries[mode].get("replay_coverage", {}).get("axes", {}).get("trajectory", {})
            for endpoint in ("start_time", "end_time"):
                reference_value = reference_axis.get(endpoint)
                mode_value = mode_axis.get(endpoint)
                if reference_value is not None and mode_value is not None and abs(mode_value - reference_value) > COVERAGE_END_TOLERANCE_SECONDS:
                    aggregate_errors.append(
                        f"trajectory {endpoint} differs by more than "
                        f"{COVERAGE_END_TOLERANCE_SECONDS}s: {mode} vs {contract_reference}")

        current_hashes = {
            summaries[mode]["status"].get("runtime_bundle_sha256")
            for mode in present_modes if mode in CURRENT_MODES
        }
        if len(current_hashes) > 1:
            aggregate_errors.append("current modes used different runtime bundle hashes")

    valid_names = {name for name, summary in summaries.items() if summary["valid"]}
    reference_name = None
    if "stock_baseline" in valid_names:
        reference_name = "stock_baseline"
    elif "all_off" in valid_names:
        reference_name = "all_off"
    comparisons = {}
    if reference_name is None:
        aggregate_errors.append("no valid stock_baseline or all_off trajectory for comparison")
    else:
        for name in sorted(valid_names):
            if name == reference_name:
                continue
            comparison = trajectory_disagreement(trajectories[reference_name], trajectories[name])
            if comparison is None:
                aggregate_errors.append(f"no common trajectory interval for {name} vs {reference_name}")
            comparisons[f"{name}_vs_{reference_name}"] = comparison

    aggregate = {
        "valid": not aggregate_errors,
        "errors": aggregate_errors,
        "missing_modes": missing_modes,
        "ignored_mode_dirs": ignored_mode_dirs,
        "requested_modes": list(requested_modes),
        "expected_lio_width": EXPECTED_LIO_WIDTH,
        "expected_vio_width": EXPECTED_VIO_WIDTH,
        "runs": summaries,
        "trajectory_reference": reference_name,
        "trajectory_comparisons": comparisons,
        "trajectory_comparison_note": (
            "Sensor-time-aligned translation disagreement over the common validated replay interval. "
            "Raw values remove initial translation only; rigid-aligned values use a "
            "no-scale Kabsch alignment. Progress-aligned values also remove wall-clock scheduling jitter by "
            "matching normalized output sequence progress. None is an accuracy metric because no ground truth "
            "is available."
        ),
    }
    write_json(root / "aggregate.json", aggregate)
    print(json.dumps(aggregate, indent=2, sort_keys=True, allow_nan=False))
    return 0 if aggregate["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
