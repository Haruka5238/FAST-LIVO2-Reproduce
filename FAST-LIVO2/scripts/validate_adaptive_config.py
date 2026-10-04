#!/usr/bin/env python3

import math
import re
import sys
from pathlib import Path

import yaml


class UniqueKeyLoader(yaml.SafeLoader):
    pass


def construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping", node.start_mark,
                f"duplicate key {key!r}", key_node.start_mark)
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, construct_unique_mapping)


def nonfinite_paths(value, prefix=""):
    paths = []
    if isinstance(value, dict):
        for key, child in value.items():
            child_prefix = f"{prefix}/{key}" if prefix else str(key)
            paths.extend(nonfinite_paths(child, child_prefix))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            paths.extend(nonfinite_paths(child, f"{prefix}[{index}]"))
    elif isinstance(value, float) and not math.isfinite(value):
        paths.append(prefix)
    return paths


def rotation_error(values):
    if not isinstance(values, list) or len(values) != 9:
        return math.inf, math.nan
    rows = [values[0:3], values[3:6], values[6:9]]
    orthogonality_error_sq = 0.0
    for row in range(3):
        for column in range(3):
            dot = sum(rows[index][row] * rows[index][column] for index in range(3))
            target = 1.0 if row == column else 0.0
            orthogonality_error_sq += (dot - target) ** 2
    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0]))
    return math.sqrt(orthogonality_error_sq), determinant


def main():
    root = Path(__file__).resolve().parents[1]
    source = (root / "src" / "LIVMapper.cpp").read_text(encoding="utf-8")
    source_keys = re.findall(r'nh\.param<[^>]+>\(\s*"adaptive/([^"]+)"', source)
    errors = []
    if len(source_keys) != len(set(source_keys)):
        duplicates = sorted(key for key in set(source_keys) if source_keys.count(key) > 1)
        errors.append(f"duplicate adaptive source reads: {duplicates}")
    expected = set(source_keys)

    config_paths = []
    for path in sorted((root / "config").glob("*.yaml")):
        if re.search(r"^adaptive:\s*$", path.read_text(encoding="utf-8"), re.MULTILINE):
            config_paths.append(path)

    for path in config_paths:
        try:
            document = yaml.load(path.read_text(encoding="utf-8"), Loader=UniqueKeyLoader) or {}
        except (OSError, UnicodeError, yaml.YAMLError) as error:
            errors.append(f"{path.name}: {error}")
            continue
        adaptive = document.get("adaptive")
        if not isinstance(adaptive, dict):
            errors.append(f"{path.name}: adaptive is not a mapping")
            continue
        actual = set(adaptive)
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        if missing or extra:
            errors.append(f"{path.name}: missing={missing}, extra={extra}")
        bad_values = nonfinite_paths(document)
        if bad_values:
            errors.append(f"{path.name}: non-finite values at {bad_values}")

        common = document.get("common", {})
        imu = document.get("imu", {})
        debug = document.get("debug", {})
        pcd_save = document.get("pcd_save", {})
        extrinsics = document.get("extrin_calib", {})
        for key in ("img_en", "lidar_en"):
            if common.get(key) not in (0, 1):
                errors.append(f"{path.name}: common/{key} must be 0 or 1")
        if common.get("lidar_en") != 1 or (common.get("img_en") == 1 and imu.get("imu_en") is not True):
            errors.append(f"{path.name}: unsupported sensor-enable combination")
        max_interval = imu.get("max_propagation_interval")
        if not isinstance(max_interval, (int, float)) or not 0.01 <= max_interval <= 10.0:
            errors.append(f"{path.name}: imu/max_propagation_interval must be in [0.01, 10]")
        for key in ("state_log_en", "imu_log_en"):
            if debug.get(key) is not False:
                errors.append(f"{path.name}: debug/{key} must default to false")
        if pcd_save.get("allow_unbounded_buffer") is not False:
            errors.append(f"{path.name}: pcd_save/allow_unbounded_buffer must default to false")
        if pcd_save.get("pcd_save_en") is True and pcd_save.get("interval", 0) <= 0:
            errors.append(f"{path.name}: enabled PCD saving requires a positive interval")
        for key in ("extrinsic_R", "Rcl"):
            orthogonality_error, determinant = rotation_error(extrinsics.get(key))
            if orthogonality_error > 1e-3 or not math.isfinite(determinant) or abs(determinant - 1.0) > 1e-3:
                errors.append(
                    f"{path.name}: extrin_calib/{key} is not SO(3) "
                    f"(orthogonality_error={orthogonality_error}, determinant={determinant})")

    if len(expected) != 105:
        errors.append(f"source adaptive key count is {len(expected)}, expected 105")
    if len(config_paths) != 6:
        errors.append(f"adaptive mapping YAML count is {len(config_paths)}, expected 6")

    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1
    print(
        f"validated {len(expected)} adaptive keys and release safety defaults "
        f"across {len(config_paths)} mapping YAML files")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
