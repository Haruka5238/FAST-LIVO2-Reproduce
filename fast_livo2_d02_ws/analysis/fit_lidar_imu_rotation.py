#!/usr/bin/env python3
"""Fit the constant LiDAR-to-IMU rotation from pose and gyroscope increments."""

import argparse
import bisect
import math

import numpy as np
import rosbag
import rospy


def quaternion_rotation(values):
    x, y, z, w = values
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def so3_log(rotation):
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    angle = math.acos(cosine)
    vee = np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ]
    )
    if angle < 1e-7:
        return vee / 2.0
    return angle * vee / (2.0 * math.sin(angle))


def rotation_euler_xyz_deg(rotation):
    return np.degrees(
        [
            math.atan2(rotation[2, 1], rotation[2, 2]),
            math.asin(np.clip(-rotation[2, 0], -1.0, 1.0)),
            math.atan2(rotation[1, 0], rotation[0, 0]),
        ]
    )


def read_poses(path):
    poses = []
    with open(path) as stream:
        for line in stream:
            values = [float(value) for value in line.split()]
            if len(values) == 8:
                poses.append(values)
    if len(poses) < 2:
        raise RuntimeError("pose file has fewer than two complete rows")
    return poses


def read_imu(bag_path, record_window):
    timestamps = []
    gyroscope = []
    acceleration = []
    with rosbag.Bag(bag_path, "r") as bag:
        record_start = bag.get_start_time()
        for _, message, _ in bag.read_messages(
            topics=["/livox/imu"],
            start_time=rospy.Time.from_sec(record_start),
            end_time=rospy.Time.from_sec(record_start + record_window),
        ):
            timestamps.append(message.header.stamp.to_sec())
            gyroscope.append(
                [message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z]
            )
            acceleration.append(
                [
                    message.linear_acceleration.x,
                    message.linear_acceleration.y,
                    message.linear_acceleration.z,
                ]
            )
    return np.array(timestamps), np.array(gyroscope), np.array(acceleration)


def paired_rates(poses, imu_times, gyroscope, fit_start, fit_end, first_pose_rel_time):
    pose_time_origin = poses[0][0] - first_pose_rel_time
    lidar_rates = []
    imu_rates = []
    relative_times = []
    for first, second in zip(poses[:-1], poses[1:]):
        start, end = first[0], second[0]
        dt = end - start
        relative_time = start - pose_time_origin
        if not (0.07 < dt < 0.13 and fit_start < relative_time < fit_end):
            continue
        begin_index = bisect.bisect_left(imu_times, start)
        end_index = bisect.bisect_right(imu_times, end)
        if end_index - begin_index < 3:
            continue
        first_rotation = quaternion_rotation(first[4:8])
        second_rotation = quaternion_rotation(second[4:8])
        lidar_rates.append(so3_log(first_rotation.T @ second_rotation) / dt)
        imu_rates.append(gyroscope[begin_index:end_index].mean(axis=0))
        relative_times.append(relative_time)
    return np.array(lidar_rates), np.array(imu_rates), np.array(relative_times)


def fit_rotation(lidar_rates, imu_rates, min_excited_rate):
    magnitudes = np.linalg.norm(lidar_rates, axis=1)
    quiet = magnitudes < 0.025
    excited = magnitudes > min_excited_rate
    if quiet.sum() < 10 or excited.sum() < 10:
        raise RuntimeError("insufficient quiet or rotationally excited pose increments")

    gyro_bias = np.median(imu_rates[quiet], axis=0)
    rotation = np.eye(3)
    for _ in range(10):
        cross_covariance = np.zeros((3, 3))
        weights = np.minimum(magnitudes[excited], 1.0)
        for lidar_rate, imu_rate, weight in zip(
            lidar_rates[excited], imu_rates[excited] - gyro_bias, weights
        ):
            cross_covariance += weight * np.outer(imu_rate, lidar_rate)
        left, _, right_transpose = np.linalg.svd(cross_covariance)
        determinant_fix = np.eye(3)
        determinant_fix[2, 2] = np.linalg.det(left @ right_transpose)
        rotation = left @ determinant_fix @ right_transpose
        residual = imu_rates - lidar_rates @ rotation.T
        gyro_bias = np.median(residual[magnitudes < 0.4], axis=0)
    return rotation, gyro_bias, quiet, excited


def vector_angle_deg(first, second):
    cosine = first.dot(second) / (np.linalg.norm(first) * np.linalg.norm(second))
    return math.degrees(math.acos(np.clip(cosine, -1.0, 1.0)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--pose", required=True)
    parser.add_argument("--record-window", type=float, default=160.0)
    parser.add_argument("--fit-start", type=float, default=5.0)
    parser.add_argument("--fit-end", type=float, default=155.0)
    parser.add_argument("--first-pose-rel-time", type=float, default=0.3)
    parser.add_argument("--min-excited-rate", type=float, default=0.08)
    args = parser.parse_args()

    poses = read_poses(args.pose)
    imu_times, gyroscope, acceleration = read_imu(args.bag, args.record_window)
    lidar_rates, imu_rates, _ = paired_rates(
        poses,
        imu_times,
        gyroscope,
        args.fit_start,
        args.fit_end,
        args.first_pose_rel_time,
    )
    rotation, gyro_bias, quiet, excited = fit_rotation(
        lidar_rates, imu_rates, args.min_excited_rate
    )

    fitted = lidar_rates @ rotation.T + gyro_bias
    identity_rms = math.sqrt(np.mean(np.sum((imu_rates - lidar_rates) ** 2, axis=1)))
    fitted_rms = math.sqrt(np.mean(np.sum((imu_rates - fitted) ** 2, axis=1)))
    initial_acceleration = acceleration[:1000].mean(axis=0)
    initial_acceleration /= np.linalg.norm(initial_acceleration)

    print(f"pose_pairs: {len(lidar_rates)}")
    print(f"excited_pairs: {int(excited.sum())}")
    print(f"quiet_pairs: {int(quiet.sum())}")
    print("gyro_bias_rad_s: [" + ", ".join(f"{value:.9f}" for value in gyro_bias) + "]")
    print("extrinsic_R:")
    for row in rotation:
        print("  [" + ", ".join(f"{value:.9f}" for value in row) + "]")
    print(
        "euler_xyz_deg: ["
        + ", ".join(f"{value:.6f}" for value in rotation_euler_xyz_deg(rotation))
        + "]"
    )
    print(f"identity_rate_rms_rad_s: {identity_rms:.9f}")
    print(f"fitted_rate_rms_rad_s: {fitted_rms:.9f}")
    print(f"fitted_z_to_initial_acc_angle_deg: {vector_angle_deg(rotation[:, 2], initial_acceleration):.6f}")


if __name__ == "__main__":
    main()
