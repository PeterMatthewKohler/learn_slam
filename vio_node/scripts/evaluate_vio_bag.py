#!/usr/bin/env python3

"""Evaluate VIO odometry against ground-truth odometry in a ROS 2 bag."""

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
from typing import Dict, List, Sequence, Tuple

import matplotlib
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


ODOMETRY_TYPE = "nav_msgs/msg/Odometry"


@dataclass
class OdomSample:
    stamp_ns: int
    frame_id: str
    child_frame_id: str
    position: np.ndarray
    quaternion_xyzw: np.ndarray
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray
    twist_available: bool


@dataclass
class OdomSeries:
    stamps_ns: np.ndarray
    frame_id: str
    child_frame_id: str
    positions: np.ndarray
    quaternions_xyzw: np.ndarray
    linear_velocities: np.ndarray
    angular_velocities: np.ndarray
    twist_available: bool
    duplicate_count: int

    def select(self, indices: Sequence[int]) -> "OdomSeries":
        selected = np.asarray(indices, dtype=int)
        return OdomSeries(
            stamps_ns=self.stamps_ns[selected],
            frame_id=self.frame_id,
            child_frame_id=self.child_frame_id,
            positions=self.positions[selected],
            quaternions_xyzw=self.quaternions_xyzw[selected],
            linear_velocities=self.linear_velocities[selected],
            angular_velocities=self.angular_velocities[selected],
            twist_available=self.twist_available,
            duplicate_count=self.duplicate_count,
        )


@dataclass
class InterpolatedGroundTruth:
    positions: np.ndarray
    quaternions_xyzw: np.ndarray
    linear_velocities: np.ndarray
    angular_velocities: np.ndarray


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare a VIO nav_msgs/Odometry topic with ground truth using "
            "initial-pose SE(3) alignment."
        )
    )
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory")
    parser.add_argument(
        "--estimate-topic",
        default="/vio/odom",
        help="Estimated odometry topic (default: /vio/odom)",
    )
    parser.add_argument(
        "--ground-truth-topic",
        default="/chassis/odom",
        help="Ground-truth odometry topic (default: /chassis/odom)",
    )
    parser.add_argument(
        "--storage-id",
        default="sqlite3",
        help="rosbag2 storage plugin (default: sqlite3)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Output directory (default: <bag>/evaluation)",
    )
    parser.add_argument(
        "--max-interpolation-gap-s",
        type=float,
        default=0.1,
        help="Maximum allowed ground-truth bracket width (default: 0.1 s)",
    )
    parser.add_argument(
        "--rpe-interval-s",
        type=float,
        default=1.0,
        help="Relative-pose-error interval (default: 1.0 s)",
    )
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Skip PNG plot generation",
    )
    return parser.parse_args()


def quaternion_normalized(quaternion_xyzw: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion_xyzw, dtype=float)
    norm = np.linalg.norm(quaternion)
    if not np.all(np.isfinite(quaternion)) or not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid quaternion")
    return quaternion / norm


def quaternion_conjugate(quaternion_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion_xyzw
    return np.array([-x, -y, -z, w], dtype=float)


def quaternion_multiply(
    first_xyzw: np.ndarray,
    second_xyzw: np.ndarray,
) -> np.ndarray:
    x1, y1, z1, w1 = first_xyzw
    x2, y2, z2, w2 = second_xyzw
    return quaternion_normalized(
        np.array(
            [
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            ]
        )
    )


def quaternion_slerp(
    first_xyzw: np.ndarray,
    second_xyzw: np.ndarray,
    alpha: float,
) -> np.ndarray:
    first = quaternion_normalized(first_xyzw)
    second = quaternion_normalized(second_xyzw)
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return quaternion_normalized((1.0 - alpha) * first + alpha * second)
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    return quaternion_normalized(
        math.sin((1.0 - alpha) * theta) / sin_theta * first
        + math.sin(alpha * theta) / sin_theta * second
    )


def quaternion_to_rotation_matrix(quaternion_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion_normalized(quaternion_xyzw)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def quaternion_error_angle_rad(
    reference_xyzw: np.ndarray,
    estimate_xyzw: np.ndarray,
) -> float:
    error = quaternion_multiply(
        quaternion_conjugate(reference_xyzw),
        estimate_xyzw,
    )
    return 2.0 * math.atan2(
        float(np.linalg.norm(error[:3])),
        abs(float(error[3])),
    )


def odom_sample_from_message(message) -> OdomSample:
    stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )
    position = np.array(
        [
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        ]
    )
    quaternion = quaternion_normalized(
        np.array(
            [
                message.pose.pose.orientation.x,
                message.pose.pose.orientation.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w,
            ]
        )
    )
    linear_velocity = np.array(
        [
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.linear.z,
        ]
    )
    angular_velocity = np.array(
        [
            message.twist.twist.angular.x,
            message.twist.twist.angular.y,
            message.twist.twist.angular.z,
        ]
    )
    twist_covariance_marker = float(message.twist.covariance[0])
    if stamp_ns < 0 or not all(
        np.all(np.isfinite(values))
        for values in (position, linear_velocity, angular_velocity)
    ) or not math.isfinite(twist_covariance_marker):
        raise ValueError("odometry message contains invalid timestamp or values")
    return OdomSample(
        stamp_ns=stamp_ns,
        frame_id=message.header.frame_id,
        child_frame_id=message.child_frame_id,
        position=position,
        quaternion_xyzw=quaternion,
        linear_velocity=linear_velocity,
        angular_velocity=angular_velocity,
        twist_available=twist_covariance_marker >= 0.0,
    )


def finalize_series(samples: List[OdomSample], topic: str) -> OdomSeries:
    if not samples:
        raise RuntimeError(f"no valid messages found on {topic}")
    samples.sort(key=lambda sample: sample.stamp_ns)
    by_stamp = {sample.stamp_ns: sample for sample in samples}
    duplicate_count = len(samples) - len(by_stamp)
    ordered = list(by_stamp.values())
    frame_ids = {sample.frame_id for sample in ordered}
    child_frame_ids = {sample.child_frame_id for sample in ordered}
    twist_availability = {sample.twist_available for sample in ordered}
    if len(frame_ids) != 1 or len(child_frame_ids) != 1:
        raise RuntimeError(
            f"{topic} changes frame IDs: parents={sorted(frame_ids)}, "
            f"children={sorted(child_frame_ids)}"
        )
    if len(twist_availability) != 1:
        raise RuntimeError(f"{topic} changes whether twist is available")
    return OdomSeries(
        stamps_ns=np.asarray([sample.stamp_ns for sample in ordered], dtype=np.int64),
        frame_id=ordered[0].frame_id,
        child_frame_id=ordered[0].child_frame_id,
        positions=np.asarray([sample.position for sample in ordered]),
        quaternions_xyzw=np.asarray([sample.quaternion_xyzw for sample in ordered]),
        linear_velocities=np.asarray([sample.linear_velocity for sample in ordered]),
        angular_velocities=np.asarray([sample.angular_velocity for sample in ordered]),
        twist_available=ordered[0].twist_available,
        duplicate_count=duplicate_count,
    )


def read_odometry_topics(
    bag_directory: Path,
    storage_id: str,
    estimate_topic: str,
    ground_truth_topic: str,
) -> Tuple[OdomSeries, OdomSeries, Dict[str, int]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    requested_topics = (estimate_topic, ground_truth_topic)
    for topic in requested_topics:
        if topic not in topic_types:
            raise RuntimeError(f"bag does not contain requested topic {topic}")
        if topic_types[topic] != ODOMETRY_TYPE:
            raise RuntimeError(
                f"{topic} has type {topic_types[topic]}, expected {ODOMETRY_TYPE}"
            )
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(requested_topics)))
    message_types = {
        topic: get_message(topic_types[topic]) for topic in requested_topics
    }
    samples: Dict[str, List[OdomSample]] = {
        estimate_topic: [],
        ground_truth_topic: [],
    }
    invalid_counts = {estimate_topic: 0, ground_truth_topic: 0}
    while reader.has_next():
        topic, serialized_data, _ = reader.read_next()
        try:
            message = deserialize_message(serialized_data, message_types[topic])
            samples[topic].append(odom_sample_from_message(message))
        except (KeyError, ValueError):
            invalid_counts[topic] += 1
    return (
        finalize_series(samples[estimate_topic], estimate_topic),
        finalize_series(samples[ground_truth_topic], ground_truth_topic),
        invalid_counts,
    )


def interpolate_ground_truth(
    estimate: OdomSeries,
    ground_truth: OdomSeries,
    max_gap_s: float,
) -> Tuple[OdomSeries, InterpolatedGroundTruth, int]:
    matched_indices: List[int] = []
    positions: List[np.ndarray] = []
    quaternions: List[np.ndarray] = []
    linear_velocities: List[np.ndarray] = []
    angular_velocities: List[np.ndarray] = []
    for estimate_index, stamp_ns in enumerate(estimate.stamps_ns):
        right = int(np.searchsorted(ground_truth.stamps_ns, stamp_ns))
        if right < ground_truth.stamps_ns.size and ground_truth.stamps_ns[right] == stamp_ns:
            left = right
            alpha = 0.0
        else:
            left = right - 1
            if left < 0 or right >= ground_truth.stamps_ns.size:
                continue
            bracket_ns = int(
                ground_truth.stamps_ns[right] - ground_truth.stamps_ns[left]
            )
            if bracket_ns <= 0 or bracket_ns * 1e-9 > max_gap_s:
                continue
            alpha = float(stamp_ns - ground_truth.stamps_ns[left]) / float(bracket_ns)
        matched_indices.append(estimate_index)
        if left == right:
            positions.append(ground_truth.positions[left])
            quaternions.append(ground_truth.quaternions_xyzw[left])
            linear_velocities.append(ground_truth.linear_velocities[left])
            angular_velocities.append(ground_truth.angular_velocities[left])
        else:
            positions.append(
                (1.0 - alpha) * ground_truth.positions[left]
                + alpha * ground_truth.positions[right]
            )
            quaternions.append(
                quaternion_slerp(
                    ground_truth.quaternions_xyzw[left],
                    ground_truth.quaternions_xyzw[right],
                    alpha,
                )
            )
            linear_velocities.append(
                (1.0 - alpha) * ground_truth.linear_velocities[left]
                + alpha * ground_truth.linear_velocities[right]
            )
            angular_velocities.append(
                (1.0 - alpha) * ground_truth.angular_velocities[left]
                + alpha * ground_truth.angular_velocities[right]
            )
    if not matched_indices:
        raise RuntimeError("no estimate timestamps have usable ground-truth coverage")
    return (
        estimate.select(matched_indices),
        InterpolatedGroundTruth(
            positions=np.asarray(positions),
            quaternions_xyzw=np.asarray(quaternions),
            linear_velocities=np.asarray(linear_velocities),
            angular_velocities=np.asarray(angular_velocities),
        ),
        estimate.stamps_ns.size - len(matched_indices),
    )


def align_ground_truth_to_estimate(
    estimate: OdomSeries,
    ground_truth: InterpolatedGroundTruth,
) -> Tuple[InterpolatedGroundTruth, np.ndarray, np.ndarray]:
    rotation_quaternion = quaternion_multiply(
        estimate.quaternions_xyzw[0],
        quaternion_conjugate(ground_truth.quaternions_xyzw[0]),
    )
    rotation_matrix = quaternion_to_rotation_matrix(rotation_quaternion)
    translation = estimate.positions[0] - rotation_matrix @ ground_truth.positions[0]
    aligned_positions = np.asarray(
        [rotation_matrix @ position + translation for position in ground_truth.positions]
    )
    aligned_quaternions = np.asarray(
        [
            quaternion_multiply(rotation_quaternion, quaternion)
            for quaternion in ground_truth.quaternions_xyzw
        ]
    )
    return (
        InterpolatedGroundTruth(
            positions=aligned_positions,
            quaternions_xyzw=aligned_quaternions,
            # Odometry twist is already expressed in the shared child frame.
            linear_velocities=ground_truth.linear_velocities,
            angular_velocities=ground_truth.angular_velocities,
        ),
        translation,
        rotation_quaternion,
    )


def scalar_statistics(values: np.ndarray) -> Dict[str, float]:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        return {}
    return {
        "rmse": float(np.sqrt(np.mean(np.square(finite)))),
        "mean": float(np.mean(finite)),
        "median": float(np.median(finite)),
        "p95": float(np.percentile(finite, 95.0)),
        "max": float(np.max(finite)),
    }


def stream_timing(series: OdomSeries) -> Dict[str, float]:
    if series.stamps_ns.size < 2:
        return {"sample_count": int(series.stamps_ns.size)}
    gaps_s = np.diff(series.stamps_ns).astype(float) * 1e-9
    duration_s = float((series.stamps_ns[-1] - series.stamps_ns[0]) * 1e-9)
    return {
        "sample_count": int(series.stamps_ns.size),
        "duration_s": duration_s,
        "average_rate_hz": float((series.stamps_ns.size - 1) / duration_s),
        "median_gap_s": float(np.median(gaps_s)),
        "p95_gap_s": float(np.percentile(gaps_s, 95.0)),
        "max_gap_s": float(np.max(gaps_s)),
        "duplicate_count": series.duplicate_count,
    }


def relative_pose_errors(
    stamps_ns: np.ndarray,
    estimate_positions: np.ndarray,
    estimate_quaternions: np.ndarray,
    ground_truth_positions: np.ndarray,
    ground_truth_quaternions: np.ndarray,
    interval_s: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    translation_errors: List[float] = []
    rotation_errors: List[float] = []
    actual_intervals: List[float] = []
    target_interval_ns = int(round(interval_s * 1e9))
    tolerance_s = max(0.05, 0.1 * interval_s)
    for first in range(stamps_ns.size - 1):
        target_ns = int(stamps_ns[first]) + target_interval_ns
        insertion = int(np.searchsorted(stamps_ns, target_ns))
        candidates = [
            index
            for index in (insertion - 1, insertion)
            if first < index < stamps_ns.size
        ]
        if not candidates:
            continue
        second = min(
            candidates,
            key=lambda index: abs(int(stamps_ns[index]) - target_ns),
        )
        actual_interval_s = float(stamps_ns[second] - stamps_ns[first]) * 1e-9
        if abs(actual_interval_s - interval_s) > tolerance_s:
            continue
        estimate_relative_position = quaternion_to_rotation_matrix(
            estimate_quaternions[first]
        ).T @ (estimate_positions[second] - estimate_positions[first])
        ground_truth_relative_position = quaternion_to_rotation_matrix(
            ground_truth_quaternions[first]
        ).T @ (ground_truth_positions[second] - ground_truth_positions[first])
        estimate_relative_orientation = quaternion_multiply(
            quaternion_conjugate(estimate_quaternions[first]),
            estimate_quaternions[second],
        )
        ground_truth_relative_orientation = quaternion_multiply(
            quaternion_conjugate(ground_truth_quaternions[first]),
            ground_truth_quaternions[second],
        )
        translation_errors.append(
            float(
                np.linalg.norm(
                    estimate_relative_position - ground_truth_relative_position
                )
            )
        )
        rotation_errors.append(
            quaternion_error_angle_rad(
                ground_truth_relative_orientation,
                estimate_relative_orientation,
            )
        )
        actual_intervals.append(actual_interval_s)
    return (
        np.asarray(translation_errors),
        np.asarray(rotation_errors),
        np.asarray(actual_intervals),
    )


def write_csv(
    output_path: Path,
    estimate: OdomSeries,
    ground_truth: InterpolatedGroundTruth,
    position_errors: np.ndarray,
    orientation_errors_deg: np.ndarray,
    linear_velocity_errors: np.ndarray,
    angular_velocity_errors: np.ndarray,
) -> None:
    relative_time_s = (estimate.stamps_ns - estimate.stamps_ns[0]).astype(float) * 1e-9
    with output_path.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.writer(output_file)
        writer.writerow(
            [
                "time_s", "stamp_ns", "estimate_x_m", "estimate_y_m",
                "estimate_z_m", "ground_truth_x_m", "ground_truth_y_m",
                "ground_truth_z_m", "position_error_x_m", "position_error_y_m",
                "position_error_z_m", "position_error_norm_m",
                "orientation_error_deg", "linear_velocity_error_m_s",
                "angular_velocity_error_rad_s",
            ]
        )
        for index in range(estimate.stamps_ns.size):
            writer.writerow(
                [
                    relative_time_s[index],
                    int(estimate.stamps_ns[index]),
                    *estimate.positions[index],
                    *ground_truth.positions[index],
                    *position_errors[index],
                    np.linalg.norm(position_errors[index]),
                    orientation_errors_deg[index],
                    linear_velocity_errors[index],
                    angular_velocity_errors[index],
                ]
            )


def write_plots(
    output_directory: Path,
    estimate: OdomSeries,
    ground_truth: InterpolatedGroundTruth,
    position_errors: np.ndarray,
    orientation_errors_deg: np.ndarray,
    linear_velocity_errors: np.ndarray,
    angular_velocity_errors: np.ndarray,
    twist_metrics_available: bool,
) -> None:
    time_s = (estimate.stamps_ns - estimate.stamps_ns[0]).astype(float) * 1e-9
    figure, axis = plt.subplots(figsize=(8, 6))
    axis.plot(
        ground_truth.positions[:, 0], ground_truth.positions[:, 1],
        label="Ground truth (aligned)",
    )
    axis.plot(
        estimate.positions[:, 0], estimate.positions[:, 1], label="VIO estimate"
    )
    axis.set(xlabel="X [m]", ylabel="Y [m]", title="Aligned XY Trajectory")
    axis.axis("equal")
    axis.grid(True)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output_directory / "trajectory_xy.png", dpi=150)
    plt.close(figure)

    figure, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(time_s, np.linalg.norm(position_errors, axis=1))
    axes[0].set_ylabel("Position error [m]")
    axes[0].grid(True)
    axes[1].plot(time_s, orientation_errors_deg)
    axes[1].set(xlabel="Time since first match [s]", ylabel="Orientation error [deg]")
    axes[1].grid(True)
    figure.suptitle("Absolute Pose Error")
    figure.tight_layout()
    figure.savefig(output_directory / "pose_errors.png", dpi=150)
    plt.close(figure)

    if twist_metrics_available:
        figure, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        axes[0].plot(time_s, linear_velocity_errors)
        axes[0].set_ylabel("Linear error [m/s]")
        axes[0].grid(True)
        axes[1].plot(time_s, angular_velocity_errors)
        axes[1].set(
            xlabel="Time since first match [s]",
            ylabel="Angular error [rad/s]",
        )
        axes[1].grid(True)
        figure.suptitle("Body-Frame Twist Error")
        figure.tight_layout()
        figure.savefig(output_directory / "velocity_errors.png", dpi=150)
        plt.close(figure)


def format_statistic_block(name: str, units: str, statistics: Dict[str, float]) -> str:
    if not statistics:
        return f"{name}: unavailable"
    return (
        f"{name} [{units}]:\n"
        f"  RMSE:   {statistics['rmse']:.6f}\n"
        f"  Mean:   {statistics['mean']:.6f}\n"
        f"  Median: {statistics['median']:.6f}\n"
        f"  P95:    {statistics['p95']:.6f}\n"
        f"  Max:    {statistics['max']:.6f}"
    )


def evaluate(arguments: argparse.Namespace) -> Tuple[Dict, str, Path]:
    bag_directory = arguments.bag.resolve()
    if not bag_directory.is_dir() or not (bag_directory / "metadata.yaml").is_file():
        raise RuntimeError(f"{bag_directory} is not a ROS 2 bag directory")
    if (
        not math.isfinite(arguments.max_interpolation_gap_s)
        or arguments.max_interpolation_gap_s <= 0.0
    ):
        raise RuntimeError("--max-interpolation-gap-s must be finite and positive")
    if not math.isfinite(arguments.rpe_interval_s) or arguments.rpe_interval_s <= 0.0:
        raise RuntimeError("--rpe-interval-s must be finite and positive")
    output_directory = (
        arguments.output_dir.resolve()
        if arguments.output_dir
        else bag_directory / "evaluation"
    )
    output_directory.mkdir(parents=True, exist_ok=True)

    estimate, ground_truth, invalid_counts = read_odometry_topics(
        bag_directory,
        arguments.storage_id,
        arguments.estimate_topic,
        arguments.ground_truth_topic,
    )
    if estimate.child_frame_id != ground_truth.child_frame_id:
        raise RuntimeError(
            "estimate and ground truth use different child frames: "
            f"{estimate.child_frame_id} vs {ground_truth.child_frame_id}"
        )
    matched_estimate, interpolated_ground_truth, unmatched_count = (
        interpolate_ground_truth(
            estimate,
            ground_truth,
            arguments.max_interpolation_gap_s,
        )
    )
    aligned_ground_truth, alignment_translation, alignment_quaternion = (
        align_ground_truth_to_estimate(matched_estimate, interpolated_ground_truth)
    )

    position_errors = matched_estimate.positions - aligned_ground_truth.positions
    position_error_norms = np.linalg.norm(position_errors, axis=1)
    orientation_errors_deg = np.degrees(
        np.asarray(
            [
                quaternion_error_angle_rad(reference, estimate_quaternion)
                for reference, estimate_quaternion in zip(
                    aligned_ground_truth.quaternions_xyzw,
                    matched_estimate.quaternions_xyzw,
                )
            ]
        )
    )
    twist_metrics_available = (
        matched_estimate.twist_available and ground_truth.twist_available
    )
    if twist_metrics_available:
        linear_velocity_errors = np.linalg.norm(
            matched_estimate.linear_velocities - aligned_ground_truth.linear_velocities,
            axis=1,
        )
        angular_velocity_errors = np.linalg.norm(
            matched_estimate.angular_velocities -
            aligned_ground_truth.angular_velocities,
            axis=1,
        )
    else:
        linear_velocity_errors = np.full(matched_estimate.stamps_ns.size, np.nan)
        angular_velocity_errors = np.full(matched_estimate.stamps_ns.size, np.nan)
    rpe_translation, rpe_rotation_rad, rpe_actual_intervals = relative_pose_errors(
        matched_estimate.stamps_ns,
        matched_estimate.positions,
        matched_estimate.quaternions_xyzw,
        aligned_ground_truth.positions,
        aligned_ground_truth.quaternions_xyzw,
        arguments.rpe_interval_s,
    )
    trajectory_distance_m = float(
        np.sum(np.linalg.norm(np.diff(aligned_ground_truth.positions, axis=0), axis=1))
    )
    final_position_error_m = float(position_error_norms[-1])
    final_drift_percent = (
        100.0 * final_position_error_m / trajectory_distance_m
        if trajectory_distance_m > 1e-9
        else None
    )
    summary = {
        "bag": str(bag_directory),
        "estimate_topic": arguments.estimate_topic,
        "ground_truth_topic": arguments.ground_truth_topic,
        "estimate_frame_id": estimate.frame_id,
        "ground_truth_frame_id": ground_truth.frame_id,
        "child_frame_id": estimate.child_frame_id,
        "alignment": {
            "method": "first_matched_pose_se3",
            "translation_xyz_m": alignment_translation.tolist(),
            "quaternion_xyzw": alignment_quaternion.tolist(),
        },
        "estimate_timing": stream_timing(estimate),
        "ground_truth_timing": stream_timing(ground_truth),
        "matched_sample_count": int(matched_estimate.stamps_ns.size),
        "unmatched_estimate_count": int(unmatched_count),
        "matched_estimate_percent": float(
            100.0 * matched_estimate.stamps_ns.size / estimate.stamps_ns.size
        ),
        "invalid_message_counts": invalid_counts,
        "twist_metrics_available": twist_metrics_available,
        "position_ate_m": scalar_statistics(position_error_norms),
        "orientation_ate_deg": scalar_statistics(orientation_errors_deg),
        "linear_velocity_error_m_s": scalar_statistics(linear_velocity_errors),
        "angular_velocity_error_rad_s": scalar_statistics(angular_velocity_errors),
        "rpe": {
            "requested_interval_s": arguments.rpe_interval_s,
            "pair_count": int(rpe_translation.size),
            "actual_interval_s": scalar_statistics(rpe_actual_intervals),
            "translation_error_m": scalar_statistics(rpe_translation),
            "rotation_error_deg": scalar_statistics(np.degrees(rpe_rotation_rad)),
        },
        "ground_truth_distance_m": trajectory_distance_m,
        "final_position_error_m": final_position_error_m,
        "final_drift_percent": final_drift_percent,
    }

    write_csv(
        output_directory / "aligned_samples.csv",
        matched_estimate,
        aligned_ground_truth,
        position_errors,
        orientation_errors_deg,
        linear_velocity_errors,
        angular_velocity_errors,
    )
    if not arguments.no_plots:
        write_plots(
            output_directory,
            matched_estimate,
            aligned_ground_truth,
            position_errors,
            orientation_errors_deg,
            linear_velocity_errors,
            angular_velocity_errors,
            twist_metrics_available,
        )
    with (output_directory / "summary.json").open("w", encoding="utf-8") as output_file:
        json.dump(summary, output_file, indent=2)
        output_file.write("\n")

    drift_text = (
        f"{final_drift_percent:.3f}%"
        if final_drift_percent is not None
        else "unavailable (stationary trajectory)"
    )
    summary_text = "\n\n".join(
        [
            "VIO Accuracy Evaluation",
            (
                f"Estimate: {arguments.estimate_topic} "
                f"({estimate.frame_id} -> {estimate.child_frame_id})\n"
                f"Ground truth: {arguments.ground_truth_topic} "
                f"({ground_truth.frame_id} -> {ground_truth.child_frame_id})"
            ),
            (
                "Header-stamp timing:\n"
                f"  Estimate: {estimate.stamps_ns.size} samples over "
                f"{summary['estimate_timing']['duration_s']:.3f} s at "
                f"{summary['estimate_timing']['average_rate_hz']:.3f} Hz\n"
                f"  Ground truth: {ground_truth.stamps_ns.size} samples over "
                f"{summary['ground_truth_timing']['duration_s']:.3f} s at "
                f"{summary['ground_truth_timing']['average_rate_hz']:.3f} Hz\n"
                f"Matched samples: {matched_estimate.stamps_ns.size}/"
                f"{estimate.stamps_ns.size} "
                f"({summary['matched_estimate_percent']:.2f}%)"
            ),
            format_statistic_block("Position ATE", "m", summary["position_ate_m"]),
            format_statistic_block(
                "Orientation ATE", "deg", summary["orientation_ate_deg"]
            ),
            format_statistic_block(
                "Linear velocity error", "m/s", summary["linear_velocity_error_m_s"]
            ),
            format_statistic_block(
                "Angular velocity error", "rad/s",
                summary["angular_velocity_error_rad_s"],
            ),
            format_statistic_block(
                f"Translation RPE ({arguments.rpe_interval_s:.3f} s)",
                "m",
                summary["rpe"]["translation_error_m"],
            ),
            format_statistic_block(
                f"Rotation RPE ({arguments.rpe_interval_s:.3f} s)",
                "deg",
                summary["rpe"]["rotation_error_deg"],
            ),
            (
                f"Ground-truth distance: {trajectory_distance_m:.6f} m\n"
                f"Final position error: {final_position_error_m:.6f} m\n"
                f"Final drift: {drift_text}"
            ),
        ]
    )
    with (output_directory / "summary.txt").open("w", encoding="utf-8") as output_file:
        output_file.write(summary_text + "\n")
    return summary, summary_text, output_directory


def main() -> int:
    arguments = parse_arguments()
    try:
        _, summary_text, output_directory = evaluate(arguments)
    except (RuntimeError, OSError, ValueError) as exception:
        print(f"Evaluation failed: {exception}", file=sys.stderr)
        return 1
    print(summary_text)
    print(f"\nOutputs written to {output_directory}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
