#!/usr/bin/env python3
"""Read-only, non-GT verification metrics for native loop-closure bags.

The evaluator recovers the final native graph topology, reconstructs the
keyframes from the frozen FAST-LIVO2 input, and measures accepted closures
against deterministic, disjoint point-cloud fit/holdout partitions. It does
not replay bags, change detector parameters, or label closures TP/FP.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np


PINNED_FAST_LIVO2_SHA = "52db4d422e00a0e1588bed86d91ac4361b21aca9"
EXPECTED_FIXED_FRAME = "camera_init"
EXPECTED_BODY_FRAME = "aft_mapped"
DEFAULT_GRAPH_TOPIC = "/vis_pose_graph"
DEFAULT_ODOM_TOPIC = "/aft_mapped_to_init"
DEFAULT_CLOUD_TOPIC = "/cloud_registered_metric"
OVERLAP_RADII_M = (0.2, 0.5)


class EvaluationError(RuntimeError):
    """An input artifact cannot support a rigorous metric result."""


@dataclass(frozen=True)
class PoseSample:
    timestamp_ns: int
    xyz: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class GraphSnapshot:
    timestamp_ns: int
    frame_id: str
    original_nodes: tuple[tuple[float, float, float], ...]
    optimized_nodes: tuple[tuple[float, float, float], ...]
    accepted_pairs: tuple[tuple[int, int], ...]


@dataclass(frozen=True)
class CloudPartition:
    fit: np.ndarray
    holdout: np.ndarray


@dataclass(frozen=True)
class RegistrationResult:
    transform: np.ndarray
    converged: bool
    correspondences: int
    fit_rmse_m: float | None
    iterations: int


def _distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def _angular_distance(a: Sequence[float], b: Sequence[float]) -> float:
    dot = abs(sum(float(x) * float(y) for x, y in zip(a, b)))
    return 2.0 * math.acos(min(1.0, max(0.0, dot)))


def percentile(values: Sequence[float], quantile: float) -> float | None:
    """Return the linearly interpolated (R-7) sample quantile."""
    if not values:
        return None
    if not 0.0 <= quantile <= 1.0:
        raise ValueError("quantile must be in [0, 1]")
    ordered = sorted(float(value) for value in values)
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _summary(values: Sequence[float], unit: str | None = None) -> dict[str, object]:
    result: dict[str, object] = {
        "count": len(values),
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
    }
    if unit is not None:
        result["unit"] = unit
    return result


def select_keyframes(
    poses: Sequence[PoseSample], distance_threshold_m: float, angle_threshold_rad: float
) -> list[PoseSample]:
    """Reproduce the node's translation-or-rotation keyframe admission rule."""
    if distance_threshold_m < 0.0 or angle_threshold_rad < 0.0:
        raise EvaluationError("keyframe thresholds must be non-negative")
    selected: list[PoseSample] = []
    for pose in poses:
        if not selected:
            selected.append(pose)
            continue
        previous = selected[-1]
        if (
            _distance(previous.xyz, pose.xyz) >= distance_threshold_m
            or _angular_distance(previous.quaternion_xyzw, pose.quaternion_xyzw)
            >= angle_threshold_rad
        ):
            selected.append(pose)
    return selected


def _point_tuple(point: object) -> tuple[float, float, float]:
    result = (float(point.x), float(point.y), float(point.z))
    if not all(math.isfinite(value) for value in result):
        raise EvaluationError("graph contains a non-finite point")
    return result


def _unique_node_index(
    point: Sequence[float], nodes: Sequence[Sequence[float]], tolerance_m: float
) -> int:
    matches = [index for index, node in enumerate(nodes) if _distance(point, node) <= tolerance_m]
    if len(matches) != 1:
        raise EvaluationError(
            "loop endpoint does not identify exactly one optimized node "
            f"(matches={matches}, tolerance_m={tolerance_m})"
        )
    return matches[0]


def parse_graph_snapshot(
    marker_array: object, storage_timestamp_ns: int, identity_tolerance_m: float
) -> GraphSnapshot:
    """Validate one native graph snapshot and recover loop pair identities."""
    by_namespace: dict[str, object] = {}
    for marker in marker_array.markers:
        if marker.ns in by_namespace:
            raise EvaluationError(f"duplicate marker namespace {marker.ns!r}")
        by_namespace[marker.ns] = marker

    required = {"original_keyframes", "optimized_keyframes", "loop_edges", "nodes"}
    missing = required.difference(by_namespace)
    if missing:
        raise EvaluationError(f"native graph snapshot is missing markers: {sorted(missing)}")

    markers = [by_namespace[name] for name in sorted(required)]
    frames = {marker.header.frame_id for marker in markers}
    if frames != {EXPECTED_FIXED_FRAME}:
        raise EvaluationError(f"graph marker frames must be {EXPECTED_FIXED_FRAME!r}, got {sorted(frames)}")
    for marker in markers:
        if marker.action != 0:
            raise EvaluationError(f"graph marker {marker.ns!r} is not an ADD snapshot")
        translation = _point_tuple(marker.pose.position)
        rotation = marker.pose.orientation
        if _distance(translation, (0.0, 0.0, 0.0)) > identity_tolerance_m or any(
            abs(value) > 1e-12 for value in (rotation.x, rotation.y, rotation.z)
        ) or abs(abs(float(rotation.w)) - 1.0) > 1e-12:
            raise EvaluationError(f"graph marker {marker.ns!r} pose is not identity")

    original_marker = by_namespace["original_keyframes"]
    optimized_marker = by_namespace["optimized_keyframes"]
    loop_marker = by_namespace["loop_edges"]
    nodes_marker = by_namespace["nodes"]
    if original_marker.type != 4:
        raise EvaluationError("original_keyframes marker is not LINE_STRIP")
    if optimized_marker.type != 5 or loop_marker.type != 5:
        raise EvaluationError("optimized_keyframes and loop_edges must be LINE_LIST markers")
    if nodes_marker.type != 7:
        raise EvaluationError("nodes marker is not SPHERE_LIST")

    original_nodes = tuple(_point_tuple(point) for point in original_marker.points)
    optimized_nodes = tuple(_point_tuple(point) for point in nodes_marker.points)
    if len(original_nodes) != len(optimized_nodes):
        raise EvaluationError("original and optimized graph node counts differ")
    if len(optimized_marker.points) != max(0, 2 * (len(optimized_nodes) - 1)):
        raise EvaluationError("optimized odometry edge count is inconsistent with graph nodes")
    for edge_index in range(len(optimized_nodes) - 1):
        p0 = _point_tuple(optimized_marker.points[2 * edge_index])
        p1 = _point_tuple(optimized_marker.points[2 * edge_index + 1])
        if _distance(p0, optimized_nodes[edge_index]) > identity_tolerance_m or _distance(
            p1, optimized_nodes[edge_index + 1]
        ) > identity_tolerance_m:
            raise EvaluationError("optimized odometry edge endpoints are inconsistent with node order")

    if len(loop_marker.points) % 2:
        raise EvaluationError("loop marker has an odd number of endpoints")
    accepted_pairs: list[tuple[int, int]] = []
    for offset in range(0, len(loop_marker.points), 2):
        source_id = _unique_node_index(
            _point_tuple(loop_marker.points[offset]), optimized_nodes, identity_tolerance_m
        )
        target_id = _unique_node_index(
            _point_tuple(loop_marker.points[offset + 1]), optimized_nodes, identity_tolerance_m
        )
        if source_id == target_id:
            raise EvaluationError("accepted closure is a self-edge")
        accepted_pairs.append((source_id, target_id))
    undirected = {tuple(sorted(pair)) for pair in accepted_pairs}
    if len(undirected) != len(accepted_pairs):
        raise EvaluationError("native loop marker contains a duplicate undirected pair")

    return GraphSnapshot(
        timestamp_ns=int(storage_timestamp_ns),
        frame_id=EXPECTED_FIXED_FRAME,
        original_nodes=original_nodes,
        optimized_nodes=optimized_nodes,
        accepted_pairs=tuple(accepted_pairs),
    )


def validate_keyframes(
    graph: GraphSnapshot, keyframes: Sequence[PoseSample], tolerance_m: float
) -> list[PoseSample]:
    if len(keyframes) < len(graph.original_nodes):
        raise EvaluationError(
            f"FAST input produced {len(keyframes)} keyframes but graph has {len(graph.original_nodes)} nodes"
        )
    matched = list(keyframes[: len(graph.original_nodes)])
    for index, (native_point, input_pose) in enumerate(zip(graph.original_nodes, matched)):
        error_m = _distance(native_point, input_pose.xyz)
        if error_m > tolerance_m:
            raise EvaluationError(
                f"native original node {index} disagrees with reconstructed FAST keyframe "
                f"by {error_m:.9g} m"
            )
    return matched


def _read_rosbag_messages(bag_path: Path, topic: str) -> Iterable[tuple[str, object, int]]:
    """Yield deserialized messages without modifying or replaying a rosbag."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise EvaluationError("ROS 2 Python rosbag/message packages are required") from exc

    if not bag_path.is_dir() or not (bag_path / "metadata.yaml").is_file():
        raise EvaluationError(f"rosbag directory has no metadata.yaml: {bag_path}")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=""),
        rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format=""),
    )
    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise EvaluationError(f"required topic {topic!r} is absent from {bag_path}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    message_class = get_message(topic_types[topic])
    while reader.has_next():
        name, serialized, storage_timestamp_ns = reader.read_next()
        yield name, deserialize_message(serialized, message_class), int(storage_timestamp_ns)


def read_last_graph(bag_path: Path, topic: str, identity_tolerance_m: float) -> GraphSnapshot:
    last_message = None
    for last_message in _read_rosbag_messages(bag_path, topic):
        pass
    if last_message is None:
        raise EvaluationError(f"no messages found on required graph topic {topic!r}")
    _, message, storage_timestamp_ns = last_message
    return parse_graph_snapshot(message, storage_timestamp_ns, identity_tolerance_m)


def read_fast_odometry(bag_path: Path, topic: str) -> list[PoseSample]:
    samples: list[PoseSample] = []
    previous_stamp = -1
    for _, message, _ in _read_rosbag_messages(bag_path, topic):
        if message.header.frame_id != EXPECTED_FIXED_FRAME or message.child_frame_id != EXPECTED_BODY_FRAME:
            raise EvaluationError(
                f"FAST odometry frames must be {EXPECTED_FIXED_FRAME}->{EXPECTED_BODY_FRAME}"
            )
        stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        if stamp <= previous_stamp:
            raise EvaluationError("FAST odometry measurement timestamps are not strictly increasing")
        previous_stamp = stamp
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        xyz = (float(position.x), float(position.y), float(position.z))
        quat = tuple(
            float(value)
            for value in (orientation.x, orientation.y, orientation.z, orientation.w)
        )
        if not all(math.isfinite(value) for value in xyz + quat):
            raise EvaluationError("FAST odometry contains a non-finite pose")
        norm = math.sqrt(sum(value * value for value in quat))
        if abs(norm - 1.0) > 1e-3:
            raise EvaluationError(f"FAST odometry quaternion norm is {norm:.9g}, expected 1")
        samples.append(PoseSample(stamp, xyz, tuple(value / norm for value in quat)))
    if not samples:
        raise EvaluationError(f"no messages found on required FAST odometry topic {topic!r}")
    return samples


def _pointcloud_xyz(message: object) -> np.ndarray:
    """Decode organized or unorganized FLOAT32 XYZ fields without ROS mutation."""
    fields = {field.name: field for field in message.fields}
    if any(name not in fields for name in ("x", "y", "z")):
        raise EvaluationError("point cloud lacks x/y/z fields")
    for name in ("x", "y", "z"):
        if fields[name].datatype != 7 or fields[name].count != 1:
            raise EvaluationError(f"point cloud field {name!r} must be scalar FLOAT32")
    point_step = int(message.point_step)
    row_step = int(message.row_step)
    width = int(message.width)
    height = int(message.height)
    if point_step <= 0 or width <= 0 or height <= 0 or row_step < point_step * width:
        raise EvaluationError("point cloud dimensions/strides are invalid")
    if len(message.data) < row_step * height:
        raise EvaluationError("point cloud data are shorter than declared dimensions")
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [endian + "f4"] * 3,
            "offsets": [int(fields[name].offset) for name in ("x", "y", "z")],
            "itemsize": point_step,
        }
    )
    structured = np.ndarray(
        shape=(height, width),
        dtype=dtype,
        buffer=message.data,
        strides=(row_step, point_step),
    )
    xyz = np.column_stack([structured[name].reshape(-1) for name in ("x", "y", "z")])
    return np.asarray(xyz[np.isfinite(xyz).all(axis=1)], dtype=np.float64)


def read_keyframe_clouds(
    bag_path: Path, topic: str, required_timestamps_ns: set[int]
) -> dict[int, np.ndarray]:
    clouds: dict[int, np.ndarray] = {}
    previous_stamp = -1
    for _, message, _ in _read_rosbag_messages(bag_path, topic):
        if message.header.frame_id != EXPECTED_FIXED_FRAME:
            raise EvaluationError(
                f"FAST cloud frame must be {EXPECTED_FIXED_FRAME!r}, got {message.header.frame_id!r}"
            )
        stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        if stamp <= previous_stamp:
            raise EvaluationError("FAST cloud timestamps are not strictly increasing")
        previous_stamp = stamp
        if stamp in required_timestamps_ns:
            if stamp in clouds:
                raise EvaluationError(f"duplicate required cloud timestamp {stamp}")
            cloud = _pointcloud_xyz(message)
            if len(cloud) == 0:
                raise EvaluationError(f"required cloud at {stamp} has no finite XYZ points")
            clouds[stamp] = cloud
    missing = sorted(required_timestamps_ns.difference(clouds))
    if missing:
        raise EvaluationError(f"missing {len(missing)} exact-timestamp keyframe clouds; first={missing[0]}")
    return clouds


def _hash_voxels(voxels: np.ndarray) -> np.ndarray:
    unsigned = voxels.astype(np.int64, copy=False).view(np.uint64)
    return (
        unsigned[:, 0] * np.uint64(0x9E3779B185EBCA87)
        ^ unsigned[:, 1] * np.uint64(0xC2B2AE3D27D4EB4F)
        ^ unsigned[:, 2] * np.uint64(0x165667B19E3779F9)
    )


def partition_cloud(
    points: np.ndarray, voxel_size_m: float, max_points_per_partition: int
) -> CloudPartition:
    """Voxel-subsample, then deterministically split into disjoint fit/holdout points."""
    if voxel_size_m <= 0.0 or max_points_per_partition <= 0:
        raise EvaluationError("cloud partition parameters must be positive")
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise EvaluationError("point cloud array must have shape (N, 3)")
    finite = points[np.isfinite(points).all(axis=1)]
    voxels = np.floor(finite / voxel_size_m).astype(np.int64)
    _, first = np.unique(voxels, axis=0, return_index=True)
    points = finite[first]
    voxels = voxels[first]
    hashes = _hash_voxels(voxels)

    def select(mask: np.ndarray) -> np.ndarray:
        selected_points = points[mask]
        selected_hashes = hashes[mask]
        if len(selected_points) > max_points_per_partition:
            order = np.argsort(selected_hashes, kind="stable")[:max_points_per_partition]
            selected_points = selected_points[order]
        return np.ascontiguousarray(selected_points)

    fit_mask = (hashes & np.uint64(1)) == 0
    result = CloudPartition(select(fit_mask), select(~fit_mask))
    if len(result.fit) < 30 or len(result.holdout) < 30:
        raise EvaluationError("point cloud has fewer than 30 points in a deterministic partition")
    return result


def _nearest_distances(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    try:
        from scipy.spatial import cKDTree
    except ImportError as exc:
        raise EvaluationError("scipy is required for offline point-cloud evaluation") from exc
    distances, indices = cKDTree(target).query(source, k=1, workers=1)
    return np.asarray(distances), np.asarray(indices)


def _rigid_transform(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = target_center - rotation @ source_center
    return transform


def _transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


def _rotation_angle(rotation: np.ndarray) -> float:
    cosine = float((np.trace(rotation) - 1.0) * 0.5)
    return math.acos(min(1.0, max(-1.0, cosine)))


def register_fit_partition(
    source: np.ndarray,
    target: np.ndarray,
    initial_transform: np.ndarray,
    max_correspondence_m: float,
    trim_fraction: float,
    max_iterations: int,
) -> RegistrationResult:
    """Deterministic trimmed point-to-point ICP using only fit-partition points."""
    if max_correspondence_m <= 0.0 or not 0.0 < trim_fraction <= 1.0 or max_iterations <= 0:
        raise EvaluationError("registration parameters are invalid")
    transform = np.asarray(initial_transform, dtype=np.float64).copy()
    previous_rmse = math.inf
    converged = False
    correspondences = 0
    rmse: float | None = None
    iteration = 0
    for iteration in range(1, max_iterations + 1):
        moved = _transform_points(source, transform)
        distances, indices = _nearest_distances(moved, target)
        valid = np.flatnonzero(distances <= max_correspondence_m)
        if len(valid) < 30:
            return RegistrationResult(transform, False, len(valid), None, iteration)
        keep = max(30, int(math.floor(len(valid) * trim_fraction)))
        valid = valid[np.argsort(distances[valid], kind="stable")[:keep]]
        new_transform = _rigid_transform(source[valid], target[indices[valid]])
        fitted = _transform_points(source[valid], new_transform)
        rmse = float(np.sqrt(np.mean(np.sum((fitted - target[indices[valid]]) ** 2, axis=1))))
        delta = new_transform @ np.linalg.inv(transform)
        transform = new_transform
        correspondences = len(valid)
        if abs(previous_rmse - rmse) < 1e-6 and np.linalg.norm(delta[:3, 3]) < 1e-5 and _rotation_angle(
            delta[:3, :3]
        ) < 1e-5:
            converged = True
            break
        previous_rmse = rmse
    return RegistrationResult(transform, converged, correspondences, rmse, iteration)


def symmetric_holdout_overlap(
    source_holdout: np.ndarray,
    target_holdout: np.ndarray,
    transform: np.ndarray,
    radii_m: Sequence[float] = OVERLAP_RADII_M,
) -> dict[str, dict[str, float]]:
    """Score only held-out points; fit points never enter this calculation."""
    moved_source = _transform_points(source_holdout, transform)
    source_to_target, _ = _nearest_distances(moved_source, target_holdout)
    target_to_source, _ = _nearest_distances(target_holdout, moved_source)
    result: dict[str, dict[str, float]] = {}
    for radius in radii_m:
        forward = float(np.mean(source_to_target <= radius))
        reverse = float(np.mean(target_to_source <= radius))
        result[f"{radius:.1f}"] = {
            "source_to_target": forward,
            "target_to_source": reverse,
            "symmetric_mean": 0.5 * (forward + reverse),
        }
    return result


def cumulative_travel_m(keyframes: Sequence[PoseSample]) -> list[float]:
    result = [0.0]
    for previous, current in zip(keyframes, keyframes[1:]):
        result.append(result[-1] + _distance(previous.xyz, current.xyz))
    return result


def admission_measurements(
    pair: tuple[int, int],
    keyframes: Sequence[PoseSample],
    travel_m: Sequence[float],
    min_time_s: float,
    min_travel_m: float,
) -> dict[str, object]:
    source_id, target_id = pair
    time_s = abs(keyframes[source_id].timestamp_ns - keyframes[target_id].timestamp_ns) / 1e9
    path_m = abs(float(travel_m[source_id]) - float(travel_m[target_id]))
    return {
        "time_separation_s": time_s,
        "travel_separation_m": path_m,
        "time_gate_strictly_greater_than_s": min_time_s,
        "travel_gate_strictly_greater_than_m": min_travel_m,
        "passes_time_gate": time_s > min_time_s,
        "passes_travel_gate": path_m > min_travel_m,
    }


def choose_admissible_nonedge_controls(
    accepted_pairs: Sequence[tuple[int, int]],
    keyframes: Sequence[PoseSample],
    travel_m: Sequence[float],
    min_time_s: float,
    min_travel_m: float,
) -> list[tuple[int, int]]:
    """Pair each source with a reused-data, admissible target that is not a native edge."""
    native = {tuple(sorted(pair)) for pair in accepted_pairs}
    target_pool = sorted({target for _, target in accepted_pairs})
    usage = {target: 0 for target in target_pool}
    controls: list[tuple[int, int]] = []
    for source, native_target in accepted_pairs:
        eligible = []
        for target in target_pool:
            pair = (source, target)
            admission = admission_measurements(
                pair, keyframes, travel_m, min_time_s, min_travel_m
            )
            if (
                source != target
                and tuple(sorted(pair)) not in native
                and admission["passes_time_gate"]
                and admission["passes_travel_gate"]
            ):
                eligible.append(target)
        if not eligible:
            raise EvaluationError(f"no admissible native-nonedge control for source node {source}")
        target = min(eligible, key=lambda value: (usage[value], -abs(value - native_target), value))
        usage[target] += 1
        controls.append((source, target))
    return controls


def _quaternion_matrix(quaternion_xyzw: Sequence[float]) -> np.ndarray:
    x, y, z, w = (float(value) for value in quaternion_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _pose_matrix(pose: PoseSample) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = _quaternion_matrix(pose.quaternion_xyzw)
    result[:3, 3] = pose.xyz
    return result


def evaluate_cloud_pairs(
    pairs: Sequence[tuple[int, int]],
    keyframes: Sequence[PoseSample],
    travel_m: Sequence[float],
    partitions: dict[int, CloudPartition],
    min_time_s: float,
    min_travel_m: float,
    max_correspondence_m: float,
    trim_fraction: float,
    max_iterations: int,
) -> list[dict[str, object]]:
    results: list[dict[str, object]] = []
    for source_id, target_id in pairs:
        source_pose = keyframes[source_id]
        target_pose = keyframes[target_id]
        initial = np.eye(4)
        initial[:3, 3] = np.asarray(target_pose.xyz) - np.asarray(source_pose.xyz)
        registration = register_fit_partition(
            partitions[source_id].fit,
            partitions[target_id].fit,
            initial,
            max_correspondence_m,
            trim_fraction,
            max_iterations,
        )
        overlap = symmetric_holdout_overlap(
            partitions[source_id].holdout,
            partitions[target_id].holdout,
            registration.transform,
        )
        results.append(
            {
                "source_id": source_id,
                "target_id": target_id,
                "source_timestamp_ns": source_pose.timestamp_ns,
                "target_timestamp_ns": target_pose.timestamp_ns,
                "fast_endpoint_distance_m": _distance(source_pose.xyz, target_pose.xyz),
                "admission": admission_measurements(
                    (source_id, target_id), keyframes, travel_m, min_time_s, min_travel_m
                ),
                "partition_point_counts": {
                    "source_fit": len(partitions[source_id].fit),
                    "source_holdout": len(partitions[source_id].holdout),
                    "target_fit": len(partitions[target_id].fit),
                    "target_holdout": len(partitions[target_id].holdout),
                },
                "fit_registration": {
                    "converged": registration.converged,
                    "correspondences": registration.correspondences,
                    "rmse_m": registration.fit_rmse_m,
                    "iterations": registration.iterations,
                    "transform_source_fixed_to_target_fixed": registration.transform.tolist(),
                },
                "holdout_overlap": overlap,
            }
        )
    return results


def _overlap_summary(results: Sequence[dict[str, object]]) -> dict[str, object]:
    summary: dict[str, object] = {}
    for radius in OVERLAP_RADII_M:
        key = f"{radius:.1f}"
        values = [float(item["holdout_overlap"][key]["symmetric_mean"]) for item in results]
        summary[f"at_{key}_m"] = _summary(values, "fraction")
    return summary


def _pair_admission_summary(results: Sequence[dict[str, object]]) -> dict[str, object]:
    time_values = [float(item["admission"]["time_separation_s"]) for item in results]
    travel_values = [float(item["admission"]["travel_separation_m"]) for item in results]
    return {
        "passing_time_gate": sum(bool(item["admission"]["passes_time_gate"]) for item in results),
        "passing_travel_gate": sum(
            bool(item["admission"]["passes_travel_gate"]) for item in results
        ),
        "time_separation": _summary(time_values, "s"),
        "travel_separation": _summary(travel_values, "m"),
    }


def _registration_summary(results: Sequence[dict[str, object]]) -> dict[str, object]:
    rmse = [
        float(item["fit_registration"]["rmse_m"])
        for item in results
        if item["fit_registration"]["rmse_m"] is not None
    ]
    correspondences = [
        float(item["fit_registration"]["correspondences"]) for item in results
    ]
    iterations = [float(item["fit_registration"]["iterations"]) for item in results]
    return {
        "settled_before_iteration_limit_count": sum(
            bool(item["fit_registration"]["converged"]) for item in results
        ),
        "fit_rmse": _summary(rmse, "m"),
        "fit_correspondences": _summary(correspondences, "points"),
        "iterations": _summary(iterations, "iterations"),
    }


def _control_contrast(
    accepted: Sequence[dict[str, object]], controls: Sequence[dict[str, object]]
) -> dict[str, object]:
    result: dict[str, object] = {
        "interpretation": "descriptive only; the control p95 is not an admission or truth threshold"
    }
    for radius in OVERLAP_RADII_M:
        key = f"{radius:.1f}"
        accepted_values = [
            float(item["holdout_overlap"][key]["symmetric_mean"]) for item in accepted
        ]
        control_values = [
            float(item["holdout_overlap"][key]["symmetric_mean"]) for item in controls
        ]
        control_p95 = percentile(control_values, 0.95)
        result[f"at_{key}_m"] = {
            "control_p95": control_p95,
            "accepted_at_or_below_control_p95": (
                None
                if control_p95 is None
                else sum(value <= control_p95 for value in accepted_values)
            ),
        }
    return result


def _topology_summary(graph: GraphSnapshot) -> dict[str, object]:
    loop_degrees = [0] * len(graph.original_nodes)
    spans = []
    original_chords = []
    optimized_chords = []
    node_displacements = [
        _distance(original, optimized)
        for original, optimized in zip(graph.original_nodes, graph.optimized_nodes)
    ]
    adjacent_length_changes = []
    for index in range(len(graph.original_nodes) - 1):
        original_length = _distance(graph.original_nodes[index], graph.original_nodes[index + 1])
        optimized_length = _distance(graph.optimized_nodes[index], graph.optimized_nodes[index + 1])
        adjacent_length_changes.append(abs(optimized_length - original_length))
    for source, target in graph.accepted_pairs:
        loop_degrees[source] += 1
        loop_degrees[target] += 1
        spans.append(abs(source - target))
        original_chords.append(_distance(graph.original_nodes[source], graph.original_nodes[target]))
        optimized_chords.append(_distance(graph.optimized_nodes[source], graph.optimized_nodes[target]))
    return {
        "unique_undirected_closure_count": len(graph.accepted_pairs),
        "closure_cycle_rank_with_connected_odometry_chain": len(graph.accepted_pairs),
        "nodes_incident_to_closures": sum(degree > 0 for degree in loop_degrees),
        "maximum_loop_degree": max(loop_degrees, default=0),
        "node_index_span": _summary(spans, "keyframe_index"),
        "original_fast_endpoint_chord": _summary(original_chords, "m"),
        "optimized_endpoint_chord_not_factor_residual": _summary(optimized_chords, "m"),
        "node_position_displacement_original_to_optimized": _summary(node_displacements, "m"),
        "adjacent_edge_absolute_length_change": _summary(adjacent_length_changes, "m"),
    }


def _registration_cycle_summary(
    accepted: Sequence[dict[str, object]], keyframes: Sequence[PoseSample]
) -> dict[str, object]:
    """Estimate adjacent-closure cycle disagreement from independent registrations."""
    by_source = sorted(accepted, key=lambda item: (int(item["source_id"]), int(item["target_id"])))
    translation_residuals: list[float] = []
    rotation_residuals: list[float] = []
    settled_translation_residuals: list[float] = []
    settled_rotation_residuals: list[float] = []
    for first, second in zip(by_source, by_source[1:]):
        s1, t1 = int(first["source_id"]), int(first["target_id"])
        s2, t2 = int(second["source_id"]), int(second["target_id"])
        a1 = np.asarray(first["fit_registration"]["transform_source_fixed_to_target_fixed"])
        a2 = np.asarray(second["fit_registration"]["transform_source_fixed_to_target_fixed"])
        ts1, tt1, ts2, tt2 = map(_pose_matrix, (keyframes[s1], keyframes[t1], keyframes[s2], keyframes[t2]))
        m1 = np.linalg.inv(tt1) @ a1 @ ts1
        m2 = np.linalg.inv(tt2) @ a2 @ ts2
        odom_s2_s1 = np.linalg.inv(ts2) @ ts1
        odom_t1_t2 = np.linalg.inv(tt1) @ tt2
        cycle = np.linalg.inv(m1) @ odom_t1_t2 @ m2 @ odom_s2_s1
        translation = float(np.linalg.norm(cycle[:3, 3]))
        rotation = _rotation_angle(cycle[:3, :3])
        translation_residuals.append(translation)
        rotation_residuals.append(rotation)
        if bool(first["fit_registration"]["converged"]) and bool(
            second["fit_registration"]["converged"]
        ):
            settled_translation_residuals.append(translation)
            settled_rotation_residuals.append(rotation)
    return {
        "definition": (
            "SE(3) cycle made from two consecutive accepted-edge fit registrations "
            "and the intervening frozen FAST odometry segments"
        ),
        "estimated_not_native_factor_residual": True,
        "cycle_count": len(translation_residuals),
        "translation_residual": _summary(translation_residuals, "m"),
        "rotation_residual": _summary(rotation_residuals, "rad"),
        "both_registrations_settled_subset": {
            "cycle_count": len(settled_translation_residuals),
            "translation_residual": _summary(settled_translation_residuals, "m"),
            "rotation_residual": _summary(settled_rotation_residuals, "rad"),
        },
    }


def evaluate(
    graph: GraphSnapshot,
    keyframes: Sequence[PoseSample],
    clouds_by_timestamp: dict[int, np.ndarray],
    *,
    min_time_s: float,
    min_travel_m: float,
    voxel_size_m: float,
    max_points_per_partition: int,
    max_correspondence_m: float,
    trim_fraction: float,
    max_iterations: int,
) -> dict[str, object]:
    travel = cumulative_travel_m(keyframes)
    controls = choose_admissible_nonedge_controls(
        graph.accepted_pairs, keyframes, travel, min_time_s, min_travel_m
    )
    required_ids = {node for pair in graph.accepted_pairs + tuple(controls) for node in pair}
    partitions = {
        node_id: partition_cloud(
            clouds_by_timestamp[keyframes[node_id].timestamp_ns], voxel_size_m, max_points_per_partition
        )
        for node_id in required_ids
    }
    accepted = evaluate_cloud_pairs(
        graph.accepted_pairs,
        keyframes,
        travel,
        partitions,
        min_time_s,
        min_travel_m,
        max_correspondence_m,
        trim_fraction,
        max_iterations,
    )
    control_results = evaluate_cloud_pairs(
        controls,
        keyframes,
        travel,
        partitions,
        min_time_s,
        min_travel_m,
        max_correspondence_m,
        trim_fraction,
        max_iterations,
    )
    time_values = [float(item["admission"]["time_separation_s"]) for item in accepted]
    travel_values = [float(item["admission"]["travel_separation_m"]) for item in accepted]
    endpoint_values = [float(item["fast_endpoint_distance_m"]) for item in accepted]
    return {
        "graph": {
            "snapshot_storage_timestamp_ns": graph.timestamp_ns,
            "frame_id": graph.frame_id,
            "keyframe_count": len(keyframes),
            "odometry_edge_count": max(0, len(keyframes) - 1),
            "topology": _topology_summary(graph),
        },
        "temporal_and_travel_admission": {
            "conditions_are_separate": True,
            "accepted_passing_time_gate": sum(
                bool(item["admission"]["passes_time_gate"]) for item in accepted
            ),
            "accepted_passing_travel_gate": sum(
                bool(item["admission"]["passes_travel_gate"]) for item in accepted
            ),
            "time_separation": _summary(time_values, "s"),
            "travel_separation": _summary(travel_values, "m"),
            "fast_endpoint_separation_not_travel": _summary(endpoint_values, "m"),
        },
        "pointcloud_verification": {
            "fit_holdout_disjoint": True,
            "accepted_closures": {
                "count": len(accepted),
                "admission": _pair_admission_summary(accepted),
                "symmetric_holdout_overlap": _overlap_summary(accepted),
                "fit_registration": _registration_summary(accepted),
                "pairs": accepted,
            },
            "admissible_nonedge_controls": {
                "count": len(control_results),
                "truth_label": None,
                "admission": _pair_admission_summary(control_results),
                "symmetric_holdout_overlap": _overlap_summary(control_results),
                "fit_registration": _registration_summary(control_results),
                "pairs": control_results,
            },
            "descriptive_control_contrast_not_admission": _control_contrast(
                accepted, control_results
            ),
        },
        "registration_cycle_consistency": _registration_cycle_summary(accepted, keyframes),
        "truth_metrics": {
            "tp": None,
            "fp": None,
            "precision": None,
            "recall": None,
            "status": "not_computed_no_ground_truth_or_independent_adjudication",
        },
    }


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend-bag", type=Path, required=True)
    parser.add_argument("--fast-bag", type=Path, required=True)
    parser.add_argument("--dataset", choices=("UGV10", "Retail", "CBD03"), required=True)
    parser.add_argument("--fast-livo2-sha", required=True)
    parser.add_argument("--output", type=Path, help="write JSON here instead of stdout")
    parser.add_argument("--graph-topic", default=DEFAULT_GRAPH_TOPIC)
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    parser.add_argument("--cloud-topic", default=DEFAULT_CLOUD_TOPIC)
    parser.add_argument("--keyframe-distance-m", type=float, default=0.5)
    parser.add_argument("--keyframe-angle-rad", type=float, default=0.3)
    parser.add_argument("--min-time-separation-s", type=float, default=30.0)
    parser.add_argument("--min-travel-separation-m", type=float, default=30.0)
    parser.add_argument("--voxel-size-m", type=float, default=0.1)
    parser.add_argument("--max-points-per-partition", type=int, default=12000)
    parser.add_argument("--max-correspondence-m", type=float, default=1.0)
    parser.add_argument("--trim-fraction", type=float, default=0.8)
    parser.add_argument("--max-iterations", type=int, default=40)
    parser.add_argument("--identity-tolerance-m", type=float, default=1e-9)
    parser.add_argument("--original-geometry-tolerance-m", type=float, default=1e-6)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        if args.fast_livo2_sha != PINNED_FAST_LIVO2_SHA:
            raise EvaluationError(
                f"FAST-LIVO2 SHA must be pinned to {PINNED_FAST_LIVO2_SHA}, got {args.fast_livo2_sha}"
            )
        numeric_positive = {
            "min_time_separation_s": args.min_time_separation_s,
            "min_travel_separation_m": args.min_travel_separation_m,
            "voxel_size_m": args.voxel_size_m,
            "max_correspondence_m": args.max_correspondence_m,
        }
        if any(value <= 0.0 for value in numeric_positive.values()):
            raise EvaluationError(f"parameters must be positive: {numeric_positive}")
        if args.identity_tolerance_m < 0.0 or args.original_geometry_tolerance_m < 0.0:
            raise EvaluationError("geometry tolerances must be non-negative")
        graph = read_last_graph(args.backend_bag.resolve(), args.graph_topic, args.identity_tolerance_m)
        fast_poses = read_fast_odometry(args.fast_bag.resolve(), args.odom_topic)
        reconstructed = select_keyframes(
            fast_poses, args.keyframe_distance_m, args.keyframe_angle_rad
        )
        keyframes = validate_keyframes(graph, reconstructed, args.original_geometry_tolerance_m)
        travel = cumulative_travel_m(keyframes)
        controls = choose_admissible_nonedge_controls(
            graph.accepted_pairs,
            keyframes,
            travel,
            args.min_time_separation_s,
            args.min_travel_separation_m,
        )
        required_ids = {node for pair in graph.accepted_pairs + tuple(controls) for node in pair}
        required_stamps = {keyframes[node].timestamp_ns for node in required_ids}
        clouds = read_keyframe_clouds(args.fast_bag.resolve(), args.cloud_topic, required_stamps)
        report = {
            "schema_version": 2,
            "dataset": args.dataset,
            "provenance": {
                "fast_livo2_sha": args.fast_livo2_sha,
                "fast_bag": str(args.fast_bag.resolve()),
                "backend_bag": str(args.backend_bag.resolve()),
                "read_only": True,
            },
            "fixed_evaluation_parameters": {
                "overlap_radii_m": list(OVERLAP_RADII_M),
                "min_time_separation_s": args.min_time_separation_s,
                "min_travel_separation_m": args.min_travel_separation_m,
                "voxel_size_m": args.voxel_size_m,
                "max_points_per_partition": args.max_points_per_partition,
                "max_correspondence_m": args.max_correspondence_m,
                "trim_fraction": args.trim_fraction,
                "max_iterations": args.max_iterations,
            },
            **evaluate(
                graph,
                keyframes,
                clouds,
                min_time_s=args.min_time_separation_s,
                min_travel_m=args.min_travel_separation_m,
                voxel_size_m=args.voxel_size_m,
                max_points_per_partition=args.max_points_per_partition,
                max_correspondence_m=args.max_correspondence_m,
                trim_fraction=args.trim_fraction,
                max_iterations=args.max_iterations,
            ),
        }
        rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
        if args.output:
            args.output.write_text(rendered, encoding="utf-8")
        else:
            sys.stdout.write(rendered)
        return 0
    except (EvaluationError, OSError, ValueError, np.linalg.LinAlgError) as exc:
        print(f"evaluation failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
