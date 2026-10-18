#!/usr/bin/env python3
"""Deterministic, read-only A/B metrics for native SimpleLoopClosure output.

This tool compares exact-stamp FAST odometry with /pgo_odom and inspects the
last native /vis_pose_graph snapshot.  It does not estimate ground truth,
re-run optimization, or reconstruct unavailable loop-factor measurements.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np


FIXED_FRAME = "camera_init"
BODY_FRAME = "aft_mapped"
ODOM_TOPIC = "/aft_mapped_to_init"
PGO_ODOM_TOPIC = "/pgo_odom"
GRAPH_TOPIC = "/vis_pose_graph"


class EvaluationError(RuntimeError):
    pass


@dataclass(frozen=True)
class PoseSample:
    timestamp_ns: int
    transform: np.ndarray


def _rotation_angle(rotation: np.ndarray) -> float:
    cosine = float((np.trace(rotation) - 1.0) * 0.5)
    return math.acos(min(1.0, max(-1.0, cosine)))


def _pose_matrix(position: Sequence[float], quaternion: Sequence[float]) -> np.ndarray:
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or abs(norm - 1.0) > 1e-3:
        raise EvaluationError(f"odometry quaternion norm is {norm:.9g}, expected 1")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    result = np.eye(4)
    result[:3, :3] = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    result[:3, 3] = tuple(float(value) for value in position)
    if not np.isfinite(result).all():
        raise EvaluationError("odometry pose contains a non-finite value")
    return result


def _percentile(values: Sequence[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    position = (len(ordered) - 1) * quantile
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def distribution(values: Sequence[float], unit: str) -> dict[str, object]:
    finite = [float(value) for value in values]
    if not all(math.isfinite(value) for value in finite):
        raise EvaluationError("metric distribution contains a non-finite value")
    return {
        "count": len(finite),
        "unit": unit,
        "min": min(finite) if finite else None,
        "mean": sum(finite) / len(finite) if finite else None,
        "p50": _percentile(finite, 0.50),
        "p95": _percentile(finite, 0.95),
        "max": max(finite) if finite else None,
    }


def correction_bands(values_m: Sequence[float]) -> dict[str, object]:
    """Descriptive scale bands; these are not correctness thresholds."""
    return {
        "definition": "translation magnitude only; descriptive, not a loop truth label",
        "centimetre_scale_lt_0_10_m": sum(value < 0.10 for value in values_m),
        "decimetre_to_sub_1_m_ge_0_10_lt_1": sum(0.10 <= value < 1.0 for value in values_m),
        "metre_scale_ge_1_lt_5": sum(1.0 <= value < 5.0 for value in values_m),
        "five_metre_scale_ge_5": sum(value >= 5.0 for value in values_m),
    }


def compare_pose_streams(
    original: dict[int, PoseSample], optimized: dict[int, PoseSample]
) -> dict[str, object]:
    missing = sorted(set(optimized).difference(original))
    if missing:
        raise EvaluationError(
            f"{len(missing)} /pgo_odom stamps lack exact FAST odometry; first={missing[0]}"
        )
    stamps = sorted(optimized)
    if not stamps:
        raise EvaluationError("no /pgo_odom samples were found")

    displacement, correction_rotation = [], []
    edge_translation, edge_rotation = [], []
    corrections = []
    for stamp in stamps:
        before = original[stamp].transform
        after = optimized[stamp].transform
        correction = after @ np.linalg.inv(before)
        translation_m = float(np.linalg.norm(after[:3, 3] - before[:3, 3]))
        rotation_rad = _rotation_angle(correction[:3, :3])
        displacement.append(translation_m)
        correction_rotation.append(rotation_rad)
        corrections.append((stamp, translation_m, rotation_rad))

    for first_stamp, second_stamp in zip(stamps, stamps[1:]):
        before_0, before_1 = original[first_stamp].transform, original[second_stamp].transform
        after_0, after_1 = optimized[first_stamp].transform, optimized[second_stamp].transform
        before_edge = np.linalg.inv(before_0) @ before_1
        after_edge = np.linalg.inv(after_0) @ after_1
        residual = np.linalg.inv(before_edge) @ after_edge
        edge_translation.append(float(np.linalg.norm(residual[:3, 3])))
        edge_rotation.append(_rotation_angle(residual[:3, :3]))

    worst = max(corrections, key=lambda item: (item[1], item[0]))
    return {
        "matched_exact_stamp_count": len(stamps),
        "unmatched_fast_sample_count": len(original) - len(stamps),
        "pose_correction": {
            "definition": "T_pgo * inverse(T_fast), compared at identical measurement stamps",
            "position_displacement": distribution(displacement, "m"),
            "rotation_angle": distribution(correction_rotation, "rad"),
            "translation_scale_bands": correction_bands(displacement),
            "maximum_translation_sample": {
                "timestamp_ns": worst[0],
                "translation_m": worst[1],
                "rotation_rad": worst[2],
            },
        },
        "sequential_edge_consistency": {
            "definition": "inverse(relative_FAST_i_j) * relative_PGO_i_j; not a GTSAM factor residual",
            "translation_residual": distribution(edge_translation, "m"),
            "rotation_residual": distribution(edge_rotation, "rad"),
        },
    }


def _xyz(point: object) -> np.ndarray:
    value = np.array((point.x, point.y, point.z), dtype=float)
    if not np.isfinite(value).all():
        raise EvaluationError("graph contains a non-finite point")
    return value


def evaluate_graph(marker_array: object, identity_tolerance_m: float = 1e-6) -> dict[str, object]:
    by_ns = {marker.ns: marker for marker in marker_array.markers}
    required = {"original_keyframes", "optimized_keyframes", "loop_edges", "nodes"}
    if len(by_ns) != len(marker_array.markers):
        raise EvaluationError("graph snapshot contains duplicate marker namespaces")
    missing = sorted(required.difference(by_ns))
    if missing:
        raise EvaluationError(f"graph snapshot is missing markers: {missing}")
    for name in required:
        marker = by_ns[name]
        if marker.header.frame_id != FIXED_FRAME:
            raise EvaluationError(f"graph marker {name!r} is not in {FIXED_FRAME!r}")
        if marker.action != 0:
            raise EvaluationError(f"graph marker {name!r} is not an ADD snapshot")
        marker_translation = _xyz(marker.pose.position)
        marker_rotation = marker.pose.orientation
        if (
            np.linalg.norm(marker_translation) > identity_tolerance_m
            or any(abs(float(value)) > 1e-12 for value in (
                marker_rotation.x, marker_rotation.y, marker_rotation.z
            ))
            or abs(abs(float(marker_rotation.w)) - 1.0) > 1e-12
        ):
            raise EvaluationError(f"graph marker {name!r} pose is not identity")

    expected_types = {
        "original_keyframes": 4,  # LINE_STRIP
        "optimized_keyframes": 5,  # LINE_LIST
        "loop_edges": 5,  # LINE_LIST
        "nodes": 7,  # SPHERE_LIST
    }
    for name, expected_type in expected_types.items():
        if by_ns[name].type != expected_type:
            raise EvaluationError(f"graph marker {name!r} has unexpected type {by_ns[name].type}")

    original = [_xyz(point) for point in by_ns["original_keyframes"].points]
    optimized = [_xyz(point) for point in by_ns["nodes"].points]
    if len(original) != len(optimized):
        raise EvaluationError("original and optimized graph node counts differ")
    optimized_edges = by_ns["optimized_keyframes"].points
    if len(optimized_edges) != 2 * max(0, len(optimized) - 1):
        raise EvaluationError("optimized sequential marker has an inconsistent endpoint count")
    for index in range(len(optimized) - 1):
        if (
            np.linalg.norm(_xyz(optimized_edges[2 * index]) - optimized[index]) > identity_tolerance_m
            or np.linalg.norm(_xyz(optimized_edges[2 * index + 1]) - optimized[index + 1])
            > identity_tolerance_m
        ):
            raise EvaluationError("optimized sequential marker does not follow node order")

    displacements = [float(np.linalg.norm(after - before)) for before, after in zip(original, optimized)]
    vector_residuals, length_changes = [], []
    for index in range(len(original) - 1):
        before_edge = original[index + 1] - original[index]
        after_edge = optimized[index + 1] - optimized[index]
        vector_residuals.append(float(np.linalg.norm(after_edge - before_edge)))
        length_changes.append(float(abs(np.linalg.norm(after_edge) - np.linalg.norm(before_edge))))

    loop_points = by_ns["loop_edges"].points
    if len(loop_points) % 2:
        raise EvaluationError("loop marker has an odd endpoint count")

    def node_id(point: object) -> int:
        matches = [
            index for index, node in enumerate(optimized)
            if np.linalg.norm(_xyz(point) - node) <= identity_tolerance_m
        ]
        if len(matches) != 1:
            raise EvaluationError(f"loop endpoint matches {len(matches)} optimized nodes")
        return matches[0]

    loops = []
    undirected_pairs = set()
    for offset in range(0, len(loop_points), 2):
        source, target = node_id(loop_points[offset]), node_id(loop_points[offset + 1])
        pair = tuple(sorted((source, target)))
        if source == target:
            raise EvaluationError("loop marker contains a self-edge")
        if pair in undirected_pairs:
            raise EvaluationError("loop marker contains a duplicate undirected edge")
        undirected_pairs.add(pair)
        before_chord = float(np.linalg.norm(original[source] - original[target]))
        after_chord = float(np.linalg.norm(optimized[source] - optimized[target]))
        loops.append({
            "source_id": source,
            "target_id": target,
            "original_endpoint_chord_m": before_chord,
            "optimized_endpoint_chord_m": after_chord,
            "absolute_chord_change_m": abs(after_chord - before_chord),
            "maximum_endpoint_displacement_m": max(displacements[source], displacements[target]),
            "five_metre_scale_correction": max(displacements[source], displacements[target]) >= 5.0,
        })

    return {
        "frame_id": FIXED_FRAME,
        "node_count": len(original),
        "accepted_loop_edge_count": len(loops),
        "node_position_displacement": distribution(displacements, "m"),
        "node_translation_scale_bands": correction_bands(displacements),
        "sequential_edge_consistency": {
            "definition": "optimized-minus-original 3D edge vector; positions only",
            "vector_residual": distribution(vector_residuals, "m"),
            "absolute_length_change": distribution(length_changes, "m"),
        },
        "accepted_loop_endpoint_chords": {
            "warning": "endpoint chord changes are not loop-factor residuals or correctness labels",
            "pairs": loops,
        },
        "unavailable_from_native_markers": [
            "optimized keyframe orientation",
            "loop-factor measurement and covariance",
            "GTSAM factor residual",
        ],
    }


def _messages(bag: Path, topic: str):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise EvaluationError("ROS 2 Humble rosbag Python packages are required") from exc
    if not (bag / "metadata.yaml").is_file():
        raise EvaluationError(f"rosbag has no metadata.yaml: {bag}")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=""),
        rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format=""),
    )
    types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if topic not in types:
        raise EvaluationError(f"required topic {topic!r} is absent from {bag}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    message_type = get_message(types[topic])
    while reader.has_next():
        name, serialized, storage_stamp = reader.read_next()
        yield name, deserialize_message(serialized, message_type), int(storage_stamp)


def read_odometry(bag: Path, topic: str) -> dict[int, PoseSample]:
    result = {}
    for _, message, _ in _messages(bag, topic):
        if message.header.frame_id != FIXED_FRAME or message.child_frame_id != BODY_FRAME:
            raise EvaluationError(
                f"{topic} frames must be {FIXED_FRAME}->{BODY_FRAME}, got "
                f"{message.header.frame_id}->{message.child_frame_id}"
            )
        stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        if stamp in result:
            raise EvaluationError(f"duplicate measurement stamp {stamp} on {topic}")
        pose = message.pose.pose
        result[stamp] = PoseSample(
            stamp,
            _pose_matrix(
                (pose.position.x, pose.position.y, pose.position.z),
                (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            ),
        )
    if not result:
        raise EvaluationError(f"no messages found on {topic}")
    return result


def read_last_graph(bag: Path, topic: str):
    last = None
    for _, message, storage_stamp in _messages(bag, topic):
        last = (message, storage_stamp)
    if last is None:
        raise EvaluationError(f"no messages found on {topic}")
    return last


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fast-bag", type=Path, required=True)
    parser.add_argument("--backend-bag", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--odom-topic", default=ODOM_TOPIC)
    parser.add_argument("--pgo-odom-topic", default=PGO_ODOM_TOPIC)
    parser.add_argument("--graph-topic", default=GRAPH_TOPIC)
    parser.add_argument("--identity-tolerance-m", type=float, default=1e-6)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        if args.identity_tolerance_m < 0.0:
            raise EvaluationError("identity tolerance must be non-negative")
        fast_bag, backend_bag = args.fast_bag.resolve(), args.backend_bag.resolve()
        original = read_odometry(fast_bag, args.odom_topic)
        optimized = read_odometry(backend_bag, args.pgo_odom_topic)
        graph_message, graph_storage_stamp = read_last_graph(backend_bag, args.graph_topic)
        report = {
            "schema_version": 1,
            "provenance": {
                "fast_bag": str(fast_bag),
                "backend_bag": str(backend_bag),
                "read_only_inputs": True,
                "graph_snapshot_storage_timestamp_ns": graph_storage_stamp,
            },
            "frame_contract": {"fixed_frame": FIXED_FRAME, "body_frame": BODY_FRAME},
            "streaming_se3_ab": compare_pose_streams(original, optimized),
            "final_native_graph_ab": evaluate_graph(graph_message, args.identity_tolerance_m),
            "interpretation": (
                "Metrics describe correction magnitude and internal consistency only; without ground truth "
                "they do not establish improvement or identify a loop as erroneous."
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
