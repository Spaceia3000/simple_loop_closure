#!/usr/bin/env python3
"""Read-only metrics for native SimpleLoopClosure rosbag artifacts.

The final native MarkerArray is treated as the accepted graph.  Loop endpoint
coordinates are resolved back to unique optimized node indices, while
keyframe timestamps and independent endpoint geometry are reconstructed from
the frozen FAST-LIVO2 odometry input.  No trajectory accuracy metric is
reported because this tool has no ground-truth trajectory contract.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping, Sequence


PINNED_FAST_LIVO2_SHA = "52db4d422e00a0e1588bed86d91ac4361b21aca9"
EXPECTED_FIXED_FRAME = "camera_init"
EXPECTED_BODY_FRAME = "aft_mapped"
DEFAULT_GRAPH_TOPIC = "/vis_pose_graph"
DEFAULT_ODOM_TOPIC = "/aft_mapped_to_init"


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


def _distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def _angular_distance(
    a: Sequence[float], b: Sequence[float]
) -> float:
    dot = abs(sum(float(x) * float(y) for x, y in zip(a, b)))
    return 2.0 * math.acos(min(1.0, max(0.0, dot)))


def select_keyframes(
    poses: Sequence[PoseSample], distance_threshold_m: float, angle_threshold_rad: float
) -> list[PoseSample]:
    """Reproduce the node's translation/rotation keyframe admission rule."""
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
    """Validate one native graph snapshot and recover selected pair identities."""
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
        if marker.action != 0:  # visualization_msgs/Marker.ADD
            raise EvaluationError(f"graph marker {marker.ns!r} is not an ADD snapshot")
        marker_translation = _point_tuple(marker.pose.position)
        marker_rotation = marker.pose.orientation
        if _distance(marker_translation, (0.0, 0.0, 0.0)) > identity_tolerance_m or any(
            abs(value) > 1e-12
            for value in (marker_rotation.x, marker_rotation.y, marker_rotation.z)
        ) or abs(abs(float(marker_rotation.w)) - 1.0) > 1e-12:
            raise EvaluationError(f"graph marker {marker.ns!r} pose is not identity")

    original_marker = by_namespace["original_keyframes"]
    optimized_marker = by_namespace["optimized_keyframes"]
    loop_marker = by_namespace["loop_edges"]
    nodes_marker = by_namespace["nodes"]
    if original_marker.type != 4:  # visualization_msgs/Marker.LINE_STRIP
        raise EvaluationError("original_keyframes marker is not LINE_STRIP")
    if optimized_marker.type != 5 or loop_marker.type != 5:  # LINE_LIST
        raise EvaluationError("optimized_keyframes and loop_edges must be LINE_LIST markers")
    if nodes_marker.type != 7:  # SPHERE_LIST
        raise EvaluationError("nodes marker is not SPHERE_LIST")

    original_nodes = tuple(_point_tuple(point) for point in original_marker.points)
    optimized_nodes = tuple(_point_tuple(point) for point in nodes_marker.points)
    if len(original_nodes) != len(optimized_nodes):
        raise EvaluationError("original and optimized graph node counts differ")
    expected_odom_points = max(0, 2 * (len(optimized_nodes) - 1))
    if len(optimized_marker.points) != expected_odom_points:
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
    undirected_pairs = {tuple(sorted(pair)) for pair in accepted_pairs}
    if len(undirected_pairs) != len(accepted_pairs):
        raise EvaluationError("native loop marker contains a duplicate pair")

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
        quat = (
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
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


def read_oracle(path: Path) -> dict[tuple[int, int], bool]:
    """Read explicit adjudications keyed by unordered keyframe timestamp pair."""
    result: dict[tuple[int, int], bool] = {}
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"source_timestamp_ns", "target_timestamp_ns", "is_true_loop"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise EvaluationError(f"oracle CSV must contain columns {sorted(required)}")
        for line_number, row in enumerate(reader, start=2):
            try:
                first = int(row["source_timestamp_ns"])
                second = int(row["target_timestamp_ns"])
            except (TypeError, ValueError) as exc:
                raise EvaluationError(f"invalid oracle timestamp at line {line_number}") from exc
            label_text = row["is_true_loop"].strip().lower()
            if label_text not in {"0", "1", "false", "true"}:
                raise EvaluationError(f"invalid oracle label at line {line_number}")
            key = tuple(sorted((first, second)))
            if first == second or key in result:
                raise EvaluationError(f"invalid or duplicate oracle pair at line {line_number}")
            result[key] = label_text in {"1", "true"}
    return result


def evaluate(
    graph: GraphSnapshot,
    keyframes: Sequence[PoseSample],
    oracle: Mapping[tuple[int, int], bool] | None,
) -> dict[str, object]:
    distances: list[float] = []
    time_separations: list[float] = []
    closures: list[dict[str, object]] = []
    tp = 0
    fp = 0
    missing_oracle: list[tuple[int, int]] = []
    for source_id, target_id in graph.accepted_pairs:
        if source_id >= len(keyframes) or target_id >= len(keyframes):
            raise EvaluationError("accepted pair references a missing reconstructed keyframe")
        source = keyframes[source_id]
        target = keyframes[target_id]
        distance_m = _distance(source.xyz, target.xyz)
        separation_s = abs(source.timestamp_ns - target.timestamp_ns) / 1e9
        timestamp_pair = tuple(sorted((source.timestamp_ns, target.timestamp_ns)))
        label = None if oracle is None else oracle.get(timestamp_pair)
        if oracle is not None and label is None:
            missing_oracle.append(timestamp_pair)
        elif label is True:
            tp += 1
        elif label is False:
            fp += 1
        distances.append(distance_m)
        time_separations.append(separation_s)
        closures.append(
            {
                "source_id": source_id,
                "target_id": target_id,
                "source_timestamp_ns": source.timestamp_ns,
                "target_timestamp_ns": target.timestamp_ns,
                "fast_input_endpoint_distance_m": distance_m,
                "time_separation_s": separation_s,
                "oracle_true_loop": label,
            }
        )
    if missing_oracle:
        raise EvaluationError(
            "oracle does not adjudicate every accepted closure timestamp pair: "
            + ", ".join(str(pair) for pair in missing_oracle)
        )

    classification: dict[str, object]
    if oracle is None:
        classification = {
            "status": "not_available_no_oracle",
            "true_positives": None,
            "false_positives": None,
        }
    else:
        classification = {
            "status": "available_for_accepted_pairs",
            "true_positives": tp,
            "false_positives": fp,
        }
    return {
        "graph": {
            "snapshot_storage_timestamp_ns": graph.timestamp_ns,
            "frame_id": graph.frame_id,
            "keyframe_count": len(keyframes),
            "odometry_edge_count": max(0, len(keyframes) - 1),
            "accepted_loop_count": len(graph.accepted_pairs),
            "keyframe_timestamps_ns": [pose.timestamp_ns for pose in keyframes],
        },
        "accepted_closures": closures,
        "independent_fast_input_geometry": {
            "definition": "3D Euclidean separation of accepted endpoints in frozen FAST input odometry",
            "accuracy_claim": False,
            "p50_m": percentile(distances, 0.50),
            "p95_m": percentile(distances, 0.95),
            "time_separation_p50_s": percentile(time_separations, 0.50),
            "time_separation_p95_s": percentile(time_separations, 0.95),
        },
        "classification": classification,
        "trajectory_accuracy": {
            "ape": None,
            "rpe": None,
            "status": "not_computed_no_ground_truth_contract",
        },
    }


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend-bag", type=Path, required=True)
    parser.add_argument("--fast-bag", type=Path, required=True)
    parser.add_argument("--dataset", choices=("UGV10", "Retail", "CBD03"), required=True)
    parser.add_argument("--fast-livo2-sha", required=True)
    parser.add_argument("--oracle", type=Path)
    parser.add_argument("--output", type=Path, help="write JSON here instead of stdout")
    parser.add_argument("--graph-topic", default=DEFAULT_GRAPH_TOPIC)
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    parser.add_argument("--keyframe-distance-m", type=float, default=0.5)
    parser.add_argument("--keyframe-angle-rad", type=float, default=0.3)
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
        if args.identity_tolerance_m < 0.0 or args.original_geometry_tolerance_m < 0.0:
            raise EvaluationError("geometry tolerances must be non-negative")
        graph = read_last_graph(args.backend_bag.resolve(), args.graph_topic, args.identity_tolerance_m)
        fast_poses = read_fast_odometry(args.fast_bag.resolve(), args.odom_topic)
        reconstructed = select_keyframes(
            fast_poses, args.keyframe_distance_m, args.keyframe_angle_rad
        )
        keyframes = validate_keyframes(graph, reconstructed, args.original_geometry_tolerance_m)
        oracle = read_oracle(args.oracle.resolve()) if args.oracle else None
        report = {
            "schema_version": 1,
            "dataset": args.dataset,
            "provenance": {
                "fast_livo2_sha": args.fast_livo2_sha,
                "fast_bag": str(args.fast_bag.resolve()),
                "backend_bag": str(args.backend_bag.resolve()),
                "read_only": True,
            },
            **evaluate(graph, keyframes, oracle),
        }
        rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
        if args.output:
            args.output.write_text(rendered, encoding="utf-8")
        else:
            sys.stdout.write(rendered)
        return 0
    except (EvaluationError, OSError) as exc:
        print(f"evaluation failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
