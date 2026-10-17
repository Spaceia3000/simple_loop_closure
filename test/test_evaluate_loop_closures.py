import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "evaluate_loop_closures.py"
SPEC = importlib.util.spec_from_file_location("evaluate_loop_closures", SCRIPT)
metrics = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = metrics
SPEC.loader.exec_module(metrics)


def point(x, y, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def marker(namespace, marker_type, points):
    return SimpleNamespace(
        ns=namespace,
        type=marker_type,
        action=0,
        header=SimpleNamespace(frame_id="camera_init"),
        pose=SimpleNamespace(
            position=point(0.0, 0.0, 0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        points=[point(*value) for value in points],
    )


def graph_message(loop_points=((2.0, 0.0, 0.0), (0.0, 0.0, 0.0))):
    nodes = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0))
    return SimpleNamespace(
        markers=[
            marker("original_keyframes", 4, nodes),
            marker("optimized_keyframes", 5, (nodes[0], nodes[1], nodes[1], nodes[2])),
            marker("loop_edges", 5, loop_points),
            marker("nodes", 7, nodes),
        ]
    )


def pose(timestamp_ns, x):
    return metrics.PoseSample(timestamp_ns, (x, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def test_native_marker_recovers_unique_undirected_pair_identity():
    graph = metrics.parse_graph_snapshot(graph_message(), 123, 1e-12)

    assert graph.accepted_pairs == ((2, 0),)
    assert graph.timestamp_ns == 123
    assert metrics._topology_summary(graph)["unique_undirected_closure_count"] == 1


def test_duplicate_reverse_pair_is_rejected():
    message = graph_message(
        ((2.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (2.0, 0.0, 0.0))
    )

    with pytest.raises(metrics.EvaluationError, match="duplicate undirected"):
        metrics.parse_graph_snapshot(message, 123, 1e-12)


def test_ambiguous_selected_pair_identity_is_rejected():
    duplicate_nodes = graph_message()
    duplicate_nodes.markers[3].points[1] = point(0.0, 0.0, 0.0)
    duplicate_nodes.markers[1] = marker(
        "optimized_keyframes",
        5,
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
    )

    with pytest.raises(metrics.EvaluationError, match="exactly one"):
        metrics.parse_graph_snapshot(duplicate_nodes, 123, 1e-12)


def test_keyframe_selection_matches_node_or_threshold_rule():
    poses = [pose(1, 0.0), pose(2, 0.29), pose(3, 0.30), pose(4, 0.59), pose(5, 0.61)]

    selected = metrics.select_keyframes(poses, 0.3, 0.2)

    assert [sample.timestamp_ns for sample in selected] == [1, 3, 5]


def test_cloud_partitions_are_deterministic_disjoint_voxel_subsamples():
    grid = np.array(
        [(x * 0.1 + 0.01, y * 0.1 + 0.01, z * 0.1 + 0.01)
         for x in range(8) for y in range(8) for z in range(4)]
    )

    first = metrics.partition_cloud(grid, 0.1, 1000)
    second = metrics.partition_cloud(grid[::-1], 0.1, 1000)

    assert {tuple(value) for value in first.fit}.isdisjoint(
        {tuple(value) for value in first.holdout}
    )
    assert {tuple(value) for value in first.fit} == {tuple(value) for value in second.fit}
    assert {tuple(value) for value in first.holdout} == {tuple(value) for value in second.holdout}


def test_synthetic_positive_registration_scores_disjoint_holdout_high():
    rng = np.random.default_rng(7)
    source = rng.uniform((-3.0, -2.0, -1.0), (4.0, 3.0, 2.0), size=(3000, 3))
    translation = np.array((1.0, -0.4, 0.2))
    target = source + translation
    source_partition = metrics.partition_cloud(source, 0.1, 5000)
    target_partition = metrics.partition_cloud(target, 0.1, 5000)
    initial = np.eye(4)
    initial[:3, 3] = translation

    registration = metrics.register_fit_partition(
        source_partition.fit, target_partition.fit, initial, 0.5, 0.8, 20
    )
    overlap = metrics.symmetric_holdout_overlap(
        source_partition.holdout, target_partition.holdout, registration.transform
    )

    assert registration.correspondences >= 30
    assert overlap["0.2"]["symmetric_mean"] > 0.99
    assert overlap["0.5"]["symmetric_mean"] > 0.99


def test_synthetic_negative_control_scores_holdout_low():
    rng = np.random.default_rng(11)
    source = rng.normal(size=(2500, 3))
    target = rng.normal(size=(2500, 3)) + np.array((20.0, 0.0, 0.0))
    source_partition = metrics.partition_cloud(source, 0.1, 5000)
    target_partition = metrics.partition_cloud(target, 0.1, 5000)

    registration = metrics.register_fit_partition(
        source_partition.fit, target_partition.fit, np.eye(4), 0.5, 0.8, 20
    )
    overlap = metrics.symmetric_holdout_overlap(
        source_partition.holdout, target_partition.holdout, registration.transform
    )

    assert registration.correspondences < 30
    assert overlap["0.5"]["symmetric_mean"] == 0.0


def test_temporal_and_travel_admission_are_separate_strict_gates():
    keyframes = [pose(0, 0.0), pose(31_000_000_000, 20.0), pose(62_000_000_000, 40.0)]
    travel = metrics.cumulative_travel_m(keyframes)

    time_only = metrics.admission_measurements((1, 0), keyframes, travel, 30.0, 30.0)
    both = metrics.admission_measurements((2, 0), keyframes, travel, 30.0, 30.0)

    assert time_only["passes_time_gate"] is True
    assert time_only["passes_travel_gate"] is False
    assert both["passes_time_gate"] is True
    assert both["passes_travel_gate"] is True


def test_percentiles_are_linear_and_empty_is_not_zero():
    assert metrics.percentile([], 0.5) is None
    assert metrics.percentile([0.0, 10.0], 0.95) == pytest.approx(9.5)


def test_cli_defaults_are_frozen_before_artifact_evaluation():
    args = metrics.build_argument_parser().parse_args(
        [
            "--backend-bag",
            "backend",
            "--fast-bag",
            "fast",
            "--dataset",
            "UGV10",
            "--fast-livo2-sha",
            metrics.PINNED_FAST_LIVO2_SHA,
        ]
    )

    assert args.keyframe_distance_m == 0.5
    assert args.keyframe_angle_rad == 0.3
    assert args.min_time_separation_s == 30.0
    assert args.min_travel_separation_m == 30.0
    assert args.voxel_size_m == 0.1
    assert args.max_correspondence_m == 1.0
