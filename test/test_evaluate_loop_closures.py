import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

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


def test_native_marker_recovers_selected_pair_identities():
    graph = metrics.parse_graph_snapshot(graph_message(), 123, 1e-12)

    assert graph.accepted_pairs == ((2, 0),)
    assert graph.timestamp_ns == 123


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


def test_geometry_and_oracle_metrics_use_fast_input_keyframes():
    graph = metrics.parse_graph_snapshot(graph_message(), 123, 1e-12)
    keyframes = [pose(100, 0.0), pose(200, 1.0), pose(400, 2.0)]

    report = metrics.evaluate(graph, keyframes, {(100, 400): False})

    assert report["graph"]["accepted_loop_count"] == 1
    assert report["graph"]["keyframe_timestamps_ns"] == [100, 200, 400]
    assert report["accepted_closures"][0]["source_id"] == 2
    assert report["accepted_closures"][0]["target_timestamp_ns"] == 100
    assert report["independent_fast_input_geometry"]["p50_m"] == 2.0
    assert report["classification"]["true_positives"] == 0
    assert report["classification"]["false_positives"] == 1
    assert report["trajectory_accuracy"]["ape"] is None
    assert report["trajectory_accuracy"]["rpe"] is None


def test_missing_oracle_adjudication_fails_closed():
    graph = metrics.parse_graph_snapshot(graph_message(), 123, 1e-12)

    with pytest.raises(metrics.EvaluationError, match="does not adjudicate every"):
        metrics.evaluate(graph, [pose(100, 0.0), pose(200, 1.0), pose(400, 2.0)], {})


def test_percentiles_are_linear_and_empty_is_not_zero():
    assert metrics.percentile([], 0.5) is None
    assert metrics.percentile([0.0, 10.0], 0.95) == pytest.approx(9.5)


def test_cli_defaults_match_deployed_keyframe_thresholds():
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
