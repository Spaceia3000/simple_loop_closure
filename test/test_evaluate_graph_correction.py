import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "evaluate_graph_correction.py"
SPEC = importlib.util.spec_from_file_location("evaluate_graph_correction", SCRIPT)
metrics = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = metrics
SPEC.loader.exec_module(metrics)


def pose(stamp, xyz, yaw=0.0):
    transform = np.eye(4)
    transform[:3, :3] = (
        (np.cos(yaw), -np.sin(yaw), 0.0),
        (np.sin(yaw), np.cos(yaw), 0.0),
        (0.0, 0.0, 1.0),
    )
    transform[:3, 3] = xyz
    return metrics.PoseSample(stamp, transform)


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


def graph(original, optimized, loops=()):
    sequential = tuple(
        endpoint
        for index in range(len(optimized) - 1)
        for endpoint in (optimized[index], optimized[index + 1])
    )
    loop_points = tuple(endpoint for pair in loops for endpoint in pair)
    return SimpleNamespace(markers=[
        marker("original_keyframes", 4, original),
        marker("optimized_keyframes", 5, sequential),
        marker("loop_edges", 5, loop_points),
        marker("nodes", 7, optimized),
    ])


def test_identical_rigid_world_correction_preserves_sequential_edges():
    original = {1: pose(1, (0.0, 0.0, 0.0)), 2: pose(2, (1.0, 0.0, 0.0))}
    optimized = {1: pose(1, (0.05, 0.0, 0.0)), 2: pose(2, (1.05, 0.0, 0.0))}

    report = metrics.compare_pose_streams(original, optimized)

    assert report["pose_correction"]["position_displacement"]["p50"] == pytest.approx(0.05)
    assert report["pose_correction"]["translation_scale_bands"]["centimetre_scale_lt_0_10_m"] == 2
    assert report["sequential_edge_consistency"]["translation_residual"]["max"] == pytest.approx(0.0)


def test_sequential_se3_residual_detects_rotation_and_translation_change():
    original = {1: pose(1, (0.0, 0.0, 0.0)), 2: pose(2, (1.0, 0.0, 0.0))}
    optimized = {1: pose(1, (0.0, 0.0, 0.0)), 2: pose(2, (2.0, 0.0, 0.0), 0.2)}

    report = metrics.compare_pose_streams(original, optimized)

    assert report["sequential_edge_consistency"]["translation_residual"]["max"] == pytest.approx(1.0)
    assert report["sequential_edge_consistency"]["rotation_residual"]["max"] == pytest.approx(0.2)


def test_exact_stamp_join_fails_closed():
    with pytest.raises(metrics.EvaluationError, match="lack exact FAST"):
        metrics.compare_pose_streams({1: pose(1, (0, 0, 0))}, {2: pose(2, (0, 0, 0))})


def test_graph_distinguishes_cm_and_five_metre_scale_without_truth_label():
    message = graph(
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
        ((0.05, 0.0, 0.0), (1.05, 0.0, 0.0), (7.0, 0.0, 0.0)),
        ((((7.0, 0.0, 0.0)), ((0.05, 0.0, 0.0))),),
    )

    report = metrics.evaluate_graph(message)

    bands = report["node_translation_scale_bands"]
    assert bands["centimetre_scale_lt_0_10_m"] == 2
    assert bands["five_metre_scale_ge_5"] == 1
    loop = report["accepted_loop_endpoint_chords"]["pairs"][0]
    assert loop["five_metre_scale_correction"] is True
    assert "not loop-factor residuals" in report["accepted_loop_endpoint_chords"]["warning"]


def test_graph_rejects_loop_endpoint_not_identifying_one_node():
    message = graph(
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ((((3.0, 0.0, 0.0)), ((0.0, 0.0, 0.0))),),
    )

    with pytest.raises(metrics.EvaluationError, match="matches 0"):
        metrics.evaluate_graph(message)
