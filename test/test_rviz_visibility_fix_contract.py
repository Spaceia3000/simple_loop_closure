"""Contract for the UGV10 visualization-only visibility changes."""

from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]


def _rviz_displays():
    document = yaml.safe_load(
        (ROOT / "rviz" / "simple_loop_closure_ab.rviz").read_text(encoding="utf-8")
    )
    return document["Visualization Manager"]["Displays"]


def test_graph_marker_width_parameters_and_candidate_values_are_separate():
    default = yaml.safe_load(
        (ROOT / "config" / "simple_loop_closure.yaml").read_text(encoding="utf-8")
    )["simple_loop_closure"]["ros__parameters"]
    candidate = yaml.safe_load(
        (ROOT / "config" / "offline_lio_livo_c1_candidate.yaml").read_text(
            encoding="utf-8"
        )
    )["simple_loop_closure"]["ros__parameters"]

    assert default["visualization"] == {
        "optimized_edge_width_m": 0.10,
        "original_edge_width_m": 0.10,
        "loop_edge_width_m": 0.10,
    }
    assert candidate["visualization"] == {
        "optimized_edge_width_m": 0.03,
        "original_edge_width_m": 0.03,
        "loop_edge_width_m": 0.008,
    }
    assert default["vis_map_cloud_frame_interval"] == 3
    assert default["vis_map_voxel_leaf_size"] == 0.8
    assert candidate["vis_map_cloud_frame_interval"] == 1
    assert candidate["vis_map_voxel_leaf_size"] == 0.4

    source = (ROOT / "src" / "simple_loop_closure_node.cpp").read_text(encoding="utf-8")
    for member in (
        "optimized_edge_width_m_",
        "original_edge_width_m_",
        "loop_edge_width_m_",
    ):
        assert f"marker_msg.scale.x = {member};" in source
    assert "edge_scale_" not in source


def test_global_and_local_displays_have_distinct_topics_and_styles():
    displays = _rviz_displays()
    by_topic = {
        display["Topic"]["Value"]: display
        for display in displays
        if "Topic" in display and display["Topic"]["Value"] != "/cloud_registered"
    }

    path = by_topic["/path"]
    assert path["Enabled"] and path["Color"] == "0; 255; 255"
    assert path["Line Width"] == 0.03

    pgo_pose = by_topic["/pgo_odom"]
    assert pgo_pose["Enabled"] and pgo_pose["Keep"] == 1
    assert pgo_pose["Shape"]["Shaft Length"] == 0.20
    assert pgo_pose["Shape"]["Shaft Radius"] == 0.008
    assert pgo_pose["Shape"]["Head Length"] == 0.05
    assert pgo_pose["Shape"]["Head Radius"] == 0.012

    for topic in (
        "/simple_loop_closure/original_keyframes",
        "/simple_loop_closure/optimized_keyframes",
        "/simple_loop_closure/loop_edges",
        "/pgo_map_cloud",
        "/cloud_registered_metric",
        "/rgb_img",
    ):
        assert by_topic[topic]["Enabled"] and by_topic[topic]["Value"] if topic != "/simple_loop_closure/original_keyframes" else not by_topic[topic]["Enabled"]

    metric = by_topic["/cloud_registered_metric"]
    assert metric["Color Transformer"] == "FlatColor"
    assert metric["Decay Time"] == 2
    assert metric["Size (Pixels)"] == 2

    topics = {display["Topic"]["Value"] for display in displays if "Topic" in display}
    assert "/simple_loop_closure/global_path" not in topics
    assert not any(topic.startswith("/rtabmap") for topic in topics)
