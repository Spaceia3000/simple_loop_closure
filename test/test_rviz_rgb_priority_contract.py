"""Regression: colored GTSAM-corrected 3D RGB map is the default visual layer."""
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


def displays(file):
    cfg = yaml.safe_load((ROOT / "rviz" / file).read_text(encoding="utf-8"))
    vm = cfg["Visualization Manager"]
    assert vm["Global Options"]["Fixed Frame"] == "camera_init"
    return vm["Displays"]


def topic_items(ds, topic):
    return [d for d in ds if isinstance(d.get("Topic"), dict) and d["Topic"]["Value"] == topic]


def test_rgb_priority_and_truthful_pgo_map():
    ds = displays("simple_loop_closure_ab.rviz")
    pgo, = topic_items(ds,"/pgo_map_cloud")
    assert not pgo["Enabled"] and pgo["Color Transformer"] == "FlatColor"
    corrected, = topic_items(ds,"/pgo_rgb_map_cloud")
    assert corrected["Enabled"] and corrected["Color Transformer"] == "RGB8"
    assert pgo["Alpha"] <= 0.2 and pgo["Size (Pixels)"] == 1
    original_rgb = topic_items(ds,"/cloud_registered")
    assert len(original_rgb) == 2
    assert all(not d["Enabled"] and d["Color Transformer"] == "RGB8" for d in original_rgb)
    assert {d["Decay Time"] for d in original_rgb} == {0, 10000}
    assert {d["Size (Pixels)"] for d in original_rgb} == {2,4}
    lidar, = topic_items(ds,"/cloud_registered_metric")
    assert lidar["Enabled"] and lidar["Decay Time"] == 2
    assert lidar["Color Transformer"] == "FlatColor"
    fast, = topic_items(ds,"/path")
    assert not fast["Enabled"] and fast["Line Width"] == 0.03
    orig, = topic_items(ds,"/simple_loop_closure/original_keyframes")
    assert not orig["Enabled"]
    assert topic_items(ds,"/simple_loop_closure/optimized_keyframes")[0]["Enabled"]
    assert topic_items(ds,"/simple_loop_closure/loop_edges")[0]["Enabled"]
    assert topic_items(ds,"/rgb_img")[0]["Enabled"]
    assert not topic_items(ds,"/simple_loop_closure/global_path")


def test_geometry_profile_is_distinct_without_new_ros_topics():
    rgb = displays("simple_loop_closure_ab.rviz")
    geo = displays("simple_loop_closure_global_geometry.rviz")
    pgo_rgb, = topic_items(rgb,"/pgo_map_cloud")
    pgo_geo, = topic_items(geo,"/pgo_map_cloud")
    assert pgo_geo["Alpha"] > pgo_rgb["Alpha"]
    assert pgo_geo["Size (Pixels)"] > pgo_rgb["Size (Pixels)"]
    assert len(topic_items(geo,"/cloud_registered")) == 2
    assert topic_items(geo,"/simple_loop_closure/optimized_keyframes")[0]["Enabled"]
    assert topic_items(geo,"/simple_loop_closure/loop_edges")[0]["Enabled"]
    assert not topic_items(geo,"/simple_loop_closure/global_path")
