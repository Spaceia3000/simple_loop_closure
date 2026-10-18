"""Static contract for Humble RViz2 A/B visualization (no rosbag needed)."""
from pathlib import Path
import yaml


def test_humble_marker_topics_and_cloud_views():
    path = Path(__file__).parents[1] / "rviz" / "simple_loop_closure_ab.rviz"
    rviz = yaml.safe_load(path.read_text(encoding="utf-8"))["Visualization Manager"]
    assert rviz["Global Options"]["Fixed Frame"] == "camera_init"
    displays = rviz["Displays"]
    markers = [item for item in displays if item["Class"] == "rviz_default_plugins/Marker"]
    expected = {
        "/simple_loop_closure/original_keyframes",
        "/simple_loop_closure/optimized_keyframes",
        "/simple_loop_closure/loop_edges",
    }
    assert len(markers) == 3
    assert {m["Topic"]["Value"] for m in markers} == expected
    assert all("Marker Topic" not in m and m["Enabled"] for m in markers)
    assert all(m["Topic"]["Reliability Policy"] == "Reliable" for m in markers)
    clouds = [d for d in displays if d["Class"] == "rviz_default_plugins/PointCloud2"]
    assert len(clouds) == 2
    assert {c["Topic"]["Value"] for c in clouds} == {
        "/pgo_map_cloud", "/cloud_registered_metric"
    }
    assert all(c["Color Transformer"] == "FlatColor" and c["Enabled"] for c in clouds)
    orbit = rviz["Views"]["Current"]
    assert orbit["Class"] == "rviz_default_plugins/Orbit"
    assert orbit["Distance"] >= 100
    assert orbit["Focal Point"]["Y"] < -30
