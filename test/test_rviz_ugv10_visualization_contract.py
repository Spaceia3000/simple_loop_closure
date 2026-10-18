"""Static contract for Humble RViz2 A/B visualization (no rosbag needed)."""
from pathlib import Path
import yaml


def test_humble_global_local_topic_and_display_contract():
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
    assert all("Marker Topic" not in m for m in markers)
    assert all(m["Topic"]["Reliability Policy"] == "Reliable" for m in markers)

    paths = {
        item["Topic"]["Value"]: item
        for item in displays
        if item["Class"] == "rviz_default_plugins/Path"
    }
    assert set(paths) == {"/path"}
    assert paths["/path"]["Color"] == "25; 255; 255"
    assert all(item["Enabled"] for item in paths.values())

    clouds = [d for d in displays if d["Class"] == "rviz_default_plugins/PointCloud2"]
    assert len(clouds) == 4
    pgo = next(c for c in clouds if c["Topic"]["Value"] == "/pgo_map_cloud")
    assert pgo["Color Transformer"] == "FlatColor" and pgo["Enabled"]
    rgb = [c for c in clouds if c["Topic"]["Value"] == "/cloud_registered"]
    assert len(rgb) == 2
    assert all(c["Color Transformer"] == "RGB8" for c in rgb)
    cumulative = next(c for c in rgb if c["Decay Time"] == 10000)
    newest = next(c for c in rgb if c["Decay Time"] == 0)
    assert cumulative["Enabled"] and cumulative["Value"]
    assert not newest["Enabled"] and not newest["Value"]
    metric = next(c for c in clouds if c["Topic"]["Value"] == "/cloud_registered_metric")
    assert not metric["Enabled"] and not metric["Value"]

    images = [d for d in displays if d["Class"] == "rviz_default_plugins/Image"]
    assert len(images) == 1
    assert images[0]["Topic"]["Value"] == "/rgb_img"
    assert images[0]["Enabled"] and images[0]["Topic"]["Reliability Policy"] == "Reliable"

    all_topics = {d["Topic"]["Value"] for d in displays if "Topic" in d}
    assert not any(topic.startswith("/rtabmap") for topic in all_topics)
    orbit = rviz["Views"]["Current"]
    assert orbit["Class"] == "rviz_default_plugins/Orbit"
    assert orbit["Distance"] == 140
    assert orbit["Focal Point"] == {"X": 14, "Y": -41, "Z": 0}
