from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_operator_topics_are_backed_by_launch_source_and_native_rviz():
    launch = (ROOT / "launch/simple_loop_closure.launch.py").read_text()
    config = (ROOT / "config/simple_loop_closure.yaml").read_text()
    source = (ROOT / "src/simple_loop_closure_node.cpp").read_text()
    rviz = (ROOT / "rviz/simple_loop_closure_ab.rviz").read_text()

    assert 'default_odometry_topic = "/aft_mapped_to_init"' in launch
    assert 'default_cloud_topic = "/cloud_registered_metric"' in launch
    assert "require_exact_timestamps: true" in config
    assert "sync_policies::ExactTime" in (
        ROOT / "include/simple_loop_closure/simple_loop_closure_node.hpp"
    ).read_text()
    for topic in (
        "/pgo_odom",
        "/pgo_map_cloud",
        "/simple_loop_closure/original_keyframes",
        "/simple_loop_closure/optimized_keyframes",
        "/simple_loop_closure/loop_edges",
    ):
        assert topic in source
        assert topic in rviz
    assert "TransformBroadcaster" not in source
