from pathlib import Path
import yaml
ROOT = Path(__file__).resolve().parents[1]

def test_bounded_rgb_parameters_and_preserved_c1():
    p = yaml.safe_load((ROOT/"config/offline_lio_livo_c1_candidate.yaml").read_text())["simple_loop_closure"]["ros__parameters"]
    rgb=p["rgb_global"]
    assert rgb["enabled"] is True
    assert 0 < rgb["frame_admission_hz"] <= 20
    assert 0 < rgb["association_max_delta_s"] <= 1.0
    assert 0 < rgb["max_cached_frames"] <= 1200
    assert 0 < rgb["max_points_per_frame"] <= 3000
    assert 0 < rgb["max_output_points"] <= 350000
    assert p["require_exact_timestamps"] is True
    assert p["mapped_cloud"] is True

def test_rviz_has_actual_rgb_corrected_global_and_native_gtsam_marker():
    for filename in ("simple_loop_closure_ab.rviz","simple_loop_closure_global_geometry.rviz"):
        root=yaml.safe_load((ROOT/"rviz"/filename).read_text())
        displays=root["Visualization Manager"]["Displays"]
        rgb=[x for x in displays if x.get("Topic",{}).get("Value")=="/pgo_rgb_map_cloud"]
        assert len(rgb)==1
        assert rgb[0]["Color Transformer"]=="RGB8"
        assert rgb[0]["Class"]=="rviz_default_plugins/PointCloud2"
        assert rgb[0]["Enabled"] is (filename=="simple_loop_closure_ab.rviz")
        assert any(x.get("Topic",{}).get("Value")=="/simple_loop_closure/optimized_keyframes"
                   and x.get("Enabled") for x in displays)
        assert any(x.get("Topic",{}).get("Value")=="/pgo_map_cloud" for x in displays)

def test_default_color_view_does_not_hide_gtsam_path():
    root=yaml.safe_load((ROOT/"rviz/simple_loop_closure_ab.rviz").read_text())
    displays=root["Visualization Manager"]["Displays"]
    names={x["Name"]:x for x in displays}
    assert not names["LOCAL FAST raw odometry (optional; path shown above)"]["Enabled"]
    assert not names["LOCAL FAST path (cyan, uncorrected geometry)"]["Enabled"]
    assert not names["LOCAL FAST accumulated RGB 3D map (not loop-corrected)"]["Enabled"]
    assert names["GLOBAL corrected 3D RGB map (GTSAM keyframe corrections)"]["Enabled"]

def test_rgb_field_is_reconstructed_from_real_rgb_not_xyzi():
    source=(ROOT/"src/simple_loop_closure_node.cpp").read_text()
    assert '"/pgo_rgb_map_cloud"' in source
    assert '"/cloud_registered"' in source
    assert 'pcl::PointXYZRGB' in source
    assert 'rgbWorldCorrection' in source
    assert 'nearestRgbPoseAnchor' in source
