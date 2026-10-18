import importlib.util
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

os.environ.setdefault("ROS_LOG_DIR", str(Path(tempfile.gettempdir()) / "simple_loop_closure_launch_test_logs"))

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def _load_launch_module():
    launch_path = Path(__file__).parents[1] / "launch" / "simple_loop_closure.launch.py"
    spec = importlib.util.spec_from_file_location("simple_loop_closure_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda _: str(launch_path.parents[1])
    return module


def test_fast_livo2_topics_are_launch_defaults():
    description = _load_launch_module().generate_launch_description()
    arguments = {
        entity.name: entity.default_value[0].text
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }

    assert arguments["odometry_topic"] == "/aft_mapped_to_init"
    assert arguments["cloud_topic"] == "/cloud_registered_metric"


def test_adapter_remaps_only_loop_closure_inputs():
    description = _load_launch_module().generate_launch_description()
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]

    assert len(nodes) == 1
    assert [
        (source[0].text, target[0].variable_name[0].text)
        for source, target in nodes[0]._Node__remappings
    ] == [
        ("/odometry", "odometry_topic"),
        ("/cloud", "cloud_topic"),
    ]


def test_default_parameters_are_fail_closed_for_metric_cloud():
    config = (Path(__file__).parents[1] / "config" / "simple_loop_closure.yaml").read_text()

    assert "mapped_cloud: true" in config
    assert "expected_fixed_frame: camera_init" in config
    assert "expected_body_frame: aft_mapped" in config
    assert "require_exact_timestamps: true" in config


def test_humble_manifest_declares_direct_adapter_dependencies():
    root = ET.parse(Path(__file__).parents[1] / "package.xml").getroot()
    dependencies = {
        element.text
        for tag in ("depend", "build_depend", "build_export_depend", "exec_depend")
        for element in root.findall(tag)
    }

    assert {"message_filters", "tf2_eigen", "gtsam", "libnanoflann-dev", "std_srvs"} <= dependencies


def test_stream_audit_taps_ingress_before_sync_and_is_opt_in():
    config = (Path(__file__).parents[1] / "config" / "simple_loop_closure.yaml").read_text()
    source = (Path(__file__).parents[1] / "src" / "simple_loop_closure_node.cpp").read_text()

    assert "simple_stream_audit: false" in config
    assert source.index("cloud_audit_connection_ =") < source.index("synchronizer_ =")
    assert source.index("odom_audit_connection_ =") < source.index("synchronizer_ =")
    assert '"~/dump_stream_audit"' in source
    assert 'emitStreamAudit("shutdown")' in source


def test_pgo_publish_attempt_return_and_failure_boundaries_are_explicit():
    source = (Path(__file__).parents[1] / "src" / "simple_loop_closure_node.cpp").read_text()
    publisher = 'create_publisher<nav_msgs::msg::Odometry>("/pgo_odom", 1)'
    attempted = "audit.pgo_publish_attempted.observe("
    completed = "audit.pgo_publish_completed.observe("
    failed = "audit.pgo_publish_failed.observe("
    publish_call = "pub_pgo_odometry_->publish(pgo_odom_msg)"

    assert publisher in source
    audited_path = source[source.index("const std::int64_t stamp_ns") :]
    assert audited_path.index(attempted) < audited_path.index(publish_call)
    assert audited_path.index(failed) > audited_path.index(publish_call)
    assert audited_path.index(completed) > audited_path.index(publish_call)
    assert "catch (...)" in audited_path
    assert audited_path.index(failed) < audited_path.index("throw;")


def test_audit_off_path_has_no_stamp_hashing_or_audit_locking():
    source = (Path(__file__).parents[1] / "src" / "simple_loop_closure_node.cpp").read_text()
    function = source[
        source.index("void SimpleLoopClosureNode::publishPoseGraphOptimizedOdometry") :
        source.index("void SimpleLoopClosureNode::pointCloudAndOdometryCallback")
    ]
    off_path = function[function.index("if (!stream_audit_context_)") : function.index("const std::int64_t stamp_ns")]

    assert "pub_pgo_odometry_->publish(pgo_odom_msg);" in off_path
    assert "return;" in off_path
    assert ".observe(" not in off_path
    assert "lock_guard" not in off_path


def test_visualization_map_stride_cannot_wrap_in_signed_arithmetic():
    source = (Path(__file__).parents[1] / "src" / "simple_loop_closure_node.cpp").read_text()

    assert "const std::size_t stride = static_cast<std::size_t>(interval) + 1U;" in source
    assert "i += stride" in source
    assert "i += (interval + 1)" not in source


def test_stream_audit_stage_taps_match_true_processing_boundaries():
    source = (Path(__file__).parents[1] / "src" / "simple_loop_closure_node.cpp").read_text()
    callback = source[source.index("void SimpleLoopClosureNode::pointCloudAndOdometryCallback") :]

    contract_check = callback.index("if (!inputContractValid")
    accepted = callback.index("audit.contract_accepted.observe")
    pgo = callback.index("publishPoseGraphOptimizedOdometry")
    keyframe_return = callback.index("if (!decision.is_keyframe)")
    deserialize = callback.index("pcl::fromROSMsg")
    converted = callback.index("audit.pcl_converted.observe")

    assert contract_check < accepted < pgo < keyframe_return < deserialize < converted


def test_generated_python_and_native_build_artifacts_are_ignored():
    ignore_rules = (Path(__file__).parents[1] / ".gitignore").read_text().splitlines()

    assert ".codex_native/" in ignore_rules
    assert "__pycache__/" in ignore_rules
    assert "*.py[cod]" in ignore_rules


def test_ab_rviz_uses_only_native_graph_and_output_topics():
    config = (Path(__file__).parents[1] / "rviz" / "simple_loop_closure_ab.rviz").read_text()

    assert "Fixed Frame: camera_init" in config
    assert "/simple_loop_closure/original_keyframes" in config
    assert "/simple_loop_closure/optimized_keyframes" in config
    assert "/simple_loop_closure/loop_edges" in config
    assert "/aft_mapped_to_init" in config
    assert "/pgo_odom" in config
    assert "/pgo_map_cloud" in config
    assert "evaluation" not in config
