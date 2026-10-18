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


def test_exact_time_policy_uses_validated_experimental_queue():
    root = Path(__file__).parents[1]
    config = (root / "config" / "simple_loop_closure.yaml").read_text()
    header = (root / "include" / "simple_loop_closure" / "simple_loop_closure_node.hpp").read_text()
    source = (root / "src" / "simple_loop_closure_node.cpp").read_text()

    assert "sync_queue_size: 128" in config
    assert "not a certified optimum" in config
    assert "operational memory-safety cap is 1024" in config
    assert 'declare_parameter<std::int64_t>("sync_queue_size", 50)' in source
    assert 'get_parameter("sync_queue_size", sync_queue_size_)' in source
    assert "validateSyncQueueSize(sync_queue_size_)" in source
    assert source.index("validateSyncQueueSize(sync_queue_size_)") < source.index(
        "SyncPolicy(validated_sync_queue_size)"
    )
    assert "sync_policies::ExactTime" in header
    assert "sync_policies::ApproximateTime" not in header
    assert "setMaxIntervalDuration" not in source


def test_pgo_output_depth_is_bounded_baseline_default_and_only_configures_pgo():
    root = Path(__file__).parents[1]
    config = (root / "config" / "simple_loop_closure.yaml").read_text()
    source = (root / "src" / "simple_loop_closure_node.cpp").read_text()

    assert "pgo_output_depth: 1" in config
    assert 'declare_parameter<std::int64_t>("pgo_output_depth", 1)' in source
    assert 'get_parameter("pgo_output_depth", pgo_output_depth_)' in source
    assert "validatePgoOutputDepth(pgo_output_depth_)" in source
    assert source.index("validatePgoOutputDepth(pgo_output_depth_)") < source.index(
        'create_publisher<nav_msgs::msg::Odometry>(\n      "/pgo_odom", rclcpp::QoS(validated_pgo_output_depth))'
    )
    assert source.count("rclcpp::QoS(validated_pgo_output_depth)") == 1
    assert "pub_pgo_odometry_->get_actual_qos()" in source
    for unchanged_endpoint in (
        'create_publisher<sensor_msgs::msg::PointCloud2>("/pgo_map_cloud", 1)',
        'create_publisher<visualization_msgs::msg::MarkerArray>("/vis_pose_graph", 1)',
        '"/simple_loop_closure/original_keyframes", 1',
        '"/simple_loop_closure/optimized_keyframes", 1',
        '"/simple_loop_closure/loop_edges", 1',
        'sub_odom_.subscribe(this, "/odometry")',
    ):
        assert unchanged_endpoint in source


def test_cloud_input_depth_only_configures_cloud_rmw_receiver_queue():
    root = Path(__file__).parents[1]
    config = (root / "config" / "simple_loop_closure.yaml").read_text()
    header = (root / "include" / "simple_loop_closure" / "simple_loop_closure_node.hpp").read_text()
    source = (root / "src" / "simple_loop_closure_node.cpp").read_text()

    assert "cloud_input_depth: 10" in config
    assert "RMW /cloud receiver queue capacity, before message_filters ExactTime" in config
    assert 'declare_parameter<std::int64_t>("cloud_input_depth", 10)' in source
    assert 'get_parameter("cloud_input_depth", cloud_input_depth_)' in source
    assert "std::int64_t cloud_input_depth_;" in header
    assert "validateCloudInputDepth(cloud_input_depth_)" in source
    assert source.index("validateCloudInputDepth(cloud_input_depth_)") < source.index(
        "rmw_qos_profile_t cloud_input_qos = rmw_qos_profile_default"
    )
    assert "cloud_input_qos.depth = validated_cloud_input_depth" in source
    assert "cloud_input_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST" in source
    assert "cloud_input_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE" in source
    assert source.count('sub_cloud_.subscribe(this, "/cloud", cloud_input_qos)') == 1
    assert source.count("cloud_input_qos") == 5
    assert source.count("validated_cloud_input_depth") == 2
    assert source.count('sub_odom_.subscribe(this, "/odometry")') == 1
    assert "sub_odom_.subscribe(this, \"/odometry\"," not in source
    assert "SyncPolicy(validated_sync_queue_size)" in source
    assert "sub_cloud_.getSubscriber()->get_actual_qos()" in source


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
    publisher = 'create_publisher<nav_msgs::msg::Odometry>(\n      "/pgo_odom", rclcpp::QoS(validated_pgo_output_depth))'
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
