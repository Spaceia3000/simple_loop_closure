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

    assert {"message_filters", "tf2_eigen", "gtsam", "libnanoflann-dev"} <= dependencies


def test_generated_python_and_native_build_artifacts_are_ignored():
    ignore_rules = (Path(__file__).parents[1] / ".gitignore").read_text().splitlines()

    assert ".codex_native/" in ignore_rules
    assert "__pycache__/" in ignore_rules
    assert "*.py[cod]" in ignore_rules
