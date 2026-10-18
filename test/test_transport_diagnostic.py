import json
import sqlite3
import sys
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import run_slc_transport_diagnostic as runner  # noqa: E402
import transport_diagnostic_common as diagnostic  # noqa: E402


CLOUD = "/cloud"
ODOM = "/odom"
PGO = "/pgo_odom"


def make_tiny_bag(path: Path) -> Path:
    path.mkdir()
    (path / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")
    database = path / "tiny.db3"
    connection = sqlite3.connect(database)
    connection.executescript(
        "CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT, type TEXT);"
        "CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB);"
    )
    connection.executemany(
        "INSERT INTO topics VALUES (?, ?, ?)",
        [(1, CLOUD, "mock/Cloud"), (2, ODOM, "mock/Odom"), (3, "/ignored", "mock/X")],
    )
    connection.executemany(
        "INSERT INTO messages VALUES (?, ?, ?, ?)",
        [
            (1, 1, 1001, (11).to_bytes(8, "little")),
            (2, 2, 1002, (11).to_bytes(8, "little")),
            (3, 1, 2001, (22).to_bytes(8, "little")),
            (4, 2, 2002, (22).to_bytes(8, "little")),
            (5, 3, 2003, (99).to_bytes(8, "little")),
        ],
    )
    connection.commit()
    connection.close()
    return path


def decode_mock(_type_name: str, payload: bytes) -> int:
    return int.from_bytes(payload, "little")


def test_reads_tiny_sqlite_bag_without_ros(tmp_path):
    bag = make_tiny_bag(tmp_path / "bag")
    records = diagnostic.read_sqlite_messages(bag, (CLOUD, ODOM), decode_mock)
    assert records == {
        CLOUD: [{"storage_ns": 1001, "header_ns": 11}, {"storage_ns": 2001, "header_ns": 22}],
        ODOM: [{"storage_ns": 1002, "header_ns": 11}, {"storage_ns": 2002, "header_ns": 22}],
    }


def test_stamp_comparison_preserves_duplicates():
    result = diagnostic.compare_stamp_sequences(
        [{"header_ns": 1}, {"header_ns": 1}, {"header_ns": 2}],
        [{"header_ns": 1}, {"header_ns": 3}],
    )
    assert result["matched_count"] == 1
    assert result["missing_header_ns"] == [1, 2]
    assert result["extra_header_ns"] == [3]
    assert result["exact_order_match"] is False


def test_report_distinguishes_input_backend_and_recorder_loss():
    source = {
        CLOUD: [{"header_ns": 1}, {"header_ns": 2}],
        ODOM: [{"header_ns": 1}, {"header_ns": 2}],
    }
    witness = [
        {"topic": CLOUD, "header_ns": 1},
        {"topic": CLOUD, "header_ns": 2},
        {"topic": ODOM, "header_ns": 1},
        {"topic": PGO, "header_ns": 1},
    ]
    recorded = {
        CLOUD: [{"header_ns": 1}],
        ODOM: [{"header_ns": 1}],
        PGO: [],
    }
    report = diagnostic.build_loss_report(source, witness, recorded, CLOUD, ODOM, PGO)
    assert report["input_delivery_loss"][CLOUD]["missing_count"] == 0
    assert report["input_delivery_loss"][ODOM]["missing_header_ns"] == [2]
    assert report["backend_output_gap"]["missing_count"] == 0
    assert report["recorder_input_loss"][CLOUD]["missing_header_ns"] == [2]
    assert report["recorder_output_loss"]["missing_header_ns"] == [1]


def test_jsonl_rejects_malformed_record(tmp_path):
    path = tmp_path / "bad.jsonl"
    path.write_text(json.dumps({"header_ns": 1}) + "\nnot-json\n", encoding="utf-8")
    with pytest.raises(diagnostic.DiagnosticError, match="bad.jsonl:2"):
        diagnostic.read_jsonl(path)


@pytest.mark.parametrize("value", ["-1", "191"])
def test_domain_id_rejects_unsafe_values(value):
    with pytest.raises(Exception, match="0..190"):
        runner._domain_id(value)


def test_udp_counter_reader_is_read_only_shape():
    counters = runner.read_udp_counters()
    assert isinstance(counters, dict)
    assert all(isinstance(section, dict) for section in counters.values())


def test_rates_and_udp_delta_are_explicit():
    stats = diagnostic.stream_stats(
        [{"header_ns": 1_000_000_000}, {"header_ns": 1_500_000_000}, {"header_ns": 2_000_000_000}]
    )
    assert stats["header_ns"]["rate_hz"] == pytest.approx(2.0)
    assert runner.udp_counter_delta(
        {"snmp:Udp": {"InErrors": 2}}, {"snmp:Udp": {"InErrors": 5}}
    ) == {"snmp:Udp": {"InErrors": 3}}


def test_run_refuses_non_realtime_rate_before_touching_bag(tmp_path):
    args = runner.parser().parse_args(
        [
            "run", "--bag", str(tmp_path / "missing"),
            "--output-dir", str(tmp_path / "out"),
            "--domain-id", "166", "--subscriber-qos-profile", "sensor-data",
            "--playback-rate", "2.0",
        ]
    )
    with pytest.raises(diagnostic.DiagnosticError, match="rate 1.0"):
        runner.run_diagnostic(args)
    assert not args.output_dir.exists()


def test_run_refuses_existing_output_before_touching_bag(tmp_path):
    output = tmp_path / "existing"
    output.mkdir()
    args = runner.parser().parse_args(
        [
            "run", "--bag", str(tmp_path / "missing"),
            "--output-dir", str(output),
            "--domain-id", "166", "--subscriber-qos-profile", "reliable",
            "--playback-rate", "1.0",
        ]
    )
    with pytest.raises(diagnostic.DiagnosticError, match="refusing to overwrite"):
        runner.run_diagnostic(args)
