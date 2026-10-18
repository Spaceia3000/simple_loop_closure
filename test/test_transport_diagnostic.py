import json
import signal
import sqlite3
import subprocess
import sys
from io import BytesIO
from pathlib import Path
from unittest.mock import Mock, call

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import run_slc_transport_diagnostic as runner  # noqa: E402
import transport_diagnostic_common as diagnostic  # noqa: E402


CLOUD = "/cloud"
ODOM = "/odom"
PGO = "/pgo_odom"


def passing_backend_audits(count=2):
    return (
        "[node] SLC_DETERMINISM_AUDIT status=PASS produced=3 copied=3 added=3 "
        "searched=1 expected_searched=1 accepted=0 graph_nodes=3 graph_factors=3\n"
        "[node] SC_SIMPLE_STREAM_AUDIT version=7 report=shutdown status=PASS "
        f"cloud_ingress_count={count} odom_ingress_count={count} "
        f"sync_callback_count={count} contract_accepted_count={count} "
        "contract_rejected_count=0 "
        f"pgo_publish_attempted_count={count} pgo_publish_completed_count={count} "
        "pgo_publish_failed_count=0 sync_accounting_ok=true "
        "contract_accepted_to_pgo_attempted_ok=true "
        "pgo_publish_outcome_accounting_ok=true pgo_publish_no_failures=true "
        "pgo_publish_attempted_to_completed_count_ok=true "
        "pgo_publish_attempted_to_completed_ordered_digest_ok=true\n"
    )


def verified_snapshot_fields(stamps=(1, 2), count=None):
    expected = runner.expected_verified_drain_fields(
        {
            CLOUD: [{"header_ns": stamp} for stamp in stamps],
            ODOM: [{"header_ns": stamp} for stamp in stamps],
        },
        CLOUD,
        ODOM,
    )
    observed_count = len(stamps) if count is None else count
    fields = {
        "version": "8", "report": "snapshot", "status": "PASS",
        "contract_rejected_count": "0", "pgo_publish_failed_count": "0",
        "sync_callback_mismatches": "0", "contract_accepted_mismatches": "0",
        "sync_accounting_ok": "true",
        "contract_accepted_to_pgo_attempted_ok": "true",
        "contract_accepted_to_pgo_attempted_count_ok": "true",
        "contract_accepted_to_pgo_attempted_ordered_digest_ok": "true",
        "pgo_publish_outcome_accounting_ok": "true",
        "pgo_publish_no_failures": "true",
        "pgo_publish_attempted_to_completed_count_ok": "true",
        "pgo_publish_attempted_to_completed_ordered_digest_ok": "true",
    }
    for key in (
        "cloud_ingress_count", "odom_ingress_count", "sync_callback_count",
        "sync_callback_cloud_count", "sync_callback_odom_count",
        "contract_accepted_count", "contract_accepted_cloud_count",
        "contract_accepted_odom_count", "pgo_publish_attempted_count",
        "pgo_publish_completed_count",
    ):
        fields[key] = str(observed_count)
    fields.update({
        "cloud_ingress_fnv64": expected["cloud_fnv64"],
        "odom_ingress_fnv64": expected["odom_fnv64"],
        "sync_callback_pair_fnv64": expected["pair_fnv64"],
        "sync_callback_cloud_fnv64": expected["cloud_fnv64"],
        "sync_callback_odom_fnv64": expected["odom_fnv64"],
        "contract_accepted_pair_fnv64": expected["pair_fnv64"],
        "contract_accepted_cloud_fnv64": expected["cloud_fnv64"],
        "contract_accepted_odom_fnv64": expected["odom_fnv64"],
        "pgo_publish_attempted_fnv64": expected["odom_fnv64"],
        "pgo_publish_completed_fnv64": expected["odom_fnv64"],
    })
    for name in (
        "cloud_ingress", "odom_ingress", "sync_callback_cloud",
        "sync_callback_odom", "contract_accepted_cloud",
        "contract_accepted_odom", "pgo_publish_attempted",
        "pgo_publish_completed",
    ):
        fields[f"{name}_adjacent_duplicates"] = str(expected["adjacent_duplicates"])
        fields[f"{name}_reversals"] = str(expected["reversals"])
        fields[f"{name}_first_ns"] = str(expected["first_ns"])
        fields[f"{name}_last_ns"] = str(expected["last_ns"])
    return fields


def stream_audit_line(fields):
    return "SC_SIMPLE_STREAM_AUDIT " + " ".join(
        f"{key}={value}" for key, value in fields.items()
    )


class SuccessfulFakeProcess:
    next_pid = 700

    def __init__(self):
        self.args = ["mock"]
        self.pid = SuccessfulFakeProcess.next_pid
        SuccessfulFakeProcess.next_pid += 1
        self.returncode = None

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        self.returncode = 0
        return 0

    def send_signal(self, _sent_signal):
        pass

    def terminate(self):
        pass

    def kill(self):
        pass


def patch_successful_fake_run(monkeypatch, fake_audit):
    source = {CLOUD: [{"header_ns": 1}], ODOM: [{"header_ns": 1}]}
    monkeypatch.setattr(runner, "inspect_bag", lambda *_args: source)
    monkeypatch.setattr(runner, "bag_content_duration_seconds", lambda _bag: 1.0)
    monkeypatch.setattr(runner, "read_udp_counters", lambda: {})
    monkeypatch.setattr(runner, "snapshot_endpoints", lambda *_args: None)
    monkeypatch.setattr(
        runner, "start_logged", lambda *_args: (SuccessfulFakeProcess(), BytesIO())
    )
    monkeypatch.setattr(runner.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(runner.time, "monotonic", Mock(side_effect=[10.0, 10.0, 11.0]))
    monkeypatch.setattr(runner, "audit_artifacts", fake_audit)


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


def test_reads_standard_rosbag_metadata_duration(tmp_path):
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n"
        "  duration:\n"
        "    nanoseconds: 265900000000\n",
        encoding="utf-8",
    )
    assert runner.bag_content_duration_seconds(bag) == pytest.approx(265.9)


def test_reads_verified_rosbag_metadata_counts_per_topic(tmp_path):
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n"
        "  topics_with_message_count:\n"
        "    - topic_metadata: {name: /cloud}\n"
        "      message_count: 2377\n"
        "    - topic_metadata: {name: /odom}\n"
        "      message_count: 2377\n",
        encoding="utf-8",
    )

    assert runner.bag_metadata_topic_counts(bag, (CLOUD, ODOM)) == {
        CLOUD: 2377,
        ODOM: 2377,
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


def test_external_gate_rejects_reordered_duplicate_multiset():
    exact = diagnostic.compare_stamp_sequences(
        [{"header_ns": 1}, {"header_ns": 1}, {"header_ns": 2}],
        [{"header_ns": 1}, {"header_ns": 1}, {"header_ns": 2}],
    )
    reordered = diagnostic.compare_stamp_sequences(
        [{"header_ns": 1}, {"header_ns": 1}, {"header_ns": 2}],
        [{"header_ns": 1}, {"header_ns": 2}, {"header_ns": 1}],
    )

    verdict = diagnostic.build_external_stamp_integrity(
        {CLOUD: reordered, ODOM: exact}, {CLOUD: exact, ODOM: exact}, exact
    )

    assert reordered["same_multiset"] is True
    assert verdict["status"] == "FAIL"
    assert verdict["discrepancies"] == [
        {
            "stage": "source_to_witness",
            "topic": CLOUD,
            "missing_count": 0,
            "extra_count": 0,
            "same_multiset": True,
            "exact_order_match": False,
        }
    ]


def _strict_report(source_cloud, source_odom, pgo_witness, pgo_recorded):
    source = {
        CLOUD: [{"header_ns": stamp} for stamp in source_cloud],
        ODOM: [{"header_ns": stamp} for stamp in source_odom],
    }
    witness = [
        *({"topic": CLOUD, "header_ns": stamp} for stamp in source_cloud),
        *({"topic": ODOM, "header_ns": stamp} for stamp in source_odom),
        *({"topic": PGO, "header_ns": stamp} for stamp in pgo_witness),
    ]
    recorded = {
        CLOUD: list(source[CLOUD]),
        ODOM: list(source[ODOM]),
        PGO: [{"header_ns": stamp} for stamp in pgo_recorded],
    }
    return diagnostic.build_loss_report(
        source,
        witness,
        recorded,
        CLOUD,
        ODOM,
        PGO,
        certify_full_source=True,
    )


@pytest.mark.parametrize("frame_count", [2660, 2377, 1351])
def test_certify_all_accepts_complete_bags_with_per_bag_derived_count(frame_count):
    stamps = list(range(frame_count))

    report = _strict_report(stamps, stamps, stamps, stamps)

    integrity = report["external_stamp_integrity"]
    assert integrity["status"] == "PASS"
    assert integrity["full_source_contract"]["derived_source_count"] == frame_count
    assert integrity["full_source_contract"]["expected_source_count"] is None


def test_certify_all_rejects_same_source_frame_missing_from_both_pgo_readers():
    report = _strict_report([1, 2, 3], [1, 2, 3], [1, 3], [1, 3])

    assert report["recorder_output_loss"]["exact_order_match"] is True
    assert report["external_stamp_integrity"]["status"] == "FAIL"
    strict = report["external_stamp_integrity"]["stages"]
    assert strict["source_to_pgo_witness"][PGO]["missing_header_ns"] == [2]
    assert strict["source_to_pgo_recorder"][PGO]["missing_header_ns"] == [2]


def test_certify_all_rejects_equal_count_replaced_pgo_stamp():
    report = _strict_report([1, 2, 3], [1, 2, 3], [1, 4, 3], [1, 4, 3])

    assert report["recorder_output_loss"]["exact_order_match"] is True
    assert report["external_stamp_integrity"]["status"] == "FAIL"
    comparison = report["external_stamp_integrity"]["stages"][
        "source_to_pgo_witness"
    ][PGO]
    assert comparison["expected_count"] == comparison["observed_count"] == 3
    assert comparison["missing_header_ns"] == [2]
    assert comparison["extra_header_ns"] == [4]


def test_certify_all_rejects_source_cloud_odometry_order_mismatch():
    report = _strict_report([1, 2, 3], [1, 3, 2], [1, 2, 3], [1, 2, 3])

    assert report["external_stamp_integrity"]["status"] == "FAIL"
    comparison = report["external_stamp_integrity"]["stages"][
        "source_cloud_to_odometry"
    ][ODOM]
    assert comparison["same_multiset"] is True
    assert comparison["exact_order_match"] is False


def test_certify_all_rejects_duplicate_source_stamp_even_when_all_streams_match():
    report = _strict_report([1, 1, 2], [1, 1, 2], [1, 1, 2], [1, 1, 2])

    assert report["external_stamp_integrity"]["status"] == "FAIL"
    assert report["external_stamp_integrity"]["discrepancies"][-1] == {
        "stage": "source_inventory_uniqueness",
        "topic": ODOM,
        "observed_count": 3,
        "unique_count": 2,
    }


def test_external_gate_returns_fail_for_malformed_comparison_without_assertions():
    exact = diagnostic.compare_stamp_sequences(
        [{"header_ns": 1}], [{"header_ns": 1}]
    )
    malformed = dict(exact)
    del malformed["same_multiset"]

    verdict = diagnostic.build_external_stamp_integrity(
        {CLOUD: malformed, ODOM: exact}, {CLOUD: exact, ODOM: exact}, exact
    )

    assert verdict["status"] == "FAIL"
    assert verdict["discrepancies"] == [
        {
            "stage": "source_to_witness",
            "topic": CLOUD,
            "validation_error": "same_multiset is not a boolean",
        }
    ]


def test_certify_all_optional_expected_source_count_fails_closed_on_mismatch():
    exact = diagnostic.compare_stamp_sequences(
        [{"header_ns": 1}], [{"header_ns": 1}]
    )
    verdict = diagnostic.build_external_stamp_integrity(
        {CLOUD: exact, ODOM: exact},
        {CLOUD: exact, ODOM: exact},
        exact,
        full_source_comparisons={
            "source_cloud_to_odometry": {ODOM: exact},
            "source_to_pgo_witness": {PGO: exact},
            "source_to_pgo_recorder": {PGO: exact},
        },
        source_topic_counts={CLOUD: 2659, ODOM: 2659},
        source_unique_topic_counts={CLOUD: 2659, ODOM: 2659},
        expected_source_count=2660,
    )

    assert verdict["status"] == "FAIL"
    assert verdict["discrepancies"][-2:] == [
        {
        "stage": "expected_source_count",
        "topic": CLOUD,
        "expected_count": 2660,
        "observed_count": 2659,
        },
        {
        "stage": "expected_source_count",
        "topic": ODOM,
        "expected_count": 2660,
        "observed_count": 2659,
        },
    ]


def test_certify_all_rejects_empty_source_inventory():
    report = _strict_report([], [], [], [])

    assert report["external_stamp_integrity"]["status"] == "FAIL"
    discrepancies = report["external_stamp_integrity"]["discrepancies"]
    assert {item["topic"] for item in discrepancies} == {CLOUD, ODOM}
    assert {item["stage"] for item in discrepancies} == {
        "source_inventory_nonempty"
    }


def test_certify_all_checks_verified_metadata_counts_per_topic():
    source = {
        CLOUD: [{"header_ns": 1}, {"header_ns": 2}],
        ODOM: [{"header_ns": 1}, {"header_ns": 2}],
    }
    witness = [
        *({"topic": CLOUD, "header_ns": stamp} for stamp in (1, 2)),
        *({"topic": ODOM, "header_ns": stamp} for stamp in (1, 2)),
        *({"topic": PGO, "header_ns": stamp} for stamp in (1, 2)),
    ]
    recorded = {**source, PGO: [{"header_ns": 1}, {"header_ns": 2}]}

    report = diagnostic.build_loss_report(
        source,
        witness,
        recorded,
        CLOUD,
        ODOM,
        PGO,
        certify_full_source=True,
        source_metadata_topic_counts={CLOUD: 2, ODOM: 3},
    )

    assert report["external_stamp_integrity"]["status"] == "FAIL"
    assert report["external_stamp_integrity"]["discrepancies"][-1] == {
        "stage": "source_metadata_count",
        "topic": ODOM,
        "metadata_count": 3,
        "sqlite_count": 2,
    }


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
    assert report["source_to_recorder"][CLOUD]["missing_header_ns"] == [2]
    assert report["external_stamp_integrity"]["status"] == "FAIL"


def test_source_to_recorder_is_independent_of_witness_snapshot():
    source = {CLOUD: [{"header_ns": 1}, {"header_ns": 2}], ODOM: [{"header_ns": 1}]}
    witness = [
        {"topic": CLOUD, "header_ns": 1},
        {"topic": ODOM, "header_ns": 1},
        {"topic": PGO, "header_ns": 1},
    ]
    recorded = {
        CLOUD: [{"header_ns": 1}, {"header_ns": 2}],
        ODOM: [{"header_ns": 1}],
        PGO: [{"header_ns": 1}],
    }

    report = diagnostic.build_loss_report(source, witness, recorded, CLOUD, ODOM, PGO)

    assert report["input_delivery_loss"][CLOUD]["missing_header_ns"] == [2]
    assert report["recorder_input_loss"][CLOUD]["extra_header_ns"] == [2]
    assert report["source_to_recorder"][CLOUD]["exact_order_match"] is True
    discrepancy_stages = {
        (item["stage"], item["topic"])
        for item in report["external_stamp_integrity"]["discrepancies"]
    }
    assert discrepancy_stages == {("source_to_witness", CLOUD)}


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


@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf"])
def test_playback_timeout_requires_finite_positive_value(value):
    with pytest.raises(SystemExit):
        runner.parser().parse_args(
            [
                "run", "--bag", "bag", "--output-dir", "out", "--domain-id", "166",
                "--subscriber-qos-profile", "reliable", "--playback-timeout-sec", value,
            ]
        )


def test_playback_timeout_is_opt_in():
    args = runner.parser().parse_args(
        [
            "run", "--bag", "bag", "--output-dir", "out", "--domain-id", "166",
            "--subscriber-qos-profile", "reliable",
        ]
    )
    assert args.playback_timeout_sec is None


@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf"])
def test_verified_drain_timeout_requires_finite_positive_value(value):
    with pytest.raises(SystemExit):
        runner.parser().parse_args(
            [
                "run", "--bag", "bag", "--output-dir", "out", "--domain-id", "166",
                "--subscriber-qos-profile", "reliable", "--verified-drain",
                "--verified-drain-timeout-sec", value,
            ]
        )


def test_verified_drain_requires_explicit_timeout_before_touching_bag(tmp_path, monkeypatch):
    args = runner.parser().parse_args(
        [
            "run", "--bag", "missing", "--output-dir", str(tmp_path / "out"),
            "--domain-id", "166", "--subscriber-qos-profile", "reliable",
            "--verified-drain",
        ]
    )
    inspect = Mock(side_effect=AssertionError("bag must not be inspected"))
    monkeypatch.setattr(runner, "inspect_bag", inspect)

    with pytest.raises(diagnostic.DiagnosticError, match="requires explicit"):
        runner.run_diagnostic(args)

    inspect.assert_not_called()


def test_stream_audit_trigger_output_parser_accepts_ros2_cli_wrapper():
    fields = verified_snapshot_fields()
    output = (
        "waiting for service to become available...\n"
        "response:\nstd_srvs.srv.Trigger_Response(success=True, message='"
        + stream_audit_line(fields)
        + "')\n"
    )

    assert runner.parse_stream_audit_snapshot(output) == fields


@pytest.mark.parametrize(
    "output",
    [
        "response without marker",
        "SC_SIMPLE_STREAM_AUDIT report=snapshot broken-token",
        "SC_SIMPLE_STREAM_AUDIT report=snapshot status=PASS\n"
        "SC_SIMPLE_STREAM_AUDIT report=snapshot status=PASS",
    ],
)
def test_stream_audit_trigger_output_parser_fails_closed(output):
    with pytest.raises(diagnostic.DiagnosticError):
        runner.parse_stream_audit_snapshot(output)


def test_verified_snapshot_requires_exact_counts_digests_and_pair_cardinality():
    expected = runner.expected_verified_drain_fields(
        {CLOUD: [{"header_ns": 1}, {"header_ns": 2}],
         ODOM: [{"header_ns": 1}, {"header_ns": 2}]},
        CLOUD,
        ODOM,
    )
    complete = verified_snapshot_fields()
    assert runner.evaluate_verified_drain_snapshot(complete, expected) == "complete"

    pending = dict(complete)
    for key in tuple(pending):
        if key.endswith("_count") and key not in (
            "contract_rejected_count", "pgo_publish_failed_count"
        ):
            pending[key] = "1"
    assert runner.evaluate_verified_drain_snapshot(pending, expected) == "pending"

    bad_digest = dict(complete, sync_callback_pair_fnv64="0x0000000000000000")
    with pytest.raises(diagnostic.DiagnosticError, match="pair_fnv64"):
        runner.evaluate_verified_drain_snapshot(bad_digest, expected)

    unexpected = dict(complete, cloud_ingress_count="3")
    with pytest.raises(diagnostic.DiagnosticError, match="irreversible count divergence"):
        runner.evaluate_verified_drain_snapshot(unexpected, expected)


def test_verified_drain_completes_only_after_two_identical_snapshots(monkeypatch):
    source = {
        CLOUD: [{"header_ns": 1}, {"header_ns": 2}],
        ODOM: [{"header_ns": 1}, {"header_ns": 2}],
    }
    pending = verified_snapshot_fields(count=1)
    complete = verified_snapshot_fields()
    snapshots = iter((pending, complete, complete))
    request = Mock(side_effect=lambda *_args: next(snapshots))
    process = Mock()
    process.poll.return_value = None
    monkeypatch.setattr(runner.time, "monotonic", Mock(return_value=10.0))
    monkeypatch.setattr(runner.time, "sleep", lambda _seconds: None)

    result = runner.wait_for_verified_drain(
        source, CLOUD, ODOM, "/audit", {}, 5.0,
        [("backend", process, object())], snapshot_request=request,
    )

    assert result["status"] == "PASS"
    assert result["attempts"] == 3
    assert result["stable_snapshot_count"] == 2


def test_verified_drain_times_out_fail_closed_when_service_is_unavailable(monkeypatch):
    source = {CLOUD: [{"header_ns": 1}], ODOM: [{"header_ns": 1}]}
    clock = Mock(side_effect=[0.0, 0.0, 0.2, 0.2, 0.4, 0.4, 0.6])
    request = Mock(
        side_effect=runner.StreamAuditServiceUnavailableError(
            "service call unavailable"
        )
    )
    process = Mock()
    process.poll.return_value = None
    monkeypatch.setattr(runner.time, "monotonic", clock)
    monkeypatch.setattr(runner.time, "sleep", lambda _seconds: None)

    with pytest.raises(runner.VerifiedDrainTimeoutError, match="service call unavailable"):
        runner.wait_for_verified_drain(
            source, CLOUD, ODOM, "/audit", {}, 0.5,
            [("backend", process, object())], snapshot_request=request,
        )


def test_verified_drain_times_out_on_incomplete_native_counts(monkeypatch):
    source = {
        CLOUD: [{"header_ns": 1}, {"header_ns": 2}],
        ODOM: [{"header_ns": 1}, {"header_ns": 2}],
    }
    now = -0.1

    def monotonic():
        nonlocal now
        now += 0.1
        return now

    process = Mock()
    process.poll.return_value = None
    monkeypatch.setattr(runner.time, "monotonic", monotonic)
    monkeypatch.setattr(runner.time, "sleep", lambda _seconds: None)

    with pytest.raises(runner.VerifiedDrainTimeoutError, match="last=pending"):
        runner.wait_for_verified_drain(
            source, CLOUD, ODOM, "/audit", {}, 0.5,
            [("backend", process, object())],
            snapshot_request=Mock(return_value=verified_snapshot_fields(count=1)),
        )


def test_verified_drain_fails_if_managed_backend_shutdowns(monkeypatch):
    source = {CLOUD: [{"header_ns": 1}], ODOM: [{"header_ns": 1}]}
    process = Mock()
    process.poll.return_value = 7
    monkeypatch.setattr(runner.time, "monotonic", Mock(side_effect=[0.0, 0.0]))

    with pytest.raises(diagnostic.DiagnosticError, match="backend=7"):
        runner.wait_for_verified_drain(
            source, CLOUD, ODOM, "/audit", {}, 1.0,
            [("backend", process, object())], snapshot_request=Mock(),
        )


def test_verified_drain_malformed_snapshot_is_error_not_timeout(monkeypatch):
    source = {CLOUD: [{"header_ns": 1}], ODOM: [{"header_ns": 1}]}
    process = Mock()
    process.poll.return_value = None
    monkeypatch.setattr(runner.time, "monotonic", Mock(side_effect=[0.0, 0.0]))
    request = Mock(return_value={"report": "snapshot"})

    with pytest.raises(diagnostic.DiagnosticError, match="missing field"):
        runner.wait_for_verified_drain(
            source, CLOUD, ODOM, "/audit", {}, 1.0,
            [("backend", process, object())], snapshot_request=request,
        )


def test_service_call_timeout_kills_owned_subprocess(monkeypatch):
    process = Mock(returncode=None)
    process.communicate.side_effect = [
        subprocess.TimeoutExpired(["ros2"], 0.25),
        ("late output", None),
    ]
    popen = Mock(return_value=process)
    monkeypatch.setattr(runner.subprocess, "Popen", popen)

    with pytest.raises(diagnostic.DiagnosticError, match="timed out"):
        runner.request_stream_audit_snapshot("/audit", {}, 0.25)

    process.kill.assert_called_once_with()
    assert process.communicate.call_args_list == [
        call(timeout=0.25), call(timeout=0.25)
    ]


@pytest.mark.parametrize("value", ["0", "-1", "1.5", "nan", "inf"])
def test_expected_source_count_requires_positive_integer(value):
    with pytest.raises(SystemExit):
        runner.parser().parse_args(
            [
                "run", "--bag", "bag", "--output-dir", "out", "--domain-id", "166",
                "--subscriber-qos-profile", "reliable", "--expected-source-count", value,
            ]
        )


def test_expected_source_count_is_optional():
    args = runner.parser().parse_args(
        [
            "run", "--bag", "bag", "--output-dir", "out", "--domain-id", "166",
            "--subscriber-qos-profile", "reliable",
        ]
    )
    assert args.expected_source_count is None


def test_recorder_qos_yaml_parser_is_opt_in_and_preserves_argument():
    base = [
        "run", "--bag", "bag", "--output-dir", "out", "--domain-id", "166",
        "--subscriber-qos-profile", "reliable",
    ]

    assert runner.parser().parse_args(base).recorder_qos_yaml is None
    assert runner.parser().parse_args(base).witness_raw_cdr_stamps is False
    args = runner.parser().parse_args(
        base + ["--recorder-qos-yaml", "relative/recorder-qos.yaml"]
    )
    assert args.recorder_qos_yaml == Path("relative/recorder-qos.yaml")


def test_run_rejects_missing_recorder_qos_yaml_before_bag_or_output(
    tmp_path, monkeypatch
):
    output = tmp_path / "out"
    args = runner.parser().parse_args(
        [
            "run", "--bag", str(tmp_path / "source"), "--output-dir", str(output),
            "--domain-id", "166", "--subscriber-qos-profile", "reliable",
            "--recorder-qos-yaml", str(tmp_path / "missing.yaml"),
        ]
    )
    inspect = Mock(side_effect=AssertionError("source bag must not be inspected"))
    monkeypatch.setattr(runner, "inspect_bag", inspect)

    with pytest.raises(diagnostic.DiagnosticError, match="recorder QoS YAML does not exist"):
        runner.run_diagnostic(args)

    inspect.assert_not_called()
    assert not output.exists()


def test_qos_and_raw_witness_arguments_are_forwarded_only_to_their_owners(
    tmp_path, monkeypatch
):
    class FakeProcess:
        next_pid = 500

        def __init__(self, command):
            self.args = command
            self.pid = FakeProcess.next_pid
            FakeProcess.next_pid += 1
            self.returncode = None

        def poll(self):
            return self.returncode

        def wait(self, timeout=None):
            self.returncode = 0
            return 0

        def send_signal(self, _sent_signal):
            pass

        def terminate(self):
            pass

        def kill(self):
            pass

    monkeypatch.chdir(tmp_path)
    qos_argument = Path("profiles/recorder-cloud-100.yaml")
    qos_path = tmp_path / qos_argument
    qos_path.parent.mkdir()
    qos_content = "/cloud:\n  subscription:\n    depth: 100\n"
    qos_path.write_text(qos_content, encoding="utf-8")
    source = {CLOUD: [{"header_ns": 1}], ODOM: [{"header_ns": 1}]}
    started = []

    def fake_start(command, _log, _env):
        copied_command = list(command)
        started.append(copied_command)
        return FakeProcess(copied_command), BytesIO()

    def fake_audit(*audit_args, run_evidence):
        report = {"run_evidence": run_evidence}
        runner.write_json(audit_args[3], report)
        return report

    monkeypatch.setattr(runner, "inspect_bag", lambda *_args: source)
    monkeypatch.setattr(runner, "bag_content_duration_seconds", lambda _bag: 1.0)
    monkeypatch.setattr(runner, "read_udp_counters", lambda: {})
    monkeypatch.setattr(runner, "snapshot_endpoints", lambda *_args: None)
    monkeypatch.setattr(runner, "start_logged", fake_start)
    monkeypatch.setattr(runner.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(runner.time, "monotonic", Mock(side_effect=[10.0, 10.0, 11.0]))
    monkeypatch.setattr(runner, "audit_artifacts", fake_audit)

    output = tmp_path / "out"
    args = runner.parser().parse_args(
        [
            "run", "--bag", "source", "--output-dir", str(output),
            "--domain-id", "166", "--subscriber-qos-profile", "reliable",
            "--cloud-topic", CLOUD, "--odom-topic", ODOM, "--pgo-topic", PGO,
            "--recorder-qos-yaml", str(qos_argument), "--startup-sec", "0.01",
            "--witness-raw-cdr-stamps",
            "--endpoint-snapshot-delay-sec", "0.01", "--drain-sec", "0.01",
            "--shutdown-timeout-sec", "0.01",
        ]
    )

    assert runner.run_diagnostic(args) == 0

    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    commands = manifest["commands"]
    expected_suffix = ["--qos-profile-overrides-path", str(qos_argument)]
    assert commands["recorder"][-2:] == expected_suffix
    assert manifest["recorder_qos"] == {"yaml": str(qos_argument)}
    assert manifest["witness_raw_cdr_stamps"] is True
    assert manifest["source_bag"] == str((tmp_path / "source").resolve())
    assert commands["playback"] == [
        "ros2", "bag", "play", str((tmp_path / "source").resolve()), "--rate", "1.0"
    ]
    assert all(
        "--qos-profile-overrides-path" not in commands[name]
        for name in ("witness", "backend", "playback")
    )
    assert commands["witness"][-1] == "--raw-cdr-stamps"
    assert all(
        "--raw-cdr-stamps" not in commands[name]
        for name in ("recorder", "backend", "playback")
    )
    assert started == [
        commands["witness"], commands["recorder"], commands["backend"], commands["playback"]
    ]
    assert qos_path.read_text(encoding="utf-8") == qos_content


def test_wait_for_playback_uses_remaining_timeout_and_records_elapsed(monkeypatch):
    process = Mock()
    process.wait.return_value = 0
    monkeypatch.setattr(runner.time, "monotonic", Mock(side_effect=[11.0, 13.5]))

    returncode, elapsed = runner.wait_for_playback(process, 5.0, started=10.0)

    assert returncode == 0
    assert elapsed == pytest.approx(3.5)
    process.wait.assert_called_once_with(timeout=4.0)


def test_wait_for_playback_timeout_is_precise(monkeypatch):
    process = Mock(args=["ros2", "bag", "play"])
    process.wait.side_effect = subprocess.TimeoutExpired(process.args, 4.0)
    monkeypatch.setattr(runner.time, "monotonic", Mock(side_effect=[11.0, 15.25]))

    with pytest.raises(
        runner.PlaybackTimeoutError,
        match=r"--playback-timeout-sec 5 after 5\.250000 monotonic seconds",
    ):
        runner.wait_for_playback(process, 5.0, started=10.0)
    process.wait.assert_called_once_with(timeout=4.0)


def test_stop_process_escalates_only_the_given_child_with_bounded_waits():
    process = Mock(pid=1234)
    process.poll.return_value = None
    process.wait.side_effect = [
        subprocess.TimeoutExpired("child", 2.0),
        subprocess.TimeoutExpired("child", 2.0),
        -9,
    ]

    runner.stop_process(process, 2.0)

    process.send_signal.assert_called_once_with(signal.SIGINT)
    process.terminate.assert_called_once_with()
    process.kill.assert_called_once_with()
    assert process.wait.call_args_list == [call(timeout=2.0)] * 3


def test_stop_process_reports_failure_after_bounded_sigkill_wait():
    process = Mock(pid=4321)
    process.poll.return_value = None
    process.wait.side_effect = subprocess.TimeoutExpired("child", 1.5)

    with pytest.raises(
        diagnostic.DiagnosticError,
        match="pid 4321 did not exit after SIGKILL within 1.5 seconds",
    ):
        runner.stop_process(process, 1.5)
    assert process.wait.call_count == 3


def test_backend_audits_require_single_passing_lines_and_complete_accounting(tmp_path):
    backend_log = tmp_path / "backend.log"
    backend_log.write_text(passing_backend_audits(), encoding="utf-8")
    source = {CLOUD: [{}, {}], ODOM: [{}, {}]}

    validation = runner.validate_backend_audits(backend_log, source, CLOUD, ODOM)

    assert validation["status"] == "PASS"
    assert validation["determinism_audit"] == {"line_count": 1, "status": "PASS"}
    stream = validation["stream_shutdown_audit"]
    assert stream["line_count"] == 1
    assert stream["report"] == "shutdown"
    assert stream["status"] == "PASS"
    assert stream["accounting"]["status"] == "PASS"
    assert stream["accounting"]["reported_counts"]["pgo_publish_completed_count"] == 2


def test_backend_final_audit_rechecks_verified_drain_exact_digests(tmp_path):
    source = {
        CLOUD: [{"header_ns": 1}, {"header_ns": 2}],
        ODOM: [{"header_ns": 1}, {"header_ns": 2}],
    }
    expected = runner.expected_verified_drain_fields(source, CLOUD, ODOM)
    fields = verified_snapshot_fields()
    fields["report"] = "shutdown"
    backend_log = tmp_path / "backend.log"
    backend_log.write_text(
        "SLC_DETERMINISM_AUDIT status=PASS\n" + stream_audit_line(fields) + "\n",
        encoding="utf-8",
    )

    validation = runner.validate_backend_audits(
        backend_log, source, CLOUD, ODOM, expected
    )
    assert validation["status"] == "PASS"
    assert validation["stream_shutdown_audit"]["verified_drain_exactness"] == {
        "status": "PASS"
    }

    fields["pgo_publish_completed_fnv64"] = "0x0000000000000000"
    backend_log.write_text(
        "SLC_DETERMINISM_AUDIT status=PASS\n" + stream_audit_line(fields) + "\n",
        encoding="utf-8",
    )
    validation = runner.validate_backend_audits(
        backend_log, source, CLOUD, ODOM, expected
    )
    assert validation["status"] == "FAIL"
    assert validation["stream_shutdown_audit"]["verified_drain_exactness"]["status"] == "FAIL"


@pytest.mark.parametrize(
    ("audit_text", "expected_status", "expected_error"),
    [
        ("", "MISSING", "found 0"),
        (
            passing_backend_audits() + passing_backend_audits(),
            "DUPLICATE",
            "found 2",
        ),
        (
            passing_backend_audits().replace(
                "SLC_DETERMINISM_AUDIT status=PASS",
                "SLC_DETERMINISM_AUDIT status=FAIL",
            ),
            "FAIL",
            "status is FAIL",
        ),
        (
            passing_backend_audits().replace(
                "report=shutdown status=PASS", "report=shutdown status=INCONCLUSIVE"
            ),
            "PASS",
            "status is INCONCLUSIVE",
        ),
        (
            passing_backend_audits(count=1),
            "PASS",
            "cloud_ingress_count=1 expected 2",
        ),
    ],
)
def test_backend_audits_fail_closed(
    tmp_path, audit_text, expected_status, expected_error
):
    backend_log = tmp_path / "backend.log"
    backend_log.write_text(audit_text, encoding="utf-8")
    source = {CLOUD: [{}, {}], ODOM: [{}, {}]}

    validation = runner.validate_backend_audits(backend_log, source, CLOUD, ODOM)

    assert validation["status"] == "FAIL"
    assert validation["determinism_audit"]["status"] == expected_status
    assert any(expected_error in error for error in validation["errors"])


@pytest.mark.parametrize(
    ("backend_log_text", "expected_completed"),
    [("", False), (passing_backend_audits(count=1), True)],
)
def test_required_backend_audits_gate_otherwise_normal_runner(
    tmp_path, monkeypatch, backend_log_text, expected_completed
):
    class FakeProcess:
        next_pid = 200

        def __init__(self):
            self.args = ["mock"]
            self.pid = FakeProcess.next_pid
            FakeProcess.next_pid += 1
            self.returncode = None

        def poll(self):
            return self.returncode

        def wait(self, timeout=None):
            self.returncode = 0
            return 0

        def send_signal(self, _sent_signal):
            pass

        def terminate(self):
            pass

        def kill(self):
            pass

    source = {CLOUD: [{"header_ns": 1}], ODOM: [{"header_ns": 1}]}

    def fake_start(command, log, _env):
        log.write_text(
            backend_log_text if log.name == "backend.log" else "", encoding="utf-8"
        )
        return FakeProcess(), BytesIO()

    def fake_audit(*audit_args, run_evidence):
        report = {"run_evidence": run_evidence}
        runner.write_json(audit_args[3], report)
        return report

    monkeypatch.setattr(runner, "inspect_bag", lambda *_args: source)
    monkeypatch.setattr(runner, "bag_content_duration_seconds", lambda _bag: 1.0)
    monkeypatch.setattr(runner, "read_udp_counters", lambda: {})
    monkeypatch.setattr(runner, "snapshot_endpoints", lambda *_args: None)
    monkeypatch.setattr(runner, "start_logged", fake_start)
    monkeypatch.setattr(runner.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(runner.time, "monotonic", Mock(side_effect=[10.0, 10.0, 11.0]))
    monkeypatch.setattr(runner, "audit_artifacts", fake_audit)

    output = tmp_path / "out"
    args = runner.parser().parse_args(
        [
            "run", "--bag", str(tmp_path / "source"), "--output-dir", str(output),
            "--domain-id", "166", "--subscriber-qos-profile", "reliable",
            "--cloud-topic", CLOUD, "--odom-topic", ODOM, "--pgo-topic", PGO,
            "--require-backend-audits", "--startup-sec", "0.01",
            "--endpoint-snapshot-delay-sec", "0.01", "--drain-sec", "0.01",
            "--shutdown-timeout-sec", "0.01",
        ]
    )

    if expected_completed:
        assert runner.run_diagnostic(args) == 0
    else:
        with pytest.raises(diagnostic.DiagnosticError, match="required backend audits did not pass"):
            runner.run_diagnostic(args)

    status = json.loads((output / "run_status.json").read_text(encoding="utf-8"))
    report = json.loads((output / "loss_report.json").read_text(encoding="utf-8"))
    expected_status = "PASS" if expected_completed else "FAIL"
    assert status["completed"] is expected_completed
    assert status["backend_validation"]["status"] == expected_status
    assert status["audit"]["pass_eligible"] is expected_completed
    assert report["run_evidence"]["pass_eligible"] is expected_completed


@pytest.mark.parametrize(("external_status", "expected_completed"), [("PASS", True), ("FAIL", False)])
def test_required_external_integrity_gates_fake_run(
    tmp_path, monkeypatch, external_status, expected_completed
):
    def fake_audit(*audit_args, run_evidence):
        discrepancy = [] if external_status == "PASS" else [
            {
                "stage": "source_to_recorder",
                "topic": CLOUD,
                "missing_count": 1,
                "extra_count": 0,
                "exact_order_match": False,
            }
        ]
        report = {
            "run_evidence": run_evidence,
            "external_stamp_integrity": {
                "schema": "slc-c1-external-stamp-integrity-v1",
                "status": external_status,
                "stages": {},
                "discrepancies": discrepancy,
            },
        }
        runner.write_json(audit_args[3], report)
        return report

    patch_successful_fake_run(monkeypatch, fake_audit)

    output = tmp_path / "out"
    args = runner.parser().parse_args(
        [
            "run", "--bag", str(tmp_path / "source"), "--output-dir", str(output),
            "--domain-id", "166", "--subscriber-qos-profile", "reliable",
            "--cloud-topic", CLOUD, "--odom-topic", ODOM, "--pgo-topic", PGO,
            "--require-external-stamp-integrity", "--startup-sec", "0.01",
            "--endpoint-snapshot-delay-sec", "0.01", "--drain-sec", "0.01",
            "--shutdown-timeout-sec", "0.01",
        ]
    )

    if expected_completed:
        assert runner.run_diagnostic(args) == 0
    else:
        with pytest.raises(
            diagnostic.DiagnosticError, match="required external stamp integrity did not pass"
        ):
            runner.run_diagnostic(args)

    status = json.loads((output / "run_status.json").read_text(encoding="utf-8"))
    report = json.loads((output / "loss_report.json").read_text(encoding="utf-8"))
    assert status["completed"] is expected_completed
    assert status["audit"] == {
        "completed": True,
        "pass_eligible": expected_completed,
        "scope": "complete",
    }
    assert status["external_stamp_integrity"]["status"] == external_status
    assert report["run_evidence"]["pass_eligible"] is expected_completed


def test_certify_all_enables_both_gates():
    args = runner.parser().parse_args(
        [
            "run", "--bag", "source", "--output-dir", "out",
            "--domain-id", "166", "--subscriber-qos-profile", "reliable",
            "--certify-all",
        ]
    )

    assert args.certify_all is True
    assert args.require_backend_audits is False
    assert args.require_external_stamp_integrity is False
    assert runner.certification_requirements(args) == (True, True)


def test_certify_all_external_pass_does_not_clear_earlier_backend_failure(
    tmp_path, monkeypatch
):
    def fake_audit(
        *audit_args, run_evidence, certify_full_source=False,
        expected_source_count=None
    ):
        assert certify_full_source is True
        assert expected_source_count is None
        report = {
            "run_evidence": run_evidence,
            "external_stamp_integrity": {
                "schema": "slc-c1-external-stamp-integrity-v2",
                "status": "PASS",
                "full_source_contract": {
                    "enforced": True,
                    "derived_source_count": 1,
                    "expected_source_count": None,
                },
                "stages": {"test_double": {}},
                "discrepancies": [],
            },
        }
        runner.write_json(audit_args[3], report)
        return report

    patch_successful_fake_run(monkeypatch, fake_audit)
    output = tmp_path / "out"
    args = runner.parser().parse_args(
        [
            "run", "--bag", "source", "--output-dir", str(output),
            "--domain-id", "166", "--subscriber-qos-profile", "reliable",
            "--cloud-topic", CLOUD, "--odom-topic", ODOM, "--pgo-topic", PGO,
            "--certify-all", "--startup-sec", "0.01",
            "--endpoint-snapshot-delay-sec", "0.01", "--drain-sec", "0.01",
            "--shutdown-timeout-sec", "0.01",
        ]
    )

    with pytest.raises(
        diagnostic.DiagnosticError,
        match="required backend audit validation failed",
    ):
        runner.run_diagnostic(args)

    status = json.loads((output / "run_status.json").read_text(encoding="utf-8"))
    report = json.loads((output / "loss_report.json").read_text(encoding="utf-8"))
    assert status["completed"] is False
    assert status["backend_validation"]["status"] == "ERROR"
    assert status["external_stamp_integrity"]["status"] == "PASS"
    assert status["audit"]["pass_eligible"] is False
    assert report["run_evidence"]["pass_eligible"] is False


def test_certification_write_ioerror_leaves_persisted_report_fail_closed(
    tmp_path, monkeypatch
):
    original_write_json = runner.write_json

    def flaky_write_json(path, value):
        if path.name == "loss_report.json" and path.exists():
            raise OSError("cannot promote certification report")
        original_write_json(path, value)

    def fake_audit(*audit_args, run_evidence):
        report = {
            "run_evidence": run_evidence,
            "external_stamp_integrity": {
                "schema": "slc-c1-external-stamp-integrity-v1",
                "status": "PASS",
                "stages": {},
                "discrepancies": [],
            },
        }
        runner.write_json(audit_args[3], report)
        return report

    monkeypatch.setattr(runner, "write_json", flaky_write_json)
    patch_successful_fake_run(monkeypatch, fake_audit)

    output = tmp_path / "out"
    args = runner.parser().parse_args(
        [
            "run", "--bag", "source", "--output-dir", str(output),
            "--domain-id", "166", "--subscriber-qos-profile", "reliable",
            "--require-external-stamp-integrity", "--startup-sec", "0.01",
            "--endpoint-snapshot-delay-sec", "0.01", "--drain-sec", "0.01",
            "--shutdown-timeout-sec", "0.01",
        ]
    )

    with pytest.raises(diagnostic.DiagnosticError, match="artifact audit failed"):
        runner.run_diagnostic(args)

    status = json.loads((output / "run_status.json").read_text(encoding="utf-8"))
    report = json.loads((output / "loss_report.json").read_text(encoding="utf-8"))
    assert status["completed"] is False
    assert status["audit"]["completed"] is False
    assert "pass_eligible" not in status["audit"]
    assert report["run_evidence"]["pass_eligible"] is False


@pytest.mark.parametrize(
    ("run_name", "expected_pass", "expected_witness_cloud", "expected_recorder_cloud"),
    [
        ("slc_s2_reader100_cbd03_lio_full_20261018", True, 2660, 2660),
        ("slc_s2_reader100_cbd03_lio_full_repeat2_20261018", True, 2660, 2660),
        ("slc_s2_reader100_cbd03_lio_full_repeat3_20261018", False, 2637, 2612),
    ],
)
def test_historical_loss_reports_expose_external_completeness(
    run_name, expected_pass, expected_witness_cloud, expected_recorder_cloud
):
    report_path = (
        Path(__file__).resolve().parents[3] / "runs" / run_name / "loss_report.json"
    )
    if not report_path.is_file():
        pytest.skip(f"read-only historical witness is unavailable: {report_path}")
    report = json.loads(report_path.read_text(encoding="utf-8"))

    comparisons = [
        *(report["input_delivery_loss"][topic] for topic in (
            "/cloud_registered_metric", "/aft_mapped_to_init"
        )),
        *(report["recorder_input_loss"][topic] for topic in (
            "/cloud_registered_metric", "/aft_mapped_to_init"
        )),
        report["recorder_output_loss"],
    ]
    historical_external_pass = all(
        comparison["missing_count"] == 0
        and comparison["extra_count"] == 0
        and comparison["exact_order_match"] is True
        for comparison in comparisons
    )

    assert historical_external_pass is expected_pass
    assert report["input_delivery_loss"]["/cloud_registered_metric"][
        "observed_count"
    ] == expected_witness_cloud
    assert report["recorder_input_loss"]["/cloud_registered_metric"][
        "observed_count"
    ] == expected_recorder_cloud


def test_timeout_run_is_failed_closed_and_salvages_partial_audit(tmp_path, monkeypatch):
    class FakeProcess:
        next_pid = 100

        def __init__(self, timeout_on_first_wait=False):
            self.args = ["mock"]
            self.pid = FakeProcess.next_pid
            FakeProcess.next_pid += 1
            self.returncode = None
            self.timeout_on_first_wait = timeout_on_first_wait
            self.wait_count = 0
            self.signals = []

        def poll(self):
            return self.returncode

        def wait(self, timeout=None):
            self.wait_count += 1
            if self.timeout_on_first_wait and self.wait_count == 1:
                raise subprocess.TimeoutExpired(self.args, timeout)
            self.returncode = 0
            return 0

        def send_signal(self, sent_signal):
            self.signals.append(sent_signal)

        def terminate(self):
            self.signals.append(signal.SIGTERM)

        def kill(self):
            self.signals.append(signal.SIGKILL)

    source = {CLOUD: [{"header_ns": 1}], ODOM: [{"header_ns": 1}]}
    processes = []

    def fake_start(command, _log, _env):
        is_playback = command[:3] == ["ros2", "bag", "play"]
        process = FakeProcess(timeout_on_first_wait=is_playback)
        processes.append(process)
        return process, BytesIO()

    def fake_audit(*audit_args, run_evidence):
        report = {"counts": "partial", "run_evidence": run_evidence}
        runner.write_json(audit_args[3], report)
        return report

    monkeypatch.setattr(runner, "inspect_bag", lambda *_args: source)
    monkeypatch.setattr(runner, "bag_content_duration_seconds", lambda _bag: 4.0)
    monkeypatch.setattr(runner, "read_udp_counters", lambda: {})
    monkeypatch.setattr(runner, "snapshot_endpoints", lambda *_args: None)
    monkeypatch.setattr(runner, "start_logged", fake_start)
    monkeypatch.setattr(runner.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(runner.time, "monotonic", Mock(side_effect=[10.0, 10.0, 15.0]))
    monkeypatch.setattr(runner, "audit_artifacts", fake_audit)

    output = tmp_path / "out"
    args = runner.parser().parse_args(
        [
            "run", "--bag", str(tmp_path / "source"), "--output-dir", str(output),
            "--domain-id", "166", "--subscriber-qos-profile", "reliable",
            "--playback-timeout-sec", "5", "--startup-sec", "0.01",
            "--endpoint-snapshot-delay-sec", "0.01", "--drain-sec", "0.01",
            "--shutdown-timeout-sec", "0.01",
        ]
    )

    with pytest.raises(runner.PlaybackTimeoutError):
        runner.run_diagnostic(args)

    status = json.loads((output / "run_status.json").read_text(encoding="utf-8"))
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    report = json.loads((output / "loss_report.json").read_text(encoding="utf-8"))
    assert status["completed"] is False
    assert status["outcome"] == "failed"
    assert status["playback"] == {
        "bag_content_duration_seconds": 4.0,
        "completed": False,
        "effective_throughput_rate": None,
        "elapsed_monotonic_seconds": 5.0,
        "timed_out": True,
        "timeout_seconds": 5.0,
    }
    assert status["audit"] == {
        "completed": True, "pass_eligible": False, "scope": "partial"
    }
    assert manifest["timings_seconds"]["playback_timeout"] == 5.0
    assert manifest["result_policy"]["aborted_replay_pass_eligible"] is False
    assert report["run_evidence"] == {
        "audit_scope": "partial", "pass_eligible": False, "replay_completed": False
    }
    assert len(processes) == 4
    assert all(process.signals == [signal.SIGINT] for process in processes)
