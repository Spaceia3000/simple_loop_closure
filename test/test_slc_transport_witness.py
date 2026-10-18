import io
import json
import sqlite3
import struct
import sys
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import slc_transport_witness as witness  # noqa: E402


FIXTURE = (
    Path(__file__).resolve().parents[3]
    / "runs"
    / "slc_s2_cbd03_livo_fixture30_20261018"
)
FIXTURE_TOPICS = ("/cloud_registered_metric", "/aft_mapped_to_init")


@pytest.mark.parametrize(
    ("encapsulation", "endian", "sec", "nanosec"),
    [
        (b"\x00\x01\x00\x00", "<", 946_687_269, 399_641_990),
        (b"\x00\x00\x00\x00", ">", -2, 123),
    ],
)
def test_raw_cdr_header_stamp_parses_little_and_big_endian(
    encapsulation, endian, sec, nanosec
):
    payload = encapsulation + struct.pack(f"{endian}iI", sec, nanosec) + b"ignored"

    assert witness.raw_cdr_header_stamp_ns(payload) == sec * 1_000_000_000 + nanosec


@pytest.mark.parametrize(
    ("payload", "message"),
    [
        (b"", "at least 12"),
        (b"\x00\x01\x00", "at least 12"),
        (b"\x00\x02\x00\x00" + b"\x00" * 8, "unsupported CDR encapsulation"),
        (b"\x00\x01\x00\x01" + b"\x00" * 8, "unsupported CDR encapsulation"),
        (
            b"\x00\x01\x00\x00" + struct.pack("<iI", 1, 1_000_000_000),
            "invalid Header nanosec",
        ),
    ],
)
def test_raw_cdr_header_stamp_rejects_malformed_payload(payload, message):
    with pytest.raises(ValueError, match=message):
        witness.raw_cdr_header_stamp_ns(payload)


def test_raw_parser_matches_all_available_30_second_fixture_messages():
    database = FIXTURE / "bag_0.db3"
    manifest_path = FIXTURE / "fixture_manifest.json"
    if not database.is_file() or not manifest_path.is_file():
        pytest.skip(f"read-only 30 second fixture is unavailable: {FIXTURE}")

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
    try:
        for topic in FIXTURE_TOPICS:
            rows = connection.execute(
                "SELECT m.timestamp, substr(m.data, 1, 12) "
                "FROM messages AS m JOIN topics AS t ON t.id = m.topic_id "
                "WHERE t.name = ? ORDER BY m.timestamp, m.id",
                (topic,),
            ).fetchall()
            assert len(rows) == manifest["verification"]["topics"][topic]["count"] == 269
            assert [witness.raw_cdr_header_stamp_ns(payload) for _, payload in rows] == [
                storage_ns for storage_ns, _ in rows
            ]
    finally:
        connection.close()


def test_observer_preserves_jsonl_schema_and_per_topic_sequence():
    stream = io.StringIO()
    sequence = {}
    clock = iter((101, 202)).__next__
    observe = witness.make_observer(
        "/cloud", stream, sequence, witness.raw_cdr_header_stamp_ns, clock
    )
    payload = b"\x00\x01\x00\x00" + struct.pack("<iI", 7, 8)

    observe(payload)
    observe(payload)

    assert [json.loads(line) for line in stream.getvalue().splitlines()] == [
        {
            "topic": "/cloud",
            "sequence": 0,
            "header_ns": 7_000_000_008,
            "receive_monotonic_ns": 101,
        },
        {
            "topic": "/cloud",
            "sequence": 1,
            "header_ns": 7_000_000_008,
            "receive_monotonic_ns": 202,
        },
    ]
