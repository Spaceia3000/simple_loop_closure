#!/usr/bin/env python3
"""Pure helpers for the SLC transport diagnostic.

This module intentionally has no ROS imports so its comparison and SQLite
inventory logic can be tested on a machine without a sourced ROS environment.
"""

from __future__ import annotations

import collections
import json
import sqlite3
from pathlib import Path
from typing import Callable, Iterable, Mapping


class DiagnosticError(RuntimeError):
    """A reproducible diagnostic precondition was not met."""


def sqlite_files(bag: Path) -> list[Path]:
    """Return a deterministic list of SQLite storage files in a rosbag."""
    if not bag.is_dir():
        raise DiagnosticError(f"bag is not a directory: {bag}")
    files = sorted(bag.glob("*.db3"))
    if not files:
        raise DiagnosticError(f"bag has no SQLite .db3 storage files: {bag}")
    return files


def read_sqlite_messages(
    bag: Path,
    topics: Iterable[str],
    decode_header_ns: Callable[[str, bytes], int],
) -> dict[str, list[dict[str, int]]]:
    """Read storage/header stamps directly from rosbag2 SQLite rows.

    ``decode_header_ns`` receives the ROS type string and serialized CDR bytes.
    Keeping it injectable makes the SQLite behavior independently testable.
    """
    wanted = tuple(dict.fromkeys(topics))
    result: dict[str, list[dict[str, int]]] = {topic: [] for topic in wanted}
    for database in sqlite_files(bag):
        connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
        try:
            rows = connection.execute(
                "SELECT t.name, t.type, m.timestamp, m.data "
                "FROM messages AS m JOIN topics AS t ON t.id = m.topic_id "
                "ORDER BY m.timestamp, m.id"
            )
            for topic, type_name, storage_ns, payload in rows:
                if topic not in result:
                    continue
                result[topic].append(
                    {
                        "storage_ns": int(storage_ns),
                        "header_ns": int(decode_header_ns(type_name, bytes(payload))),
                    }
                )
        finally:
            connection.close()
    return result


def read_jsonl(path: Path) -> list[dict[str, object]]:
    records: list[dict[str, object]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise DiagnosticError(f"invalid JSONL at {path}:{line_number}: {exc}") from exc
    return records


def _stamps(records: Iterable[Mapping[str, object]]) -> list[int]:
    return [int(record["header_ns"]) for record in records]


def compare_stamp_sequences(
    expected: Iterable[Mapping[str, object]],
    observed: Iterable[Mapping[str, object]],
) -> dict[str, object]:
    """Compare header stamps as multisets, preserving duplicate evidence."""
    expected_stamps = _stamps(expected)
    observed_stamps = _stamps(observed)
    expected_counts = collections.Counter(expected_stamps)
    observed_counts = collections.Counter(observed_stamps)
    missing = list((expected_counts - observed_counts).elements())
    extra = list((observed_counts - expected_counts).elements())
    return {
        "expected_count": len(expected_stamps),
        "observed_count": len(observed_stamps),
        "matched_count": sum((expected_counts & observed_counts).values()),
        "missing_count": len(missing),
        "extra_count": len(extra),
        "missing_header_ns": sorted(missing),
        "extra_header_ns": sorted(extra),
        "exact_order_match": expected_stamps == observed_stamps,
    }


def records_for_topic(
    records: Iterable[Mapping[str, object]], topic: str
) -> list[Mapping[str, object]]:
    return [record for record in records if record.get("topic") == topic]


def stream_stats(records: Iterable[Mapping[str, object]]) -> dict[str, object]:
    """Summarize explicit header/storage/receive time spans and rates."""
    items = list(records)
    result: dict[str, object] = {"count": len(items)}
    for field in ("header_ns", "storage_ns", "receive_monotonic_ns"):
        values = [int(item[field]) for item in items if field in item]
        if not values:
            continue
        span_ns = max(values) - min(values)
        result[field] = {
            "first": min(values),
            "last": max(values),
            "span_ns": span_ns,
            "rate_hz": ((len(values) - 1) * 1_000_000_000 / span_ns)
            if len(values) > 1 and span_ns > 0
            else None,
        }
    return result


def build_loss_report(
    source: Mapping[str, list[dict[str, int]]],
    witness: list[dict[str, object]],
    recorded: Mapping[str, list[dict[str, int]]],
    cloud_topic: str,
    odom_topic: str,
    pgo_topic: str,
) -> dict[str, object]:
    """Build comparisons whose labels distinguish input, backend and recorder loss."""
    witness_by_topic = {
        topic: records_for_topic(witness, topic)
        for topic in (cloud_topic, odom_topic, pgo_topic)
    }
    input_delivery = {
        topic: compare_stamp_sequences(source[topic], witness_by_topic[topic])
        for topic in (cloud_topic, odom_topic)
    }
    recorder_input = {
        topic: compare_stamp_sequences(witness_by_topic[topic], recorded[topic])
        for topic in (cloud_topic, odom_topic)
    }

    cloud_seen = collections.Counter(_stamps(witness_by_topic[cloud_topic]))
    odom_seen = collections.Counter(_stamps(witness_by_topic[odom_topic]))
    paired_input = [
        {"header_ns": stamp}
        for stamp in sorted((cloud_seen & odom_seen).elements())
    ]
    backend_output = compare_stamp_sequences(paired_input, witness_by_topic[pgo_topic])
    recorder_output = compare_stamp_sequences(witness_by_topic[pgo_topic], recorded[pgo_topic])

    return {
        "interpretation": {
            "input_delivery_loss": "source SQLite header stamps missing at the independent subscriber",
            "backend_output_gap": "cloud/odom stamps seen by the witness but absent from /pgo_odom",
            "recorder_input_loss": "input stamps seen by the witness but absent from the output bag",
            "recorder_output_loss": "/pgo_odom stamps seen by the witness but absent from the output bag",
            "caveat": "independent subscribers can differ under their explicit QoS profiles; inspect endpoint and UDP evidence before assigning cause",
        },
        "input_delivery_loss": input_delivery,
        "backend_output_gap": backend_output,
        "recorder_input_loss": recorder_input,
        "recorder_output_loss": recorder_output,
        "paired_input_count_at_witness": len(paired_input),
        "stream_statistics": {
            "source_sqlite": {topic: stream_stats(source[topic]) for topic in (cloud_topic, odom_topic)},
            "witness": {topic: stream_stats(witness_by_topic[topic]) for topic in (cloud_topic, odom_topic, pgo_topic)},
            "recorded_sqlite": {topic: stream_stats(recorded[topic]) for topic in (cloud_topic, odom_topic, pgo_topic)},
        },
    }


def write_json(path: Path, value: object) -> None:
    with path.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")
