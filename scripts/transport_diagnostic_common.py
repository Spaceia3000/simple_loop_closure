#!/usr/bin/env python3
"""Pure helpers for the SLC transport diagnostic.

This module intentionally has no ROS imports so its comparison and SQLite
inventory logic can be tested on a machine without a sourced ROS environment.
"""

from __future__ import annotations

import collections
import json
import os
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
        "same_multiset": not missing and not extra,
        "exact_order_match": expected_stamps == observed_stamps,
    }


def _comparison_validation_error(comparison: object) -> str | None:
    """Validate comparison evidence without relying on removable assertions."""
    if not isinstance(comparison, Mapping):
        return "comparison is not a mapping"
    integer_fields = (
        "expected_count",
        "observed_count",
        "matched_count",
        "missing_count",
        "extra_count",
    )
    for field in integer_fields:
        value = comparison.get(field)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            return f"{field} is not a non-negative integer"
    for field in ("missing_header_ns", "extra_header_ns"):
        values = comparison.get(field)
        if not isinstance(values, list) or any(
            isinstance(value, bool) or not isinstance(value, int) for value in values
        ):
            return f"{field} is not a list of integer stamps"
    for field in ("same_multiset", "exact_order_match"):
        if not isinstance(comparison.get(field), bool):
            return f"{field} is not a boolean"
    if comparison["missing_count"] != len(comparison["missing_header_ns"]):
        return "missing_count does not match missing_header_ns"
    if comparison["extra_count"] != len(comparison["extra_header_ns"]):
        return "extra_count does not match extra_header_ns"
    if comparison["expected_count"] != (
        comparison["matched_count"] + comparison["missing_count"]
    ):
        return "expected_count accounting is inconsistent"
    if comparison["observed_count"] != (
        comparison["matched_count"] + comparison["extra_count"]
    ):
        return "observed_count accounting is inconsistent"
    if comparison["same_multiset"] != (
        comparison["missing_count"] == 0 and comparison["extra_count"] == 0
    ):
        return "same_multiset is inconsistent with missing/extra counts"
    return None


def _comparison_discrepancy(
    stage: str, topic: object, comparison: object
) -> dict[str, object] | None:
    validation_error = _comparison_validation_error(comparison)
    if validation_error is not None:
        return {
            "stage": stage,
            "topic": str(topic),
            "validation_error": validation_error,
        }
    if not isinstance(comparison, Mapping):
        # The validator above makes this unreachable, but keep the production
        # path structurally safe if it is changed independently later.
        return {
            "stage": stage,
            "topic": str(topic),
            "validation_error": "comparison is not a mapping",
        }
    if (
        comparison["missing_count"] == 0
        and comparison["extra_count"] == 0
        and comparison["same_multiset"] is True
        and comparison["exact_order_match"] is True
    ):
        return None
    return {
        "stage": stage,
        "topic": str(topic),
        "missing_count": comparison["missing_count"],
        "extra_count": comparison["extra_count"],
        "same_multiset": comparison["same_multiset"],
        "exact_order_match": comparison["exact_order_match"],
    }


def build_external_stamp_integrity(
    source_to_witness: object,
    source_to_recorder: object,
    pgo_witness_to_recorder: object,
    pgo_topic: str = "/pgo_odom",
    *,
    full_source_comparisons: object | None = None,
    source_topic_counts: Mapping[str, int] | None = None,
    source_unique_topic_counts: Mapping[str, int] | None = None,
    source_metadata_topic_counts: Mapping[str, int] | None = None,
    expected_source_count: int | None = None,
) -> dict[str, object]:
    """Build exact external evidence, optionally enforcing the full-source contract."""
    raw_stages: dict[str, object] = {
        "source_to_witness": source_to_witness,
        "source_to_recorder": source_to_recorder,
        "pgo_witness_to_recorder": {pgo_topic: pgo_witness_to_recorder},
    }
    if full_source_comparisons is not None:
        if isinstance(full_source_comparisons, Mapping):
            raw_stages.update(full_source_comparisons)
        else:
            raw_stages["full_source_comparisons"] = full_source_comparisons

    stages: dict[str, dict[str, object]] = {}
    discrepancies: list[dict[str, object]] = []
    for stage, comparisons in raw_stages.items():
        if not isinstance(comparisons, Mapping):
            stages[stage] = {}
            discrepancies.append(
                {
                    "stage": stage,
                    "topic": "*",
                    "validation_error": "stage comparisons are not a mapping",
                }
            )
            continue
        stages[stage] = dict(comparisons)
        if not comparisons:
            discrepancies.append(
                {
                    "stage": stage,
                    "topic": "*",
                    "validation_error": "stage has no comparisons",
                }
            )
            continue
        for topic, comparison in comparisons.items():
            discrepancy = _comparison_discrepancy(stage, topic, comparison)
            if discrepancy is not None:
                discrepancies.append(discrepancy)

    full_source_contract: dict[str, object] = {
        "enforced": full_source_comparisons is not None,
    }
    if full_source_comparisons is not None:
        counts = dict(source_topic_counts or {})
        unique_counts = dict(source_unique_topic_counts or {})
        metadata_counts = (
            dict(source_metadata_topic_counts)
            if source_metadata_topic_counts is not None
            else None
        )
        valid_counts = bool(counts) and all(
            isinstance(value, int) and not isinstance(value, bool) and value >= 0
            for value in counts.values()
        )
        valid_unique_counts = set(unique_counts) == set(counts) and all(
            isinstance(value, int) and not isinstance(value, bool) and value >= 0
            for value in unique_counts.values()
        )
        expected_count_is_valid = expected_source_count is None or (
            isinstance(expected_source_count, int)
            and not isinstance(expected_source_count, bool)
            and expected_source_count > 0
        )
        derived_source_count = (
            next(iter(counts.values()))
            if valid_counts and len(set(counts.values())) == 1
            else None
        )
        full_source_contract.update(
            {
                "derived_source_count": derived_source_count,
                "source_topic_counts": counts,
                "source_unique_topic_counts": unique_counts,
                "expected_source_count": expected_source_count,
                "metadata_topic_counts": metadata_counts,
                "metadata_counts_verified": metadata_counts is not None,
            }
        )
        if not valid_counts or not valid_unique_counts:
            discrepancies.append(
                {
                    "stage": "source_inventory",
                    "topic": "source",
                    "validation_error": (
                        "source and unique topic counts must be matching mappings "
                        "of non-negative integers"
                    ),
                }
            )
        else:
            for topic, observed_count in counts.items():
                if observed_count == 0:
                    discrepancies.append(
                        {
                            "stage": "source_inventory_nonempty",
                            "topic": topic,
                            "observed_count": 0,
                        }
                    )
                if unique_counts[topic] != observed_count:
                    discrepancies.append(
                        {
                            "stage": "source_inventory_uniqueness",
                            "topic": topic,
                            "observed_count": observed_count,
                            "unique_count": unique_counts[topic],
                        }
                    )
        if not expected_count_is_valid:
            discrepancies.append(
                {
                    "stage": "expected_source_count",
                    "topic": "source",
                    "validation_error": "expected source count must be a positive integer",
                }
            )
        elif expected_source_count is not None:
            for topic, observed_count in counts.items():
                if observed_count != expected_source_count:
                    discrepancies.append(
                        {
                            "stage": "expected_source_count",
                            "topic": topic,
                            "expected_count": expected_source_count,
                            "observed_count": observed_count,
                        }
                    )
        if metadata_counts is not None:
            for topic, observed_count in counts.items():
                metadata_count = metadata_counts.get(topic)
                if metadata_count != observed_count:
                    discrepancies.append(
                        {
                            "stage": "source_metadata_count",
                            "topic": topic,
                            "metadata_count": metadata_count,
                            "sqlite_count": observed_count,
                        }
                    )
    return {
        "schema": "slc-c1-external-stamp-integrity-v2",
        "status": "PASS" if not discrepancies else "FAIL",
        "full_source_contract": full_source_contract,
        "stages": stages,
        "discrepancies": discrepancies,
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
    *,
    certify_full_source: bool = False,
    expected_source_count: int | None = None,
    source_metadata_topic_counts: Mapping[str, int] | None = None,
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
    source_to_recorder = {
        topic: compare_stamp_sequences(source[topic], recorded[topic])
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
    full_source_comparisons = None
    if certify_full_source:
        full_source_comparisons = {
            "source_cloud_to_odometry": {
                odom_topic: compare_stamp_sequences(source[cloud_topic], source[odom_topic])
            },
            "source_to_pgo_witness": {
                pgo_topic: compare_stamp_sequences(
                    source[cloud_topic], witness_by_topic[pgo_topic]
                )
            },
            "source_to_pgo_recorder": {
                pgo_topic: compare_stamp_sequences(
                    source[cloud_topic], recorded[pgo_topic]
                )
            },
        }
    external_stamp_integrity = build_external_stamp_integrity(
        input_delivery,
        source_to_recorder,
        recorder_output,
        pgo_topic,
        full_source_comparisons=full_source_comparisons,
        source_topic_counts=(
            {topic: len(source[topic]) for topic in (cloud_topic, odom_topic)}
            if certify_full_source
            else None
        ),
        source_unique_topic_counts=(
            {
                topic: len(set(_stamps(source[topic])))
                for topic in (cloud_topic, odom_topic)
            }
            if certify_full_source
            else None
        ),
        source_metadata_topic_counts=(
            source_metadata_topic_counts if certify_full_source else None
        ),
        expected_source_count=(expected_source_count if certify_full_source else None),
    )

    return {
        "interpretation": {
            "input_delivery_loss": "source SQLite header stamps missing at the independent subscriber",
            "source_to_recorder": "source SQLite header stamps compared independently with recorded SQLite rows",
            "backend_output_gap": "cloud/odom stamps seen by the witness but absent from /pgo_odom",
            "recorder_input_loss": "input stamps seen by the witness but absent from the output bag",
            "recorder_output_loss": "/pgo_odom stamps seen by the witness but absent from the output bag",
            "caveat": "independent subscribers can differ under their explicit QoS profiles; inspect endpoint and UDP evidence before assigning cause",
        },
        "input_delivery_loss": input_delivery,
        "source_to_recorder": source_to_recorder,
        "backend_output_gap": backend_output,
        "recorder_input_loss": recorder_input,
        "recorder_output_loss": recorder_output,
        "external_stamp_integrity": external_stamp_integrity,
        "paired_input_count_at_witness": len(paired_input),
        "stream_statistics": {
            "source_sqlite": {topic: stream_stats(source[topic]) for topic in (cloud_topic, odom_topic)},
            "witness": {topic: stream_stats(witness_by_topic[topic]) for topic in (cloud_topic, odom_topic, pgo_topic)},
            "recorded_sqlite": {topic: stream_stats(recorded[topic]) for topic in (cloud_topic, odom_topic, pgo_topic)},
        },
    }


def write_json(path: Path, value: object) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    try:
        with temporary.open("x", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)
