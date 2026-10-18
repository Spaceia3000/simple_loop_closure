#!/usr/bin/env python3
"""Run and audit an isolated rate-1 SLC transport diagnostic.

The wrapper does not change DDS, kernel, sysctl, or node parameters.  It starts
an independent subscriber, the native backend, rosbag recording, and playback;
then compares header stamps from source SQLite rows, subscriber observations,
backend /pgo_odom, and the recorded SQLite rows.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Sequence

from transport_diagnostic_common import (
    DiagnosticError,
    build_loss_report,
    read_jsonl,
    read_sqlite_messages,
    write_json,
)


DEFAULT_CLOUD_TOPIC = "/cloud_registered_metric"
DEFAULT_ODOM_TOPIC = "/aft_mapped_to_init"
DEFAULT_PGO_TOPIC = "/pgo_odom"


class PlaybackTimeoutError(DiagnosticError):
    """The explicitly bounded rosbag playback interval expired."""

    def __init__(self, timeout: float, elapsed: float):
        self.timeout_seconds = timeout
        self.elapsed_seconds = elapsed
        super().__init__(
            "rosbag playback exceeded configured "
            f"--playback-timeout-sec {timeout:g} after {elapsed:.6f} monotonic seconds"
        )


def _positive(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return parsed


def _domain_id(value: str) -> int:
    parsed = int(value)
    if not 0 <= parsed <= 190:
        raise argparse.ArgumentTypeError("must be in the safe diagnostic range 0..190")
    return parsed


def parser() -> argparse.ArgumentParser:
    top = argparse.ArgumentParser(description=__doc__)
    subparsers = top.add_subparsers(dest="action", required=True)

    run = subparsers.add_parser("run", help="execute replay and create an audit report")
    run.add_argument("--bag", type=Path, required=True)
    run.add_argument("--output-dir", type=Path, required=True)
    run.add_argument("--domain-id", type=_domain_id, required=True)
    run.add_argument("--subscriber-qos-profile", choices=("sensor-data", "reliable"), required=True)
    run.add_argument("--publisher-qos-yaml", type=Path)
    run.add_argument("--playback-rate", type=_positive, default=1.0)
    run.add_argument("--startup-sec", type=_positive, default=3.0)
    run.add_argument("--endpoint-snapshot-delay-sec", type=_positive, default=0.5)
    run.add_argument("--drain-sec", type=_positive, default=2.0)
    run.add_argument("--shutdown-timeout-sec", type=_positive, default=10.0)
    run.add_argument(
        "--playback-timeout-sec",
        type=_positive,
        help="optional finite wall-clock timeout for rosbag playback",
    )
    run.add_argument("--cloud-topic", default=DEFAULT_CLOUD_TOPIC)
    run.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    run.add_argument("--pgo-topic", default=DEFAULT_PGO_TOPIC)
    run.add_argument(
        "--backend-command",
        help="shell-like argv string, executed without a shell; default launches this package",
    )

    audit = subparsers.add_parser("audit", help="rebuild a report from completed artifacts")
    audit.add_argument("--source-bag", type=Path, required=True)
    audit.add_argument("--recorded-bag", type=Path, required=True)
    audit.add_argument("--witness-jsonl", type=Path, required=True)
    audit.add_argument("--report", type=Path, required=True)
    audit.add_argument("--cloud-topic", default=DEFAULT_CLOUD_TOPIC)
    audit.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    audit.add_argument("--pgo-topic", default=DEFAULT_PGO_TOPIC)
    return top


def ros_header_decoder():
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise DiagnosticError(f"sourced ROS 2 Python environment is required: {exc}") from exc

    message_types: dict[str, object] = {}

    def decode(type_name: str, payload: bytes) -> int:
        if type_name not in message_types:
            message_types[type_name] = get_message(type_name)
        message_class = message_types[type_name]
        message = deserialize_message(payload, message_class)
        if not hasattr(message, "header"):
            raise DiagnosticError(f"topic type has no header stamp: {type_name}")
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

    return decode


def inspect_bag(bag: Path, topics: Sequence[str]) -> dict[str, list[dict[str, int]]]:
    if not (bag / "metadata.yaml").is_file():
        raise DiagnosticError(f"rosbag has no metadata.yaml: {bag}")
    return read_sqlite_messages(bag, topics, ros_header_decoder())


def bag_content_duration_seconds(bag: Path) -> float | None:
    """Return rosbag2's declared content duration when it can be read safely."""
    try:
        import yaml
    except ImportError:
        return None
    try:
        metadata = yaml.safe_load((bag / "metadata.yaml").read_text(encoding="utf-8"))
        duration_ns = metadata["rosbag2_bagfile_information"]["duration"]["nanoseconds"]
        duration = int(duration_ns) / 1_000_000_000
    except (KeyError, TypeError, ValueError, OSError, yaml.YAMLError):
        return None
    return duration if math.isfinite(duration) and duration >= 0.0 else None


def audit_artifacts(
    source_bag: Path,
    recorded_bag: Path,
    witness_jsonl: Path,
    report_path: Path,
    cloud_topic: str,
    odom_topic: str,
    pgo_topic: str,
    run_evidence: dict[str, object] | None = None,
) -> dict[str, object]:
    source = inspect_bag(source_bag, (cloud_topic, odom_topic))
    recorded = inspect_bag(recorded_bag, (cloud_topic, odom_topic, pgo_topic))
    report = build_loss_report(
        source,
        read_jsonl(witness_jsonl),
        recorded,
        cloud_topic,
        odom_topic,
        pgo_topic,
    )
    if run_evidence is not None:
        report["run_evidence"] = run_evidence
    write_json(report_path, report)
    return report


def read_udp_counters() -> dict[str, object]:
    """Read Linux UDP MIBs only; this is deliberately read-only."""
    counters: dict[str, object] = {}
    for filename in ("/proc/net/snmp", "/proc/net/netstat"):
        path = Path(filename)
        if not path.is_file():
            continue
        lines = path.read_text(encoding="utf-8").splitlines()
        for index in range(0, len(lines) - 1, 2):
            names = lines[index].split()
            values = lines[index + 1].split()
            if not names or not values or names[0] != values[0]:
                continue
            section = names[0].rstrip(":")
            if not section.lower().startswith("udp"):
                continue
            counters[f"{path.name}:{section}"] = {
                name: int(value) for name, value in zip(names[1:], values[1:])
            }
    snmp6 = Path("/proc/net/snmp6")
    if snmp6.is_file():
        udp6: dict[str, int] = {}
        for line in snmp6.read_text(encoding="utf-8").splitlines():
            fields = line.split()
            if len(fields) == 2 and fields[0].startswith("Udp6"):
                udp6[fields[0]] = int(fields[1])
        counters["snmp6:Udp6"] = udp6
    return counters


def udp_counter_delta(
    before: dict[str, object], after: dict[str, object]
) -> dict[str, dict[str, int]]:
    delta: dict[str, dict[str, int]] = {}
    for section in sorted(set(before) | set(after)):
        before_values = before.get(section, {})
        after_values = after.get(section, {})
        if not isinstance(before_values, dict) or not isinstance(after_values, dict):
            continue
        delta[section] = {
            key: int(after_values.get(key, 0)) - int(before_values.get(key, 0))
            for key in sorted(set(before_values) | set(after_values))
        }
    return delta


def snapshot_endpoints(
    topics: Sequence[str], destination: Path, env: dict[str, str]
) -> None:
    snapshots: dict[str, object] = {}
    for topic in topics:
        completed = subprocess.run(
            ["ros2", "topic", "info", "--verbose", topic],
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=10.0,
            check=False,
        )
        snapshots[topic] = {"returncode": completed.returncode, "output": completed.stdout}
    write_json(destination, snapshots)


def stop_process(process: subprocess.Popen[bytes], timeout: float) -> None:
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            process.kill()
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired as exc:
                raise DiagnosticError(
                    f"child process pid {process.pid} did not exit after SIGKILL "
                    f"within {timeout:g} seconds"
                ) from exc


def wait_for_playback(
    process: subprocess.Popen[bytes], timeout: float | None, started: float | None = None
) -> tuple[int, float]:
    """Wait for the exact playback child and measure monotonic wall time."""
    if started is None:
        started = time.monotonic()
    try:
        if timeout is None:
            returncode = process.wait()
        else:
            remaining = timeout - (time.monotonic() - started)
            if remaining <= 0.0:
                raise subprocess.TimeoutExpired(process.args, timeout)
            returncode = process.wait(timeout=remaining)
    except subprocess.TimeoutExpired as exc:
        elapsed = time.monotonic() - started
        assert timeout is not None
        raise PlaybackTimeoutError(timeout, elapsed) from exc
    return returncode, time.monotonic() - started


def start_logged(command: Sequence[str], log: Path, env: dict[str, str]):
    stream = log.open("xb")
    try:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, env=env)
    except Exception:
        stream.close()
        raise
    return process, stream


def run_diagnostic(args: argparse.Namespace) -> int:
    if args.playback_rate != 1.0:
        raise DiagnosticError("this diagnostic requires explicit real-time --playback-rate 1.0")
    source_bag = args.bag.resolve()
    output_dir = args.output_dir.resolve()
    if output_dir.exists():
        raise DiagnosticError(f"refusing to overwrite existing output path: {output_dir}")
    if args.publisher_qos_yaml:
        qos_yaml = args.publisher_qos_yaml.resolve()
        if not qos_yaml.is_file():
            raise DiagnosticError(f"publisher QoS YAML does not exist: {qos_yaml}")
    else:
        qos_yaml = None

    topics = (args.cloud_topic, args.odom_topic, args.pgo_topic)
    source_inventory = inspect_bag(source_bag, topics[:2])
    bag_duration = bag_content_duration_seconds(source_bag)
    output_dir.mkdir(parents=True, exist_ok=False)
    write_json(output_dir / "source_sqlite_stamps.json", source_inventory)

    recorded_bag = output_dir / "recorded"
    witness_path = output_dir / "witness.jsonl"
    script_dir = Path(__file__).resolve().parent
    backend = (
        shlex.split(args.backend_command)
        if args.backend_command
        else [
            "ros2", "launch", "simple_loop_closure", "simple_loop_closure.launch.py",
            f"cloud_topic:={args.cloud_topic}", f"odometry_topic:={args.odom_topic}",
        ]
    )
    if not backend:
        raise DiagnosticError("backend command cannot be empty")
    witness = [
        sys.executable, str(script_dir / "slc_transport_witness.py"),
        "--output", str(witness_path),
        "--cloud-topic", args.cloud_topic,
        "--odom-topic", args.odom_topic,
        "--pgo-topic", args.pgo_topic,
        "--qos-profile", args.subscriber_qos_profile,
    ]
    recorder = [
        "ros2", "bag", "record", "--storage", "sqlite3", "--output", str(recorded_bag),
        args.cloud_topic, args.odom_topic, args.pgo_topic,
    ]
    playback = ["ros2", "bag", "play", str(source_bag), "--rate", "1.0"]
    if qos_yaml:
        playback.extend(["--qos-profile-overrides-path", str(qos_yaml)])

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(args.domain_id)
    manifest = {
        "ros_domain_id": args.domain_id,
        "source_bag": str(source_bag),
        "bag_content_duration_seconds": bag_duration,
        "output_dir": str(output_dir),
        "topics": {"cloud": args.cloud_topic, "odometry": args.odom_topic, "pgo_odometry": args.pgo_topic},
        "subscriber_qos_profile": args.subscriber_qos_profile,
        "subscriber_qos_definitions": {
            "sensor-data": "keep_last(10), best_effort, volatile",
            "reliable": "keep_last(100), reliable, volatile",
        },
        "publisher_qos": "rosbag2 default" if qos_yaml is None else {"yaml": str(qos_yaml)},
        "result_policy": {
            "fail_closed": True,
            "aborted_replay_pass_eligible": False,
            "partial_audit_requires_readable_artifacts": True,
        },
        "timings_seconds": {
            "playback_rate": 1.0,
            "startup": args.startup_sec,
            "endpoint_snapshot_delay": args.endpoint_snapshot_delay_sec,
            "drain": args.drain_sec,
            "shutdown_timeout": args.shutdown_timeout_sec,
            "playback_timeout": args.playback_timeout_sec,
        },
        "commands": {"witness": witness, "backend": backend, "recorder": recorder, "playback": playback},
        "constraints": [
            "no kernel or sysctl changes", "no DDS configuration changes",
            "no implicit publisher QoS override", "subscriber QoS selected by explicit test profile",
        ],
    }
    write_json(output_dir / "manifest.json", manifest)
    udp_before = read_udp_counters()
    write_json(output_dir / "udp_before.json", udp_before)

    processes: list[tuple[str, subprocess.Popen[bytes], object]] = []
    status: dict[str, object] = {
        "completed": False,
        "outcome": "running",
        "playback": {
            "completed": False,
            "timed_out": False,
            "timeout_seconds": args.playback_timeout_sec,
            "bag_content_duration_seconds": bag_duration,
            "elapsed_monotonic_seconds": None,
            "effective_throughput_rate": None,
        },
    }
    primary_error: BaseException | None = None
    primary_traceback = None
    cleanup_errors: list[str] = []
    try:
        for name, command in (("witness", witness), ("recorder", recorder), ("backend", backend)):
            process, stream = start_logged(command, output_dir / f"{name}.log", env)
            processes.append((name, process, stream))
        time.sleep(args.startup_sec)
        failed = [(name, process.returncode) for name, process, _ in processes if process.poll() is not None]
        if failed:
            raise DiagnosticError(f"process exited during startup: {failed}")

        snapshot_endpoints(topics, output_dir / "endpoints_before_playback.json", env)
        playback_started = time.monotonic()
        play_process, play_stream = start_logged(playback, output_dir / "playback.log", env)
        processes.append(("playback", play_process, play_stream))
        time.sleep(args.endpoint_snapshot_delay_sec)
        snapshot_endpoints(topics, output_dir / "endpoints_during_playback.json", env)
        playback_returncode, playback_elapsed = wait_for_playback(
            play_process, args.playback_timeout_sec, playback_started
        )
        status["playback"]["elapsed_monotonic_seconds"] = playback_elapsed
        if playback_returncode != 0:
            raise DiagnosticError(f"rosbag playback failed with status {playback_returncode}")
        status["playback"]["completed"] = True
        if bag_duration is not None and playback_elapsed > 0.0:
            status["playback"]["effective_throughput_rate"] = bag_duration / playback_elapsed
        time.sleep(args.drain_sec)
    except BaseException as exc:
        if isinstance(exc, PlaybackTimeoutError):
            status["playback"]["timed_out"] = True
            status["playback"]["elapsed_monotonic_seconds"] = exc.elapsed_seconds
        status["error"] = f"{type(exc).__name__}: {exc}"
        primary_error = exc
        primary_traceback = exc.__traceback__
    finally:
        for name, process, _stream in reversed(processes):
            try:
                stop_process(process, args.shutdown_timeout_sec)
            except Exception as exc:
                cleanup_errors.append(f"{name}: {type(exc).__name__}: {exc}")
        for name, process, stream in processes:
            try:
                stream.close()
            except Exception as exc:
                cleanup_errors.append(f"{name} log: {type(exc).__name__}: {exc}")
            status.setdefault("returncodes", {})[name] = process.returncode
        if cleanup_errors:
            status["cleanup_errors"] = cleanup_errors

    try:
        udp_after = read_udp_counters()
        write_json(output_dir / "udp_after.json", udp_after)
        write_json(output_dir / "udp_delta.json", udp_counter_delta(udp_before, udp_after))
    except Exception as exc:
        evidence_error = f"{type(exc).__name__}: {exc}"
        status["evidence_error"] = evidence_error
        if primary_error is None:
            primary_error = DiagnosticError(f"failed to preserve UDP evidence: {evidence_error}")
            primary_traceback = primary_error.__traceback__

    run_complete_for_audit = (
        primary_error is None
        and not cleanup_errors
        and bool(status["playback"]["completed"])
    )
    run_evidence = {
        "audit_scope": "complete" if run_complete_for_audit else "partial",
        "replay_completed": bool(status["playback"]["completed"]),
        "pass_eligible": run_complete_for_audit,
    }
    try:
        report = audit_artifacts(
            source_bag, recorded_bag, witness_path, output_dir / "loss_report.json",
            args.cloud_topic, args.odom_topic, args.pgo_topic,
            run_evidence=run_evidence,
        )
        status["audit"] = {
            "completed": True,
            "scope": run_evidence["audit_scope"],
            "pass_eligible": run_evidence["pass_eligible"],
        }
    except Exception as exc:
        audit_error = f"{type(exc).__name__}: {exc}"
        status["audit"] = {"completed": False, "error": audit_error}
        if primary_error is None and not cleanup_errors:
            primary_error = DiagnosticError(f"artifact audit failed: {audit_error}")
            primary_traceback = primary_error.__traceback__

    if primary_error is None and cleanup_errors:
        primary_error = DiagnosticError(
            "managed child cleanup failed: " + "; ".join(cleanup_errors)
        )
        primary_traceback = primary_error.__traceback__

    status["completed"] = primary_error is None
    status["outcome"] = "completed" if status["completed"] else "failed"
    write_json(output_dir / "run_status.json", status)
    if primary_error is not None:
        raise primary_error.with_traceback(primary_traceback)
    return 0


def main() -> int:
    args = parser().parse_args()
    try:
        if args.action == "audit":
            if args.report.exists():
                raise DiagnosticError(f"refusing to overwrite existing report: {args.report}")
            audit_artifacts(
                args.source_bag.resolve(), args.recorded_bag.resolve(), args.witness_jsonl.resolve(),
                args.report.resolve(), args.cloud_topic, args.odom_topic, args.pgo_topic,
            )
            return 0
        return run_diagnostic(args)
    except (DiagnosticError, OSError, subprocess.SubprocessError) as exc:
        print(f"transport diagnostic error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
