#!/usr/bin/env python3
"""Independent lightweight cloud/odometry subscriber for transport diagnosis."""

from __future__ import annotations

import argparse
import json
import signal
import struct
import time
from pathlib import Path
from typing import Callable, TextIO


CDR_HEADER_SIZE = 4
ROS_HEADER_STAMP_SIZE = 8
CDR_ENDIAN = {
    b"\x00\x00\x00\x00": ">",
    b"\x00\x01\x00\x00": "<",
}


def raw_cdr_header_stamp_ns(payload: bytes | bytearray | memoryview) -> int:
    """Read the leading std_msgs/Header stamp from a ROS 2 CDR payload."""
    view = memoryview(payload)
    required_size = CDR_HEADER_SIZE + ROS_HEADER_STAMP_SIZE
    if view.nbytes < required_size:
        raise ValueError(
            f"raw CDR message is {view.nbytes} bytes; at least {required_size} are required"
        )
    encapsulation = bytes(view[:CDR_HEADER_SIZE])
    try:
        endian = CDR_ENDIAN[encapsulation]
    except KeyError as exc:
        raise ValueError(f"unsupported CDR encapsulation: {encapsulation.hex()}") from exc
    sec, nanosec = struct.unpack_from(f"{endian}iI", view, CDR_HEADER_SIZE)
    if nanosec >= 1_000_000_000:
        raise ValueError(f"invalid Header nanosec value: {nanosec}")
    return sec * 1_000_000_000 + nanosec


def deserialized_header_stamp_ns(message: object) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def make_observer(
    topic: str,
    stream: TextIO,
    sequence: dict[str, int],
    stamp_decoder: Callable[[object], int],
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
):
    """Build the minimal callback shared by deserialized and raw subscriptions."""

    def callback(message: object) -> None:
        header_ns = stamp_decoder(message)
        received_ns = monotonic_ns()
        index = sequence.get(topic, 0)
        sequence[topic] = index + 1
        stream.write(
            json.dumps(
                {
                    "topic": topic,
                    "sequence": index,
                    "header_ns": header_ns,
                    "receive_monotonic_ns": received_ns,
                },
                sort_keys=True,
            )
            + "\n"
        )

    return callback


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cloud-topic", required=True)
    parser.add_argument("--odom-topic", required=True)
    parser.add_argument("--pgo-topic", required=True)
    parser.add_argument(
        "--qos-profile",
        choices=("sensor-data", "reliable"),
        required=True,
        help="explicit experimental subscriber profile; never inferred from publishers",
    )
    parser.add_argument(
        "--raw-cdr-stamps",
        action="store_true",
        help="extract Header stamps directly from raw CDR instead of deserializing messages",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2
    except ImportError as exc:
        raise SystemExit(f"ROS 2 Python packages are required: {exc}")

    depth = 10 if args.qos_profile == "sensor-data" else 100
    reliability = (
        ReliabilityPolicy.BEST_EFFORT
        if args.qos_profile == "sensor-data"
        else ReliabilityPolicy.RELIABLE
    )
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
    )

    rclpy.init()
    node = Node("slc_transport_witness")
    stream = args.output.open("x", encoding="utf-8", buffering=1)
    sequence: dict[str, int] = {}

    stamp_decoder = (
        raw_cdr_header_stamp_ns
        if args.raw_cdr_stamps
        else deserialized_header_stamp_ns
    )

    def subscribe(message_type: object, topic: str):
        subscription_args = (
            message_type,
            topic,
            make_observer(topic, stream, sequence, stamp_decoder),
            qos,
        )
        if args.raw_cdr_stamps:
            return node.create_subscription(*subscription_args, raw=True)
        return node.create_subscription(*subscription_args)

    subscriptions = [
        subscribe(PointCloud2, args.cloud_topic),
        subscribe(Odometry, args.odom_topic),
        subscribe(Odometry, args.pgo_topic),
    ]

    stopping = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        subscriptions.clear()
        stream.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
