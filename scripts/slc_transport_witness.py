#!/usr/bin/env python3
"""Independent lightweight cloud/odometry subscriber for transport diagnosis."""

from __future__ import annotations

import argparse
import json
import signal
import time
from pathlib import Path


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

    def observe(topic: str):
        def callback(message: object) -> None:
            stamp = message.header.stamp
            index = sequence.get(topic, 0)
            sequence[topic] = index + 1
            stream.write(
                json.dumps(
                    {
                        "topic": topic,
                        "sequence": index,
                        "header_ns": int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec),
                        "receive_monotonic_ns": time.monotonic_ns(),
                    },
                    sort_keys=True,
                )
                + "\n"
            )

        return callback

    subscriptions = [
        node.create_subscription(PointCloud2, args.cloud_topic, observe(args.cloud_topic), qos),
        node.create_subscription(Odometry, args.odom_topic, observe(args.odom_topic), qos),
        node.create_subscription(Odometry, args.pgo_topic, observe(args.pgo_topic), qos),
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
