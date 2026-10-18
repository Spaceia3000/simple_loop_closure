# SLC F1 playback watchdog — 2026-10-18

## Scope and motivating evidence

This change is limited to `scripts/run_slc_transport_diagnostic.py` and its
mocked tests. It does not change SimpleLoopClosure, FAST-LIVO2, DDS, kernel
settings, rosbag publisher QoS, or the witness subscriber QoS definitions.

The motivating CBD03 LIO ExactTime128 observations were:

- the rosbag2 LiDAR publisher using default reliable keep-last depth 10 remained
  in rosbag playback for more than seven minutes for 265.9 seconds of bag
  content at rate 1.0, emitted repeated CycloneDDS
  `ddsi_udp_conn_write retcode -1` messages, and ended with
  `run_status.completed=false`;
- a separate reliable keep-last depth 100 run completed with exact 2660/2660
  header stamps at all audited frontiers.

Those observations motivate a bounded diagnostic runner. They do not, by
themselves, identify a DDS, kernel, or backend root cause.

## Explicit watchdog contract

Playback remains unbounded unless the operator explicitly supplies the new
finite positive option:

```bash
--playback-timeout-sec 360
```

Zero, negative, NaN, and infinite values are rejected by argument parsing. The
configured value is written to `manifest.json`. When configured, the runner
waits only for the exact `ros2 bag play` child, using the remaining portion of
the wall-clock budget. Expiry raises `PlaybackTimeoutError` (reported by the
CLI as a diagnostic error), and includes both the configured limit and measured
monotonic elapsed seconds.

Cleanup addresses only the exact `Popen` children created by the runner. Each
still-running child receives SIGINT, then `terminate()`, then `kill()` if
needed. Every wait, including the post-kill wait, is bounded by
`--shutdown-timeout-sec`; there is no `pkill`, name match, or process-tree-wide
signal.

## Evidence and fail-closed interpretation

`run_status.json` now records a `playback` object containing:

- whether playback returned status zero;
- whether the configured watchdog expired;
- configured timeout and monotonic elapsed playback wall time;
- bag content duration, when the standard rosbag2 metadata duration is
  readable;
- effective throughput rate, defined as bag content seconds divided by
  monotonic playback wall seconds.

The throughput value is emitted only after playback returns status zero and
both inputs to the ratio are available and positive. It is not inferred for a
timeout, signal-driven shutdown, or nonzero playback exit.

After any run failure, cleanup and log closure happen before a partial audit is
attempted. A partial `loss_report.json` is written only if the recorded SQLite
bag, source SQLite bag, and witness JSONL can actually be read. Its
`run_evidence` marks `audit_scope=partial`, `replay_completed=false` when
applicable, and `pass_eligible=false`. Failure to salvage an audit is recorded
in `run_status.json` and does not replace the original playback error.

`run_status.completed` becomes true only when playback, managed-child cleanup,
UDP evidence capture, and the artifact audit all succeed. Thus an aborted
replay cannot be reported as a passing diagnostic even if its partial stamp
counts happen to match.

## Verification

No real bag, ROS node, DDS participant, or backend was started. The focused
tests use deterministic process doubles for:

- configured playback timeout and precise error text;
- successful bounded playback completion and elapsed time;
- SIGINT/terminate/kill escalation with bounded waits;
- failure after a bounded post-SIGKILL wait;
- timeout-run manifest/status output and partial-audit pass ineligibility.

Run the focused checks with:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test/test_transport_diagnostic.py
git diff --check
```
