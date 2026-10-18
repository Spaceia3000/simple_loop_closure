# F1 independent transport diagnostic

This is an experimental transport audit for `simple_loop_closure`. It does not
edit the backend, tune its parameters, change the kernel or sysctls, or install
a Cyclone DDS configuration. Do not run a full dataset without supervisor
approval; use a tiny synthetic or short pilot bag first.

The diagnostic keeps four evidence paths separate:

| Evidence | Source | What a missing header stamp suggests |
|---|---|---|
| source input | read-only SQL query of source `.db3` rows | baseline, including SQLite storage and message-header stamps |
| input witness | independent Python cloud/odometry subscriber | input delivery/QoS/transport loss before the backend |
| backend witness | the same independent process subscribing to `/pgo_odom` | a backend-output gap after both inputs reached the witness |
| recorder | read-only SQL query of the newly recorded `.db3` rows | recorder-side input or output loss |

Header stamps are compared as multisets, so duplicate stamps remain visible.
SQLite storage stamps and subscriber monotonic receive times are retained as
separate fields and are not treated as interchangeable clocks.

## Reproducible invocation

Source the same ROS 2 environment and workspace overlay used for the backend.
Choose an unused domain explicitly in the range 0 through 190. The output path
must not exist; the wrapper refuses to overwrite it. The only supported replay
rate is explicitly `1.0`.

```bash
python3 scripts/run_slc_transport_diagnostic.py run \
  --bag /absolute/path/to/tiny_source_bag \
  --output-dir /absolute/path/to/new_f1_output \
  --domain-id 166 \
  --playback-rate 1.0 \
  --subscriber-qos-profile sensor-data \
  --startup-sec 3.0 \
  --endpoint-snapshot-delay-sec 0.5 \
  --drain-sec 2.0 \
  --shutdown-timeout-sec 10.0 \
  --playback-timeout-sec 360.0 \
  --require-backend-audits
```

`--playback-timeout-sec` is optional and has no implicit default. Supply a
finite positive value to bound playback wall time explicitly; omitting it
retains an unbounded playback wait for compatibility with prior experiments.

`--require-backend-audits` is also opt-in so generic fixture backends retain
their previous behavior. Enable it for native SLC graph-certification runs.
After bounded child shutdown and log closure, the runner requires exactly one
`SLC_DETERMINISM_AUDIT status=PASS` line and exactly one
`SC_SIMPLE_STREAM_AUDIT report=shutdown status=PASS` line. Missing, duplicate,
malformed, `FAIL`, `INCONCLUSIVE`, or other non-PASS audits fail closed.

The stream shutdown audit must also account for the complete source bag:
cloud and odometry ingress, synchronized callbacks, accepted contracts, and
attempted/completed PGO publishes must equal the corresponding source count;
rejected contracts and failed publishes must be zero. Native sync and PGO
accounting flags must be true. The parsed result and explicit failure reasons
are recorded under `backend_validation` in `run_status.json`. A launch-wrapper
return code is retained as evidence but is never a substitute for these node
audits.

The default backend command is:

```text
ros2 launch simple_loop_closure simple_loop_closure.launch.py cloud_topic:=/cloud_registered_metric odometry_topic:=/aft_mapped_to_init
```

To run a known installed executable or launch file instead, pass one argv
string with `--backend-command`. It is parsed with `shlex` and executed without
a shell. The manifest records the resulting argv exactly.

The replay publisher uses rosbag2 defaults. A publisher override is applied
only when a YAML file is explicitly supplied:

```bash
  --publisher-qos-yaml /absolute/path/to/reviewed_qos.yaml
```

There is no automatic QoS discovery or parameter tuning. The witness subscriber
profile is independently and explicitly selected for the experiment:

- `sensor-data`: keep-last depth 10, best-effort, volatile
- `reliable`: keep-last depth 100, reliable, volatile

Run separate output directories to compare those profiles. Do not change a
profile during a run or infer that its result is the publisher's QoS.

## Artifacts

The exclusive output directory contains:

- `manifest.json`: domain, topics, exact commands, QoS selection, and all fixed timings
- `source_sqlite_stamps.json`: source storage/header stamps read before replay
- `witness.jsonl`: topic, header stamp, sequence, and monotonic receive time
- `recorded/`: new SQLite rosbag containing both inputs and `/pgo_odom`
- `loss_report.json`: labeled comparisons plus explicit header/storage/receive rates
- `endpoints_*.json`: verbose ROS endpoint snapshots before and during playback
- `udp_before.json`, `udp_after.json`, and `udp_delta.json`: read-only host UDP MIB evidence
- one log per managed process and fail-closed `run_status.json`, including
  monotonic playback elapsed time, metadata duration when readable, and a
  completion-only effective throughput ratio; with
  `--require-backend-audits`, it also includes native backend validation status

An audit can be rebuilt without replay:

```bash
python3 scripts/run_slc_transport_diagnostic.py audit \
  --source-bag /absolute/path/to/tiny_source_bag \
  --recorded-bag /absolute/path/to/new_f1_output/recorded \
  --witness-jsonl /absolute/path/to/new_f1_output/witness.jsonl \
  --report /absolute/path/to/new_f1_output/loss_report_recheck.json
```

## Interpretation limits

The witness and rosbag recorder are independent subscribers and can observe
different delivery under different QoS behavior. A `/pgo_odom` gap is therefore
classified only after cloud and odometry with the same header stamp were both
seen by the witness; it is diagnostic evidence, not by itself proof of a native
backend defect. Endpoint snapshots give the offered/requested QoS needed to
interpret that evidence.

Linux UDP MIB values are host-wide, cumulative counters. Their delta can show
that UDP errors changed during the bounded run, but cannot attribute an error
to a ROS node or DDS participant. Very short bags can finish before the second
endpoint snapshot completes; the exact snapshot delay remains in the manifest.
Only rosbag2 SQLite (`.db3`) storage and messages with a ROS `header.stamp` are
supported. No MCAP fallback is silently selected.

On an aborted run, the wrapper first performs bounded cleanup of its exact
children and then attempts a partial audit. It writes `loss_report.json` only
when the SQLite and witness artifacts are readable, and marks that report
`pass_eligible=false`; an aborted replay is never a passing diagnostic.
The same partial/ineligible rule applies when replay and artifact collection
finish but required backend audit validation fails. Thus a wrapper `rc=0`, or
even complete source/witness/recorder counts, cannot certify a backend whose
native shutdown audit is missing or incomplete.

## Small tests

The tests create a tiny mock rosbag2-style SQLite database; they do not start
ROS, DDS, the backend, or a real dataset.

```bash
python3 -m py_compile \
  scripts/transport_diagnostic_common.py \
  scripts/slc_transport_witness.py \
  scripts/run_slc_transport_diagnostic.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test/test_transport_diagnostic.py
```
