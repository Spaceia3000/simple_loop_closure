# S2 `/cloud` reader QoS experiment — 2026-10-18

## Scope and provenance

This isolated change starts from commit
`eaccc4328bb951f43ca6fcdd98c93885c28feec5` and changes only the SimpleLoopClosure
`/cloud` subscription's RMW receiver history, plus its validation, audit label,
tests, parameter documentation, and this experiment protocol. It does not change
FAST-LIVO2, `/odometry` QoS, S1 `ExactTime(128)`, S3 `/pgo_odom` history, input
timestamps, input contracts, keyframe selection, ICP, GTSAM, loop scheduling, or
published message contents.

The inspected ROS 2 Humble installation contains `ros-humble-message-filters`
`4.3.20-1jammy.20260907.223809` and `ros-humble-rmw`
`6.1.4-1jammy.20260907.194822`. In
`/opt/ros/humble/include/message_filters/message_filters/subscriber.h`, the
`Subscriber::subscribe(NodeType *, const std::string &, rmw_qos_profile_t)`
overload passes the supplied profile to `create_subscription`. In
`/opt/ros/humble/include/rmw/rmw/qos_profiles.h`,
`rmw_qos_profile_default` is reliable `KEEP_LAST(10)`; the profile `depth` field
is `size_t` in `rmw/types.h`.

The new signed integer parameter is `cloud_input_depth`. Its code and checked-in
configuration default is 10, preserving the prior implicit Humble default.
Values outside `[1, 1024]` fail during initialization before conversion or
subscription creation. The upper bound is only an operational resource cap; it
is not an optimum, a delivery guarantee, or a tuning conclusion.

After validation, the node copies `rmw_qos_profile_default`, explicitly sets
`depth=cloud_input_depth`, `history=KEEP_LAST`, and `reliability=RELIABLE`, and
passes that copy only to `sub_cloud_`. The odometry subscriber continues to use
`sub_odom_.subscribe(this, "/odometry")`, so its default QoS is unchanged.

The stream audit reports
`cloud_subscriber_qos=<reliability>_<history>_<depth>` from the live
subscription's `get_actual_qos()`. This is endpoint configuration evidence, not
evidence that DDS delivered every source sample. Audit stage counts and status
logic are unchanged; the report schema version advances from 7 to 8.

## Queue distinction and invariants

The two relevant queues have different boundaries:

1. `cloud_input_depth` is the RMW `/cloud` reader's `KEEP_LAST` capacity. It
   bounds cloud samples retained by middleware while they wait to enter the
   subscription callback.
2. `sync_queue_size=128` is the native `message_filters::ExactTime` tuple-map
   capacity after callback ingress. It stores unmatched cloud/odometry stamps.

Increasing the second queue cannot recover a cloud already discarded at the
RMW reader. Increasing the first queue cannot recover a tuple later evicted by
ExactTime. The treatment therefore changes only the first boundary.

Correctness conditions are:

- depth 10 reproduces the former reliable `KEEP_LAST(10)` cloud subscription;
- validation occurs on signed 64-bit input before bounded unsigned conversion,
  rejecting negative values, zero, values above 1024, and signed extrema;
- only `/cloud` receives the constructed profile; `/odometry` remains unchanged;
- S1 remains native exact-stamp synchronization with queue 128;
- S3 remains reliable `KEEP_LAST(1)` for `/pgo_odom` in the checked-in config;
- audit reporting never converts internal consistency into a claim of source,
  DDS, witness, recorder, or finalized-bag completeness.

## Supplied full-run observation

The pre-change run
`slc_s2_recorder100_directnode_full_repeat2_20261018` is evidence motivating the
experiment, not an S2 result. The supplied counts are:

- source bag: 2660 cloud and 2660 odometry;
- independent witness: 2660 cloud and 2660 odometry;
- finalized recorder bag: 2660 cloud and 2660 odometry, with reliable
  `KEEP_LAST(100)` cloud writer and reliable `KEEP_LAST(100)` recorder reader;
- SimpleLoopClosure ingress: 2630 cloud and 2660 odometry;
- ExactTime callbacks and PGO outputs: 2630 each;
- missing cloud indices: `1305..1321`, `1324..1333`, `1578`, `1579`, and `2659`
  (30 total).

Those observations localize the missing samples no later than the SLC cloud
ingress tap under that run's instrumentation, but do not by themselves prove
that the depth-10 RMW queue was the cause.

## Controlled A/B protocol

Run exactly two treatments against the same fixed bag:

| Treatment | `cloud_input_depth` | All other settings |
|---|---:|---|
| A | 10 | fixed as specified below |
| B | 100 | identical to A |

For both treatments fix the cloud writer to reliable `KEEP_LAST(100)`, the
rosbag2 recorder reader to reliable `KEEP_LAST(100)`, and the independent
observer reader to reliable `KEEP_LAST(100)`. Keep S1 at native
`ExactTime(128)`, S3 at `pgo_output_depth=10` for this experiment, localhost
transport, playback rate 1.0x, the same RMW implementation, storage plugin,
process startup order, CPU/load conditions, topic remappings, audit setting,
commands, drain rule, and shutdown order. Use fresh output bags and log
directories. Record treatment order and repeat trials in both orders if
practical; do not tune any other parameter between A and B.

For every run retain native-backend evidence at each boundary:

- source bag cloud/odometry counts and exact ordered timestamp digests;
- online writer-side and independent observer counts/digests;
- SLC `cloud_ingress`, `odom_ingress`, `sync_callback`, contract, keyframe, and
  PGO audit counts, first/last stamps, duplicate/reversal counters, and ordered
  FNV-64 digests;
- audited `cloud_subscriber_qos` and `pgo_publisher_qos`;
- recorder-observed and independently re-read finalized-bag counts/digests;
- RMW implementation/version, rosbag2/storage versions, commands, host-load
  notes, and wall-clock timing for last source, last observer, SLC shutdown
  audit, recorder stop request, recorder exit, and finalized-bag read.

After the last source stamp, wait for the observer and SLC audit boundary to
reach their terminal stamps and apply the same bounded quiet/drain interval in
both treatments. Stop the recorder normally, wait for process exit and storage
finalization, then inspect the completed bag with an independent reader.

## Decision rule and no-false-PASS policy

Compare exact ordered stamp sequences, not only counts. A treatment is
`INCONCLUSIVE` if source, observer, shutdown, or finalized-bag evidence is
missing; if the audit is a live snapshot; if the endpoint QoS does not match the
intended treatment; or if drain completion is not demonstrated. Internal audit
`PASS` means only internal stage accounting and must never be relabeled as
zero-loss delivery.

Depth 100 may be considered an improvement only if repeated controlled trials
show a corresponding reduction in cloud-ingress omissions without sequence
corruption and without a conflicting witness/finalized-bag boundary. Even a
2660/2660 result does not prove zero loss in general, identify depth as the sole
cause, establish an optimum, or justify other workloads. Equal A/B results do
not prove that receiver depth is irrelevant outside the tested conditions.

No full rosbag replay is performed as part of this implementation task. The
table remains a protocol until the supervisor supplies controlled runtime
evidence; no treatment is marked PASS here.

## Bounded isolated validation

All artifacts are under `.codex_native/s2_input/`; no shared build, install, or
log directory was used.

- Build command: `timeout 600s colcon --log-base
  .codex_native/s2_input/log build --base-paths . --build-base
  .codex_native/s2_input/build --install-base
  .codex_native/s2_input/install --event-handlers console_cohesion+
  --cmake-args -DBUILD_TESTING=ON` after sourcing Humble. One package finished
  in 1m30s. The node and six native test executables built successfully with
  the configured `-Wall -Wextra -Wpedantic`; no compiler warning was emitted.
  Log: `.codex_native/s2_input/log/build_2026-10-18_16-06-29/`.
- Test command: `timeout 300s colcon --log-base
  .codex_native/s2_input/test_log test --base-paths . --build-base
  .codex_native/s2_input/build --install-base
  .codex_native/s2_input/install --event-handlers console_cohesion+
  --return-code-on-test-failure` after sourcing Humble and the private install.
  The final full rerun finished in 1m02s and all 10/10 CTest targets passed.
  Their result XML contains 93 framework test
  cases; `colcon test-result` reports 103 aggregate records when the 10 CTest
  wrapper records are included: 0 errors, 0 failures, 0 skipped. The launch
  contract passed 14/14, the keyframe/math/parameter target passed 18/18, and
  the stream-audit target passed 10/10. Log:
  `.codex_native/s2_input/test_log_final/test_2026-10-18_16-10-21/`.
- `git diff --check`: clean after the final documentation edit.

These local tests establish validation and wiring contracts only; they cannot
establish end-to-end loss behavior.
