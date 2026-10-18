# S2 optional ingress telemetry — 2026-10-18

## Scope and invariant

This isolated change adds `cloud_input_telemetry` with code and YAML default
`false`. With the parameter disabled, the original native subscription calls
remain the executed path: `/cloud` uses the validated Reliable `KEEP_LAST`
depth and `/odometry` uses `rmw_qos_profile_default`. ExactTime construction,
exact-stamp matching, the synchronized callback body, keyframe selection, PGO
graph processing, and publisher QoS are not changed.

When enabled, Humble's native four-argument
`message_filters::Subscriber::subscribe(node, topic, qos, SubscriptionOptions)`
overload installs `message_lost` and requested-incompatible-QoS callbacks on
both input subscriptions. If the active RMW throws
`rclcpp::UnsupportedEventTypeException`, the affected subscription is retried
with its original options, a warning is emitted, and its event support field is
false. An event count of zero is not proof of loss-free transport: an RMW may
not expose every discarded or missing sample through `message_lost`.

## Runtime schema

Exactly one `SLC_C1_RUNTIME_TELEMETRY version=1 report=shutdown` line is emitted
when telemetry is enabled; no telemetry line is emitted when disabled. All hot
path state is fixed-size and uses relaxed atomics. Gap counters compile out the
service-only threshold checks. There is no per-message log, message copy,
timestamp-indexed container, or telemetry mutex.

- `cloud|odom_callback_count`: pre-synchronizer F0 callback invocations.
- `*_interarrival_{count,total_ns,peak_ns,ge_100ms,ge_500ms,ge_1s}`:
  steady-clock time between consecutive callbacks on that stream.
- `*_latest_peer_age_*`: absolute steady-clock distance to the most recently
  observed callback on the opposite stream. This is a scheduling/backlog clue,
  not the latency of the exact-stamp counterpart and not causal attribution.
- `*_event_support`: whether both requested native handlers were installed.
- `*_message_lost_callbacks`, `*_message_lost_total_change`, and
  `*_message_lost_rmw_total_last`: callback accounting and RMW-reported totals.
- `*_incompatible_qos_*`: equivalent requested-incompatible-QoS accounting,
  including the last policy kind reported by the RMW.
- `sync_service_*`: wall service duration of the full synchronized callback,
  including all early returns. Native Humble ExactTime invokes this callback
  while holding its policy mutex, so long durations identify local synchronous
  blocking exposure without proving upstream or DDS loss.

The duration thresholds are inclusive (`>=100 ms`, `>=500 ms`, `>=1 s`). The
line explicitly states `causal_attribution=not_established` and
`event_counts_semantics=rmw_reported_only_missing_samples_may_be_unreported`.

## Evidence gates

No bag playback is part of this change. Acceptance requires:

1. A private bounded build of this package on ROS 2 Humble.
2. Unit tests for threshold boundaries, event accumulation, reset-to-empty,
   latest-peer/interarrival aggregates, one-line schema, and the causal caveat.
3. Launch-contract tests proving the parameter defaults false, the unmodified
   subscription calls remain on the disabled path, and telemetry taps precede
   synchronizer construction.
4. Existing ExactTime, stream-audit, keyframe, graph, and analysis tests remain
   green.
5. A later controlled run must compare source, F0 ingress, ExactTime, PGO, and
   external witness/recorder counts. Telemetry alone must not assign loss to
   FAST-LIVO2, DDS, the executor, ExactTime, or downstream recorders.

## Local validation

- Private Humble build completed successfully with no compiler diagnostics in
  `.codex_native/s2_ingress_telemetry/`.
- The final full package run passed all 11 CTest targets: 111 aggregate test
  records, 0 errors, 0 failures, 0 skipped. This includes six telemetry tests,
  nine native ExactTime tests, and 15 launch/source contract tests.
- No bag was played. A no-data process smoke test could not reach node creation
  because the restricted sandbox prevented CycloneDDS from enumerating network
  interfaces. Consequently, runtime event support for the active RMW remains a
  controlled-run gate; the compile-time Humble API and fallback path are tested.

## Prior motivating evidence (not reproduced here)

- CBD03 LIO: full 2660/2660 ingress on all three nodes with cloud reader
  Reliable `KEEP_LAST(100)`, ExactTime(128), and PGO output depth 10; the third
  witness/recorder still lost clouds while the backend completed.
- CBD03 LIVO: source cloud/odom 2377/2377; backend cloud 2341, odom 2377,
  ExactTime 2341; witness cloud 2309; recorder cloud 2305. Backend cloud indices
  1623--1685 (source time about 177.8--184.5 s) were absent.
- Independent monotonic cloud-vs-odom arrival lag had p99 about 9.94 s and max
  about 11.5 s. These observations motivate causal telemetry rather than a
  deeper unmeasured queue; they do not identify the loss mechanism.
