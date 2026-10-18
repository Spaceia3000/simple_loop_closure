# LIVO DDS telemetry integration — 2026-10-18

## Integrated scope and invariants

This feature worktree starts at integration candidate `4b78dfe` and contains
the exact nine-file change from optional telemetry commit `ee4477e`, plus this
integration record, an inert rosbag2 depth-100 experiment profile, and explicit
`noexcept` annotations on the fixed-cost telemetry callbacks. The linked
worktree Git administration directory was read-only, so `git cherry-pick
--no-commit ee4477e` could not create `index.lock`; the commit patch was applied
to the writable worktree without staging. Before the integration-only changes,
all nine resulting files byte-matched `ee4477e`.

The C1 gate remains strict: `--certify-all` still requires the native
determinism and shutdown stream audits, exact ordered source-to-witness and
source-to-recorder integrity, and exact full-source-to-PGO coverage. No frame
count was added to C1. The command below supplies the independent CBD03 LIVO
expectation of 2,377 only through the existing `--expected-source-count` gate.

Production behavior remains unchanged. Both the C++ declaration and
`config/simple_loop_closure.yaml` default `cloud_input_telemetry=false`.
With that default, the original two subscription calls execute, no telemetry
object or taps are created, and no `SLC_C1_RUNTIME_TELEMETRY` line is emitted.
The production defaults also remain `cloud_input_depth=10`,
`sync_queue_size=128`, and `pgo_output_depth=1`. The depth-100 YAML in this
change is never loaded implicitly.

## Humble API and hot-path safety review

The installed ROS 2 Humble headers expose the used four-argument
`message_filters::Subscriber::subscribe(node, topic, rmw_qos_profile_t,
rclcpp::SubscriptionOptions)` overload. Humble defines the callback payloads as
`rclcpp::QOSMessageLostInfo` and
`rclcpp::QOSRequestedIncompatibleQoSInfo`. If either requested event cannot be
created, `rclcpp::UnsupportedEventTypeException` propagates out of subscription
construction; this integration catches it once per input, warns once during
initialization, and retries that input with its original subscription call and
no requested event callbacks. Other subscription failures are not swallowed.

The ingress and event callbacks are explicitly `noexcept`. Their bodies use
only `steady_clock::now()`, a weak-pointer lock, and fixed-size relaxed atomic
loads/exchanges/additions/maximum updates. They do not log, allocate a
timestamp container, copy a ROS message, take a telemetry mutex, perform I/O,
or format the report. Formatting and the single output line occur only during
staged shutdown. The synchronized-callback timer likewise uses fixed-size
relaxed atomics and includes all early returns.

Humble `message_filters::Signal1` already holds its per-subscriber signal mutex
while invoking every registered tap. Enabling telemetry adds its bounded
callback duration to that existing critical section; it does not introduce a
new user mutex. Native `ExactTime` separately holds its policy mutex while
calling the synchronized backend callback, so `sync_service_peak_ns` measures
the local blocking exposure of that callback. It does not prove why an earlier
sample was absent. The node uses a single-threaded executor, and staged
shutdown stops spinning before disconnecting taps and taking the final
snapshot.

`message_lost` is RMW-reported evidence only. A zero value is not proof that no
sample was dropped, overwritten, or never delivered. The unsupported-event
warning means only that native event counters are unavailable for that input;
arrival and service telemetry remain active. Incompatible-QoS events and event
support must be read from the shutdown line, not inferred from a quiet log.

## Precise loss-localization rules

The prior CBD03 LIVO observation remains a failure, not a certification:

- source cloud/odometry: 2,377 / 2,377;
- backend cloud/odometry and ExactTime: 2,341 / 2,377 and 2,341;
- independent witness cloud: 2,309 / 2,377;
- recorder cloud: 2,305 / 2,377;
- maximum observed witness excess/arrival lag: about 11.5 seconds.

Those independent readers are not a serial pipeline: witness 2,309 and
recorder 2,305 cannot be used to claim that the backend's 36 missing clouds
were caused by either reader. Host-wide UDP counter changes are likewise not
causal attribution to a ROS node, DDS participant, topic, or sample.

For a controlled rerun, localize only to measured boundaries:

| Observation | Defensible localization | Not established |
|---|---|---|
| source has 2,377, `cloud_callback_count` is lower | absent no later than the backend pre-ExactTime callback | publisher, DDS, RMW queue, or executor as the specific cause |
| both ingress callback counts are 2,377, sync count is lower | between ingress taps and ExactTime emission | which ExactTime eviction/purge or scheduling event caused it without stamp evidence |
| sync/accepted count is 2,377, PGO attempt/completion is lower | backend contract/publish boundary identified by the native audit | external reader loss |
| backend is complete but witness or recorder is lower | loss is confined to that independent external observation path | loss at the backend subscription |

Only exact ordered stamp comparisons qualify. Counts, event totals,
interarrival peaks, peer age, and the 11.5-second observation are diagnostic
clues and never replace C1.

## Exact supervised CBD03 LIVO verification command

Do not run this while the integration is uncommitted: the diagnostic manifest
records `HEAD`, which is still `4b78dfe` until the supervisor creates the final
feature commit. After that commit, require a clean tree and a fresh output path.
This command pins localhost-only CycloneDDS, cloud writer depth 100, recorder
cloud reader depth 100, independent witness depth 100, backend cloud reader
depth 100, native ExactTime depth 128, PGO publisher depth 10, active optional
telemetry, native stream audit, and strict `--certify-all` with the independent
2,377-frame expectation:

```bash
cd /home/breidy/Documents/simple_loop_closure_ws/worktrees/slc_livo_dds_integration_20261018
test "$(git branch --show-current)" = feat/slc-livo-dds-telemetry-20261018
git diff --exit-code
git diff --cached --exit-code
source /opt/ros/humble/setup.bash
source "$PWD/.codex_native/livo_dds/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=184
export ROS_LOG_DIR="$PWD/.codex_native/livo_dds/cbd03_livo_cert_ros_logs"
mkdir -p "$ROS_LOG_DIR"
python3 scripts/run_slc_transport_diagnostic.py run \
  --bag /home/breidy/Documents/fast_livo2_rtabmap_lc_ws/mvp1_runs/cbd03_livo_jetson_full_normalized \
  --output-dir "$PWD/.codex_native/livo_dds/cbd03_livo_dds_telemetry_cert_r1" \
  --domain-id 184 \
  --playback-rate 1.0 \
  --subscriber-qos-profile reliable \
  --publisher-qos-yaml "$PWD/config/cbd03_livo_cloud_depth100_qos.yaml" \
  --recorder-qos-yaml "$PWD/config/cbd03_livo_cloud_depth100_qos.yaml" \
  --startup-sec 3.0 \
  --endpoint-snapshot-delay-sec 0.5 \
  --drain-sec 20.0 \
  --shutdown-timeout-sec 60.0 \
  --playback-timeout-sec 600.0 \
  --expected-source-count 2377 \
  --certify-all \
  --backend-command 'ros2 run simple_loop_closure simple_loop_closure_node --ros-args --params-file /home/breidy/Documents/simple_loop_closure_ws/worktrees/slc_livo_dds_integration_20261018/config/simple_loop_closure.yaml -p simple_stream_audit:=true -p cloud_input_telemetry:=true -p cloud_input_depth:=100 -p sync_queue_size:=128 -p pgo_output_depth:=10 -r /cloud:=/cloud_registered_metric -r /odometry:=/aft_mapped_to_init'
```

Acceptance requires `run_status.json` to report a completed, pass-eligible run,
`loss_report.json` to report C1 external integrity `PASS`, exactly one passing
native determinism line, exactly one passing native shutdown stream-audit line,
and exactly one telemetry shutdown line. That telemetry line must show 2,377
cloud callbacks, 2,377 odometry callbacks, 2,377 synchronized services, the
requested effective QoS, and zero incompatible-QoS callbacks. Nonzero native
message-lost evidence is a diagnostic failure clue; zero is not an independent
proof of delivery. Repeat the same frozen treatment three times before calling
CBD03 LIVO certified. Full certification still requires the corresponding
three clean runs for every UGV10, Retail, and CBD03 LIO/LIVO cell.

## Local validation performed without a real bag

The package was built only under `.codex_native/livo_dds/` against ROS 2
Humble. All 11 CTest targets passed; `colcon test-result` reports 111 aggregate
records with 0 errors, 0 failures, and 0 skipped. This includes six ingress
telemetry tests, nine ExactTime cases, and 15 launch/source-contract cases.

No real bag was run. A CycloneDDS no-data smoke could not create a participant
because the sandbox denied interface enumeration. An otherwise identical Fast
DDS no-data smoke could construct and stop the node despite its blocked UDP
transport warnings. With the production default it emitted no telemetry line;
with the opt-in enabled it emitted exactly one zero-count shutdown line with
both event-support fields true. These smokes validate construction/default
behavior only, not transport delivery or the requested CBD03 certification.
