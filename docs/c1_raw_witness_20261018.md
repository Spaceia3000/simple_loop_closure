# C1 raw-CDR witness experiment (2026-10-18)

## Motivation

In the third CBD03 LIVO rate-1 full run, the Python diagnostic witness reported
13 missing clouds. The SLC backend, rosbag recorder cloud stream, and PGO stream
each accounted for all 2377 inputs. The witness also showed an approximately
11.4-second cloud-versus-odometry arrival backlog while using reliable,
keep-last depth 100 subscriptions. Because each `PointCloud2` is about 1.1 MiB
and the witness used only its header stamp, full Python message deserialization
is a plausible witness-local source of delay.

This evidence identifies a witness-only discrepancy; it does not establish its
cause or show that transport was healthy in every layer.

## Optional experiment

The diagnostic runner now accepts `--witness-raw-cdr-stamps`. It forwards
`--raw-cdr-stamps` only to `slc_transport_witness.py`. The backend, publisher,
recorder, playback command, topics, QoS profile, and witness JSONL schema are
unchanged. Without the option, the previous deserialized-message path remains
the default.

With the option enabled, all witness subscriptions (`PointCloud2` and both
`Odometry` streams) use Humble `rclpy` raw subscriptions. The callback validates
the CDR encapsulation, selects its declared byte order, and reads the leading
`std_msgs/Header` stamp as signed `int32 sec` plus `uint32 nanosec` at byte
offset 4. Short data, unsupported or nonzero-option encapsulations, and
nanoseconds outside `[0, 1000000000)` raise an error rather than emitting a
stamp. The JSONL fields remain exactly `topic`, `sequence`, `header_ns`, and
`receive_monotonic_ns`.

Example:

```bash
python3 scripts/run_slc_transport_diagnostic.py run \
  --bag /path/to/source_bag \
  --output-dir /path/to/new_output \
  --domain-id 166 \
  --subscriber-qos-profile reliable \
  --witness-raw-cdr-stamps
```

## DDS packet constraints and interpretation

Raw subscription changes the Python endpoint's handling only after DDS/rmw has
delivered a serialized sample. It does not make a 1.1 MiB cloud fit in one UDP
datagram, bypass RTPS fragmentation/reassembly, enlarge kernel socket buffers,
change DDS history/resource limits, or alter reliability behavior. Packet loss,
fragment loss, retransmission, receive-buffer pressure, and implementation
resource limits therefore remain possible upstream constraints. The existing
endpoint snapshots, UDP counters, recorder evidence, and native stream audits
remain necessary when interpreting a run.

The experiment deliberately keeps reliable depth 100 unchanged so that raw and
deserialized witness runs differ by one variable. Queue depth should not be
tuned blindly: a deeper queue can hide callback latency, increase memory and
drain time, and leave the underlying bottleneck unresolved.

This change is not evidence that the witness loss is fixed. That conclusion
requires a new controlled replay with the optional flag, complete artifacts,
and comparison against the source SQLite inventory, recorder, backend audits,
and PGO output. No replay was performed as part of this implementation.
