# SLC-S2 recorder cloud QoS A/B — 2026-10-18

## Scope and hypothesis

The completed CBD03 LIO run `slc_s3_directnode_full_drain_20261018` had 2,660
cloud messages at the source, backend ingress, ExactTime synchronization, PGO
publisher/recorder, and independent witness. The rosbag2 recorder stored 2,654;
source indices 2636, 2637, 2638, 2657, 2658, and 2659 were absent. This localizes
the observation but does not establish its cause.

Test one variable: the rosbag2 recorder subscription for
`/cloud_registered_metric`.

- A: recorder cloud reader `reliable`, `keep_last`, depth 10 (existing default).
- B: recorder cloud reader `reliable`, `keep_last`, depth 100, supplied explicitly
  with `--recorder-qos-yaml PATH`.

The B YAML must override only the cloud recorder subscription. Recorder readers
for odometry and `/pgo_odom` remain unchanged. The option is forwarded only to
`ros2 bag record`; it does not alter the rosbag player, witness, or SLC backend.
Omitting it preserves all prior behavior. The runner checks that the supplied
file exists, records the exact CLI path in `manifest.json`, and performs no
implicit QoS tuning.

## Constants and gate

Hold `ROS_LOCALHOST_ONLY`, ROS domain, source bag, host load, and startup/drain
timings constant. Replay at exactly 1.0x with the cloud writer at reliable
keep-last depth 100. Keep S1 ExactTime depth 128, S3 PGO publisher depth 10, the
direct backend invocation, the playback watchdog, and
`--require-backend-audits` identical between A and B.

Use a fresh output directory for each run. A valid comparison requires completed
playback, bounded clean shutdown, passing native determinism and shutdown stream
audits, and exact 2,660/2,660 source/backend/sync/PGO/witness accounting. Recorder
cloud success is 2,660/2,660 (100%), with source and recorded cloud header
timestamps matching exactly in multiplicity and order: no missing, extra, or
reordered timestamp. Report both arms regardless of outcome; a B improvement is
evidence about this isolated reader-depth change, not proof of root cause.
