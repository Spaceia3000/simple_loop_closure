# Deterministic keyframe and loop schedule

The loop worker processes a fixed input sequence in global keyframe-ID order.
For each ID it adds exactly one prior/odometry factor, updates iSAM2, and then
searches that ID only when `id % (loop_search_frame_interval + 1) == 0`.
An accepted loop factor is applied before the next odometry ID. Consequently,
one-by-one and burst delivery produce the same ordered GTSAM update sequence.

Each candidate search uses an immutable optimized-pose prefix ending at the
candidate ID. KD-tree membership, timestamps, trajectory distance, target
neighborhood clouds, and source cloud indices are all bounded by that prefix;
later copied keyframes cannot participate. Radius-search results use an
explicit `(squared distance, keyframe ID)` ordering to make equal-distance
selection deterministic. PCL ICP, GTSAM, and all existing geometric acceptance
thresholds remain native and unchanged.

SIGINT and SIGTERM use a staged shutdown while the ROS context is still alive:

1. stop executor callback intake and the visualization thread;
2. copy and drain all produced keyframes through candidate search;
3. apply the final optimizer update;
4. publish the final native pose-graph snapshot and wait up to one second for
   reliable acknowledgements;
5. emit `SLC_DETERMINISM_AUDIT`, then shut down ROS.

The audit reports produced, copied, added, searched, accepted, graph-node, and
graph-factor counts plus stable FNV-1a digests of the ordered searched IDs and
accepted `(source, target)` pairs. `status=PASS` requires exact count parity,
the complete eligible-ID prefix, accepted-pair parity, and
`graph_factors == added + accepted`.

The determinism guarantee is schedule invariance for an identical ordered
input stream on the same software/platform configuration. It does not claim
bit-identical floating-point results across different PCL/GTSAM versions,
architectures, compiler floating-point modes, or changed input ordering.
