# SimpleLoopClosure G4/G5 read-only metrics

Date: 2026-10-17

## Scope and evidence boundary

`scripts/evaluate_loop_closures.py` reads existing ROS 2 bags without replaying
them or changing them. It accepts only the named datasets `UGV10`, `Retail`,
and `CBD03`, and rejects any declared FAST-LIVO2 revision other than the Jetson
pin `52db4d422e00a0e1588bed86d91ac4361b21aca9`.

The tool does not infer a closure from proximity alone. Its accepted-closure
source is the last native `/vis_pose_graph` `MarkerArray`. In the node,
`constructLoopEdge()` appends `(source keyframe i, selected target_id)` only
after ICP converges and its score is below `fitness_score_th`; the same stored
pair is later published by `constructVisualizationLoopEdges()` as two red
`LINE_LIST` endpoints. The relevant implementation is
`src/simple_loop_closure_node.cpp:815-875` for acceptance and
`src/simple_loop_closure_node.cpp:499-539` for publication.

Markers do not carry integer pair IDs. The evaluator therefore requires each
loop endpoint to match exactly one node in the *same final native optimized
graph snapshot* (default tolerance `1e-9 m`). It reports those ordered node
indices as `source_id` and `target_id`. Missing, off-graph, duplicate, or
ambiguous identities are errors; nearest-node guessing is not allowed.

Keyframe timestamps are reconstructed from the frozen FAST odometry using the
deployed YAML admission thresholds (`0.5 m`, `0.3 rad`) and the node's rule:
accept when translation **or** rotation reaches its threshold. The reconstructed
keyframe positions must agree with every published native
`original_keyframes` point (default tolerance `1e-6 m`) before any metric is
emitted. Frames must be `camera_init -> aft_mapped`, timestamps must be strictly
increasing, poses finite, and quaternions unit length.

`independent_fast_input_geometry.p50_m` and `p95_m` are R-7 linearly
interpolated percentiles of the 3D Euclidean separation between each accepted
pair's endpoints in the frozen, pre-PGO FAST odometry. They do not use optimized
marker coordinates. This is an endpoint-geometry diagnostic, not an accuracy
metric: FAST odometry may itself drift. Empty accepted sets produce JSON
`null`, never a fabricated zero.

TP/FP are emitted only when `--oracle` supplies an explicit adjudication for
every accepted timestamp pair. The CSV contract is:

```csv
source_timestamp_ns,target_timestamp_ns,is_true_loop
123000000000,456000000000,true
```

Pair matching is unordered, timestamps are exact integer nanoseconds, labels
are `true/false` or `1/0`, and missing or duplicate adjudications fail closed.
An oracle is never derived from FAST proximity, graph geometry, logs, or ICP.
Without one, TP and FP are JSON `null`. APE and RPE are always `null` because
this evaluator has no ground-truth trajectory and alignment contract.

## Invocation

From this feature checkout, after sourcing the ROS 2 environment:

```bash
python3 scripts/evaluate_loop_closures.py \
  --backend-bag /absolute/path/to/saved_simple_loop_closure_bag \
  --fast-bag /absolute/path/to/the_matching_frozen_fast_bag \
  --dataset UGV10 \
  --fast-livo2-sha 52db4d422e00a0e1588bed86d91ac4361b21aca9
```

Use `--oracle oracle.csv` only for a separately curated oracle. If a run used
non-default keyframe thresholds, pass the recorded values explicitly with
`--keyframe-distance-m` and `--keyframe-angle-rad`; otherwise the graph/input
cross-check is expected to fail. `--output` may create a new JSON report, but
the input bags are always opened read-only and no ROS nodes or players are
started.

## Tests run

Exact lightweight source test command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  test/test_evaluate_loop_closures.py test/test_launch_contract.py
```

Result on 2026-10-17: **12 passed in 0.12 s**. The seven metric tests cover
selected-pair recovery, ambiguous identity rejection, keyframe admission,
independent geometry, explicit FP classification, missing-oracle rejection,
empty/linear percentiles, and deployed CLI threshold defaults. The five
existing launch/config contract tests also passed.

An isolated build was invoked with both `CMAKE_BUILD_PARALLEL_LEVEL=2` and
`MAKEFLAGS=-j2`. The node and GTest targets reached 100% successfully. The
colcon wrapper then stopped emitting output and did not return, so it was
interrupted; no successful colcon-wrapper exit is claimed. Tests were run
directly from that completed build tree:

```bash
source /opt/ros/humble/setup.bash
/tmp/slc_metrics_build/simple_loop_closure/test_fast_livo2_contract
cd /tmp/slc_metrics_build/simple_loop_closure
CTEST_OUTPUT_ON_FAILURE=1 ctest --output-on-failure -j2
```

Result: the GTest binary passed **9/9** assertions, and CTest passed **3/3**
test targets (`test_fast_livo2_contract`, `test_launch_contract`, and
`test_evaluate_loop_closures`) in 0.25 s on the final rerun.

Exact read-only artifact check:

```bash
python3 scripts/evaluate_loop_closures.py \
  --backend-bag /home/breidy/Documents/simple_loop_closure_ws/runs/ugv10_jetson_frontend_smoke/backend_messages \
  --fast-bag /home/breidy/Documents/fast_livo2_rtabmap_lc_ws/mvp1_runs/ugv10_livo_jetson_time_normalized \
  --dataset UGV10 \
  --fast-livo2-sha 52db4d422e00a0e1588bed86d91ac4361b21aca9
```

Result: PASS for artifact consistency, with 2 keyframes, 1 odometry edge,
`camera_init`, and 0 accepted loops. Consequently geometry p50/p95 and TP/FP
are all `null`. This remains only the previously documented 19.05 s / 1.05 m
frontend smoke; it is not a true loop-closure evaluation.

## Current blockers to G4/G5 claims

- No full SimpleLoopClosure backend bag containing the native final graph was
  produced for the frozen UGV10, Retail, or CBD03 FAST runs in this task.
  Frontend bags alone cannot prove an accepted backend closure. No rosbag was
  replayed here, as required.
- The available UGV10 backend smoke contains no accepted closure and is too
  short to exercise a revisit. It cannot yield non-empty pair geometry.
- No independently curated closure oracle was supplied for UGV10, Retail, or
  CBD03. TP/FP are therefore not available and are not inferred.
- No ground-truth trajectory plus time association/alignment contract was
  supplied. APE/RPE are not evaluated or claimed.
- The CLI enforces the pinned SHA as declared run provenance; it cannot recover
  a source revision from serialized ROS messages. The supervisor must pair each
  backend bag with the already frozen and audited FAST artifact/manifest.
- The isolated colcon wrapper did not return after all build targets reached
  100%; it was interrupted. Direct execution of every generated test target
  passed, but a clean colcon command exit remains unverified in this sandbox.

These blockers require supervisor-provided saved artifacts or oracle evidence,
not code changes to the production loop detector.
