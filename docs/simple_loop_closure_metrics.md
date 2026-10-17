# Read-only loop-closure verification metrics

Date: 2026-10-17

## Scope

`scripts/evaluate_loop_closures.py` is an offline evaluator for saved native
SimpleLoopClosure and FAST frontend bags. It does not replay a bag, start a
node, modify an input artifact, or change the production detector. The CLI
accepts UGV10, Retail, and CBD03 and requires the declared FAST revision to be
the pinned Jetson revision
`52db4d422e00a0e1588bed86d91ac4361b21aca9`. It uses only the common FAST
`/aft_mapped_to_init` and `/cloud_registered_metric` contract, so the same
measurement path applies to LO, LIO, and LIVO artifacts satisfying that
contract.

This evaluator deliberately reports no TP, FP, precision, recall, APE, or RPE.
No ground truth or independent human adjudication is available in the supplied
artifacts.

## Measurement contract

### Native graph topology

The last native `/vis_pose_graph` snapshot is the sole source of accepted
edges. Marker endpoints must each identify exactly one optimized node in that
same snapshot. Self edges, off-graph endpoints, ambiguous endpoints, and
duplicate undirected pairs (including a repeated reverse pair) fail closed.
The resulting count is therefore the count of unique undirected closures, not
the number of marker endpoints or detector log lines.

The evaluator reconstructs keyframes from frozen FAST odometry with the
deployed translation-or-rotation rule (`0.5 m` or `0.3 rad`) and requires every
reconstructed input position to agree with the native `original_keyframes`
marker. Cloud association uses exact keyframe timestamps. Frames, monotonic
timestamps, finite values, cloud layout, and quaternion norms are validated.

### Separate time and travel admission

The native search conditions are retained as two separate measurements:

- absolute timestamp separation must be strictly greater than `30 s`;
- cumulative FAST path travel between keyframes must be strictly greater than
  `30 m`.

Endpoint Euclidean distance is reported separately. It is not substituted for
cumulative travel.

### Disjoint fit/holdout cloud verification

Each required exact-timestamp cloud is voxel subsampled at `0.1 m`. A stable
integer voxel hash divides representatives into mutually exclusive fit and
holdout point sets, capped at 12,000 points per partition. A deterministic
trimmed rigid ICP uses only fit points:

- initial rotation is identity in the shared FAST fixed frame;
- initial translation is target FAST position minus source FAST position;
- maximum correspondence distance is `1.0 m`;
- closest 80% of valid correspondences are retained;
- at most 40 iterations are used.

The fitted transform is then frozen. Holdout points never enter fitting.
Forward and reverse nearest-neighbor support are measured on the holdout sets
at exactly `0.2 m` and `0.5 m`; their arithmetic mean is the symmetric overlap.
R-7 linear p50 and p95 are reported across pairs. A registration that reaches
the iteration bound is still scored at its finite final iterate, with the
number settling early reported separately.

For every native source, the evaluator also constructs one control using a
target drawn from the native target pool. A control must pass both time and
travel gates and must not be a native undirected edge. Target use is balanced,
then a target far in node index from the native target is preferred. Controls
go through the identical fit/holdout pipeline. They are admissible non-edges,
not ground-truth negatives.

### Residual and cycle estimates

The bag does not serialize native loop-factor transforms, covariances, ICP
scores, or optimized orientations. Consequently an actual GTSAM factor
residual cannot be reconstructed. The evaluator safely reports:

- original and optimized endpoint chord lengths (explicitly not factor
  residuals);
- node displacement and adjacent edge-length change from original to optimized
  marker positions;
- an estimated SE(3) consistency cycle for each consecutive accepted-edge
  pair, made from the two independent fit registrations and the intervening
  frozen FAST odometry segments.

Cycle p50/p95 are given for all consecutive pairs and separately for the subset
where both registrations settled before the iteration limit.

## UGV10 full-run result

Inputs (read only):

- FAST bag:
  `/home/breidy/Documents/fast_livo2_rtabmap_lc_ws/mvp1_runs/ugv10_livo_jetson_full_normalized`
- Native backend bag:
  `/home/breidy/Documents/simple_loop_closure_ws/runs/ugv10_livo_jetson_full/backend_messages`
- FAST revision: `52db4d422e00a0e1588bed86d91ac4361b21aca9`

Exact invocation:

```bash
source /opt/ros/humble/setup.bash
python3 scripts/evaluate_loop_closures.py \
  --backend-bag /home/breidy/Documents/simple_loop_closure_ws/runs/ugv10_livo_jetson_full/backend_messages \
  --fast-bag /home/breidy/Documents/fast_livo2_rtabmap_lc_ws/mvp1_runs/ugv10_livo_jetson_full_normalized \
  --dataset UGV10 \
  --fast-livo2-sha 52db4d422e00a0e1588bed86d91ac4361b21aca9 \
  --output /tmp/ugv10_loop_verification.json
```

The complete JSON contains every accepted and control pair, timestamps,
separate admission results, partition sizes, fit diagnostics, transforms, and
both directional holdout overlaps. Aggregate measured results are:

| Measurement | Count | p50 | p95 |
|---|---:|---:|---:|
| Keyframes | 391 | — | — |
| Odometry edges | 390 | — | — |
| Unique undirected closures | 86 | — | — |
| Closure graph cycle-rank increment | 86 | — | — |
| Nodes incident to closures | 145 | — | — |
| Maximum loop-only node degree | 7 | — | — |
| Closure node-index span | 86 | 233.5 | 328.5 |
| FAST endpoint chord (m) | 86 | 0.9474404691 | 9.3194000677 |
| Optimized endpoint chord, not residual (m) | 86 | 0.9536733826 | 9.3156873136 |
| Node original-to-optimized displacement (m) | 391 | 0.0181344113 | 0.0337255233 |
| Adjacent-edge absolute length change (m) | 390 | 0.0000420631 | 0.0002309336 |
| Accepted time separation (s) | 86 | 75.6689909175 | 110.5702756883 |
| Accepted cumulative travel separation (m) | 86 | 147.8866752798 | 204.8588800841 |

All 86 closures independently pass the strict time gate and all 86
independently pass the strict travel gate.

Holdout point-cloud results:

| Cohort and metric | Count | p50 | p95 |
|---|---:|---:|---:|
| Accepted symmetric overlap at 0.2 m | 86 | 0.5099689234 | 0.6068326073 |
| Accepted symmetric overlap at 0.5 m | 86 | 0.8239032090 | 0.8727732872 |
| Control symmetric overlap at 0.2 m | 86 | 0.0953976551 | 0.2381159438 |
| Control symmetric overlap at 0.5 m | 86 | 0.2992984429 | 0.4584440564 |
| Accepted fit RMSE (m) | 86 | 0.1938129382 | 0.4566881596 |
| Control fit RMSE (m) | 86 | 0.3879000680 | 0.4922691284 |
| Accepted fit correspondences | 86 | 4569.5 | 8950.25 |
| Control fit correspondences | 86 | 2632.5 | 4700.5 |
| Accepted fit iterations | 86 | 36 | 40 |
| Control fit iterations | 86 | 40 | 40 |

Of the 86 accepted registrations, 49 settled before the 40-iteration limit;
10 of 86 controls did. Control admission remained nontrivial: its time
separation was p50 `61.009373341 s`, p95 `87.836845442 s`, and cumulative
travel was p50 `120.2139479325 m`, p95 `169.3741763494 m`; all 86 controls
passed both gates.

As a descriptive contrast only, 29 accepted pairs were at or below the control
p95 overlap at `0.2 m`, and 28 were at or below the control p95 at `0.5 m`.
Those empirical control p95 values are not detector admission thresholds and
do not assign truth labels.

Estimated consecutive-edge cycle consistency:

| Cycle estimate | Count | p50 | p95 |
|---|---:|---:|---:|
| Translation residual, all consecutive pairs (m) | 85 | 0.0418078573 | 2.6072139889 |
| Rotation residual, all consecutive pairs (rad) | 85 | 0.0057025212 | 0.0797244030 |
| Translation residual, both fits settled (m) | 40 | 0.0233409479 | 0.0550912841 |
| Rotation residual, both fits settled (rad) | 40 | 0.0037488958 | 0.0073874362 |

The accepted cohort is substantially above the admissible non-edge control
cohort in median holdout overlap. That is useful independent geometric support
for the cohort, not proof that all 86 are correct. The low-overlap accepted
tail and the all-cycle translation p95 prevent a defensible blanket
"86 verified true loops" claim.

## Tests

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  test/test_evaluate_loop_closures.py test/test_launch_contract.py
```

Result: **15 passed in 0.34 s**. Synthetic coverage includes a known rigid
positive with greater than 0.99 holdout overlap at both radii, a separated
negative with zero 0.5 m holdout overlap, deterministic partition disjointness,
separate time/travel gates, exact topology recovery, ambiguous identity
rejection, and reverse-edge duplicate rejection.

## Limitations

- Fit and holdout partitions are disjoint for this evaluator, and the scoring
  implementation is independent of the native ICP. They are derived from the
  same saved sensor clouds that the native detector saw, so they are not a new
  sensor acquisition or a detector-blind dataset.
- An admissible native non-edge is not known false. It can include an
  unselected real revisit. Controls calibrate the pipeline; they do not define
  FP truth.
- FAST global registration, endpoint-centering initialization, repeated
  structure, ground surfaces, viewpoint change, occlusion, and dynamic objects
  can all affect overlap. Symmetric support reduces one-way density bias but
  does not eliminate these effects.
- The fixed parameters above were chosen before inspecting UGV10 scores and
  were not swept or tuned on this run. No overlap cutoff was selected.
- A final iterate that did not settle before 40 iterations is retained and
  clearly counted. The settled-only cycle subset is therefore the safer
  consistency statistic.
- Marker positions alone cannot recover native loop measurement transforms,
  optimized orientations, robust weights, or GTSAM factor residuals. Optimized
  endpoint chord is not presented as a residual.
- Only the supplied UGV10 full artifacts were measured here. Retail and CBD03,
  and additional LO/LIO/LIVO bags, require their corresponding saved native
  backend bags; no result is extrapolated to absent artifacts.
- Without ground truth or independent adjudication, TP, FP, precision, recall,
  APE, and RPE remain unavailable by design.
