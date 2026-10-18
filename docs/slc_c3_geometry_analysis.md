# SLC-C3 independent heldout geometry analysis

Date: 2026-10-18

## Scope and audit result

`scripts/analyze_loop_geometry.py` is an additive, read-only v3 analyzer. It
does not alter the v2 output of `scripts/evaluate_loop_closures.py`, native
backend code, detector parameters, bags, or graph state. It reports no ground
truth, TP/FP labels, acceptance decision, or new rejection threshold.

The audit identified two data/geometry issues that the standalone path avoids:

1. Backend keyframe admission runs only for synchronized cloud/odometry
   callbacks. A frontend bag may contain odometry messages that never reached
   that callback (the current CBD03 LIVO artifact has 2,377 frontend odometry
   messages but 2,369 captured `/pgo_odom` ingress messages). Reconstructing
   keyframes from every frontend odometry sample can therefore diverge from
   the native graph. V3 reads the ordered `/pgo_odom` timestamps and selects
   only the exactly corresponding frozen frontend poses before applying the
   native translation-or-rotation keyframe rule. Missing or non-monotonic
   ingress evidence fails closed.
2. `/cloud_registered_metric` contains global `camera_init` points, while the
   backend stores `inverse(T_fixed_body) * p_fixed`. The v2 analysis initializes
   registration of global clouds with only `target_xyz - source_xyz`; that is
   not the full native pose convention and omits rotation. V3 first partitions
   in the common global voxel coordinates, then transforms each disjoint
   partition into its keyframe body frame. Its source-to-target seed is
   `inverse(T_fixed_target) @ T_fixed_source`; reverse fitting uses the inverse.

Partitioning before the pose transform keeps the deterministic global-voxel
assignment consistent between scans. Fit and heldout arrays are disjoint.
Each direction is fit independently on fit points and frozen before heldout
nearest-neighbor residuals are measured.

## Controls, uncertainty, diagnostics, and ranking

For each native accepted pair, one control is chosen with the same source and
a strictly earlier target from all admissible native nonedges. Native pairs
with `target_id >= source_id`, non-earlier target timestamps, or out-of-range
indices fail closed. Selection minimizes relative mismatch in the accepted
pair's absolute time and cumulative-travel separations. There is no match
cutoff. Point clouds, fit results, heldout results, and candidate ranks are not
inputs to selection. A selected nonedge is a hard matched control, not a known
negative.

V3 reports forward and reverse heldout mean, RMSE, median, p95, MAD, overlap at
the existing 0.2 m and 0.5 m descriptive radii, and deterministic point-level
bootstrap intervals. The intervals explicitly do not model spatial
correlation and are descriptive rather than inferential confidence claims.
The reported bidirectional or "symmetric" overlap is the equal-weight mean of
two one-way heldout overlaps after separately fitting source-to-target and
target-to-source transformations. It is not an overlap computed in both
directions under one frozen transform.

The additive `paired_group_summary` reports, for every overlap radius, both
all matched pairs and the sensitivity subset where both directions settled
before the iteration limit for both the native candidate and its control. Each
summary distinguishes marginal candidate means/medians from the paired mean
and median delta (`native - control`) and reports native wins, ties, and control
wins. Its deterministic bootstrap defaults to 10,000 replicates and resamples
whole matched pairs, preserving the within-pair association. It remains
descriptive: target reuse, shared clouds, spatial dependence, and temporal
trajectory correlation across pairs are not modeled. If the paired mean-delta
interval includes zero, the output explicitly says `inconclusive`. No
preregistered effect threshold exists, so the analyzer emits no acceptance
decision even if a descriptive interval does not include zero.

`--geometry-diagnostics` optionally reports the point-to-point normal-matrix
rank, eigenvalues, condition number, and marginal local linearization standard
deviations. These diagnostics do not filter or rank candidates. Candidate
ranking is metric-specific (each overlap, bidirectional p95 residual, and
inverse-consistency translation); there is no composite score or binary label.
All final transforms remain in the descriptive all-candidate rankings, but
each candidate now carries an explicit ranking convergence qualification.
Unsettled transforms are not silently presented as equivalent to settled
ones, and convergence is not repurposed into a post-hoc acceptance cutoff.

These fields are additive to schema v3: existing field names and meanings are
unchanged, so no schema-version bump is required.

## CBD03 LIVO interpretation after independent peer review

The completed artifact contains 28 native candidates and 28 matched controls.
At 0.5 m, 50.45% versus 45.89% are the two marginal group medians, not a
paired advantage. The marginal means are 47.47% and 47.04%; the native
candidate wins only 11 of 28 matched comparisons, and the mean paired delta is
about +0.43 percentage points. A diagnostic paired bootstrap interval includes
zero. In addition, only 17/28 native candidates and 13/28 controls settled in
both registration directions before iteration 40. Therefore this completed
comparison is inconclusive and must not be cited as evidence that native
accepted closures have better geometry. The analyzer adds no ground truth and
no post-hoc acceptance threshold.

## Lightweight verification

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  test/test_analyze_loop_geometry.py test/test_evaluate_loop_closures.py \
  test/test_launch_contract.py
```

The synthetic coverage includes exact backend-ingress filtering, native-edge
exclusion in either direction, fail-closed future-target prevention,
source/time/travel matching without geometry inputs, full-pose transforms with
nonzero rotations, two independently fitted heldout directions, deterministic
point and matched-pair uncertainty, all-pair versus joint-convergence
summaries, convergence-qualified rankings, inconclusive reporting, and
rank-deficient geometry.

## Optional future read-only run

Do not overwrite an existing report. If a distinct future artifact is needed,
run this read-only command against the captured backend and frozen frontend
bags with a new output path:

```bash
source /opt/ros/humble/setup.bash
python3 scripts/analyze_loop_geometry.py \
  --backend-bag /home/breidy/Documents/simple_loop_closure_ws/runs/cbd03_livo_jetson_full_simple_attempt10/backend_messages \
  --fast-bag /home/breidy/Documents/fast_livo2_rtabmap_lc_ws/mvp1_runs/cbd03_livo_jetson_full_normalized \
  --dataset CBD03 \
  --mode LIVO \
  --fast-livo2-sha 52db4d422e00a0e1588bed86d91ac4361b21aca9 \
  --geometry-diagnostics \
  --output /tmp/cbd03_livo_loop_geometry_v3_revised.json
```

This command reads both bags sequentially and writes only the requested JSON
file. It does not replay either bag.
