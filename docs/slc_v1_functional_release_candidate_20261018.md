# SLC V1 — Functional release candidate freeze — 2026-10-18

## Exact source and purpose

- Canonical functional candidate: integration/slc-v1-drain-dds-candidate-20261018, base 75a1def (C1 + bounded verified drain + optional DDS telemetry), plus operator YAML 96937f0 and optional raw CDR witness 2f34e54.
- Source frontend: FAST-LIVO2 refactor/fast-livo-yaml-first @52db4d4, frozen.
- Stable SimpleLoopClosure baseline: integration/slc-v1-20261018 @502225d, frozen and unchanged.
- Functionally tested scope: LiDAR+IMU (LIO) and LiDAR+IMU+camera (LIVO) normalized frontend streams; source metric cloud and odometry with identical header timestamps.
- LO, sensor live UGV calibration, actual geometric accuracy, ground-truth APE/RPE and scientific F3 are NOT certified by this milestone.

## Runtime parameters

Use config/offline_lio_livo_c1_candidate.yaml, explicitly: ExactTime sync queue 128, reliable cloud DDS reader depth100, PGO publisher depth10, strict equal timestamps, enabled internal audit. The default YAML is unchanged.

For reproducible operator commands use docs/slc_v1_operator_3term_20261018.md (3 terminals) and docs/slc_v1_operator_validated_20261018.md. The PGO backend publishes /pgo_odom and /pgo_map_cloud and RViz overlay; it does not correct FAST-LIVO2 internal odometry or broadcast a replacement TF.

## Verified evidence

- CBD03 LIO original bag 2660 cloud +2660 odometry, 3 repeat complete backend/synchronization/PGO trials; deterministic 351 keyframes, 176 searches, 26 accepted loop factors, 377 total pose graph factors. Backend all PASS; external witness/recorder 2/3 PASS, third witness cloud 2637 and recorder cloud 2612 (FAIL strict C1).
- CBD03 LIVO source 2377 pairs: 2 complete trials all source/witness/recorder/PGO 2377 (strict C1 PASS); third source/backend/PGO/recorder complete 2377 but Python witness cloud 2364 (strict C1 FAIL). This is witness-only loss, not proven SLAM loss.
- Merged bounded drain + optional DDS telemetry candidate 75a1def: native tests 111 PASS; small LIVO stress265/265 native and external audit PASS.
- Optional raw CDR witness enabled via --witness-raw-cdr-stamps; same JSONL schema and QoS, new helper directly extracts first ROS Header sec/nanosec from CDR payload. Source tested against actual CDR SQLite and synthetic endian/invalid cases; 129 Python tests PASS.
- One minimal ROS pilot with raw mode: runs/slc_raw_witness_livo_stress265_20261018, source cloud/odom 265, witness cloud/odom/PGO 265, recorder cloud/odom/PGO 265, native and strict C1 audits PASS, all processes exit 0, effective playback 0.9595x. This proves raw path works but does not establish 3/3 full-run witness reliability.
- User SSD: about 41GiB free after verified archival cleanup on Robotics USB.

## Scientific F3 — NOT CLOSED, do not merge

- F3 isolated branch feat/slc-f3-rank-gate-20261018 @2c13f0d. Unit dimensional ICP fix, source-safe submaps, native final-correspondence shape observability, rank gate, SPD left/right SE3 covariance frame tests passed.
- Default F3 intentionally disables loop factors without production-calibrated ICP distance, registration noise/covariance, predeclared and heldout-validated condition threshold. The normalized point-to-point Hessian alone cannot determine a GTSAM between-factor covariance, and a 6D rank PASS does not prove a true closure.
- Independent ground truth and fit/heldout protocol for six dataset/mode pairs, false positive/false negative closure metrics, and APE/RPE are still not present in the evidence reviewed. Do not invent them or silently enable uncertified override.
- Existing accepted closures on functional candidate still use legacy geometry; functional demonstration, not a safety/scientific certificate.

## Future promotion gate, minimum indispensable only

1. Choose independent truth/reference and calibrate ICP, covariance in right-local GTSAM Pose3 coordinates; validate correspondence rank/condition in heldout.
2. Run the minimum heldout experiments needed to verify true loops and corrected trajectory and map on UGV10/Retail/CBD03 LIO/LIVO; no new broad search or unbounded parameter sweeps.
3. Compare exact cloud+odom+PGO stamps end-to-end, plus witness and recorder on required final certification runs; final RViz map/trajectory and on-UGV demonstration.
4. Promote isolated F3 to stable only when all gates PASS; do not treat this functional RC as full scientific closure.

No production system files modified, no code from F3 merged into functional candidate and no Git remote push performed.
