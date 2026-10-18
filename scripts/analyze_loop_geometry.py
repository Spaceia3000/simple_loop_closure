#!/usr/bin/env python3
"""Standalone v3 read-only geometry analysis for native loop candidates.

This module deliberately does not change the v2 report produced by
``evaluate_loop_closures.py``.  It uses the same validated graph, odometry,
cloud reader, and deterministic global-voxel partition, then performs the
pose-frame and bidirectional analysis described in its own v3 schema.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Sequence

import numpy as np

import evaluate_loop_closures as base


SCHEMA_NAME = "simple_loop_geometry_analysis"
SCHEMA_VERSION = 3
DEFAULT_BACKEND_INGRESS_TOPIC = "/pgo_odom"
PAIR_COMPARISON_SEED = 71_903
DEFAULT_PAIRED_BOOTSTRAP_REPLICATES = 10_000


def _require_strictly_past_target(
    source_id: int,
    target_id: int,
    keyframes: Sequence[base.PoseSample],
    *,
    context: str,
) -> None:
    """Fail closed unless a target precedes its source by index and timestamp."""
    if not 0 <= source_id < len(keyframes) or not 0 <= target_id < len(keyframes):
        raise base.EvaluationError(
            f"{context} pair ({source_id}, {target_id}) is outside the keyframe range"
        )
    if target_id >= source_id:
        raise base.EvaluationError(
            f"{context} target {target_id} must be strictly earlier than source {source_id}"
        )
    if keyframes[target_id].timestamp_ns >= keyframes[source_id].timestamp_ns:
        raise base.EvaluationError(
            f"{context} target timestamp must be strictly earlier than its source"
        )


def read_backend_ingress_timestamps(bag_path: Path, topic: str) -> list[int]:
    """Read the exact synchronized callback sequence captured by backend output."""
    timestamps: list[int] = []
    for _, message, _ in base._read_rosbag_messages(bag_path, topic):
        if (
            message.header.frame_id != base.EXPECTED_FIXED_FRAME
            or message.child_frame_id != base.EXPECTED_BODY_FRAME
        ):
            raise base.EvaluationError(
                f"backend ingress evidence must be {base.EXPECTED_FIXED_FRAME}"
                f"->{base.EXPECTED_BODY_FRAME}"
            )
        stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        if timestamps and stamp <= timestamps[-1]:
            raise base.EvaluationError(
                "backend ingress evidence timestamps are not strictly increasing"
            )
        timestamps.append(stamp)
    if not timestamps:
        raise base.EvaluationError(f"no messages found on backend ingress topic {topic!r}")
    return timestamps


def select_backend_ingress_poses(
    fast_poses: Sequence[base.PoseSample], ingress_timestamps_ns: Sequence[int]
) -> list[base.PoseSample]:
    """Restrict odometry to samples that actually entered the synchronized callback."""
    by_timestamp = {pose.timestamp_ns: pose for pose in fast_poses}
    if len(by_timestamp) != len(fast_poses):
        raise base.EvaluationError("FAST odometry timestamps are not unique")
    missing = [stamp for stamp in ingress_timestamps_ns if stamp not in by_timestamp]
    if missing:
        raise base.EvaluationError(
            f"backend ingress has {len(missing)} timestamps absent from FAST odometry; "
            f"first={missing[0]}"
        )
    return [by_timestamp[stamp] for stamp in ingress_timestamps_ns]


def choose_source_time_travel_matched_controls(
    accepted_pairs: Sequence[tuple[int, int]],
    keyframes: Sequence[base.PoseSample],
    travel_m: Sequence[float],
    min_time_s: float,
    min_travel_m: float,
) -> list[dict[str, object]]:
    """Choose one admissible native non-edge per source without geometry leakage.

    Matching minimizes relative disagreement with the accepted pair's time and
    cumulative-travel separations.  Targets must strictly precede their source.
    There is no matching cutoff and neither point clouds nor any fitted/held-out
    score is an input to this function.
    """
    native = {tuple(sorted(pair)) for pair in accepted_pairs}
    usage = [0] * len(keyframes)
    controls: list[dict[str, object]] = []
    for source_id, accepted_target_id in accepted_pairs:
        _require_strictly_past_target(
            source_id, accepted_target_id, keyframes, context="native accepted"
        )
        reference = base.admission_measurements(
            (source_id, accepted_target_id), keyframes, travel_m, min_time_s, min_travel_m
        )
        reference_time = float(reference["time_separation_s"])
        reference_travel = float(reference["travel_separation_m"])
        eligible: list[tuple[tuple[float, float, int, int], int, dict[str, object]]] = []
        for target_id in range(source_id):
            if tuple(sorted((source_id, target_id))) in native:
                continue
            if keyframes[target_id].timestamp_ns >= keyframes[source_id].timestamp_ns:
                continue
            admission = base.admission_measurements(
                (source_id, target_id), keyframes, travel_m, min_time_s, min_travel_m
            )
            if not admission["passes_time_gate"] or not admission["passes_travel_gate"]:
                continue
            time_error = abs(float(admission["time_separation_s"]) - reference_time) / max(
                reference_time, np.finfo(float).eps
            )
            travel_error = abs(
                float(admission["travel_separation_m"]) - reference_travel
            ) / max(reference_travel, np.finfo(float).eps)
            key = (
                max(time_error, travel_error),
                time_error + travel_error,
                usage[target_id],
                target_id,
            )
            eligible.append((key, target_id, admission))
        if not eligible:
            raise base.EvaluationError(
                f"no admissible source/time/travel-matched non-edge for source node {source_id}"
            )
        _, target_id, admission = min(eligible, key=lambda item: item[0])
        usage[target_id] += 1
        controls.append(
            {
                "pair": (source_id, target_id),
                "matched_accepted_pair": (source_id, accepted_target_id),
                "selection_inputs": (
                    "source_id,strictly_past_target,time_separation_s,travel_separation_m,"
                    "native_edge_set"
                ),
                "target_strictly_precedes_source": True,
                "heldout_geometry_used_for_selection": False,
                "absolute_time_match_error_s": abs(
                    float(admission["time_separation_s"]) - reference_time
                ),
                "absolute_travel_match_error_m": abs(
                    float(admission["travel_separation_m"]) - reference_travel
                ),
            }
        )
    return controls


def _partition_in_body_frame(
    partition: base.CloudPartition, pose: base.PoseSample
) -> base.CloudPartition:
    """Convert a partition of global points to the keyframe body coordinates."""
    fixed_from_body = base._pose_matrix(pose)
    body_from_fixed = np.linalg.inv(fixed_from_body)
    return base.CloudPartition(
        base._transform_points(partition.fit, body_from_fixed),
        base._transform_points(partition.holdout, body_from_fixed),
    )


def _residuals(source: np.ndarray, target: np.ndarray, transform: np.ndarray) -> np.ndarray:
    moved = base._transform_points(source, transform)
    distances, _ = base._nearest_distances(moved, target)
    return distances


def _bootstrap_interval(
    values: np.ndarray, statistic: str, replicates: int, seed: int
) -> list[float] | None:
    if replicates == 0:
        return None
    rng = np.random.default_rng(seed)
    sample = np.asarray(values, dtype=np.float64)
    if len(sample) > 2048:
        sample = sample[rng.choice(len(sample), 2048, replace=False)]
    estimates = np.empty(replicates, dtype=np.float64)
    for index in range(replicates):
        draw = sample[rng.integers(0, len(sample), len(sample))]
        if statistic == "median":
            estimates[index] = np.median(draw)
        elif statistic == "rmse":
            estimates[index] = np.sqrt(np.mean(draw * draw))
        else:
            radius = float(statistic)
            estimates[index] = np.mean(draw <= radius)
    return [float(value) for value in np.quantile(estimates, (0.025, 0.975))]


def residual_summary(
    distances_m: np.ndarray, *, bootstrap_replicates: int, seed: int
) -> dict[str, object]:
    values = np.asarray(distances_m, dtype=np.float64)
    if values.ndim != 1 or len(values) == 0 or not np.isfinite(values).all():
        raise base.EvaluationError("heldout residuals must be a non-empty finite vector")
    median = float(np.median(values))
    result: dict[str, object] = {
        "count": len(values),
        "mean_m": float(np.mean(values)),
        "rmse_m": float(np.sqrt(np.mean(values * values))),
        "median_m": median,
        "p95_m": float(np.quantile(values, 0.95)),
        "mad_about_median_m": float(np.median(np.abs(values - median))),
        "point_resampling_uncertainty": {
            "replicates": bootstrap_replicates,
            "maximum_points_per_replicate": 2048,
            "median_m_ci95": _bootstrap_interval(values, "median", bootstrap_replicates, seed),
            "rmse_m_ci95": _bootstrap_interval(values, "rmse", bootstrap_replicates, seed + 1),
            "interpretation": "descriptive only; spatial point dependence is not modeled",
        },
        "overlap": {},
    }
    for radius in base.OVERLAP_RADII_M:
        result["overlap"][f"at_{radius:.1f}_m"] = {
            "fraction": float(np.mean(values <= radius)),
            "point_resampling_ci95": _bootstrap_interval(
                values, str(radius), bootstrap_replicates, seed + int(radius * 1000)
            ),
        }
    return result


def geometry_diagnostics(
    source_fit: np.ndarray,
    target_fit: np.ndarray,
    registration: base.RegistrationResult,
    max_correspondence_m: float,
    trim_fraction: float,
) -> dict[str, object]:
    """Describe local point-to-point information; never accept/reject a pair."""
    moved = base._transform_points(source_fit, registration.transform)
    distances, indices = base._nearest_distances(moved, target_fit)
    valid = np.flatnonzero(distances <= max_correspondence_m)
    if len(valid) >= 30:
        keep = max(30, int(math.floor(len(valid) * trim_fraction)))
        valid = valid[np.argsort(distances[valid], kind="stable")[:keep]]
    if len(valid) < 3:
        return {
            "correspondence_count": len(valid),
            "normal_matrix_rank": 0,
            "condition_number": None,
            "marginal_standard_deviation": None,
            "used_for_acceptance_or_ranking": False,
        }
    selected = moved[valid]
    jacobian = np.zeros((3 * len(valid), 6), dtype=np.float64)
    for row, point in enumerate(selected):
        x, y, z = point
        skew = np.array(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)))
        jacobian[3 * row : 3 * row + 3, :3] = -skew
        jacobian[3 * row : 3 * row + 3, 3:] = np.eye(3)
    normal = jacobian.T @ jacobian
    eigenvalues = np.linalg.eigvalsh(normal)
    tolerance = float(np.finfo(float).eps * max(normal.shape) * eigenvalues[-1])
    rank = int(np.sum(eigenvalues > tolerance))
    condition = None if eigenvalues[0] <= tolerance else float(eigenvalues[-1] / eigenvalues[0])
    residual = selected - target_fit[indices[valid]]
    sigma2 = float(np.sum(residual * residual) / max(1, 3 * len(valid) - rank))
    covariance = sigma2 * np.linalg.pinv(normal, rcond=np.finfo(float).eps * 6)
    standard_deviation = np.sqrt(np.maximum(0.0, np.diag(covariance)))
    return {
        "correspondence_count": len(valid),
        "normal_matrix_rank": rank,
        "normal_matrix_eigenvalues": eigenvalues.tolist(),
        "condition_number": condition,
        "marginal_standard_deviation": {
            "rotation_rad": standard_deviation[:3].tolist(),
            "translation_m": standard_deviation[3:].tolist(),
        },
        "used_for_acceptance_or_ranking": False,
    }


def analyze_pair(
    pair: tuple[int, int],
    role: str,
    keyframes: Sequence[base.PoseSample],
    travel_m: Sequence[float],
    partitions: dict[int, base.CloudPartition],
    *,
    min_time_s: float,
    min_travel_m: float,
    max_correspondence_m: float,
    trim_fraction: float,
    max_iterations: int,
    bootstrap_replicates: int,
    include_geometry_diagnostics: bool,
) -> dict[str, object]:
    source_id, target_id = pair
    _require_strictly_past_target(source_id, target_id, keyframes, context=role)
    source_pose, target_pose = keyframes[source_id], keyframes[target_id]
    source = _partition_in_body_frame(partitions[source_id], source_pose)
    target = _partition_in_body_frame(partitions[target_id], target_pose)
    fixed_from_source = base._pose_matrix(source_pose)
    fixed_from_target = base._pose_matrix(target_pose)
    target_from_source = np.linalg.inv(fixed_from_target) @ fixed_from_source
    source_from_target = np.linalg.inv(target_from_source)
    forward = base.register_fit_partition(
        source.fit,
        target.fit,
        target_from_source,
        max_correspondence_m,
        trim_fraction,
        max_iterations,
    )
    reverse = base.register_fit_partition(
        target.fit,
        source.fit,
        source_from_target,
        max_correspondence_m,
        trim_fraction,
        max_iterations,
    )
    forward_residuals = _residuals(source.holdout, target.holdout, forward.transform)
    reverse_residuals = _residuals(target.holdout, source.holdout, reverse.transform)
    forward_summary = residual_summary(
        forward_residuals,
        bootstrap_replicates=bootstrap_replicates,
        seed=source_id * 1_000_003 + target_id * 101 + 17,
    )
    reverse_summary = residual_summary(
        reverse_residuals,
        bootstrap_replicates=bootstrap_replicates,
        seed=source_id * 1_000_003 + target_id * 101 + 53,
    )
    inverse_cycle = reverse.transform @ forward.transform
    forward_status = _registration_status(forward, max_iterations)
    reverse_status = _registration_status(reverse, max_iterations)
    result: dict[str, object] = {
        "candidate_id": f"{role}:{source_id}->{target_id}",
        "candidate_role_not_truth_label": role,
        "source_id": source_id,
        "target_id": target_id,
        "admission": base.admission_measurements(
            pair, keyframes, travel_m, min_time_s, min_travel_m
        ),
        "pose_and_cloud_convention": {
            "input_cloud_frame": base.EXPECTED_FIXED_FRAME,
            "registration_cloud_frames": [f"body_at_{source_id}", f"body_at_{target_id}"],
            "forward_seed": "inverse(T_fixed_target) @ T_fixed_source",
            "partition_before_pose_transform": True,
        },
        "forward_source_to_target": {
            "fit": _registration_json(forward, max_iterations),
            "heldout_residual": forward_summary,
        },
        "reverse_target_to_source": {
            "fit": _registration_json(reverse, max_iterations),
            "heldout_residual": reverse_summary,
        },
        "bidirectional": {
            "overlap_definition": (
                "equal-weight mean of two one-way heldout overlaps after separately "
                "fitting source-to-target and target-to-source transformations"
            ),
            "separately_fitted_forward_and_reverse_transforms": True,
            "mean_median_residual_m": 0.5
            * (float(forward_summary["median_m"]) + float(reverse_summary["median_m"])),
            "mean_p95_residual_m": 0.5
            * (float(forward_summary["p95_m"]) + float(reverse_summary["p95_m"])),
            "inverse_consistency_translation_m": float(np.linalg.norm(inverse_cycle[:3, 3])),
            "inverse_consistency_rotation_rad": base._rotation_angle(inverse_cycle[:3, :3]),
            "symmetric_overlap": {
                f"at_{radius:.1f}_m": 0.5
                * (
                    float(forward_summary["overlap"][f"at_{radius:.1f}_m"]["fraction"])
                    + float(reverse_summary["overlap"][f"at_{radius:.1f}_m"]["fraction"])
                )
                for radius in base.OVERLAP_RADII_M
            },
        },
        "convergence": {
            "forward_settled_before_iteration_limit": forward.converged,
            "reverse_settled_before_iteration_limit": reverse.converged,
            "both_directions_settled_before_iteration_limit": (
                forward.converged and reverse.converged
            ),
            "forward_status": forward_status,
            "reverse_status": reverse_status,
            "unsettled_metrics_retained_as_descriptive": not (
                forward.converged and reverse.converged
            ),
        },
        "ranks": {},
    }
    if include_geometry_diagnostics:
        result["geometry_diagnostics_not_filter"] = {
            "forward": geometry_diagnostics(
                source.fit, target.fit, forward, max_correspondence_m, trim_fraction
            ),
            "reverse": geometry_diagnostics(
                target.fit, source.fit, reverse, max_correspondence_m, trim_fraction
            ),
        }
    return result


def _registration_status(
    registration: base.RegistrationResult, max_iterations: int
) -> str:
    if registration.converged:
        return "settled_before_iteration_limit"
    if registration.iterations >= max_iterations:
        return "iteration_limit_reached"
    return "stopped_before_settling"


def _registration_json(
    registration: base.RegistrationResult, max_iterations: int
) -> dict[str, object]:
    return {
        "settled_before_iteration_limit": registration.converged,
        "termination_status": _registration_status(registration, max_iterations),
        "correspondences": registration.correspondences,
        "fit_rmse_m": registration.fit_rmse_m,
        "iterations": registration.iterations,
        "transform": registration.transform.tolist(),
    }


def rank_candidates(candidates: list[dict[str, object]]) -> None:
    """Add descriptive ranks and explicitly qualify each rank by fit convergence."""
    metrics = (
        (
            "symmetric_overlap_at_0.2_m",
            True,
            lambda item: item["bidirectional"]["symmetric_overlap"]["at_0.2_m"],
        ),
        (
            "symmetric_overlap_at_0.5_m",
            True,
            lambda item: item["bidirectional"]["symmetric_overlap"]["at_0.5_m"],
        ),
        (
            "mean_p95_residual_m",
            False,
            lambda item: item["bidirectional"]["mean_p95_residual_m"],
        ),
        (
            "inverse_consistency_translation_m",
            False,
            lambda item: item["bidirectional"]["inverse_consistency_translation_m"],
        ),
    )
    for name, descending, getter in metrics:
        values = sorted(
            {float(getter(item)) for item in candidates}, reverse=descending
        )
        rank_by_value = {value: rank for rank, value in enumerate(values, start=1)}
        for item in candidates:
            item["ranks"][name] = rank_by_value[float(getter(item))]
    for item in candidates:
        convergence = item.get("convergence")
        if convergence is None:
            status = "not_available"
        elif convergence["both_directions_settled_before_iteration_limit"]:
            status = "both_directions_settled"
        elif "iteration_limit_reached" in (
            convergence.get("forward_status"),
            convergence.get("reverse_status"),
        ):
            status = "one_or_both_directions_reached_iteration_limit"
        else:
            status = "one_or_both_directions_unsettled"
        item["ranking_qualification"] = {
            "convergence_status": status,
            "rank_is_descriptive_not_acceptance": True,
            "rank_population_includes_unsettled_final_transforms": True,
        }


def _paired_mean_bootstrap_ci95(
    deltas: np.ndarray, replicates: int, seed: int
) -> list[float] | None:
    """Bootstrap matched-pair deltas while preserving each within-pair association."""
    if replicates == 0 or len(deltas) == 0:
        return None
    rng = np.random.default_rng(seed)
    estimates = np.empty(replicates, dtype=np.float64)
    for index in range(replicates):
        draw = deltas[rng.integers(0, len(deltas), len(deltas))]
        estimates[index] = np.mean(draw)
    return [float(value) for value in np.quantile(estimates, (0.025, 0.975))]


def _paired_overlap_summary(
    paired_candidates: Sequence[tuple[dict[str, object], dict[str, object]]],
    radius_key: str,
    *,
    bootstrap_replicates: int,
    seed: int,
) -> dict[str, object]:
    native = np.asarray(
        [pair[0]["bidirectional"]["symmetric_overlap"][radius_key] for pair in paired_candidates],
        dtype=np.float64,
    )
    controls = np.asarray(
        [pair[1]["bidirectional"]["symmetric_overlap"][radius_key] for pair in paired_candidates],
        dtype=np.float64,
    )
    deltas = native - controls
    ci95 = _paired_mean_bootstrap_ci95(deltas, bootstrap_replicates, seed)
    includes_zero = ci95 is not None and ci95[0] <= 0.0 <= ci95[1]
    evidence_status = (
        "inconclusive" if ci95 is None or includes_zero else "descriptive_separation_only"
    )
    return {
        "matched_pair_count": len(paired_candidates),
        "native_mean_fraction": float(np.mean(native)) if len(native) else None,
        "native_median_fraction": float(np.median(native)) if len(native) else None,
        "control_mean_fraction": float(np.mean(controls)) if len(controls) else None,
        "control_median_fraction": float(np.median(controls)) if len(controls) else None,
        "paired_mean_delta_native_minus_control_fraction": (
            float(np.mean(deltas)) if len(deltas) else None
        ),
        "paired_median_delta_native_minus_control_fraction": (
            float(np.median(deltas)) if len(deltas) else None
        ),
        "native_wins": int(np.sum(deltas > 0.0)),
        "ties": int(np.sum(deltas == 0.0)),
        "control_wins": int(np.sum(deltas < 0.0)),
        "paired_mean_delta_bootstrap_ci95": ci95,
        "paired_mean_delta_ci95_includes_zero": includes_zero if ci95 is not None else None,
        "preregistered_effect_or_acceptance_threshold": None,
        "evidence_status": evidence_status,
        "acceptance_decision": None,
    }


def paired_group_summaries(
    candidates: Sequence[dict[str, object]],
    control_matches: Sequence[dict[str, object]],
    *,
    bootstrap_replicates: int,
) -> dict[str, object]:
    """Summarize native/control matched pairs with convergence sensitivity."""
    by_id = {
        (item["candidate_role_not_truth_label"], item["source_id"], item["target_id"]): item
        for item in candidates
    }
    paired: list[tuple[dict[str, object], dict[str, object]]] = []
    for match in control_matches:
        source_id, control_target_id = match["pair"]
        native_source_id, native_target_id = match["matched_accepted_pair"]
        if source_id != native_source_id:
            raise base.EvaluationError("control/native match does not preserve its source")
        try:
            paired.append(
                (
                    by_id[("native_accepted", source_id, native_target_id)],
                    by_id[("matched_admissible_nonedge", source_id, control_target_id)],
                )
            )
        except KeyError as exc:
            raise base.EvaluationError(f"missing candidate for control match {match!r}") from exc

    both_converged = [
        pair
        for pair in paired
        if pair[0]["convergence"]["both_directions_settled_before_iteration_limit"]
        and pair[1]["convergence"]["both_directions_settled_before_iteration_limit"]
    ]
    summaries: dict[str, object] = {}
    for radius_index, radius in enumerate(base.OVERLAP_RADII_M):
        radius_key = f"at_{radius:.1f}_m"
        summaries[radius_key] = {
            "all_matched_pairs": _paired_overlap_summary(
                paired,
                radius_key,
                bootstrap_replicates=bootstrap_replicates,
                seed=PAIR_COMPARISON_SEED + radius_index * 100,
            ),
            "both_candidates_bidirectionally_converged_only": _paired_overlap_summary(
                both_converged,
                radius_key,
                bootstrap_replicates=bootstrap_replicates,
                seed=PAIR_COMPARISON_SEED + radius_index * 100 + 1,
            ),
        }
    return {
        "metric_definition": (
            "equal-weight mean of two one-way heldout overlaps after separately fitted "
            "forward and reverse transformations"
        ),
        "delta_definition": "native_accepted minus its source-matched admissible nonedge",
        "all_pairs_and_convergence_sensitivity_reported": True,
        "uncertainty": {
            "method": "deterministic bootstrap of matched-pair mean deltas",
            "replicates": bootstrap_replicates,
            "within_pair_association_preserved": True,
            "robust_to_cross_pair_correlation": False,
            "correlation_caveat": (
                "descriptive only: resampling preserves each native/control pair but does "
                "not model dependence from reused targets, shared clouds, spatial points, "
                "or temporal trajectory correlation"
            ),
        },
        "candidate_convergence_counts": {
            "native_both_directions_settled": sum(
                bool(pair[0]["convergence"]["both_directions_settled_before_iteration_limit"])
                for pair in paired
            ),
            "control_both_directions_settled": sum(
                bool(pair[1]["convergence"]["both_directions_settled_before_iteration_limit"])
                for pair in paired
            ),
            "jointly_both_candidates_bidirectionally_settled": len(both_converged),
            "total_matched_pairs": len(paired),
        },
        "overlap": summaries,
    }


def analyze(
    graph: base.GraphSnapshot,
    keyframes: Sequence[base.PoseSample],
    clouds_by_timestamp: dict[int, np.ndarray],
    *,
    min_time_s: float,
    min_travel_m: float,
    voxel_size_m: float,
    max_points_per_partition: int,
    max_correspondence_m: float,
    trim_fraction: float,
    max_iterations: int,
    bootstrap_replicates: int,
    paired_bootstrap_replicates: int,
    include_geometry_diagnostics: bool,
) -> dict[str, object]:
    travel = base.cumulative_travel_m(keyframes)
    control_matches = choose_source_time_travel_matched_controls(
        graph.accepted_pairs, keyframes, travel, min_time_s, min_travel_m
    )
    controls = [tuple(match["pair"]) for match in control_matches]
    required_ids = {node for pair in graph.accepted_pairs + tuple(controls) for node in pair}
    partitions = {
        node_id: base.partition_cloud(
            clouds_by_timestamp[keyframes[node_id].timestamp_ns],
            voxel_size_m,
            max_points_per_partition,
        )
        for node_id in required_ids
    }
    candidates = [
        analyze_pair(
            pair,
            "native_accepted",
            keyframes,
            travel,
            partitions,
            min_time_s=min_time_s,
            min_travel_m=min_travel_m,
            max_correspondence_m=max_correspondence_m,
            trim_fraction=trim_fraction,
            max_iterations=max_iterations,
            bootstrap_replicates=bootstrap_replicates,
            include_geometry_diagnostics=include_geometry_diagnostics,
        )
        for pair in graph.accepted_pairs
    ]
    candidates.extend(
        analyze_pair(
            pair,
            "matched_admissible_nonedge",
            keyframes,
            travel,
            partitions,
            min_time_s=min_time_s,
            min_travel_m=min_travel_m,
            max_correspondence_m=max_correspondence_m,
            trim_fraction=trim_fraction,
            max_iterations=max_iterations,
            bootstrap_replicates=bootstrap_replicates,
            include_geometry_diagnostics=include_geometry_diagnostics,
        )
        for pair in controls
    )
    rank_candidates(candidates)
    group_summaries = paired_group_summaries(
        candidates,
        control_matches,
        bootstrap_replicates=paired_bootstrap_replicates,
    )
    return {
        "schema_name": SCHEMA_NAME,
        "schema_version": SCHEMA_VERSION,
        "interpretation": {
            "ground_truth_used": False,
            "tp_fp_labels_produced": False,
            "rejection_threshold_added": False,
            "candidate_ranking_only": True,
            "matched_nonedge_is_known_negative": False,
            "group_effect_is_inferential_claim": False,
        },
        "control_matching": {
            "method": (
                "same source and strictly earlier target; minimize relative time/travel "
                "mismatch among admissible native nonedges"
            ),
            "future_targets_allowed": False,
            "pointcloud_or_holdout_score_used": False,
            "match_cutoff": None,
            "matches": control_matches,
        },
        "analysis_parameters": {
            "existing_descriptive_overlap_radii_m": list(base.OVERLAP_RADII_M),
            "min_time_separation_s": min_time_s,
            "min_travel_separation_m": min_travel_m,
            "voxel_size_m": voxel_size_m,
            "max_points_per_partition": max_points_per_partition,
            "max_correspondence_m": max_correspondence_m,
            "trim_fraction": trim_fraction,
            "max_iterations": max_iterations,
            "bootstrap_replicates": bootstrap_replicates,
            "paired_bootstrap_replicates": paired_bootstrap_replicates,
            "geometry_diagnostics_enabled": include_geometry_diagnostics,
        },
        "ranking": {
            "kind": "metric_specific_dense_ordinal",
            "composite_score": None,
            "used_as_rejection_rule": False,
            "population": "all candidates including unsettled final transforms",
            "convergence_status_attached_to_each_candidate": True,
        },
        "paired_group_summary": group_summaries,
        "candidates": candidates,
    }


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend-bag", type=Path, required=True)
    parser.add_argument("--fast-bag", type=Path, required=True)
    parser.add_argument("--dataset", choices=("UGV10", "Retail", "CBD03"), required=True)
    parser.add_argument("--mode", choices=("LIO", "LIVO"), required=True)
    parser.add_argument("--fast-livo2-sha", required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--graph-topic", default=base.DEFAULT_GRAPH_TOPIC)
    parser.add_argument("--backend-ingress-topic", default=DEFAULT_BACKEND_INGRESS_TOPIC)
    parser.add_argument("--odom-topic", default=base.DEFAULT_ODOM_TOPIC)
    parser.add_argument("--cloud-topic", default=base.DEFAULT_CLOUD_TOPIC)
    parser.add_argument("--keyframe-distance-m", type=float, default=0.5)
    parser.add_argument("--keyframe-angle-rad", type=float, default=0.3)
    parser.add_argument("--min-time-separation-s", type=float, default=30.0)
    parser.add_argument("--min-travel-separation-m", type=float, default=30.0)
    parser.add_argument("--voxel-size-m", type=float, default=0.1)
    parser.add_argument("--max-points-per-partition", type=int, default=12000)
    parser.add_argument("--max-correspondence-m", type=float, default=1.0)
    parser.add_argument("--trim-fraction", type=float, default=0.8)
    parser.add_argument("--max-iterations", type=int, default=40)
    parser.add_argument("--bootstrap-replicates", type=int, default=200)
    parser.add_argument(
        "--paired-bootstrap-replicates",
        type=int,
        default=DEFAULT_PAIRED_BOOTSTRAP_REPLICATES,
    )
    parser.add_argument("--geometry-diagnostics", action="store_true")
    parser.add_argument("--identity-tolerance-m", type=float, default=1e-9)
    parser.add_argument("--original-geometry-tolerance-m", type=float, default=1e-6)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        if args.fast_livo2_sha != base.PINNED_FAST_LIVO2_SHA:
            raise base.EvaluationError(
                f"FAST-LIVO2 SHA must be pinned to {base.PINNED_FAST_LIVO2_SHA}"
            )
        if args.bootstrap_replicates < 0 or args.paired_bootstrap_replicates < 0:
            raise base.EvaluationError("bootstrap replicates must be non-negative")
        graph = base.read_last_graph(
            args.backend_bag.resolve(), args.graph_topic, args.identity_tolerance_m
        )
        poses = base.read_fast_odometry(args.fast_bag.resolve(), args.odom_topic)
        ingress_timestamps = read_backend_ingress_timestamps(
            args.backend_bag.resolve(), args.backend_ingress_topic
        )
        ingress_poses = select_backend_ingress_poses(poses, ingress_timestamps)
        keyframes = base.validate_keyframes(
            graph,
            base.select_keyframes(
                ingress_poses, args.keyframe_distance_m, args.keyframe_angle_rad
            ),
            args.original_geometry_tolerance_m,
        )
        travel = base.cumulative_travel_m(keyframes)
        matches = choose_source_time_travel_matched_controls(
            graph.accepted_pairs,
            keyframes,
            travel,
            args.min_time_separation_s,
            args.min_travel_separation_m,
        )
        required_ids = {
            node
            for pair in graph.accepted_pairs + tuple(tuple(match["pair"]) for match in matches)
            for node in pair
        }
        clouds = base.read_keyframe_clouds(
            args.fast_bag.resolve(),
            args.cloud_topic,
            {keyframes[node].timestamp_ns for node in required_ids},
        )
        report = analyze(
            graph,
            keyframes,
            clouds,
            min_time_s=args.min_time_separation_s,
            min_travel_m=args.min_travel_separation_m,
            voxel_size_m=args.voxel_size_m,
            max_points_per_partition=args.max_points_per_partition,
            max_correspondence_m=args.max_correspondence_m,
            trim_fraction=args.trim_fraction,
            max_iterations=args.max_iterations,
            bootstrap_replicates=args.bootstrap_replicates,
            paired_bootstrap_replicates=args.paired_bootstrap_replicates,
            include_geometry_diagnostics=args.geometry_diagnostics,
        )
        report["dataset"] = args.dataset
        report["mode"] = args.mode
        report["provenance"] = {
            "fast_livo2_sha": args.fast_livo2_sha,
            "fast_bag": str(args.fast_bag.resolve()),
            "backend_bag": str(args.backend_bag.resolve()),
            "read_only": True,
            "frontend_odometry_count": len(poses),
            "backend_synchronized_ingress_count": len(ingress_poses),
            "keyframes_reconstructed_from_backend_ingress_only": True,
        }
        rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
        if args.output:
            args.output.write_text(rendered, encoding="utf-8")
        else:
            sys.stdout.write(rendered)
        return 0
    except (base.EvaluationError, OSError, ValueError, np.linalg.LinAlgError) as exc:
        print(f"geometry analysis failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
