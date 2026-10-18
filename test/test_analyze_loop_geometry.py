import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
SCRIPT = SCRIPTS / "analyze_loop_geometry.py"
SPEC = importlib.util.spec_from_file_location("analyze_loop_geometry", SCRIPT)
geometry = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = geometry
SPEC.loader.exec_module(geometry)
base = geometry.base


def pose(timestamp_ns, xyz, yaw_rad=0.0):
    half = 0.5 * yaw_rad
    return base.PoseSample(
        timestamp_ns,
        tuple(xyz),
        (0.0, 0.0, np.sin(half), np.cos(half)),
    )


def test_control_is_same_source_and_closest_time_travel_nonedge_without_score_input():
    keyframes = [pose(index * 10_000_000_000, (float(index * 10), 0.0, 0.0)) for index in range(9)]
    travel = base.cumulative_travel_m(keyframes)

    matches = geometry.choose_source_time_travel_matched_controls(
        [(8, 2)], keyframes, travel, 15.0, 15.0
    )

    assert matches[0]["pair"] == (8, 1)
    assert matches[0]["matched_accepted_pair"] == (8, 2)
    assert matches[0]["absolute_time_match_error_s"] == pytest.approx(10.0)
    assert matches[0]["absolute_travel_match_error_m"] == pytest.approx(10.0)
    assert matches[0]["heldout_geometry_used_for_selection"] is False


def test_keyframes_use_only_exact_backend_ingress_sequence():
    fast = [
        pose(10, (0.0, 0.0, 0.0)),
        pose(20, (100.0, 0.0, 0.0)),  # odometry-only sample must not create a keyframe
        pose(30, (0.4, 0.0, 0.0)),
        pose(40, (0.6, 0.0, 0.0)),
    ]

    ingress = geometry.select_backend_ingress_poses(fast, [10, 30, 40])
    selected = base.select_keyframes(ingress, 0.5, 0.3)

    assert [sample.timestamp_ns for sample in ingress] == [10, 30, 40]
    assert [sample.timestamp_ns for sample in selected] == [10, 40]


def test_missing_backend_ingress_pose_fails_closed():
    with pytest.raises(base.EvaluationError, match="absent from FAST odometry"):
        geometry.select_backend_ingress_poses([pose(10, (0.0, 0.0, 0.0))], [10, 20])


def test_control_selection_excludes_native_edges_in_either_direction():
    keyframes = [pose(index * 20_000_000_000, (float(index * 20), 0.0, 0.0)) for index in range(7)]
    travel = base.cumulative_travel_m(keyframes)

    matches = geometry.choose_source_time_travel_matched_controls(
        [(6, 1), (5, 0)], keyframes, travel, 10.0, 10.0
    )

    native = {(1, 6), (0, 5)}
    assert all(tuple(sorted(match["pair"])) not in native for match in matches)
    assert [match["pair"][0] for match in matches] == [6, 5]
    assert all(match["pair"][1] < match["pair"][0] for match in matches)


def test_future_native_target_fails_closed_before_control_selection():
    keyframes = [pose(index * 20_000_000_000, (float(index * 20), 0.0, 0.0)) for index in range(7)]
    travel = base.cumulative_travel_m(keyframes)

    with pytest.raises(base.EvaluationError, match="must be strictly earlier"):
        geometry.choose_source_time_travel_matched_controls(
            [(0, 6)], keyframes, travel, 10.0, 10.0
        )


def test_full_pose_frame_transform_and_bidirectional_holdout_are_consistent():
    rng = np.random.default_rng(42)
    world = rng.uniform((-4.0, -3.0, -1.0), (5.0, 4.0, 2.0), size=(4000, 3))
    partition = base.partition_cloud(world, 0.1, 5000)
    keyframes = [
        pose(0, (1.2, -0.5, 0.2), 0.7),
        pose(40_000_000_000, (-2.0, 1.5, -0.1), -0.4),
    ]

    result = geometry.analyze_pair(
        (1, 0),
        "native_accepted",
        keyframes,
        [0.0, 4.0],
        {0: partition, 1: partition},
        min_time_s=1.0,
        min_travel_m=1.0,
        max_correspondence_m=0.5,
        trim_fraction=0.8,
        max_iterations=20,
        bootstrap_replicates=20,
        include_geometry_diagnostics=True,
    )

    assert result["forward_source_to_target"]["heldout_residual"]["p95_m"] < 1e-10
    assert result["reverse_target_to_source"]["heldout_residual"]["p95_m"] < 1e-10
    assert result["bidirectional"]["inverse_consistency_translation_m"] < 1e-10
    assert result["bidirectional"]["inverse_consistency_rotation_rad"] < 1e-7
    assert result["pose_and_cloud_convention"]["partition_before_pose_transform"] is True
    assert result["convergence"]["both_directions_settled_before_iteration_limit"] is True


def test_registration_status_distinguishes_iteration_limit_from_other_unsettled_stop():
    transform = np.eye(4)
    at_limit = base.RegistrationResult(transform, False, 100, 0.1, 40)
    stopped_early = base.RegistrationResult(transform, False, 12, None, 1)

    assert geometry._registration_status(at_limit, 40) == "iteration_limit_reached"
    assert geometry._registration_status(stopped_early, 40) == "stopped_before_settling"


def test_residual_uncertainty_is_deterministic_and_descriptive():
    distances = np.linspace(0.01, 0.8, 200)

    first = geometry.residual_summary(distances, bootstrap_replicates=50, seed=9)
    second = geometry.residual_summary(distances, bootstrap_replicates=50, seed=9)

    assert first == second
    assert first["point_resampling_uncertainty"]["median_m_ci95"][0] < 0.405
    assert first["point_resampling_uncertainty"]["median_m_ci95"][1] > 0.405
    assert "spatial point dependence" in first["point_resampling_uncertainty"]["interpretation"]


def test_geometry_diagnostic_reports_degenerate_line_without_filtering():
    x = np.linspace(-3.0, 3.0, 100)
    source = np.column_stack((x, np.zeros_like(x), np.zeros_like(x)))
    registration = base.RegistrationResult(np.eye(4), True, 100, 0.0, 1)

    diagnostic = geometry.geometry_diagnostics(source, source, registration, 0.2, 1.0)

    assert diagnostic["normal_matrix_rank"] == 5
    assert diagnostic["condition_number"] is None
    assert diagnostic["used_for_acceptance_or_ranking"] is False


def test_candidate_ranking_is_metric_specific_and_has_no_truth_labels():
    candidates = [
        {
            "candidate_id": "native_accepted:2->0",
            "bidirectional": {
                "symmetric_overlap": {"at_0.2_m": 0.8, "at_0.5_m": 0.9},
                "mean_p95_residual_m": 0.2,
                "inverse_consistency_translation_m": 0.05,
            },
            "convergence": {
                "both_directions_settled_before_iteration_limit": True,
                "forward_status": "settled_before_iteration_limit",
                "reverse_status": "settled_before_iteration_limit",
            },
            "ranks": {},
        },
        {
            "candidate_id": "matched_admissible_nonedge:2->1",
            "bidirectional": {
                "symmetric_overlap": {"at_0.2_m": 0.2, "at_0.5_m": 0.5},
                "mean_p95_residual_m": 0.7,
                "inverse_consistency_translation_m": 0.2,
            },
            "convergence": {
                "both_directions_settled_before_iteration_limit": False,
                "forward_status": "iteration_limit_reached",
                "reverse_status": "settled_before_iteration_limit",
            },
            "ranks": {},
        },
    ]

    geometry.rank_candidates(candidates)

    assert candidates[0]["ranks"] == {
        "symmetric_overlap_at_0.2_m": 1,
        "symmetric_overlap_at_0.5_m": 1,
        "mean_p95_residual_m": 1,
        "inverse_consistency_translation_m": 1,
    }
    assert candidates[0]["ranking_qualification"] == {
        "convergence_status": "both_directions_settled",
        "rank_is_descriptive_not_acceptance": True,
        "rank_population_includes_unsettled_final_transforms": True,
    }
    assert candidates[1]["ranking_qualification"]["convergence_status"] == (
        "one_or_both_directions_reached_iteration_limit"
    )
    assert all("tp" not in candidate and "fp" not in candidate for candidate in candidates)


def candidate(role, source_id, target_id, overlap, converged):
    return {
        "candidate_role_not_truth_label": role,
        "source_id": source_id,
        "target_id": target_id,
        "bidirectional": {
            "symmetric_overlap": {
                "at_0.2_m": overlap,
                "at_0.5_m": overlap,
            }
        },
        "convergence": {
            "both_directions_settled_before_iteration_limit": converged,
        },
    }


def test_paired_summary_is_deterministic_and_stratifies_joint_convergence():
    values = [
        (0.4, 0.5, True, True),
        (0.6, 0.5, True, True),
        (0.7, 0.5, True, False),
        (0.4, 0.5, False, True),
    ]
    candidates = []
    matches = []
    for offset, (native, control, native_converged, control_converged) in enumerate(values):
        source_id = 10 + offset
        candidates.extend(
            (
                candidate("native_accepted", source_id, 0, native, native_converged),
                candidate(
                    "matched_admissible_nonedge",
                    source_id,
                    1,
                    control,
                    control_converged,
                ),
            )
        )
        matches.append(
            {
                "pair": (source_id, 1),
                "matched_accepted_pair": (source_id, 0),
            }
        )

    first = geometry.paired_group_summaries(
        candidates, matches, bootstrap_replicates=500
    )
    second = geometry.paired_group_summaries(
        candidates, matches, bootstrap_replicates=500
    )
    all_pairs = first["overlap"]["at_0.5_m"]["all_matched_pairs"]
    converged = first["overlap"]["at_0.5_m"][
        "both_candidates_bidirectionally_converged_only"
    ]

    assert first == second
    assert all_pairs["matched_pair_count"] == 4
    assert all_pairs["paired_mean_delta_native_minus_control_fraction"] == pytest.approx(0.025)
    assert (all_pairs["native_wins"], all_pairs["ties"], all_pairs["control_wins"]) == (2, 0, 2)
    assert all_pairs["paired_mean_delta_ci95_includes_zero"] is True
    assert all_pairs["evidence_status"] == "inconclusive"
    assert all_pairs["preregistered_effect_or_acceptance_threshold"] is None
    assert converged["matched_pair_count"] == 2
    assert converged["paired_mean_delta_native_minus_control_fraction"] == pytest.approx(0.0)
    assert "reused targets" in first["uncertainty"]["correlation_caveat"]


def test_bidirectional_metric_states_transforms_are_fitted_separately():
    rng = np.random.default_rng(7)
    world = rng.normal(size=(1000, 3))
    partition = base.partition_cloud(world, 0.1, 2000)
    keyframes = [pose(0, (0.0, 0.0, 0.0)), pose(40_000_000_000, (1.0, 0.0, 0.0))]

    result = geometry.analyze_pair(
        (1, 0),
        "native_accepted",
        keyframes,
        [0.0, 1.0],
        {0: partition, 1: partition},
        min_time_s=1.0,
        min_travel_m=0.5,
        max_correspondence_m=0.5,
        trim_fraction=0.8,
        max_iterations=10,
        bootstrap_replicates=0,
        include_geometry_diagnostics=False,
    )

    assert result["bidirectional"]["separately_fitted_forward_and_reverse_transforms"] is True
    assert "two one-way" in result["bidirectional"]["overlap_definition"]
