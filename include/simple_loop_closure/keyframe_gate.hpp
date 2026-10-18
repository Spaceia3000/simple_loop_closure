#pragma once

#include <Eigen/Geometry>

namespace simple_loop_closure
{

struct KeyframeDecision
{
  bool is_keyframe;
  double translation_distance;
  double angular_distance;
};

inline KeyframeDecision decideKeyframe(
    const Eigen::Affine3d *previous_pose,
    const Eigen::Affine3d &current_pose,
    const double translation_threshold,
    const double angular_threshold)
{
  if (previous_pose == nullptr)
    return {true, 0.0, 0.0};

  const double translation_distance =
      (previous_pose->translation() - current_pose.translation()).norm();
  const Eigen::Quaterniond previous_rotation(previous_pose->rotation());
  const Eigen::Quaterniond current_rotation(current_pose.rotation());
  const double angular_distance = previous_rotation.angularDistance(current_rotation);

  return {
      translation_distance >= translation_threshold || angular_distance >= angular_threshold,
      translation_distance,
      angular_distance};
}

inline double nextTrajectoryDistance(
    const bool first_keyframe,
    const double previous_trajectory_distance,
    const KeyframeDecision &decision)
{
  return first_keyframe ? 0.0 : previous_trajectory_distance + decision.translation_distance;
}

}  // namespace simple_loop_closure
