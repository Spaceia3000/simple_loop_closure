#pragma once

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace simple_loop_closure
{
struct RgbPoseAnchor
{
  std::int64_t stamp_ns{0};
  Eigen::Affine3d raw_pose{Eigen::Affine3d::Identity()};
  Eigen::Affine3d optimized_pose{Eigen::Affine3d::Identity()};
};

inline int nearestRgbPoseAnchor(
    const std::vector<RgbPoseAnchor>& anchors,
    const std::int64_t stamp_ns,
    const std::int64_t maximum_delta_ns) noexcept
{
  if (stamp_ns <= 0 || maximum_delta_ns < 0)
    return -1;
  std::int64_t nearest_delta = std::numeric_limits<std::int64_t>::max();
  int nearest_index = -1;
  for (std::size_t i = 0; i < anchors.size(); ++i)
  {
    const auto candidate = anchors[i].stamp_ns;
    if (candidate <= 0)
      continue;
    const std::int64_t delta =
        (candidate > stamp_ns) ? candidate - stamp_ns : stamp_ns - candidate;
    if (delta <= maximum_delta_ns && delta < nearest_delta)
    {
      nearest_delta = delta;
      nearest_index = static_cast<int>(i);
    }
  }
  return nearest_index;
}

// Both original and corrected frame are in FAST's camera_init fixed frame.
// Raw RGB points are already in that frame, not in LiDAR/body coordinates.
inline Eigen::Affine3d rgbWorldCorrection(const RgbPoseAnchor& anchor)
{
  return anchor.optimized_pose * anchor.raw_pose.inverse();
}

inline bool validRgbWorldCorrection(const Eigen::Affine3d& transform)
{
  return transform.matrix().allFinite() &&
      std::abs(transform.linear().determinant() - 1.0) < 1e-3 &&
      (transform.linear().transpose() * transform.linear() -
       Eigen::Matrix3d::Identity()).norm() < 1e-3;
}
} // namespace simple_loop_closure
