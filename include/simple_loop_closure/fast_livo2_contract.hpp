#pragma once

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <algorithm>
#include <string>

namespace simple_loop_closure
{

struct FastLivo2Contract
{
  std::string fixed_frame{"camera_init"};
  std::string body_frame{"aft_mapped"};
  bool require_exact_stamp{true};

  bool framesMatch(
      const std::string &cloud_frame,
      const std::string &odom_frame,
      const std::string &odom_child_frame) const
  {
    return !fixed_frame.empty() && !body_frame.empty() &&
           cloud_frame == fixed_frame && odom_frame == fixed_frame &&
           odom_child_frame == body_frame;
  }

  bool stampsMatch(const std::int64_t cloud_stamp_ns, const std::int64_t odom_stamp_ns) const
  {
    return !require_exact_stamp || cloud_stamp_ns == odom_stamp_ns;
  }

  bool poseIsValid(
      const double x, const double y, const double z,
      const double qx, const double qy, const double qz, const double qw) const
  {
    const bool finite = std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
                        std::isfinite(qx) && std::isfinite(qy) &&
                        std::isfinite(qz) && std::isfinite(qw);
    if (!finite)
      return false;

    const double quaternion_squared_norm = qx * qx + qy * qy + qz * qz + qw * qw;
    return std::abs(quaternion_squared_norm - 1.0) <= 1e-6;
  }

  template <typename FieldContainer>
  bool cloudFieldsMatch(
      const FieldContainer &fields, const std::uint8_t float32_datatype) const
  {
    const auto has_float32_field = [&fields, float32_datatype](const std::string &name) {
      return std::any_of(
          fields.begin(), fields.end(),
          [&name, float32_datatype](const auto &field) {
            return field.name == name && field.datatype == float32_datatype &&
                   field.count == 1;
          });
    };

    return has_float32_field("x") && has_float32_field("y") &&
           has_float32_field("z") && has_float32_field("intensity");
  }
};

inline Eigen::Vector3d globalPointToLocal(
    const Eigen::Affine3d &fixed_T_body,
    const Eigen::Vector3d &point_fixed)
{
  return fixed_T_body.inverse() * point_fixed;
}

inline Eigen::Vector3d localPointToGlobal(
    const Eigen::Affine3d &fixed_T_body,
    const Eigen::Vector3d &point_body)
{
  return fixed_T_body * point_body;
}

}  // namespace simple_loop_closure
