#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

namespace simple_loop_closure
{

inline void requireFiniteNonnegative(const char* name, const double value)
{
  if (!std::isfinite(value) || value < 0.0)
    throw std::invalid_argument(std::string(name) + " must be finite and nonnegative");
}

inline void requireNonnegative(const char* name, const int value)
{
  if (value < 0)
    throw std::invalid_argument(std::string(name) + " must be nonnegative");
}

inline void validateMathSafetyParameters(
    const double keyframe_distance_threshold,
    const double keyframe_angular_threshold,
    const double time_stamp_tolerance,
    const int vis_map_cloud_frame_interval)
{
  requireFiniteNonnegative("keyframe_dist_th", keyframe_distance_threshold);
  requireFiniteNonnegative("keyframe_angular_dist_th", keyframe_angular_threshold);
  requireFiniteNonnegative("time_stamp_tolerance", time_stamp_tolerance);
  requireNonnegative("vis_map_cloud_frame_interval", vis_map_cloud_frame_interval);
}

}  // namespace simple_loop_closure
