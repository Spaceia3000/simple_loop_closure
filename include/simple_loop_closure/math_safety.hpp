#pragma once

#include <cmath>
#include <cstdint>
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

// Operational memory-safety guard, not an empirically optimal matching depth.
inline constexpr std::uint32_t kMaxSyncQueueSize = 1024U;

inline std::uint32_t validateSyncQueueSize(const std::int64_t sync_queue_size)
{
  if (sync_queue_size <= 0 ||
      static_cast<std::uint64_t>(sync_queue_size) > kMaxSyncQueueSize)
    throw std::invalid_argument("sync_queue_size must be in the operational safety range [1, 1024]");
  return static_cast<std::uint32_t>(sync_queue_size);
}

// Operational receiver-resource cap, not an empirically selected input depth.
inline constexpr std::uint32_t kMaxCloudInputDepth = 1024U;

inline std::uint32_t validateCloudInputDepth(const std::int64_t cloud_input_depth)
{
  if (cloud_input_depth <= 0 ||
      static_cast<std::uint64_t>(cloud_input_depth) > kMaxCloudInputDepth)
    throw std::invalid_argument(
        "cloud_input_depth must be in the operational safety range [1, 1024]");
  return static_cast<std::uint32_t>(cloud_input_depth);
}

// Operational resource cap, not an empirically selected delivery setting.
inline constexpr std::uint32_t kMaxPgoOutputDepth = 1024U;

inline std::uint32_t validatePgoOutputDepth(const std::int64_t pgo_output_depth)
{
  if (pgo_output_depth <= 0 ||
      static_cast<std::uint64_t>(pgo_output_depth) > kMaxPgoOutputDepth)
    throw std::invalid_argument("pgo_output_depth must be in the operational safety range [1, 1024]");
  return static_cast<std::uint32_t>(pgo_output_depth);
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
