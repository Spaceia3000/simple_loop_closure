#pragma once

#include <gtsam/geometry/Pose3.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>

#include <stdexcept>
#include <utility>
#include <vector>

namespace simple_loop_closure
{

inline nav_msgs::msg::Path makeGlobalPath(
    const std::vector<gtsam::Pose3>& ordered_poses,
    const std_msgs::msg::Header& header)
{
  if (header.frame_id.empty())
    throw std::invalid_argument("global path frame_id must not be empty");

  nav_msgs::msg::Path path;
  path.header = header;
  path.poses.reserve(ordered_poses.size());
  for (const gtsam::Pose3& pose : ordered_poses)
  {
    geometry_msgs::msg::PoseStamped stamped_pose;
    stamped_pose.header = header;
    stamped_pose.pose.position.x = pose.x();
    stamped_pose.pose.position.y = pose.y();
    stamped_pose.pose.position.z = pose.z();

    const auto quaternion = pose.rotation().toQuaternion();
    stamped_pose.pose.orientation.x = quaternion.x();
    stamped_pose.pose.orientation.y = quaternion.y();
    stamped_pose.pose.orientation.z = quaternion.z();
    stamped_pose.pose.orientation.w = quaternion.w();
    path.poses.emplace_back(std::move(stamped_pose));
  }
  return path;
}

}  // namespace simple_loop_closure
