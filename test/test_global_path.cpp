#include <gtest/gtest.h>

#include <simple_loop_closure/global_path.hpp>

#include <gtsam/geometry/Rot3.h>

#include <cmath>
#include <vector>

TEST(GlobalPath, PreservesOrderedPoseGeometryAndHeader)
{
  std_msgs::msg::Header header;
  header.frame_id = "camera_init";
  header.stamp.sec = 17;
  header.stamp.nanosec = 42;

  const std::vector<gtsam::Pose3> poses{
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 2.0, 3.0)),
      gtsam::Pose3(gtsam::Rot3::Rz(std::acos(-1.0) / 2.0),
                   gtsam::Point3(-4.0, 5.0, -6.0))};

  const nav_msgs::msg::Path path = simple_loop_closure::makeGlobalPath(poses, header);

  ASSERT_EQ(path.poses.size(), 2U);
  EXPECT_EQ(path.header.frame_id, "camera_init");
  EXPECT_EQ(path.header.stamp.sec, 17);
  EXPECT_EQ(path.header.stamp.nanosec, 42U);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.z, 3.0);
  EXPECT_DOUBLE_EQ(path.poses[1].pose.position.x, -4.0);
  EXPECT_DOUBLE_EQ(path.poses[1].pose.position.y, 5.0);
  EXPECT_DOUBLE_EQ(path.poses[1].pose.position.z, -6.0);
  EXPECT_NEAR(path.poses[1].pose.orientation.x, 0.0, 1e-12);
  EXPECT_NEAR(path.poses[1].pose.orientation.y, 0.0, 1e-12);
  EXPECT_NEAR(path.poses[1].pose.orientation.z, std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(path.poses[1].pose.orientation.w, std::sqrt(0.5), 1e-12);
  for (const auto& pose : path.poses)
  {
    EXPECT_EQ(pose.header.frame_id, path.header.frame_id);
    EXPECT_EQ(pose.header.stamp, path.header.stamp);
  }
}

TEST(GlobalPath, RejectsAnEmptyFrame)
{
  EXPECT_THROW(
      simple_loop_closure::makeGlobalPath({}, std_msgs::msg::Header{}),
      std::invalid_argument);
}
