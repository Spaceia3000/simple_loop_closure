#include <gtest/gtest.h>
#include <simple_loop_closure/rgb_global_map_math.hpp>

using simple_loop_closure::RgbPoseAnchor;
using simple_loop_closure::nearestRgbPoseAnchor;
using simple_loop_closure::rgbWorldCorrection;

TEST(RgbGlobalMapMath, MatchesNearestOnlyWithinExplicitBound)
{
  std::vector<RgbPoseAnchor> anchors(3);
  anchors[0].stamp_ns = 1000000000LL;
  anchors[1].stamp_ns = 1500000000LL;
  anchors[2].stamp_ns = 2500000000LL;
  EXPECT_EQ(nearestRgbPoseAnchor(anchors, 1480000000LL, 100000000LL), 1);
  EXPECT_EQ(nearestRgbPoseAnchor(anchors, 1700000000LL, 100000000LL), -1);
  EXPECT_EQ(nearestRgbPoseAnchor(anchors, 0, 100000000LL), -1);
  EXPECT_EQ(nearestRgbPoseAnchor(anchors, 1480000000LL, -1), -1);
}

TEST(RgbGlobalMapMath, AppliesOptimizedTimesRawInverseInWorld)
{
  RgbPoseAnchor anchor;
  anchor.stamp_ns = 1250000000LL;
  anchor.raw_pose = Eigen::Translation3d(2, 4, 0) *
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ());
  anchor.optimized_pose = Eigen::Translation3d(4, 5, 1) *
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ());
  const Eigen::Vector3d point_in_body(3, 2, 0.7);
  const Eigen::Vector3d point_in_world = anchor.raw_pose * point_in_body;
  const Eigen::Vector3d corrected = rgbWorldCorrection(anchor) * point_in_world;
  EXPECT_LT((corrected - anchor.optimized_pose * point_in_body).norm(), 1e-10);
  EXPECT_TRUE(simple_loop_closure::validRgbWorldCorrection(rgbWorldCorrection(anchor)));
}

TEST(RgbGlobalMapMath, RejectsNonrigidCorrection)
{
  Eigen::Affine3d invalid = Eigen::Affine3d::Identity();
  invalid.linear()(0,0) = 0.8;
  EXPECT_FALSE(simple_loop_closure::validRgbWorldCorrection(invalid));
}

TEST(RgbGlobalMapMath, ChoosesEarlierAnchorOnTies)
{
  std::vector<RgbPoseAnchor> a(2);
  a[0].stamp_ns = 1000000000LL;
  a[1].stamp_ns = 1100000000LL;
  EXPECT_EQ(nearestRgbPoseAnchor(a,1050000000LL,50000000LL),0);
}
