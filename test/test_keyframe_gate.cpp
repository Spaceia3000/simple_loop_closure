#include <simple_loop_closure/keyframe_gate.hpp>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace
{

constexpr double kTranslationThreshold = 0.3;
constexpr double kAngularThreshold = 0.2;

Eigen::Affine3d pose(
    const Eigen::Vector3d &translation = Eigen::Vector3d::Zero(),
    const Eigen::AngleAxisd &rotation = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()))
{
  Eigen::Affine3d result = Eigen::Affine3d::Identity();
  result.translation() = translation;
  result.linear() = rotation.toRotationMatrix();
  return result;
}

TEST(KeyframeGate, FirstFrameIsKeyframeAtZeroTrajectoryDistance)
{
  const auto decision = simple_loop_closure::decideKeyframe(
      nullptr, pose(), kTranslationThreshold, kAngularThreshold);

  EXPECT_TRUE(decision.is_keyframe);
  EXPECT_DOUBLE_EQ(decision.translation_distance, 0.0);
  EXPECT_DOUBLE_EQ(decision.angular_distance, 0.0);
  EXPECT_DOUBLE_EQ(simple_loop_closure::nextTrajectoryDistance(true, 17.0, decision), 0.0);
}

TEST(KeyframeGate, NoMotionIsNotKeyframe)
{
  const Eigen::Affine3d previous = pose();
  const auto decision = simple_loop_closure::decideKeyframe(
      &previous, pose(), kTranslationThreshold, kAngularThreshold);

  EXPECT_FALSE(decision.is_keyframe);
  EXPECT_DOUBLE_EQ(decision.translation_distance, 0.0);
  EXPECT_DOUBLE_EQ(decision.angular_distance, 0.0);
}

TEST(KeyframeGate, RotationOnlyUsesFullThreeDimensionalAngle)
{
  const Eigen::Affine3d previous = pose();
  const auto decision = simple_loop_closure::decideKeyframe(
      &previous, pose(Eigen::Vector3d::Zero(), Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitX())),
      kTranslationThreshold, kAngularThreshold);

  EXPECT_TRUE(decision.is_keyframe);
  EXPECT_DOUBLE_EQ(decision.translation_distance, 0.0);
  EXPECT_NEAR(decision.angular_distance, 0.25, 1e-12);
}

TEST(KeyframeGate, DistanceOnlyIsKeyframe)
{
  const Eigen::Affine3d previous = pose();
  const auto decision = simple_loop_closure::decideKeyframe(
      &previous, pose(Eigen::Vector3d(0.31, 0.0, 0.0)),
      kTranslationThreshold, kAngularThreshold);

  EXPECT_TRUE(decision.is_keyframe);
  EXPECT_NEAR(decision.translation_distance, 0.31, 1e-12);
  EXPECT_DOUBLE_EQ(decision.angular_distance, 0.0);
}

TEST(KeyframeGate, EqualityAtEitherThresholdIsKeyframe)
{
  const Eigen::Affine3d previous = pose();
  const auto distance_boundary = simple_loop_closure::decideKeyframe(
      &previous, pose(Eigen::Vector3d(kTranslationThreshold, 0.0, 0.0)),
      kTranslationThreshold, kAngularThreshold);
  const auto rotation_boundary = simple_loop_closure::decideKeyframe(
      &previous,
      pose(Eigen::Vector3d::Zero(), Eigen::AngleAxisd(kAngularThreshold, Eigen::Vector3d::UnitY())),
      kTranslationThreshold, kAngularThreshold);

  EXPECT_TRUE(distance_boundary.is_keyframe);
  EXPECT_TRUE(rotation_boundary.is_keyframe);
}

TEST(KeyframeGate, AcceptedKeyframeTrajectoryDistanceIsMonotonic)
{
  std::vector<Eigen::Affine3d> accepted_poses{pose()};
  std::vector<double> trajectory_distances{0.0};
  const std::vector<Eigen::Affine3d> input_poses{
      pose(Eigen::Vector3d(0.1, 0.0, 0.0)),
      pose(Eigen::Vector3d(0.3, 0.0, 0.0)),
      pose(Eigen::Vector3d(0.3, 0.0, 0.0), Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ())),
      pose(Eigen::Vector3d(0.7, 0.0, 0.0))};

  for (const auto &current_pose : input_poses)
  {
    const auto decision = simple_loop_closure::decideKeyframe(
        &accepted_poses.back(), current_pose, kTranslationThreshold, kAngularThreshold);
    if (!decision.is_keyframe)
      continue;
    trajectory_distances.push_back(simple_loop_closure::nextTrajectoryDistance(
        false, trajectory_distances.back(), decision));
    accepted_poses.push_back(current_pose);
  }

  ASSERT_EQ(trajectory_distances.size(), 4U);
  for (std::size_t i = 1; i < trajectory_distances.size(); ++i)
    EXPECT_GE(trajectory_distances[i], trajectory_distances[i - 1]);
  EXPECT_DOUBLE_EQ(trajectory_distances[1], 0.3);
  EXPECT_DOUBLE_EQ(trajectory_distances[2], 0.3);
  EXPECT_DOUBLE_EQ(trajectory_distances[3], 0.7);
}

TEST(KeyframeCallbackStatic, PublishesPgoPerAcceptedPairBeforeCheapEarlyReturn)
{
  const std::string source_path =
      std::string(SLC_SOURCE_DIR) + "/src/simple_loop_closure_node.cpp";
  std::ifstream source_file(source_path);
  ASSERT_TRUE(source_file.is_open());
  const std::string source{
      std::istreambuf_iterator<char>(source_file), std::istreambuf_iterator<char>()};
  const std::size_t callback_begin = source.find(
      "void SimpleLoopClosureNode::pointCloudAndOdometryCallback");
  const std::size_t callback_end = source.find(
      "pcl::PointCloud<pcl::PointXYZI>::Ptr SimpleLoopClosureNode::constructPointCloudMap",
      callback_begin);
  ASSERT_NE(callback_begin, std::string::npos);
  ASSERT_NE(callback_end, std::string::npos);
  const std::string callback = source.substr(callback_begin, callback_end - callback_begin);

  const std::size_t validation = callback.find("if (!inputContractValid");
  const std::size_t publish = callback.find("publishPoseGraphOptimizedOdometry");
  const std::size_t gate_return = callback.find("if (!decision.is_keyframe)");
  const std::size_t deserialize = callback.find("pcl::fromROSMsg");
  const std::size_t inverse_transform = callback.find("pcl::transformPointCloud");
  ASSERT_NE(validation, std::string::npos);
  ASSERT_NE(publish, std::string::npos);
  ASSERT_NE(gate_return, std::string::npos);
  ASSERT_NE(deserialize, std::string::npos);
  ASSERT_NE(inverse_transform, std::string::npos);
  EXPECT_LT(validation, publish);
  EXPECT_LT(publish, gate_return);
  EXPECT_LT(gate_return, deserialize);
  EXPECT_LT(deserialize, inverse_transform);
  EXPECT_EQ(callback.find("publishPoseGraphOptimizedOdometry", publish + 1), std::string::npos);
}

TEST(KeyframeCallbackStatic, AuditsSelectedAndSkippedStampsAroundEarlyGate)
{
  const std::string source_path =
      std::string(SLC_SOURCE_DIR) + "/src/simple_loop_closure_node.cpp";
  std::ifstream source_file(source_path);
  ASSERT_TRUE(source_file.is_open());
  const std::string source{
      std::istreambuf_iterator<char>(source_file), std::istreambuf_iterator<char>()};
  const std::size_t callback_begin = source.find(
      "void SimpleLoopClosureNode::pointCloudAndOdometryCallback");
  const std::size_t callback_end = source.find(
      "pcl::PointCloud<pcl::PointXYZI>::Ptr SimpleLoopClosureNode::constructPointCloudMap",
      callback_begin);
  ASSERT_NE(callback_begin, std::string::npos);
  ASSERT_NE(callback_end, std::string::npos);
  const std::string callback = source.substr(callback_begin, callback_end - callback_begin);

  const std::size_t decision = callback.find("decideKeyframe");
  const std::size_t selected_audit = callback.find("keyframe_selected.observe");
  const std::size_t skipped_audit = callback.find("keyframe_skipped.observe");
  const std::size_t gate_return = callback.find("if (!decision.is_keyframe)");
  const std::size_t deserialize = callback.find("pcl::fromROSMsg");
  const std::size_t inserted_audit = callback.find("keyframes_inserted.observe");
  ASSERT_NE(decision, std::string::npos);
  ASSERT_NE(selected_audit, std::string::npos);
  ASSERT_NE(skipped_audit, std::string::npos);
  ASSERT_NE(gate_return, std::string::npos);
  ASSERT_NE(deserialize, std::string::npos);
  ASSERT_NE(inserted_audit, std::string::npos);
  EXPECT_LT(decision, selected_audit);
  EXPECT_LT(decision, skipped_audit);
  EXPECT_LT(selected_audit, gate_return);
  EXPECT_LT(skipped_audit, gate_return);
  EXPECT_LT(gate_return, deserialize);
  EXPECT_LT(deserialize, inserted_audit);
}

}  // namespace
