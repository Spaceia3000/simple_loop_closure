#include <simple_loop_closure/keyframe_gate.hpp>
#include <simple_loop_closure/math_safety.hpp>
#include <simple_loop_closure/stream_audit.hpp>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
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

Eigen::Affine3d quaternionPose(const Eigen::Quaterniond &rotation)
{
  Eigen::Affine3d result = Eigen::Affine3d::Identity();
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

TEST(KeyframeGate, NextafterBracketsExactTranslationAndAngularEquality)
{
  const Eigen::Affine3d previous = pose();
  const Eigen::Affine3d translated = pose(Eigen::Vector3d(kTranslationThreshold, 0.0, 0.0));
  const auto translation_measurement = simple_loop_closure::decideKeyframe(
      &previous, translated, 1.0, 1.0);
  const double translation = translation_measurement.translation_distance;

  EXPECT_TRUE(simple_loop_closure::decideKeyframe(
      &previous, translated, std::nextafter(translation, 0.0), 1.0).is_keyframe);
  EXPECT_TRUE(simple_loop_closure::decideKeyframe(
      &previous, translated, translation, 1.0).is_keyframe);
  EXPECT_FALSE(simple_loop_closure::decideKeyframe(
      &previous, translated, std::nextafter(translation, 1.0), 1.0).is_keyframe);

  const Eigen::Affine3d rotated = pose(
      Eigen::Vector3d::Zero(), Eigen::AngleAxisd(kAngularThreshold, Eigen::Vector3d::UnitY()));
  const auto angular_measurement = simple_loop_closure::decideKeyframe(
      &previous, rotated, 1.0, 1.0);
  const double angle = angular_measurement.angular_distance;

  EXPECT_TRUE(simple_loop_closure::decideKeyframe(
      &previous, rotated, 1.0, std::nextafter(angle, 0.0)).is_keyframe);
  EXPECT_TRUE(simple_loop_closure::decideKeyframe(
      &previous, rotated, 1.0, angle).is_keyframe);
  EXPECT_FALSE(simple_loop_closure::decideKeyframe(
      &previous, rotated, 1.0, std::nextafter(angle, 1.0)).is_keyframe);
}

TEST(KeyframeGate, QuaternionSignDoesNotChangeAngularDistanceOrDecision)
{
  const Eigen::Affine3d previous = pose();
  Eigen::Quaterniond rotation(Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond negated = rotation;
  negated.coeffs() *= -1.0;

  const auto positive = simple_loop_closure::decideKeyframe(
      &previous, quaternionPose(rotation), 1.0, kAngularThreshold);
  const auto negative = simple_loop_closure::decideKeyframe(
      &previous, quaternionPose(negated), 1.0, kAngularThreshold);

  EXPECT_DOUBLE_EQ(positive.angular_distance, negative.angular_distance);
  EXPECT_EQ(positive.is_keyframe, negative.is_keyframe);
  EXPECT_TRUE(positive.is_keyframe);
}

TEST(KeyframeGate, WrappedYawAcrossPlusMinus179UsesShortestRotation)
{
  const double pi = std::acos(-1.0);
  const double yaw_179 = 179.0 * pi / 180.0;
  const Eigen::Affine3d positive = pose(
      Eigen::Vector3d::Zero(), Eigen::AngleAxisd(yaw_179, Eigen::Vector3d::UnitZ()));
  const Eigen::Affine3d negative = pose(
      Eigen::Vector3d::Zero(), Eigen::AngleAxisd(-yaw_179, Eigen::Vector3d::UnitZ()));

  const auto forward = simple_loop_closure::decideKeyframe(&positive, negative, 1.0, 0.04);
  const auto reverse = simple_loop_closure::decideKeyframe(&negative, positive, 1.0, 0.04);

  EXPECT_NEAR(forward.angular_distance, 2.0 * pi / 180.0, 1e-12);
  EXPECT_DOUBLE_EQ(forward.angular_distance, reverse.angular_distance);
  EXPECT_FALSE(forward.is_keyframe);
  EXPECT_FALSE(reverse.is_keyframe);
}

TEST(KeyframeGate, ZeroThresholdPreservesInclusiveBoundarySemantics)
{
  const Eigen::Affine3d previous = pose();
  const auto decision = simple_loop_closure::decideKeyframe(&previous, pose(), 0.0, 0.0);

  EXPECT_TRUE(decision.is_keyframe);
  EXPECT_DOUBLE_EQ(decision.translation_distance, 0.0);
  EXPECT_DOUBLE_EQ(decision.angular_distance, 0.0);
}

TEST(MathSafetyParameters, RejectsEveryNonfiniteAndNegativeRequiredDomain)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const double negative = -std::numeric_limits<double>::denorm_min();

  for (const double invalid : {nan, infinity, -infinity, negative})
  {
    EXPECT_THROW(
        simple_loop_closure::validateMathSafetyParameters(invalid, 0.2, 0.01, 3),
        std::invalid_argument);
    EXPECT_THROW(
        simple_loop_closure::validateMathSafetyParameters(0.3, invalid, 0.01, 3),
        std::invalid_argument);
    EXPECT_THROW(
        simple_loop_closure::validateMathSafetyParameters(0.3, 0.2, invalid, 3),
        std::invalid_argument);
  }
  EXPECT_THROW(
      simple_loop_closure::validateMathSafetyParameters(0.3, 0.2, 0.01, -1),
      std::invalid_argument);
}

TEST(MathSafetyParameters, AcceptsZeroAndFiniteNonnegativeBoundaries)
{
  EXPECT_NO_THROW(simple_loop_closure::validateMathSafetyParameters(0.0, 0.0, 0.0, 0));
  EXPECT_NO_THROW(simple_loop_closure::validateMathSafetyParameters(
      std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
      std::numeric_limits<double>::max(), std::numeric_limits<int>::max()));
}

TEST(VisualizationLineWidth, EnforcesFiniteInclusiveBounds)
{
  EXPECT_DOUBLE_EQ(
      simple_loop_closure::validateVisualizationLineWidth("width", 0.002), 0.002);
  EXPECT_DOUBLE_EQ(
      simple_loop_closure::validateVisualizationLineWidth("width", 0.10), 0.10);
  EXPECT_DOUBLE_EQ(
      simple_loop_closure::validateVisualizationLineWidth("width", 0.20), 0.20);
  EXPECT_THROW(
      simple_loop_closure::validateVisualizationLineWidth("width", 0.0019),
      std::invalid_argument);
  EXPECT_THROW(
      simple_loop_closure::validateVisualizationLineWidth("width", 0.2001),
      std::invalid_argument);
  EXPECT_THROW(
      simple_loop_closure::validateVisualizationLineWidth(
          "width", std::numeric_limits<double>::quiet_NaN()),
      std::invalid_argument);
  EXPECT_THROW(
      simple_loop_closure::validateVisualizationLineWidth(
          "width", std::numeric_limits<double>::infinity()),
      std::invalid_argument);
}

TEST(SyncQueueSize, EnforcesOperationalMemorySafetyBoundaries)
{
  EXPECT_THROW(simple_loop_closure::validateSyncQueueSize(0), std::invalid_argument);
  EXPECT_EQ(simple_loop_closure::validateSyncQueueSize(1), 1U);
  EXPECT_EQ(simple_loop_closure::validateSyncQueueSize(50), 50U);
  EXPECT_EQ(simple_loop_closure::validateSyncQueueSize(128), 128U);
  EXPECT_EQ(
      simple_loop_closure::validateSyncQueueSize(simple_loop_closure::kMaxSyncQueueSize),
      1024U);
  EXPECT_THROW(simple_loop_closure::validateSyncQueueSize(1025), std::invalid_argument);
  EXPECT_THROW(
      simple_loop_closure::validateSyncQueueSize(
          std::numeric_limits<std::uint32_t>::max()),
      std::invalid_argument);
}

TEST(PgoOutputDepth, EnforcesOperationalResourceBoundaries)
{
  EXPECT_THROW(simple_loop_closure::validatePgoOutputDepth(0), std::invalid_argument);
  EXPECT_EQ(simple_loop_closure::validatePgoOutputDepth(1), 1U);
  EXPECT_EQ(simple_loop_closure::validatePgoOutputDepth(10), 10U);
  EXPECT_EQ(simple_loop_closure::validatePgoOutputDepth(100), 100U);
  EXPECT_EQ(
      simple_loop_closure::validatePgoOutputDepth(simple_loop_closure::kMaxPgoOutputDepth),
      1024U);
  EXPECT_THROW(simple_loop_closure::validatePgoOutputDepth(1025), std::invalid_argument);
  EXPECT_THROW(
      simple_loop_closure::validatePgoOutputDepth(std::numeric_limits<std::int64_t>::max()),
      std::invalid_argument);
}

TEST(CloudInputDepth, ValidatesSignedValueBeforeBoundedUnsignedConversion)
{
  EXPECT_THROW(
      simple_loop_closure::validateCloudInputDepth(std::numeric_limits<std::int64_t>::min()),
      std::invalid_argument);
  EXPECT_THROW(simple_loop_closure::validateCloudInputDepth(-1), std::invalid_argument);
  EXPECT_THROW(simple_loop_closure::validateCloudInputDepth(0), std::invalid_argument);
  EXPECT_EQ(simple_loop_closure::validateCloudInputDepth(1), 1U);
  EXPECT_EQ(simple_loop_closure::validateCloudInputDepth(10), 10U);
  EXPECT_EQ(simple_loop_closure::validateCloudInputDepth(100), 100U);
  EXPECT_EQ(
      simple_loop_closure::validateCloudInputDepth(
          simple_loop_closure::kMaxCloudInputDepth),
      1024U);
  EXPECT_THROW(simple_loop_closure::validateCloudInputDepth(1025), std::invalid_argument);
  EXPECT_THROW(
      simple_loop_closure::validateCloudInputDepth(std::numeric_limits<std::int64_t>::max()),
      std::invalid_argument);
}

TEST(KeyframeGate, ThresholdCrossingsRemainConsistentAcrossAuditedStages)
{
  const Eigen::Affine3d previous = pose();
  const std::array<double, 3> distances{
      std::nextafter(kTranslationThreshold, 0.0),
      kTranslationThreshold,
      std::nextafter(kTranslationThreshold, 1.0)};
  simple_loop_closure::StreamAuditSnapshot snapshot;

  for (std::size_t index = 0; index < distances.size(); ++index)
  {
    const std::int64_t stamp = static_cast<std::int64_t>((index + 1) * 100);
    const auto decision = simple_loop_closure::decideKeyframe(
        &previous, pose(Eigen::Vector3d(distances[index], 0.0, 0.0)),
        kTranslationThreshold, kAngularThreshold);
    auto& audit = snapshot.audit;
    audit.sync_callbacks.observe(stamp, stamp);
    audit.contract_accepted.observe(stamp, stamp);
    audit.pgo_publish_attempted.observe(stamp);
    audit.pgo_publish_completed.observe(stamp);
    if (decision.is_keyframe)
    {
      audit.keyframe_selected.observe(stamp);
      audit.pcl_converted.observe(stamp, stamp);
      audit.keyframes_inserted.observe(stamp);
      ++snapshot.keyframes_stored;
    }
    else
    {
      audit.keyframe_skipped.observe(stamp);
    }
  }

  EXPECT_EQ(snapshot.audit.keyframe_skipped.count, 1U);
  EXPECT_EQ(snapshot.audit.keyframe_selected.count, 2U);
  const std::string report =
      simple_loop_closure::formatStreamAuditReport(snapshot, "snapshot");
  EXPECT_NE(report.find(" status=PASS "), std::string::npos);
  EXPECT_NE(report.find(" contract_accepted_to_pgo_attempted_ok=true"), std::string::npos);
  EXPECT_NE(report.find(" selected_to_inserted_ok=true"), std::string::npos);
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
  const std::size_t accepted_audit = callback.find("contract_accepted.observe");
  const std::size_t publish = callback.find("publishPoseGraphOptimizedOdometry");
  const std::size_t gate_return = callback.find("if (!decision.is_keyframe)");
  const std::size_t deserialize = callback.find("pcl::fromROSMsg");
  const std::size_t inverse_transform = callback.find("pcl::transformPointCloud");
  ASSERT_NE(validation, std::string::npos);
  ASSERT_NE(accepted_audit, std::string::npos);
  ASSERT_NE(publish, std::string::npos);
  ASSERT_NE(gate_return, std::string::npos);
  ASSERT_NE(deserialize, std::string::npos);
  ASSERT_NE(inverse_transform, std::string::npos);
  EXPECT_LT(validation, accepted_audit);
  EXPECT_LT(accepted_audit, publish);
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
  const std::size_t converted_audit = callback.find("pcl_converted.observe");
  const std::size_t inserted_audit = callback.find("keyframes_inserted.observe");
  ASSERT_NE(decision, std::string::npos);
  ASSERT_NE(selected_audit, std::string::npos);
  ASSERT_NE(skipped_audit, std::string::npos);
  ASSERT_NE(gate_return, std::string::npos);
  ASSERT_NE(deserialize, std::string::npos);
  ASSERT_NE(converted_audit, std::string::npos);
  ASSERT_NE(inserted_audit, std::string::npos);
  EXPECT_LT(decision, selected_audit);
  EXPECT_LT(decision, skipped_audit);
  EXPECT_LT(selected_audit, gate_return);
  EXPECT_LT(skipped_audit, gate_return);
  EXPECT_LT(gate_return, deserialize);
  EXPECT_LT(deserialize, converted_audit);
  EXPECT_LT(converted_audit, inserted_audit);
}

}  // namespace
