#include <simple_loop_closure/fast_livo2_contract.hpp>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <limits>
#include <string>
#include <vector>

namespace
{

struct TestPointField
{
  std::string name;
  std::uint8_t datatype;
  std::uint32_t count;
};

TEST(FastLivo2Contract, AcceptsFrozenFrontendFramesAndExactStamp)
{
  const simple_loop_closure::FastLivo2Contract contract;
  EXPECT_TRUE(contract.framesMatch("camera_init", "camera_init", "aft_mapped"));
  EXPECT_TRUE(contract.stampsMatch(290869707354LL, 290869707354LL));
}

TEST(FastLivo2Contract, RejectsWrongOrEmptyFrames)
{
  const simple_loop_closure::FastLivo2Contract contract;
  EXPECT_FALSE(contract.framesMatch("", "camera_init", "aft_mapped"));
  EXPECT_FALSE(contract.framesMatch("livox", "camera_init", "aft_mapped"));
  EXPECT_FALSE(contract.framesMatch("camera_init", "map", "aft_mapped"));
  EXPECT_FALSE(contract.framesMatch("camera_init", "camera_init", "base_link"));
}

TEST(FastLivo2Contract, RejectsNearButNonidenticalStampByDefault)
{
  simple_loop_closure::FastLivo2Contract contract;
  EXPECT_FALSE(contract.stampsMatch(1000000000LL, 1000000001LL));
  contract.require_exact_stamp = false;
  EXPECT_TRUE(contract.stampsMatch(1000000000LL, 1000000001LL));
}

TEST(FastLivo2Contract, AcceptsFiniteNormalizedPose)
{
  const simple_loop_closure::FastLivo2Contract contract;
  EXPECT_TRUE(contract.poseIsValid(1.0, -2.0, 3.0, 0.0, 0.0, 0.0, 1.0));
  EXPECT_TRUE(contract.poseIsValid(0.0, 0.0, 0.0, 0.0, 0.0, 0.7071067811865475, 0.7071067811865475));
}

TEST(FastLivo2Contract, RejectsInvalidPose)
{
  const simple_loop_closure::FastLivo2Contract contract;
  EXPECT_FALSE(contract.poseIsValid(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(contract.poseIsValid(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0));
  EXPECT_FALSE(contract.poseIsValid(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0));
}

TEST(FastLivo2Contract, AcceptsPointXyziFloat32Schema)
{
  const simple_loop_closure::FastLivo2Contract contract;
  const std::vector<TestPointField> fields{
      {"x", 7, 1}, {"y", 7, 1}, {"z", 7, 1}, {"intensity", 7, 1}};
  EXPECT_TRUE(contract.cloudFieldsMatch(fields, 7));
}

TEST(FastLivo2Contract, RejectsIncompatibleCloudSchema)
{
  const simple_loop_closure::FastLivo2Contract contract;
  const std::vector<TestPointField> missing_intensity{
      {"x", 7, 1}, {"y", 7, 1}, {"z", 7, 1}};
  const std::vector<TestPointField> double_precision{
      {"x", 8, 1}, {"y", 8, 1}, {"z", 8, 1}, {"intensity", 8, 1}};
  EXPECT_FALSE(contract.cloudFieldsMatch(missing_intensity, 7));
  EXPECT_FALSE(contract.cloudFieldsMatch(double_precision, 7));
}

TEST(FastLivo2Contract, FixedBodyRoundTripIsIdentity)
{
  Eigen::Affine3d fixed_T_body = Eigen::Affine3d::Identity();
  fixed_T_body.translation() = Eigen::Vector3d(2.5, -1.25, 0.75);
  fixed_T_body.linear() =
      (Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()))
          .toRotationMatrix();
  const Eigen::Vector3d point_fixed(8.0, -3.0, 1.5);

  const Eigen::Vector3d point_body =
      simple_loop_closure::globalPointToLocal(fixed_T_body, point_fixed);
  const Eigen::Vector3d recovered_fixed =
      simple_loop_closure::localPointToGlobal(fixed_T_body, point_body);

  EXPECT_TRUE(recovered_fixed.isApprox(point_fixed, 1e-12));
}

TEST(FastLivo2Contract, IdentityPoseDoesNotMoveCloudPoint)
{
  const Eigen::Affine3d identity = Eigen::Affine3d::Identity();
  const Eigen::Vector3d point(1.0, 2.0, 3.0);
  EXPECT_TRUE(simple_loop_closure::globalPointToLocal(identity, point).isApprox(point, 0.0));
}

}  // namespace
