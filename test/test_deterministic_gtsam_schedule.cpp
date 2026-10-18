#include <gtest/gtest.h>

#include <simple_loop_closure/deterministic_loop_schedule.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <cstddef>
#include <string>
#include <vector>

namespace
{

struct GraphRun
{
  gtsam::Values estimate;
  std::size_t factor_count;
  std::string searched_digest;
  std::string accepted_digest;
};

GraphRun runGraph(const std::vector<std::size_t>& batches)
{
  simple_loop_closure::DeterministicLoopSchedule schedule(2);
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  gtsam::ISAM2 isam(parameters);
  const auto prior_noise = gtsam::noiseModel::Isotropic::Variance(6, 1e-12);
  const auto odometry_noise = gtsam::noiseModel::Isotropic::Variance(6, 1e-6);
  const auto loop_noise = gtsam::noiseModel::Isotropic::Variance(6, 1e-4);

  std::size_t available = 0U;
  for (const std::size_t batch : batches)
  {
    available += batch;
    while (schedule.nextOdometryId() < available)
    {
      const std::size_t id = schedule.nextOdometryId();
      const gtsam::Pose3 raw_pose(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.01 * id),
                                 gtsam::Point3(static_cast<double>(id), 0.1 * id, 0.0));
      gtsam::NonlinearFactorGraph graph;
      gtsam::Values initial;
      if (id == 0U)
      {
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(0U, raw_pose, prior_noise));
        initial.insert(0U, raw_pose);
      }
      else
      {
        const gtsam::Pose3 raw_previous(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.01 * (id - 1U)),
                                       gtsam::Point3(static_cast<double>(id - 1U), 0.1 * (id - 1U), 0.0));
        const gtsam::Pose3 delta = raw_previous.between(raw_pose);
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(id - 1U, id, delta, odometry_noise));
        initial.insert(id, isam.calculateEstimate<gtsam::Pose3>(id - 1U).compose(delta));
      }
      isam.update(graph, initial);
      isam.update();
      schedule.markOdometryAdded(id);

      if (!schedule.candidateReady())
        continue;
      const std::size_t source_id = schedule.nextCandidateId();
      schedule.markCandidateSearched(source_id);
      if (source_id < schedule.stride())
        continue;

      const std::size_t target_id = source_id - schedule.stride();
      const gtsam::Pose3 source = isam.calculateEstimate<gtsam::Pose3>(source_id);
      const gtsam::Pose3 target = isam.calculateEstimate<gtsam::Pose3>(target_id);
      gtsam::NonlinearFactorGraph loop_graph;
      loop_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
          source_id, target_id, source.between(target), loop_noise));
      isam.update(loop_graph);
      isam.update();
      schedule.recordAcceptedEdge(source_id, target_id);
    }
  }

  return {isam.calculateEstimate(), isam.getFactorsUnsafe().size(),
          schedule.searchedDigest(), schedule.acceptedDigest()};
}

TEST(DeterministicGtsamSchedule, OneByOneAndBurstProduceSameGraphState)
{
  const GraphRun one_by_one = runGraph(std::vector<std::size_t>(19U, 1U));
  const GraphRun burst = runGraph({7U, 12U});

  ASSERT_EQ(one_by_one.factor_count, burst.factor_count);
  ASSERT_EQ(one_by_one.estimate.size(), burst.estimate.size());
  EXPECT_EQ(one_by_one.searched_digest, burst.searched_digest);
  EXPECT_EQ(one_by_one.accepted_digest, burst.accepted_digest);
  for (std::size_t id = 0; id < one_by_one.estimate.size(); ++id)
  {
    const gtsam::Pose3 lhs = one_by_one.estimate.at<gtsam::Pose3>(id);
    const gtsam::Pose3 rhs = burst.estimate.at<gtsam::Pose3>(id);
    EXPECT_TRUE(lhs.equals(rhs, 1e-12)) << "pose " << id << " differs";
  }
}

}  // namespace
