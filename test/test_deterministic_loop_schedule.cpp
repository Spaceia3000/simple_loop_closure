#include <gtest/gtest.h>

#include <simple_loop_closure/deterministic_loop_schedule.hpp>

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <thread>
#include <vector>

namespace
{

using Schedule = simple_loop_closure::DeterministicLoopSchedule;

struct Result
{
  std::size_t added_count;
  std::vector<std::size_t> searched_ids;
  std::vector<Schedule::LoopEdge> accepted_edges;
  std::string searched_digest;
  std::string accepted_digest;
};

Result runArrivalPattern(const std::vector<std::size_t>& batch_sizes, const int interval)
{
  Schedule schedule(interval);
  std::size_t available = 0U;
  for (const std::size_t batch_size : batch_sizes)
  {
    available += batch_size;
    while (schedule.nextOdometryId() < available)
    {
      const std::size_t id = schedule.nextOdometryId();
      schedule.markOdometryAdded(id);
      if (schedule.candidateReady())
      {
        const std::size_t source_id = schedule.nextCandidateId();
        schedule.markCandidateSearched(source_id);
        if (source_id >= schedule.stride())
          schedule.recordAcceptedEdge(source_id, source_id - schedule.stride());
      }
    }
  }
  return {schedule.addedCount(), schedule.searchedIds(), schedule.acceptedEdges(),
          schedule.searchedDigest(), schedule.acceptedDigest()};
}

TEST(DeterministicLoopSchedule, OneByOneAndBurstInsertionsMatch)
{
  const Result one_by_one = runArrivalPattern(std::vector<std::size_t>(17U, 1U), 2);
  const Result burst = runArrivalPattern({5U, 1U, 8U, 3U}, 2);

  EXPECT_EQ(one_by_one.added_count, burst.added_count);
  EXPECT_EQ(one_by_one.searched_ids, burst.searched_ids);
  EXPECT_EQ(one_by_one.accepted_edges, burst.accepted_edges);
  EXPECT_EQ(one_by_one.searched_digest, burst.searched_digest);
  EXPECT_EQ(one_by_one.accepted_digest, burst.accepted_digest);
}

TEST(DeterministicLoopSchedule, TerminalTailIsAddedButNotInventedAsCandidate)
{
  Schedule schedule(3);
  for (std::size_t id = 0; id < 10U; ++id)
  {
    schedule.markOdometryAdded(id);
    if (schedule.candidateReady())
      schedule.markCandidateSearched(schedule.nextCandidateId());
  }

  EXPECT_EQ(schedule.addedCount(), 10U);
  EXPECT_EQ(schedule.searchedIds(), (std::vector<std::size_t>{0U, 4U, 8U}));
  EXPECT_TRUE(schedule.searchedPrefixMatches(10U));
  EXPECT_EQ(schedule.nextCandidateId(), 12U);
}

TEST(DeterministicLoopSchedule, AcceptedEdgesAndDigestsAreReproducible)
{
  const Result first = runArrivalPattern({23U}, 1);
  const Result second = runArrivalPattern({2U, 7U, 1U, 13U}, 1);

  ASSERT_EQ(first.accepted_edges, second.accepted_edges);
  EXPECT_FALSE(first.accepted_edges.empty());
  EXPECT_EQ(first.searched_digest, second.searched_digest);
  EXPECT_EQ(first.accepted_digest, second.accepted_digest);
}

TEST(DeterministicLoopSchedule, ProducerConsumerConcurrencyPreservesGlobalOrder)
{
  constexpr std::size_t keyframe_count = 257U;
  Schedule schedule(4);
  std::mutex mutex;
  std::condition_variable changed;
  std::size_t available = 0U;
  bool producer_done = false;

  std::thread producer([&]() {
    for (std::size_t id = 0; id < keyframe_count; ++id)
    {
      {
        std::lock_guard<std::mutex> guard(mutex);
        ++available;
      }
      changed.notify_one();
    }
    {
      std::lock_guard<std::mutex> guard(mutex);
      producer_done = true;
    }
    changed.notify_one();
  });

  std::thread consumer([&]() {
    for (;;)
    {
      std::size_t available_snapshot;
      {
        std::unique_lock<std::mutex> lock(mutex);
        changed.wait(lock, [&]() { return schedule.nextOdometryId() < available || producer_done; });
        available_snapshot = available;
        if (schedule.nextOdometryId() == available_snapshot && producer_done)
          break;
      }
      while (schedule.nextOdometryId() < available_snapshot)
      {
        schedule.markOdometryAdded(schedule.nextOdometryId());
        if (schedule.candidateReady())
          schedule.markCandidateSearched(schedule.nextCandidateId());
      }
    }
  });

  producer.join();
  consumer.join();

  EXPECT_EQ(schedule.addedCount(), keyframe_count);
  EXPECT_TRUE(schedule.searchedPrefixMatches(keyframe_count));
}

TEST(DeterministicLoopSchedule, RejectsOutOfOrderOrFutureBookkeeping)
{
  Schedule schedule(1);
  EXPECT_THROW(schedule.markOdometryAdded(1U), std::logic_error);
  schedule.markOdometryAdded(0U);
  EXPECT_THROW(schedule.markCandidateSearched(2U), std::logic_error);
  schedule.markCandidateSearched(0U);
  EXPECT_THROW(schedule.recordAcceptedEdge(0U, 1U), std::logic_error);
}

}  // namespace
