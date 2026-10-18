#include <gtest/gtest.h>

#include <message_filters/simple_filter.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{

using Cloud = sensor_msgs::msg::PointCloud2;
using Odometry = nav_msgs::msg::Odometry;
using ExactPolicy = message_filters::sync_policies::ExactTime<Cloud, Odometry>;
using ExactSynchronizer = message_filters::Synchronizer<ExactPolicy>;

template<typename Message>
class TestSource : public message_filters::SimpleFilter<Message>
{
public:
  void add(const std::shared_ptr<const Message>& message)
  {
    this->signalMessage(message);
  }
};

template<typename Message>
std::shared_ptr<Message> stampedMessage(
    const std::uint32_t nanosec, const std::string& marker = {})
{
  auto message = std::make_shared<Message>();
  message->header.stamp.nanosec = nanosec;
  message->header.frame_id = marker;
  return message;
}

struct PairRecord
{
  std::uint32_t cloud_stamp;
  std::uint32_t odom_stamp;
  std::string cloud_marker;
};

struct PairRecorder
{
  void callback(const Cloud::ConstSharedPtr& cloud, const Odometry::ConstSharedPtr& odom)
  {
    pairs.push_back(PairRecord{
        cloud->header.stamp.nanosec,
        odom->header.stamp.nanosec,
        cloud->header.frame_id});
  }

  std::vector<PairRecord> pairs;
};

class ExactTimeSyncTest : public testing::Test
{
protected:
  explicit ExactTimeSyncTest(const std::uint32_t queue_size = 8)
  : synchronizer(ExactPolicy(queue_size), clouds, odometry)
  {
    synchronizer.registerCallback(&PairRecorder::callback, &recorder);
  }

  TestSource<Cloud> clouds;
  TestSource<Odometry> odometry;
  ExactSynchronizer synchronizer;
  PairRecorder recorder;
};

TEST_F(ExactTimeSyncTest, PairsOnlyEqualNanosecondStamps)
{
  clouds.add(stampedMessage<Cloud>(354U));
  odometry.add(stampedMessage<Odometry>(354U));
  clouds.add(stampedMessage<Cloud>(500U));
  odometry.add(stampedMessage<Odometry>(501U));

  ASSERT_EQ(recorder.pairs.size(), 1U);
  EXPECT_EQ(recorder.pairs[0].cloud_stamp, 354U);
  EXPECT_EQ(recorder.pairs[0].odom_stamp, 354U);
}

TEST_F(ExactTimeSyncTest, OutOfOrderArrivalsPairWhenMatchesArriveInTimestampOrder)
{
  clouds.add(stampedMessage<Cloud>(300U));
  clouds.add(stampedMessage<Cloud>(100U));
  clouds.add(stampedMessage<Cloud>(200U));
  odometry.add(stampedMessage<Odometry>(100U));
  odometry.add(stampedMessage<Odometry>(200U));
  odometry.add(stampedMessage<Odometry>(300U));

  ASSERT_EQ(recorder.pairs.size(), 3U);
  EXPECT_EQ(recorder.pairs[0].cloud_stamp, 100U);
  EXPECT_EQ(recorder.pairs[1].cloud_stamp, 200U);
  EXPECT_EQ(recorder.pairs[2].cloud_stamp, 300U);
}

TEST_F(ExactTimeSyncTest, UnmatchedTimestampsNeverFormACrossTimestampPair)
{
  clouds.add(stampedMessage<Cloud>(100U));
  odometry.add(stampedMessage<Odometry>(101U));
  EXPECT_TRUE(recorder.pairs.empty());

  clouds.add(stampedMessage<Cloud>(101U));
  ASSERT_EQ(recorder.pairs.size(), 1U);
  EXPECT_EQ(recorder.pairs[0].cloud_stamp, 101U);
  EXPECT_EQ(recorder.pairs[0].odom_stamp, 101U);

  odometry.add(stampedMessage<Odometry>(100U));
  EXPECT_EQ(recorder.pairs.size(), 1U);
}

TEST(ExactTimeSync, QueueEvictsSmallestUnmatchedTimestampAtCapacity)
{
  TestSource<Cloud> clouds;
  TestSource<Odometry> odometry;
  ExactSynchronizer synchronizer(ExactPolicy(2), clouds, odometry);
  PairRecorder recorder;
  synchronizer.registerCallback(&PairRecorder::callback, &recorder);

  clouds.add(stampedMessage<Cloud>(300U));
  clouds.add(stampedMessage<Cloud>(100U));
  clouds.add(stampedMessage<Cloud>(200U));
  odometry.add(stampedMessage<Odometry>(100U));
  odometry.add(stampedMessage<Odometry>(200U));
  odometry.add(stampedMessage<Odometry>(300U));

  ASSERT_EQ(recorder.pairs.size(), 2U);
  EXPECT_EQ(recorder.pairs[0].cloud_stamp, 200U);
  EXPECT_EQ(recorder.pairs[1].cloud_stamp, 300U);
}

TEST_F(ExactTimeSyncTest, SameStampDuplicateReplacesPendingMessageInItsSlot)
{
  clouds.add(stampedMessage<Cloud>(100U, "first"));
  clouds.add(stampedMessage<Cloud>(100U, "second"));
  odometry.add(stampedMessage<Odometry>(100U));

  ASSERT_EQ(recorder.pairs.size(), 1U);
  EXPECT_EQ(recorder.pairs[0].cloud_marker, "second");
}

TEST_F(ExactTimeSyncTest, NewerCompletionClearsOlderPendingTuple)
{
  clouds.add(stampedMessage<Cloud>(100U));
  clouds.add(stampedMessage<Cloud>(200U));
  odometry.add(stampedMessage<Odometry>(200U));

  ASSERT_EQ(recorder.pairs.size(), 1U);
  EXPECT_EQ(recorder.pairs[0].cloud_stamp, 200U);

  // Completion at T purges pending tuples with stamp <= T, so the old cloud
  // cannot pair when its odometry counterpart arrives later.
  odometry.add(stampedMessage<Odometry>(100U));
  EXPECT_EQ(recorder.pairs.size(), 1U);
}

TEST_F(ExactTimeSyncTest, RecreatedOlderTupleCanEmitAfterNewerTuple)
{
  clouds.add(stampedMessage<Cloud>(200U));
  odometry.add(stampedMessage<Odometry>(200U));

  // The purge is not a permanent lower timestamp bound: recreating a complete
  // old tuple after T has emitted produces the callback sequence T, then t<T.
  clouds.add(stampedMessage<Cloud>(100U));
  odometry.add(stampedMessage<Odometry>(100U));

  ASSERT_EQ(recorder.pairs.size(), 2U);
  EXPECT_EQ(recorder.pairs[0].cloud_stamp, 200U);
  EXPECT_EQ(recorder.pairs[1].cloud_stamp, 100U);
}

TEST_F(ExactTimeSyncTest, FullyCompletedSameStampCanEmitTwice)
{
  clouds.add(stampedMessage<Cloud>(100U, "first"));
  odometry.add(stampedMessage<Odometry>(100U));
  clouds.add(stampedMessage<Cloud>(100U, "second"));
  odometry.add(stampedMessage<Odometry>(100U));

  ASSERT_EQ(recorder.pairs.size(), 2U);
  EXPECT_EQ(recorder.pairs[0].cloud_marker, "first");
  EXPECT_EQ(recorder.pairs[1].cloud_marker, "second");
}

TEST(ExactTimeSync, DelayedCloudStreamNeedsQAtLeastLToPairEveryInput)
{
  constexpr std::uint32_t message_count = 8U;
  constexpr std::uint32_t cloud_lag = 3U;

  const auto run_stream = [=](const std::uint32_t queue_size) {
    TestSource<Cloud> clouds;
    TestSource<Odometry> odometry;
    ExactSynchronizer synchronizer(ExactPolicy(queue_size), clouds, odometry);
    PairRecorder recorder;
    synchronizer.registerCallback(&PairRecorder::callback, &recorder);

    // Every stamp 1..N is injected exactly once on both topics. A cloud for
    // stamp s arrives L odometry arrivals later, immediately before s+L.
    for (std::uint32_t stamp = 1U; stamp <= message_count; ++stamp)
    {
      if (stamp > cloud_lag)
        clouds.add(stampedMessage<Cloud>(stamp - cloud_lag));
      odometry.add(stampedMessage<Odometry>(stamp));
    }
    for (std::uint32_t stamp = message_count - cloud_lag + 1U;
         stamp <= message_count; ++stamp)
    {
      clouds.add(stampedMessage<Cloud>(stamp));
    }

    std::vector<std::uint32_t> paired_stamps;
    for (const auto& pair : recorder.pairs)
      paired_stamps.push_back(pair.cloud_stamp);
    return paired_stamps;
  };

  // L > Q evicts the oldest pending odometry before its cloud arrives; only
  // the final Q odometry tuples survive when no newer odometry is inserted.
  EXPECT_EQ(run_stream(cloud_lag - 1U), (std::vector<std::uint32_t>{7U, 8U}));

  // Q >= L retains every delayed counterpart for this explicitly ordered
  // stream, so all N complete tuples emit in timestamp order.
  EXPECT_EQ(
      run_stream(cloud_lag),
      (std::vector<std::uint32_t>{1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U}));
}

}  // namespace
