#include <simple_loop_closure/ingress_telemetry.hpp>

#include <gtest/gtest.h>

namespace
{

TEST(IngressTelemetry, AggregatesIngressIntervalsAndLatestPeerAgeWithoutSamples)
{
  simple_loop_closure::IngressTelemetry telemetry;
  telemetry.observeCloudArrival(100);
  telemetry.observeOdomArrival(130);
  telemetry.observeCloudArrival(180);
  telemetry.observeOdomArrival(250);

  const auto snapshot = telemetry.snapshot();
  EXPECT_EQ(snapshot.cloud.callback_count, 2U);
  EXPECT_EQ(snapshot.cloud.interarrival.count, 1U);
  EXPECT_EQ(snapshot.cloud.interarrival.total_ns, 80U);
  EXPECT_EQ(snapshot.cloud.latest_peer_age.count, 1U);
  EXPECT_EQ(snapshot.cloud.latest_peer_age.peak_ns, 50U);
  EXPECT_EQ(snapshot.odom.callback_count, 2U);
  EXPECT_EQ(snapshot.odom.interarrival.peak_ns, 120U);
  EXPECT_EQ(snapshot.odom.latest_peer_age.count, 2U);
  EXPECT_EQ(snapshot.odom.latest_peer_age.total_ns, 100U);
}

TEST(IngressTelemetry, ServiceThresholdsAreInclusiveAndNested)
{
  simple_loop_closure::AtomicDurationCounter<true> counter;
  for (const std::uint64_t duration : {
      99999999ULL, 100000000ULL, 499999999ULL, 500000000ULL, 1000000000ULL})
  {
    counter.observe(duration);
  }

  const auto snapshot = counter.snapshot();
  EXPECT_EQ(snapshot.count, 5U);
  EXPECT_EQ(snapshot.peak_ns, 1000000000U);
  EXPECT_EQ(snapshot.ge_100ms, 4U);
  EXPECT_EQ(snapshot.ge_500ms, 2U);
  EXPECT_EQ(snapshot.ge_1s, 1U);
}

TEST(IngressTelemetry, IngressAggregatesCompileOutServiceThresholdAccounting)
{
  simple_loop_closure::AtomicDurationCounter<false> counter;
  counter.observe(1000000000ULL);

  const auto snapshot = counter.snapshot();
  EXPECT_EQ(snapshot.count, 1U);
  EXPECT_EQ(snapshot.peak_ns, 1000000000U);
  EXPECT_EQ(snapshot.ge_100ms, 0U);
  EXPECT_EQ(snapshot.ge_500ms, 0U);
  EXPECT_EQ(snapshot.ge_1s, 0U);
}

TEST(IngressTelemetry, EventCountersRetainCallbacksChangesAndLastRmwValues)
{
  simple_loop_closure::AtomicSubscriptionEvents events;
  events.setSupported(true);
  events.observeMessageLost(3, 3);
  events.observeMessageLost(5, 2);
  events.observeIncompatibleQos(1, 1, 7);

  const auto snapshot = events.snapshot();
  EXPECT_TRUE(snapshot.supported);
  EXPECT_EQ(snapshot.message_lost_callbacks, 2U);
  EXPECT_EQ(snapshot.message_lost_total_change, 5U);
  EXPECT_EQ(snapshot.message_lost_rmw_total_last, 5);
  EXPECT_EQ(snapshot.incompatible_qos_callbacks, 1U);
  EXPECT_EQ(snapshot.incompatible_qos_total_change, 1U);
  EXPECT_EQ(snapshot.incompatible_qos_rmw_total_last, 1);
  EXPECT_EQ(snapshot.incompatible_qos_last_policy_kind, 7);
}

TEST(IngressTelemetry, ResetReturnsEveryCounterToDisabledEmptyState)
{
  simple_loop_closure::IngressTelemetry telemetry;
  telemetry.observeCloudArrival(100);
  telemetry.observeOdomArrival(200);
  telemetry.cloud_events.setSupported(true);
  telemetry.cloud_events.observeMessageLost(4, 4);
  telemetry.odom_events.setSupported(true);
  telemetry.odom_events.observeIncompatibleQos(2, 2, 5);
  telemetry.synchronized_callback_service.observe(1000000000ULL);

  telemetry.reset();
  const auto snapshot = telemetry.snapshot();
  EXPECT_EQ(snapshot.cloud.callback_count, 0U);
  EXPECT_EQ(snapshot.odom.callback_count, 0U);
  EXPECT_EQ(snapshot.cloud.interarrival.count, 0U);
  EXPECT_EQ(snapshot.odom.latest_peer_age.count, 0U);
  EXPECT_FALSE(snapshot.cloud_events.supported);
  EXPECT_FALSE(snapshot.odom_events.supported);
  EXPECT_EQ(snapshot.cloud_events.message_lost_callbacks, 0U);
  EXPECT_EQ(snapshot.odom_events.incompatible_qos_callbacks, 0U);
  EXPECT_EQ(snapshot.synchronized_callback_service.count, 0U);
  EXPECT_EQ(snapshot.synchronized_callback_service.peak_ns, 0U);
}

TEST(IngressTelemetry, ReportIsOneBoundedLineAndDisclaimsCausalAttribution)
{
  simple_loop_closure::IngressTelemetry telemetry;
  const std::string report = simple_loop_closure::formatIngressTelemetryReport(
      telemetry.snapshot(), "reliable_keep_last_100", "reliable_keep_last_10");

  EXPECT_EQ(report.find('\n'), std::string::npos);
  EXPECT_EQ(report.find('\r'), std::string::npos);
  EXPECT_EQ(report.rfind("SLC_C1_RUNTIME_TELEMETRY version=1 report=shutdown enabled=true", 0), 0U);
  EXPECT_NE(report.find("causal_attribution=not_established"), std::string::npos);
  EXPECT_NE(report.find("missing_samples_may_be_unreported"), std::string::npos);
  EXPECT_NE(report.find("sync_service_ge_100ms=0"), std::string::npos);
}

}  // namespace
