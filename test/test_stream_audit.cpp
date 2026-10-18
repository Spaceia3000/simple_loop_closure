#include <simple_loop_closure/stream_audit.hpp>

#include <gtest/gtest.h>

namespace
{

TEST(StreamAudit, PreservesOrderedFnvDigestInConstantSpace)
{
  simple_loop_closure::TimestampSequenceAudit audit;
  for (const std::int64_t stamp : {100LL, 101LL, 500LL, 501LL, 502LL})
    audit.observe(stamp);

  EXPECT_EQ(audit.count, 5U);
  EXPECT_EQ(audit.digest, 0x5d8f3933cc870eb4ULL);
  EXPECT_EQ(audit.first_stamp_ns, 100);
  EXPECT_EQ(audit.last_stamp_ns, 502);
  EXPECT_EQ(audit.adjacent_duplicates, 0U);
  EXPECT_EQ(audit.reversals, 0U);
}

TEST(StreamAudit, IndependentIngressExposesLossBeforeSynchronization)
{
  simple_loop_closure::StreamAudit audit;
  for (const std::int64_t stamp : {10LL, 20LL, 30LL, 40LL})
    audit.cloud_ingress.observe(stamp);
  for (const std::int64_t stamp : {10LL, 30LL, 40LL})
    audit.odom_ingress.observe(stamp);
  for (const std::int64_t stamp : {10LL, 30LL, 40LL})
    audit.sync_callbacks.observe(stamp, stamp);

  EXPECT_EQ(audit.cloud_ingress.count, 4U);
  EXPECT_EQ(audit.odom_ingress.count, 3U);
  EXPECT_EQ(audit.sync_callbacks.count(), 3U);
  EXPECT_FALSE(audit.cloud_ingress.sameCountAndDigest(audit.sync_callbacks.cloud));
  EXPECT_TRUE(audit.odom_ingress.sameCountAndDigest(audit.sync_callbacks.odom));
}

TEST(StreamAudit, AcceptedRejectedAndKeyframePathsAreFullyAccounted)
{
  simple_loop_closure::StreamAuditSnapshot snapshot;
  auto& audit = snapshot.audit;

  audit.sync_callbacks.observe(10, 10);
  audit.pcl_converted.observe(10, 10);
  audit.pgo_publish_calls.observe(10);
  audit.keyframe_selected.observe(10);
  audit.keyframes_inserted.observe(10);

  audit.sync_callbacks.observe(20, 21);
  audit.contract_rejected.observe(20, 21);

  audit.sync_callbacks.observe(30, 30);
  audit.pcl_converted.observe(30, 30);
  audit.pgo_publish_calls.observe(30);
  audit.keyframe_skipped.observe(30);

  snapshot.keyframes_stored = 1;
  const std::string line = simple_loop_closure::formatStreamAuditReport(snapshot, "snapshot");

  EXPECT_NE(line.find(" sync_callback_count=3 "), std::string::npos);
  EXPECT_NE(line.find(" contract_rejected_count=1 "), std::string::npos);
  EXPECT_NE(line.find(" pcl_converted_count=2 "), std::string::npos);
  EXPECT_NE(line.find(" pgo_publish_call_count=2 "), std::string::npos);
  EXPECT_NE(line.find(" keyframe_inserted_count=1 "), std::string::npos);
  EXPECT_NE(line.find(" sync_accounting_ok=true"), std::string::npos);
  EXPECT_NE(line.find(" converted_to_pgo_ok=true"), std::string::npos);
  EXPECT_NE(line.find(" keyframe_decision_accounting_ok=true"), std::string::npos);
  EXPECT_NE(line.find(" selected_to_inserted_ok=true"), std::string::npos);
  EXPECT_NE(line.find(" inserted_to_storage_ok=true"), std::string::npos);
}

TEST(StreamAudit, MissingTerminalStageCannotBeSilent)
{
  simple_loop_closure::StreamAuditSnapshot snapshot;
  snapshot.audit.sync_callbacks.observe(100, 100);

  const std::string line = simple_loop_closure::formatStreamAuditReport(snapshot, "shutdown");
  EXPECT_NE(line.find(" sync_accounted=0"), std::string::npos);
  EXPECT_NE(line.find(" sync_accounting_ok=false"), std::string::npos);
}

TEST(StreamAudit, DepthOnePgoLossIsVisibleAgainstRecorderCountAndDigest)
{
  simple_loop_closure::TimestampSequenceAudit emitted;
  for (const std::int64_t stamp : {1000LL, 2000LL, 3000LL})
    emitted.observe(stamp);

  // A slow recorder observing only the newest sample from KEEP_LAST(1) cannot
  // match the node's publish-call sequence.
  simple_loop_closure::TimestampSequenceAudit lossy_recorder;
  lossy_recorder.observe(3000);
  EXPECT_FALSE(emitted.sameCountAndDigest(lossy_recorder));

  simple_loop_closure::TimestampSequenceAudit complete_recorder;
  for (const std::int64_t stamp : {1000LL, 2000LL, 3000LL})
    complete_recorder.observe(stamp);
  EXPECT_TRUE(emitted.sameCountAndDigest(complete_recorder));

  simple_loop_closure::TimestampSequenceAudit reordered_recorder;
  for (const std::int64_t stamp : {1000LL, 3000LL, 2000LL})
    reordered_recorder.observe(stamp);
  EXPECT_EQ(emitted.count, reordered_recorder.count);
  EXPECT_NE(emitted.digest, reordered_recorder.digest);
}

TEST(StreamAudit, ShutdownReportLabelsExternalRecorderComparison)
{
  const simple_loop_closure::StreamAuditSnapshot snapshot;
  const std::string snapshot_line =
      simple_loop_closure::formatStreamAuditReport(snapshot, "snapshot");
  const std::string shutdown_line =
      simple_loop_closure::formatStreamAuditReport(snapshot, "shutdown");

  EXPECT_NE(snapshot_line, shutdown_line);
  EXPECT_EQ(shutdown_line.find('\n'), std::string::npos);
  EXPECT_EQ(shutdown_line.find('\r'), std::string::npos);
  EXPECT_EQ(shutdown_line.rfind("SC_SIMPLE_STREAM_AUDIT version=4 report=shutdown", 0), 0U);
  EXPECT_NE(shutdown_line.find(" pgo_recorder=external_compare_required"), std::string::npos);
  EXPECT_NE(shutdown_line.find(" pgo_publisher_qos=reliable_keep_last_1"), std::string::npos);
}

}  // namespace
