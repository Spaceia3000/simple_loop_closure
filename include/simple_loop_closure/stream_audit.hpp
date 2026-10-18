#pragma once

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace simple_loop_closure
{

constexpr std::uint64_t kFnv1a64OffsetBasis = 14695981039346656037ULL;
constexpr std::uint64_t kFnv1a64Prime = 1099511628211ULL;

inline std::uint64_t appendStampFnv1a64(std::uint64_t digest, const std::int64_t stamp_ns)
{
  std::uint64_t value = static_cast<std::uint64_t>(stamp_ns);
  for (unsigned int byte = 0; byte < 8; ++byte)
  {
    digest ^= value & 0xffU;
    digest *= kFnv1a64Prime;
    value >>= 8;
  }
  return digest;
}

// Exact repeated stamps in a monotonic stream are adjacent, so this remains
// constant-space while still exposing the expected sensor-stream failures.
struct TimestampSequenceAudit
{
  void observe(const std::int64_t stamp_ns)
  {
    digest = appendStampFnv1a64(digest, stamp_ns);
    if (count == 0)
      first_stamp_ns = stamp_ns;
    else
    {
      adjacent_duplicates += stamp_ns == last_stamp_ns;
      reversals += stamp_ns < last_stamp_ns;
    }
    last_stamp_ns = stamp_ns;
    ++count;
  }

  bool sameCountAndDigest(const TimestampSequenceAudit& other) const
  {
    return count == other.count && digest == other.digest;
  }

  std::uint64_t count{0};
  std::uint64_t digest{kFnv1a64OffsetBasis};
  std::uint64_t adjacent_duplicates{0};
  std::uint64_t reversals{0};
  std::int64_t first_stamp_ns{0};
  std::int64_t last_stamp_ns{0};
};

struct PairedTimestampSequenceAudit
{
  void observe(const std::int64_t cloud_stamp_ns, const std::int64_t odom_stamp_ns)
  {
    cloud.observe(cloud_stamp_ns);
    odom.observe(odom_stamp_ns);
    ordered_pair_digest = appendStampFnv1a64(ordered_pair_digest, cloud_stamp_ns);
    ordered_pair_digest = appendStampFnv1a64(ordered_pair_digest, odom_stamp_ns);
    mismatches += cloud_stamp_ns != odom_stamp_ns;
  }

  std::uint64_t count() const
  {
    return cloud.count;
  }

  TimestampSequenceAudit cloud;
  TimestampSequenceAudit odom;
  std::uint64_t ordered_pair_digest{kFnv1a64OffsetBasis};
  std::uint64_t mismatches{0};
};

struct StreamAudit
{
  TimestampSequenceAudit cloud_ingress;
  TimestampSequenceAudit odom_ingress;
  PairedTimestampSequenceAudit sync_callbacks;
  PairedTimestampSequenceAudit contract_rejected;
  PairedTimestampSequenceAudit contract_accepted;
  PairedTimestampSequenceAudit pcl_converted;
  // These describe only the local rclcpp Publisher::publish call boundary.
  // A normal return is not evidence that DDS delivered the sample.
  TimestampSequenceAudit pgo_publish_attempted;
  TimestampSequenceAudit pgo_publish_completed;
  TimestampSequenceAudit pgo_publish_failed;
  TimestampSequenceAudit keyframe_selected;
  TimestampSequenceAudit keyframe_skipped;
  TimestampSequenceAudit keyframes_inserted;
};

struct StreamAuditSnapshot
{
  StreamAudit audit;
  std::uint64_t keyframes_stored{0};
  std::uint64_t loop_factors{0};
};

inline void appendTimestampSequence(
    std::ostringstream& out, const char* name, const TimestampSequenceAudit& sequence)
{
  out << ' ' << name << "_count=" << sequence.count
      << ' ' << name << "_fnv64=0x" << std::hex << std::setw(16) << std::setfill('0')
      << sequence.digest << std::dec << std::setfill(' ')
      << ' ' << name << "_adjacent_duplicates=" << sequence.adjacent_duplicates
      << ' ' << name << "_reversals=" << sequence.reversals
      << ' ' << name << "_first_ns=" << sequence.first_stamp_ns
      << ' ' << name << "_last_ns=" << sequence.last_stamp_ns;
}

inline void appendPairSequence(
    std::ostringstream& out, const char* name, const PairedTimestampSequenceAudit& sequence)
{
  out << ' ' << name << "_count=" << sequence.count()
      << ' ' << name << "_pair_fnv64=0x" << std::hex << std::setw(16) << std::setfill('0')
      << sequence.ordered_pair_digest << std::dec << std::setfill(' ')
      << ' ' << name << "_mismatches=" << sequence.mismatches;
  const std::string cloud_name = std::string(name) + "_cloud";
  const std::string odom_name = std::string(name) + "_odom";
  appendTimestampSequence(out, cloud_name.c_str(), sequence.cloud);
  appendTimestampSequence(out, odom_name.c_str(), sequence.odom);
}

inline std::string formatStreamAuditReport(
    const StreamAuditSnapshot& snapshot, const char* report_kind)
{
  const StreamAudit& audit = snapshot.audit;
  const std::uint64_t sync_accounted =
      audit.contract_rejected.count() + audit.contract_accepted.count();
  const std::uint64_t keyframe_decisions =
      audit.keyframe_selected.count + audit.keyframe_skipped.count;
  const bool contract_accepted_to_pgo_attempted_count_ok =
      audit.contract_accepted.count() == audit.pgo_publish_attempted.count;
  const bool contract_accepted_to_pgo_attempted_ordered_digest_ok =
      audit.contract_accepted.odom.digest == audit.pgo_publish_attempted.digest;
  const std::uint64_t pgo_publish_outcomes =
      audit.pgo_publish_completed.count + audit.pgo_publish_failed.count;
  const bool pgo_publish_outcome_accounting_ok =
      audit.pgo_publish_attempted.count == pgo_publish_outcomes;
  const bool pgo_publish_no_failures = audit.pgo_publish_failed.count == 0;
  const bool pgo_publish_attempted_to_completed_count_ok =
      audit.pgo_publish_attempted.count == audit.pgo_publish_completed.count;
  const bool pgo_publish_attempted_to_completed_ordered_digest_ok =
      audit.pgo_publish_attempted.digest == audit.pgo_publish_completed.digest;
  const bool selected_to_pcl_converted_count_ok =
      audit.keyframe_selected.count == audit.pcl_converted.count();
  const bool selected_to_pcl_converted_ordered_digest_ok =
      audit.keyframe_selected.digest == audit.pcl_converted.odom.digest;
  const bool selected_to_inserted_count_ok =
      audit.keyframe_selected.count == audit.keyframes_inserted.count;
  const bool selected_to_inserted_ordered_digest_ok =
      audit.keyframe_selected.digest == audit.keyframes_inserted.digest;
  const bool sync_accounting_ok = audit.sync_callbacks.count() == sync_accounted;
  const bool keyframe_decision_accounting_ok =
      audit.contract_accepted.count() == keyframe_decisions;
  const bool inserted_to_storage_ok =
      audit.keyframes_inserted.count == snapshot.keyframes_stored;
  const bool internal_consistency_ok =
      sync_accounting_ok && contract_accepted_to_pgo_attempted_count_ok &&
      contract_accepted_to_pgo_attempted_ordered_digest_ok &&
      pgo_publish_outcome_accounting_ok && pgo_publish_no_failures &&
      pgo_publish_attempted_to_completed_count_ok &&
      pgo_publish_attempted_to_completed_ordered_digest_ok &&
      keyframe_decision_accounting_ok && selected_to_pcl_converted_count_ok &&
      selected_to_pcl_converted_ordered_digest_ok && selected_to_inserted_count_ok &&
      selected_to_inserted_ordered_digest_ok && inserted_to_storage_ok;
  const bool any_data =
      audit.cloud_ingress.count != 0 || audit.odom_ingress.count != 0 ||
      audit.sync_callbacks.count() != 0 || audit.contract_rejected.count() != 0 ||
      audit.contract_accepted.count() != 0 || audit.pcl_converted.count() != 0 ||
      audit.pgo_publish_attempted.count != 0 || audit.pgo_publish_completed.count != 0 ||
      audit.pgo_publish_failed.count != 0 || audit.keyframe_selected.count != 0 ||
      audit.keyframe_skipped.count != 0 || audit.keyframes_inserted.count != 0 ||
      snapshot.keyframes_stored != 0 || snapshot.loop_factors != 0;
  const bool final_report = std::string(report_kind) == "shutdown";
  const char* status = !any_data ? "NO_DATA"
                       : audit.sync_callbacks.count() == 0 ||
                                 (!internal_consistency_ok && !final_report)
                           ? "INCONCLUSIVE"
                       : internal_consistency_ok ? "PASS"
                                                 : "FAIL";

  std::ostringstream out;
  out << "SC_SIMPLE_STREAM_AUDIT version=6 report=" << report_kind
      << " status=" << status
      << " status_scope=internal_call_and_stage_accounting"
      << " external_source_count=not_observed"
      << " pgo_recorder=external_compare_required"
      << " pgo_publisher_qos=reliable_keep_last_1"
      << " dds_delivery_status=INCONCLUSIVE"
      << " pgo_publish_completed_semantics=rclcpp_publish_returned_normally";
  appendTimestampSequence(out, "cloud_ingress", audit.cloud_ingress);
  appendTimestampSequence(out, "odom_ingress", audit.odom_ingress);
  appendPairSequence(out, "sync_callback", audit.sync_callbacks);
  appendPairSequence(out, "contract_rejected", audit.contract_rejected);
  appendPairSequence(out, "contract_accepted", audit.contract_accepted);
  appendPairSequence(out, "pcl_converted", audit.pcl_converted);
  appendTimestampSequence(out, "pgo_publish_attempted", audit.pgo_publish_attempted);
  appendTimestampSequence(out, "pgo_publish_completed", audit.pgo_publish_completed);
  appendTimestampSequence(out, "pgo_publish_failed", audit.pgo_publish_failed);
  appendTimestampSequence(out, "keyframe_selected", audit.keyframe_selected);
  appendTimestampSequence(out, "keyframe_skipped", audit.keyframe_skipped);
  appendTimestampSequence(out, "keyframe_inserted", audit.keyframes_inserted);
  out << " sync_accounted=" << sync_accounted
      << " sync_accounting_ok="
      << (sync_accounting_ok ? "true" : "false")
      << " contract_accepted_to_pgo_attempted_count_ok="
      << (contract_accepted_to_pgo_attempted_count_ok ? "true" : "false")
      << " contract_accepted_to_pgo_attempted_ordered_digest_ok="
      << (contract_accepted_to_pgo_attempted_ordered_digest_ok ? "true" : "false")
      << " contract_accepted_to_pgo_attempted_ok="
      << (contract_accepted_to_pgo_attempted_count_ok &&
                  contract_accepted_to_pgo_attempted_ordered_digest_ok
              ? "true"
              : "false")
      << " pgo_publish_outcomes=" << pgo_publish_outcomes
      << " pgo_publish_outcome_accounting_ok="
      << (pgo_publish_outcome_accounting_ok ? "true" : "false")
      << " pgo_publish_no_failures=" << (pgo_publish_no_failures ? "true" : "false")
      << " pgo_publish_attempted_to_completed_count_ok="
      << (pgo_publish_attempted_to_completed_count_ok ? "true" : "false")
      << " pgo_publish_attempted_to_completed_ordered_digest_ok="
      << (pgo_publish_attempted_to_completed_ordered_digest_ok ? "true" : "false")
      << " keyframe_decisions=" << keyframe_decisions
      << " keyframe_decision_accounting_ok="
      << (keyframe_decision_accounting_ok ? "true" : "false")
      << " selected_to_pcl_converted_count_ok="
      << (selected_to_pcl_converted_count_ok ? "true" : "false")
      << " selected_to_pcl_converted_ordered_digest_ok="
      << (selected_to_pcl_converted_ordered_digest_ok ? "true" : "false")
      << " selected_to_pcl_converted_ok="
      << (selected_to_pcl_converted_count_ok && selected_to_pcl_converted_ordered_digest_ok
              ? "true"
              : "false")
      << " selected_to_inserted_count_ok="
      << (selected_to_inserted_count_ok ? "true" : "false")
      << " selected_to_inserted_ordered_digest_ok="
      << (selected_to_inserted_ordered_digest_ok ? "true" : "false")
      << " selected_to_inserted_ok="
      << (selected_to_inserted_count_ok && selected_to_inserted_ordered_digest_ok
              ? "true"
              : "false")
      << " inserted_to_storage_ok="
      << (inserted_to_storage_ok ? "true" : "false")
      << " keyframes_stored=" << snapshot.keyframes_stored
      << " loop_factors=" << snapshot.loop_factors;
  return out.str();
}

}  // namespace simple_loop_closure
