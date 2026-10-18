#pragma once

#include <atomic>
#include <cstdint>
#include <sstream>
#include <string>

namespace simple_loop_closure
{

struct DurationCounterSnapshot
{
  std::uint64_t count{0};
  std::uint64_t total_ns{0};
  std::uint64_t peak_ns{0};
  std::uint64_t ge_100ms{0};
  std::uint64_t ge_500ms{0};
  std::uint64_t ge_1s{0};
};

template<bool TrackThresholds>
class AtomicDurationCounter
{
public:
  void observe(const std::uint64_t duration_ns) noexcept
  {
    count_.fetch_add(1, std::memory_order_relaxed);
    total_ns_.fetch_add(duration_ns, std::memory_order_relaxed);
    updateMaximum(peak_ns_, duration_ns);
    if constexpr (TrackThresholds)
    {
      if (duration_ns >= 100000000ULL)
        ge_100ms_.fetch_add(1, std::memory_order_relaxed);
      if (duration_ns >= 500000000ULL)
        ge_500ms_.fetch_add(1, std::memory_order_relaxed);
      if (duration_ns >= 1000000000ULL)
        ge_1s_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  DurationCounterSnapshot snapshot() const noexcept
  {
    return {
      count_.load(std::memory_order_relaxed),
      total_ns_.load(std::memory_order_relaxed),
      peak_ns_.load(std::memory_order_relaxed),
      ge_100ms_.load(std::memory_order_relaxed),
      ge_500ms_.load(std::memory_order_relaxed),
      ge_1s_.load(std::memory_order_relaxed)};
  }

  // Reset is intended for initialization and tests, not concurrently with observe().
  void reset() noexcept
  {
    count_.store(0, std::memory_order_relaxed);
    total_ns_.store(0, std::memory_order_relaxed);
    peak_ns_.store(0, std::memory_order_relaxed);
    ge_100ms_.store(0, std::memory_order_relaxed);
    ge_500ms_.store(0, std::memory_order_relaxed);
    ge_1s_.store(0, std::memory_order_relaxed);
  }

private:
  static void updateMaximum(
      std::atomic<std::uint64_t>& destination, const std::uint64_t candidate) noexcept
  {
    std::uint64_t current = destination.load(std::memory_order_relaxed);
    while (current < candidate &&
           !destination.compare_exchange_weak(
               current, candidate, std::memory_order_relaxed, std::memory_order_relaxed))
    {
    }
  }

  std::atomic<std::uint64_t> count_{0};
  std::atomic<std::uint64_t> total_ns_{0};
  std::atomic<std::uint64_t> peak_ns_{0};
  std::atomic<std::uint64_t> ge_100ms_{0};
  std::atomic<std::uint64_t> ge_500ms_{0};
  std::atomic<std::uint64_t> ge_1s_{0};
};

struct IngressStreamSnapshot
{
  std::uint64_t callback_count{0};
  DurationCounterSnapshot interarrival;
  DurationCounterSnapshot latest_peer_age;
};

class AtomicIngressStream
{
public:
  void observeArrival(const std::uint64_t now_ns, const std::uint64_t latest_peer_ns) noexcept
  {
    callback_count_.fetch_add(1, std::memory_order_relaxed);
    const std::uint64_t previous = latest_arrival_ns_.exchange(now_ns, std::memory_order_relaxed);
    if (previous != 0)
      interarrival_.observe(absoluteDifference(now_ns, previous));
    if (latest_peer_ns != 0)
      latest_peer_age_.observe(absoluteDifference(now_ns, latest_peer_ns));
  }

  std::uint64_t latestArrivalNs() const noexcept
  {
    return latest_arrival_ns_.load(std::memory_order_relaxed);
  }

  IngressStreamSnapshot snapshot() const noexcept
  {
    return {
      callback_count_.load(std::memory_order_relaxed),
      interarrival_.snapshot(), latest_peer_age_.snapshot()};
  }

  void reset() noexcept
  {
    callback_count_.store(0, std::memory_order_relaxed);
    latest_arrival_ns_.store(0, std::memory_order_relaxed);
    interarrival_.reset();
    latest_peer_age_.reset();
  }

private:
  static std::uint64_t absoluteDifference(
      const std::uint64_t lhs, const std::uint64_t rhs) noexcept
  {
    return lhs >= rhs ? lhs - rhs : rhs - lhs;
  }

  std::atomic<std::uint64_t> callback_count_{0};
  std::atomic<std::uint64_t> latest_arrival_ns_{0};
  AtomicDurationCounter<false> interarrival_;
  AtomicDurationCounter<false> latest_peer_age_;
};

struct SubscriptionEventSnapshot
{
  bool supported{false};
  std::uint64_t message_lost_callbacks{0};
  std::uint64_t message_lost_total_change{0};
  std::uint64_t message_lost_rmw_total_last{0};
  std::uint64_t incompatible_qos_callbacks{0};
  std::uint64_t incompatible_qos_total_change{0};
  std::int64_t incompatible_qos_rmw_total_last{0};
  std::int32_t incompatible_qos_last_policy_kind{0};
};

class AtomicSubscriptionEvents
{
public:
  void setSupported(const bool supported) noexcept
  {
    supported_.store(supported, std::memory_order_relaxed);
  }

  void observeMessageLost(
      const std::uint64_t total_count, const std::uint64_t total_count_change) noexcept
  {
    message_lost_callbacks_.fetch_add(1, std::memory_order_relaxed);
    message_lost_total_change_.fetch_add(total_count_change, std::memory_order_relaxed);
    message_lost_rmw_total_last_.store(total_count, std::memory_order_relaxed);
  }

  void observeIncompatibleQos(
      const std::int32_t total_count, const std::int32_t total_count_change,
      const std::int32_t last_policy_kind) noexcept
  {
    incompatible_qos_callbacks_.fetch_add(1, std::memory_order_relaxed);
    if (total_count_change > 0)
      incompatible_qos_total_change_.fetch_add(
          static_cast<std::uint64_t>(total_count_change), std::memory_order_relaxed);
    incompatible_qos_rmw_total_last_.store(total_count, std::memory_order_relaxed);
    incompatible_qos_last_policy_kind_.store(last_policy_kind, std::memory_order_relaxed);
  }

  SubscriptionEventSnapshot snapshot() const noexcept
  {
    return {
      supported_.load(std::memory_order_relaxed),
      message_lost_callbacks_.load(std::memory_order_relaxed),
      message_lost_total_change_.load(std::memory_order_relaxed),
      message_lost_rmw_total_last_.load(std::memory_order_relaxed),
      incompatible_qos_callbacks_.load(std::memory_order_relaxed),
      incompatible_qos_total_change_.load(std::memory_order_relaxed),
      incompatible_qos_rmw_total_last_.load(std::memory_order_relaxed),
      incompatible_qos_last_policy_kind_.load(std::memory_order_relaxed)};
  }

  void reset() noexcept
  {
    supported_.store(false, std::memory_order_relaxed);
    message_lost_callbacks_.store(0, std::memory_order_relaxed);
    message_lost_total_change_.store(0, std::memory_order_relaxed);
    message_lost_rmw_total_last_.store(0, std::memory_order_relaxed);
    incompatible_qos_callbacks_.store(0, std::memory_order_relaxed);
    incompatible_qos_total_change_.store(0, std::memory_order_relaxed);
    incompatible_qos_rmw_total_last_.store(0, std::memory_order_relaxed);
    incompatible_qos_last_policy_kind_.store(0, std::memory_order_relaxed);
  }

private:
  std::atomic<bool> supported_{false};
  std::atomic<std::uint64_t> message_lost_callbacks_{0};
  std::atomic<std::uint64_t> message_lost_total_change_{0};
  std::atomic<std::uint64_t> message_lost_rmw_total_last_{0};
  std::atomic<std::uint64_t> incompatible_qos_callbacks_{0};
  std::atomic<std::uint64_t> incompatible_qos_total_change_{0};
  std::atomic<std::int64_t> incompatible_qos_rmw_total_last_{0};
  std::atomic<std::int32_t> incompatible_qos_last_policy_kind_{0};
};

struct IngressTelemetrySnapshot
{
  IngressStreamSnapshot cloud;
  IngressStreamSnapshot odom;
  SubscriptionEventSnapshot cloud_events;
  SubscriptionEventSnapshot odom_events;
  DurationCounterSnapshot synchronized_callback_service;
};

class IngressTelemetry
{
public:
  void observeCloudArrival(const std::uint64_t now_ns) noexcept
  {
    cloud_.observeArrival(now_ns, odom_.latestArrivalNs());
  }

  void observeOdomArrival(const std::uint64_t now_ns) noexcept
  {
    odom_.observeArrival(now_ns, cloud_.latestArrivalNs());
  }

  IngressTelemetrySnapshot snapshot() const noexcept
  {
    return {
      cloud_.snapshot(), odom_.snapshot(), cloud_events.snapshot(), odom_events.snapshot(),
      synchronized_callback_service.snapshot()};
  }

  void reset() noexcept
  {
    cloud_.reset();
    odom_.reset();
    cloud_events.reset();
    odom_events.reset();
    synchronized_callback_service.reset();
  }

  AtomicSubscriptionEvents cloud_events;
  AtomicSubscriptionEvents odom_events;
  AtomicDurationCounter<true> synchronized_callback_service;

private:
  AtomicIngressStream cloud_;
  AtomicIngressStream odom_;
};

inline void appendDurationCounter(
    std::ostringstream& out, const char* name, const DurationCounterSnapshot& counter)
{
  out << ' ' << name << "_count=" << counter.count
      << ' ' << name << "_total_ns=" << counter.total_ns
      << ' ' << name << "_peak_ns=" << counter.peak_ns
      << ' ' << name << "_ge_100ms=" << counter.ge_100ms
      << ' ' << name << "_ge_500ms=" << counter.ge_500ms
      << ' ' << name << "_ge_1s=" << counter.ge_1s;
}

inline void appendIngressStream(
    std::ostringstream& out, const char* name, const IngressStreamSnapshot& stream)
{
  out << ' ' << name << "_callback_count=" << stream.callback_count;
  appendDurationCounter(out, (std::string(name) + "_interarrival").c_str(), stream.interarrival);
  appendDurationCounter(out, (std::string(name) + "_latest_peer_age").c_str(), stream.latest_peer_age);
}

inline void appendSubscriptionEvents(
    std::ostringstream& out, const char* name, const SubscriptionEventSnapshot& events)
{
  out << ' ' << name << "_event_support=" << (events.supported ? "true" : "false")
      << ' ' << name << "_message_lost_callbacks=" << events.message_lost_callbacks
      << ' ' << name << "_message_lost_total_change=" << events.message_lost_total_change
      << ' ' << name << "_message_lost_rmw_total_last=" << events.message_lost_rmw_total_last
      << ' ' << name << "_incompatible_qos_callbacks=" << events.incompatible_qos_callbacks
      << ' ' << name << "_incompatible_qos_total_change=" << events.incompatible_qos_total_change
      << ' ' << name << "_incompatible_qos_rmw_total_last="
      << events.incompatible_qos_rmw_total_last
      << ' ' << name << "_incompatible_qos_last_policy_kind="
      << events.incompatible_qos_last_policy_kind;
}

inline std::string formatIngressTelemetryReport(
    const IngressTelemetrySnapshot& snapshot, const std::string& cloud_qos,
    const std::string& odom_qos)
{
  std::ostringstream out;
  out << "SLC_C1_RUNTIME_TELEMETRY version=1 report=shutdown enabled=true"
      << " clock=steady_clock durations_unit=ns"
      << " latest_peer_age_semantics=absolute_age_to_most_recent_opposite_stream_callback"
      << " event_counts_semantics=rmw_reported_only_missing_samples_may_be_unreported"
      << " causal_attribution=not_established"
      << " cloud_subscriber_qos=" << cloud_qos
      << " odom_subscriber_qos=" << odom_qos;
  appendIngressStream(out, "cloud", snapshot.cloud);
  appendIngressStream(out, "odom", snapshot.odom);
  appendSubscriptionEvents(out, "cloud", snapshot.cloud_events);
  appendSubscriptionEvents(out, "odom", snapshot.odom_events);
  appendDurationCounter(out, "sync_service", snapshot.synchronized_callback_service);
  return out.str();
}

}  // namespace simple_loop_closure
