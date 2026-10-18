#pragma once

#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace simple_loop_closure
{

class DeterministicLoopSchedule
{
public:
  using LoopEdge = std::pair<std::size_t, std::size_t>;

  explicit DeterministicLoopSchedule(const int loop_search_frame_interval)
  {
    if (loop_search_frame_interval < 0)
      throw std::invalid_argument("loop_search_frame_interval must not be negative");
    stride_ = static_cast<std::size_t>(loop_search_frame_interval) + 1U;
  }

  std::size_t nextOdometryId() const noexcept { return added_count_; }
  std::size_t addedCount() const noexcept { return added_count_; }
  std::size_t searchedCount() const noexcept { return searched_ids_.size(); }
  std::size_t acceptedCount() const noexcept { return accepted_edges_.size(); }
  std::size_t nextCandidateId() const noexcept { return next_candidate_id_; }
  std::size_t stride() const noexcept { return stride_; }

  bool candidateReady() const noexcept { return next_candidate_id_ < added_count_; }

  void markOdometryAdded(const std::size_t id)
  {
    if (id != added_count_)
      throw std::logic_error("odometry IDs must be committed in global order");
    ++added_count_;
  }

  void markCandidateSearched(const std::size_t id)
  {
    if (!candidateReady() || id != next_candidate_id_)
      throw std::logic_error("loop candidate IDs must be searched in global order");
    searched_ids_.push_back(id);
    next_candidate_id_ += stride_;
  }

  void recordAcceptedEdge(const std::size_t source_id, const std::size_t target_id)
  {
    if (searched_ids_.empty() || searched_ids_.back() != source_id)
      throw std::logic_error("an accepted edge must belong to the last searched candidate");
    if (target_id > source_id)
      throw std::logic_error("a loop edge must not target a future keyframe");
    accepted_edges_.emplace_back(source_id, target_id);
  }

  const std::vector<std::size_t>& searchedIds() const noexcept { return searched_ids_; }
  const std::vector<LoopEdge>& acceptedEdges() const noexcept { return accepted_edges_; }

  std::size_t expectedSearchedCount(const std::size_t keyframe_count) const noexcept
  {
    return keyframe_count == 0U ? 0U : ((keyframe_count - 1U) / stride_) + 1U;
  }

  bool searchedPrefixMatches(const std::size_t keyframe_count) const noexcept
  {
    if (searched_ids_.size() != expectedSearchedCount(keyframe_count))
      return false;
    for (std::size_t i = 0; i < searched_ids_.size(); ++i)
    {
      if (searched_ids_[i] != i * stride_)
        return false;
    }
    return true;
  }

  std::string searchedDigest() const { return digestIds(searched_ids_); }

  std::string acceptedDigest() const
  {
    std::uint64_t hash = kFnvOffset;
    for (const auto& edge : accepted_edges_)
    {
      hashInteger(hash, edge.first);
      hashByte(hash, 0xffU);
      hashInteger(hash, edge.second);
      hashByte(hash, 0xfeU);
    }
    return toHex(hash);
  }

private:
  static constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
  static constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

  static void hashByte(std::uint64_t& hash, const std::uint8_t value) noexcept
  {
    hash ^= value;
    hash *= kFnvPrime;
  }

  static void hashInteger(std::uint64_t& hash, std::size_t value) noexcept
  {
    for (std::size_t byte = 0; byte < sizeof(std::uint64_t); ++byte)
      hashByte(hash, static_cast<std::uint8_t>((static_cast<std::uint64_t>(value) >> (byte * 8U)) & 0xffU));
  }

  static std::string toHex(const std::uint64_t hash)
  {
    std::ostringstream stream;
    stream << std::hex << std::setfill('0') << std::setw(16) << hash;
    return stream.str();
  }

  static std::string digestIds(const std::vector<std::size_t>& ids)
  {
    std::uint64_t hash = kFnvOffset;
    for (const std::size_t id : ids)
    {
      hashInteger(hash, id);
      hashByte(hash, 0xfdU);
    }
    return toHex(hash);
  }

  std::size_t stride_{1U};
  std::size_t added_count_{0U};
  std::size_t next_candidate_id_{0U};
  std::vector<std::size_t> searched_ids_;
  std::vector<LoopEdge> accepted_edges_;
};

}  // namespace simple_loop_closure
