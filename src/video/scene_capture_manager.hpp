/*
 * Defines capture-manager state, submission tickets and completed frames. The manager turns the capture
 * evidence the hooks record on each tracked command list (d3d12_objects.hpp) into private GPU copies placed
 * on the actual queue timeline, keeping ownership, ordering and retirement explicit.
 */

#pragma once
#include <array>
#include <atomic>
#include <mutex>
#include "../shared/configuration.hpp"
#include "d3d12_objects.hpp"
#include "queue_hooks.hpp"
#include "scene_capture_d3d12.hpp"
#include "scene_handoff.hpp"
#include "scene_source_state.hpp"

namespace azp {

/*
 * Process-lifetime manager for the simulator's one device. Every native DIRECT queue must be hooked
 * (queue_hooks.cpp) before recorded captures/consumers are admitted; a failed queue observation calls
 * submission_refused. Submission entry points never hold a lock needed by another submission. It guards the
 * CaptureRecording part of every tracked list with its mutex. No method calls the private simulator engine.
 */
class SceneCaptureManager {
 public:
  static constexpr std::size_t MaximumPackets = 16;
  static constexpr std::uint64_t MaximumBytes = 256ull * 1024 * 1024;
  using List = standalone::List;
  struct FrameOrder {
    /*
     * Actual serialized device timeline and list position within that submit. Allocation tokens and CPU
     * retirement times do not order rendered images.
     */
    std::uint64_t submission = 0;
    std::uint32_t position = 0;
    bool newer_than(const FrameOrder& previous) const noexcept {
      return submission && position &&
             (submission > previous.submission || (submission == previous.submission && position > previous.position));
    }
  };
  struct Frame {
    std::uint64_t token = 0;
    SceneCopyMatch match;
    ID3D12Resource* resource = nullptr; /* Borrowed owned snapshot, COPY_DEST. */
    FrameOrder order;
    /*
     * CPU monotonic time immediately before the native producer submission.
     */
    std::uint64_t capture_time_us = 0;
  };
  struct Submission {
    std::uint64_t receipt = 0;
    ID3D12Fence* fence = nullptr; /* Process-lifetime borrowed timeline. */
    std::uint64_t value = 0;      /* Queued only by successful end. */
  };
  /*
   * `captures` counts recorded copies and `source_draws` draws observed on a camera source.
   * `escaped_invalidations` counts unordered submissions whose effect on capture could not be excluded;
   * `tail_status` says why the last queue tail did or did not capture.
   */
  struct InvalidationStatistics {
    std::uint64_t global = 0, scoped = 0;
    std::uint64_t pass_begin = 0, pass_state = 0, barrier_batch = 0, split_barrier = 0, alias_or_discard = 0;
    std::uint64_t unobserved_work = 0, observer_disabled = 0, reset_failed = 0;
    std::uint64_t execute_bundle = 0, execute_indirect = 0, predication = 0;
    std::uint64_t unknown_batches = 0, unknown_lists = 0, invalid_recordings = 0;
    std::uint64_t discard_calls = 0, source_discards = 0;
    std::uint64_t legacy_aliases = 0, source_aliases = 0;
    std::uint64_t legacy_source_splits = 0, enhanced_source_splits = 0, enhanced_source_discards = 0;
    std::uint64_t legacy_rt = 0, enhanced_rt = 0;
    bool operator==(const InvalidationStatistics&) const = default;
  };
  struct Statistics {
    std::uint64_t captures = 0, source_draws = 0;
    std::uint64_t escaped_invalidations = 0;
    InvalidationStatistics invalidations;
    const char* tail_status = "not_started";
  };
  /*
   * Completed copies into the WASM input image: total, one-second rate, capture-to-completion latency and
   * age of the last completion.
   */
  struct InputStatistics {
    std::uint64_t completed_frames = 0;
    float frames_per_second = 0;
    float latency_ms = 0;
    float age_ms = 0;
    bool latency_valid = false;
  };

  explicit SceneCaptureManager(SceneHandoff& handoff) noexcept;
  ~SceneCaptureManager();
  SceneCaptureManager(const SceneCaptureManager&) = delete;
  SceneCaptureManager& operator=(const SceneCaptureManager&) = delete;

  bool register_device(ID3D12Device* native_device) noexcept;
  /*
   * Accepts a new tracked list of the device. A list first seen mid-recording (`observed` false) holds no
   * capture evidence until its next successful native Reset.
   */
  bool admit_list(List&, bool observed) noexcept;
  /*
   * A successful native Reset retires the previous recording's packets and leases and starts a new one.
   */
  void successful_reset(List&) noexcept;
  /*
   * The native list was destroyed: its recording is retired for good.
   */
  void retire_list(List&) noexcept;

  /*
   * Candidate/model observation starts at resource creation, before scene publication. Only begin/stop
   * controls tail capture; neither invents a state.
   */
  void begin_source_tracking() noexcept;
  void stop_source_tracking() noexcept;
  /*
   * Suppress NEW GPU capture work only. Source state/alias/draw observation, immutable replay effects and
   * existing packet/consumer ordering must continue while idle, including for persistent textures created
   * directly in RT state.
   */
  void set_capture_enabled(bool enabled) noexcept;
  bool register_source_candidate(ID3D12Resource*,
                                 std::uint64_t resource_generation,
                                 const D3D12_RESOURCE_DESC&,
                                 source_state::Model initial = source_state::Model::unknown) noexcept;
  void unregister_source_candidate(ID3D12Resource*, std::uint64_t resource_generation) noexcept;
  /*
   * Records the actual targets of one native draw, after it was recorded.
   */
  void observe_source_draw_after(List&,
                                 UINT count,
                                 ID3D12Resource* const* targets,
                                 const std::uint64_t* resource_generations,
                                 bool allowed) noexcept;
  void observe_source_legacy(List&, const D3D12_RESOURCE_BARRIER&) noexcept;
  void observe_source_enhanced(List&, const D3D12_TEXTURE_BARRIER&) noexcept;
  void observe_source_discard(List&, ID3D12Resource*) noexcept;
  /*
   * Records why hook-level uncertainty revoked source evidence. `scoped` means that only positively
   * identified bound targets were affected; global events invalidate the complete recording.
   */
  void note_source_invalidation(std::uint32_t reasons, bool scoped) noexcept;
  void invalidate_source_recording(List&, bool affects_all_sources) noexcept;
  /*
   * Known render-pass/indirect targets affect only their exact resource keys. Missing/truncated target
   * metadata retains the conservative global guard.
   */
  void invalidate_source_targets(List&, UINT count, ID3D12Resource* const* targets, const std::uint64_t* resource_generations) noexcept;

  /*
   * Call when the current application recording reads the one stable output. Keeps every later replay on the
   * same timeline as private output writes.
   */
  bool register_consumer_recording(ID3D12GraphicsCommandList* native_list) noexcept;
  /*
   * Metrics only: called after a recorded copy into the WASM input image. Registration above still protects
   * all recorded buffer reads.
   */
  void mark_input_copy(ID3D12GraphicsCommandList* native_list) noexcept;
  /*
   * Starts or stops input-copy accounting; a change restarts the rate, latency and age measurements.
   */
  void set_input_active(bool active) noexcept;
  InputStatistics input_statistics() noexcept;

  /*
   * Called by the ExecuteCommandLists hook. Lists seen for the first time are registered first, without
   * submission serialization. Only fully observed, unrelated recordings bypass ordering; all other batches
   * revalidate under both manager and submission locks before retaining leases or queuing a fence.
   * A contended or same-thread re-entrant submit is first classified without waiting. Known unrelated lists
   * can forward without disturbing capture; uncertain or camera-related batches bracket their native
   * forward with deferred invalidation markers.
   */
  std::uint64_t before_submission(ID3D12CommandQueue*, UINT, ID3D12CommandList* const*) noexcept;
  bool unordered_submission_affects_capture(ID3D12CommandQueue*, UINT, ID3D12CommandList* const*) noexcept;
  void defer_unordered_submission() noexcept;
  void after_submission(ID3D12CommandQueue*, std::uint64_t receipt) noexcept;
  void submission_refused(ID3D12CommandQueue*, queue_hooks::Refusal) noexcept;

  /*
   * begin/end MUST run on the same thread, with no intervening manager API that begins another submission.
   * The private DIRECT queue must not be registered for ordinary capture notifications. No CPU waits for GPU
   * completion occur.
   */
  Submission begin_private_submission(ID3D12CommandQueue*) noexcept;
  bool end_private_submission(std::uint64_t receipt, std::uint64_t image_id = 0, std::uint64_t capture_time_us = 0) noexcept;
  void abort_private_submission(std::uint64_t receipt) noexcept;

  /*
   * Each returned frame is leased exactly once, after successful native Reset or destruction retires its
   * application recording, all receipts return, and its producer fence completes. Caller supplies fixed
   * bounded storage. Buffer-slot order is unspecified. Consumers must retain the newest order, including
   * across polls, to reject delayed older captures.
   */
  std::size_t poll_completed_frames(Frame* frames, std::size_t capacity) noexcept;
  bool finish_consumption(std::uint64_t token, ID3D12Fence*, std::uint64_t value) noexcept;
  /*
   * Only for a leased frame on which NO private work was recorded/submitted.
   */
  bool discard_frame(std::uint64_t token) noexcept;
  Statistics statistics() const noexcept;

 private:
  using CaptureRecording = standalone::CaptureRecording;
  using SourceLease = standalone::SourceLease;
  using Leases = std::array<SourceLease, source_state::Tracker::capacity>;
  struct Device {
    ID3D12Device* native = nullptr;
    ID3D12Fence* timeline = nullptr;
    std::uint64_t last_signal = 0;
    bool failed = false;
    source_state::Tracker source_states;
    std::uint64_t image_id = 0, capture_time_us = 0;
    bool input_active = false;
    std::uint64_t input_epoch = 0, input_last_image = 0;
    struct InputCompletion {
      std::uint64_t fence_value = 0, capture_time_us = 0;
    };
    std::array<InputCompletion, 64> input_pending{};
    std::array<std::uint64_t, 128> input_completions_us{};
    std::size_t input_window_cursor = 0;
    std::uint64_t input_last_completion_us = 0, input_last_completion_value = 0;
    InputStatistics input;
  };
  struct Packet {
    SceneCaptureD3D12 gpu;
    D3D12_RESOURCE_DESC description{};
    SceneCopyMatch match;
    std::uint64_t token = 0, submitted = 0;
    FrameOrder order;
    std::uint64_t capture_time_us = 0;
    ID3D12CommandQueue* producer = nullptr; /* Registered queue retained by hook. */
    unsigned in_flight = 0;
    bool assigned = false, retired = false, quarantined = false, leased = false;
    ID3D12CommandAllocator* tail_allocator = nullptr;
    ID3D12GraphicsCommandList* tail_list = nullptr;
    ID3D12GraphicsCommandList7* tail_list7 = nullptr;
  };
  struct Transaction {
    std::uint64_t id = 0;
    ID3D12CommandQueue* queue = nullptr;
    std::uint64_t value = 0;
    std::uint16_t packets = 0;
    bool private_work = false;
    bool source_work = false;
    Leases source_leases{};
    std::size_t source_lease_count = 0;
    std::array<std::uint32_t, MaximumPackets> packet_positions{};
    std::uint32_t last_position = 0;
    std::uint64_t started_us = 0, input_epoch = 0, image_id = 0, capture_time_us = 0;
  };
  struct SourceCandidate {
    ID3D12Resource* native = nullptr; /* Registry identity only; never a GPU lease. */
    std::uint64_t generation = 0;
    D3D12_RESOURCE_DESC description{};
  };
  bool usable() const noexcept { return device_.native && !device_.failed; }
  void retire_recording(CaptureRecording&) noexcept;
  void apply_source_draw(CaptureRecording&, source_state::Key, bool allowed) noexcept;
  static void release_source_leases(Leases&, std::size_t&) noexcept;
  static bool retain_source_lease(Leases&, std::size_t&, source_state::Key, ID3D12Resource*) noexcept;
  void quarantine(Packet&) noexcept;
  void fail_device() noexcept;
  void collect() noexcept;
  void collect_input() noexcept;
  SourceCandidate* source_candidate(ID3D12Resource*) noexcept;
  bool may_be_source(ID3D12Resource*) const noexcept;
  bool prepare_tail(Packet&) noexcept;
  bool record_tail_capture(Packet&, const SceneCopyMatch&, ID3D12Resource* source, bool enhanced) noexcept;
  void record_queue_tail(Transaction&) noexcept;
  bool compatible_queue(ID3D12CommandQueue*) noexcept;
  void invalidate_source_after_escape() noexcept;
  bool apply_deferred_escape() noexcept;
  Submission begin_transaction(ID3D12CommandQueue*, std::uint16_t, bool) noexcept;
  bool finish_transaction(std::uint64_t, bool refused, bool fatal = true) noexcept;

  SceneHandoff& handoff_;
  mutable std::mutex mutex_;
  std::mutex submission_mutex_;
  Device device_;
  std::array<Packet, MaximumPackets> packets_{};
  std::uint64_t packet_bytes_ = 0;
  std::array<SourceCandidate, source_state::Tracker::capacity> sources_{};
  std::array<std::atomic<std::uint64_t>, source_state::Tracker::capacity> source_handles_{}, source_generations_{};
  std::array<std::atomic<std::uint64_t>, 16> source_filter_{};
  bool source_tracking_ = false;
  bool capture_enabled_ = true;
  std::atomic<bool> deferred_escape_{false};
  Transaction transaction_;
  std::uint64_t next_token_ = 0, next_receipt_ = 0;
  Statistics stats_;
};

} /* namespace azp */
