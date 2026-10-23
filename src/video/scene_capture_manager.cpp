/*
 * Coordinates camera-image capture across command-list recordings and queue submissions. It records a copy
 * at each admissible queue tail and tracks fences through composition and consumption.
 */

#include "scene_capture_manager.hpp"
#include <chrono>
#include <limits>
#include "../shared/configuration.hpp"
#include "command_list_hooks.hpp"
#include "resource_guards.hpp"

namespace azp {
namespace {
thread_local SceneCaptureManager* transaction_owner = nullptr;
thread_local std::uint64_t thread_receipt = 0;
constexpr auto MaximumCounter = std::numeric_limits<std::uint64_t>::max() - 1;
std::uint64_t steady_us() noexcept {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
}
bool same_description(const D3D12_RESOURCE_DESC& a, const D3D12_RESOURCE_DESC& b) noexcept {
  return a.Dimension == b.Dimension && a.Width == b.Width && a.Height == b.Height && a.DepthOrArraySize == b.DepthOrArraySize &&
         a.MipLevels == b.MipLevels && a.Format == b.Format && a.SampleDesc.Count == b.SampleDesc.Count &&
         a.SampleDesc.Quality == b.SampleDesc.Quality;
}
} /* namespace */

SceneCaptureManager::SceneCaptureManager(SceneHandoff& handoff) noexcept : handoff_(handoff) {}
SceneCaptureManager::~SceneCaptureManager() = default; /* Native device/timeline leases intentionally process-lifetime. */

/*
 * Retains the simulator's device with its fence timeline. A second registration must agree with the first
 * rather than creating another ordering domain.
 */
bool SceneCaptureManager::register_device(ID3D12Device* native) noexcept {
  const std::lock_guard lock(mutex_);
  if (!native)
    return false;
  if (device_.native)
    return !device_.failed && device_.native == native;
  ID3D12Fence* timeline = nullptr;
  if (FAILED(native->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&timeline))))
    return false;
  native->AddRef();
  device_.native = native;
  device_.timeline = timeline;
  return true;
}

void SceneCaptureManager::quarantine(Packet& packet) noexcept {
  packet.quarantined = true;
}
/*
 * Stops all capture on the device: source states are forgotten and assigned packets are quarantined rather
 * than reused.
 */
void SceneCaptureManager::fail_device() noexcept {
  device_.failed = true;
  device_.source_states.invalidate_all();
  for (auto& packet : packets_)
    if (packet.assigned)
      quarantine(packet);
}

/*
 * Admits a direct list of the device. An unobserved initial recording is kept distinct from an admissible
 * reset recording.
 */
bool SceneCaptureManager::admit_list(List& item, bool observed) noexcept {
  const std::lock_guard lock(mutex_);
  if (!item.native || !usable() || item.native->GetType() != D3D12_COMMAND_LIST_TYPE_DIRECT ||
      !same_native_device(item.native, device_.native))
    return false;
  item.capture.admitted = true;
  item.capture.awaiting_native_reset = !observed;
  return true;
}

/*
 * Releases recording-owned source references and clears lease storage before destruction callbacks can
 * reenter tracking.
 *
 * Release can synchronously call unregister_source_candidate, which is deliberately lock-free. Clear each
 * owned slot before that callback.
 */
void SceneCaptureManager::release_source_leases(Leases& leases, std::size_t& count) noexcept {
  while (count) {
    auto& lease = leases[--count];
    auto* resource = lease.native;
    lease = {};
    resource->Release();
  }
}
/*
 * Retains one reference per recording only from a currently live native argument or an existing owned lease.
 * Registry identity alone does not authorize AddRef.
 *
 * Caller must already have a live application draw argument or an owned recording lease. The candidate
 * identity registry is never sufficient.
 */
bool SceneCaptureManager::retain_source_lease(Leases& leases,
                                              std::size_t& count,
                                              source_state::Key key,
                                              ID3D12Resource* resource) noexcept {
  for (std::size_t index = 0; index < count; ++index)
    if (leases[index].key == key)
      return leases[index].native == resource;
  if (!resource || count == leases.size())
    return false;
  resource->AddRef();
  leases[count++] = {key, resource};
  return true;
}
/*
 * Retires a command-list recording and its source references while preserving submitted packet obligations.
 */
void SceneCaptureManager::retire_recording(CaptureRecording& item) noexcept {
  for (std::size_t index = 0; index < packets_.size(); ++index)
    if (item.packets & (1u << index))
      packets_[index].retired = true;
  item.packets = 0;
  item.consumer = false;
  item.input_epoch = 0;
  item.source_effects.reset();
  item.source_touched = false;
  release_source_leases(item.source_leases, item.source_lease_count);
}

SceneCaptureManager::SourceCandidate* SceneCaptureManager::source_candidate(ID3D12Resource* resource) noexcept {
  for (std::size_t index = 0; index < sources_.size(); ++index)
    if (sources_[index].native == resource && resource && source_generations_[index].load(std::memory_order_acquire))
      return &sources_[index];
  return nullptr;
}
bool SceneCaptureManager::may_be_source(ID3D12Resource* resource) const noexcept {
  const auto hash = (reinterpret_cast<std::uint64_t>(resource) >> 4) * 0x9e3779b97f4a7c15ull;
  return resource && (source_filter_[(hash >> 6) & 15].load(std::memory_order_acquire) & (1ull << (hash & 63)));
}
void SceneCaptureManager::begin_source_tracking() noexcept {
  const std::lock_guard lock(mutex_);
  source_tracking_ = true;
  stats_.tail_status = "awaiting_ordered_source_state";
}
/*
 * Stops tail capture and releases every recording's source references, under the same lock as the draws
 * that retain them. The registry lock is a leaf: it is held only while the list snapshot is copied.
 */
void SceneCaptureManager::stop_source_tracking() noexcept {
  const std::lock_guard lock(mutex_);
  const auto lists = standalone::live_lists();
  source_tracking_ = false;
  for (const auto& item : lists)
    release_source_leases(item->capture.source_leases, item->capture.source_lease_count);
  stats_.tail_status = "stopped";
}

void SceneCaptureManager::set_capture_enabled(bool enabled) noexcept {
  const std::lock_guard lock(mutex_);
  capture_enabled_ = enabled;
}
/*
 * Validates source shape, format and size before registering its generation-qualified identity in the
 * ordered source tracker.
 */
bool SceneCaptureManager::register_source_candidate(ID3D12Resource* resource,
                                                    std::uint64_t generation,
                                                    const D3D12_RESOURCE_DESC& desc,
                                                    source_state::Model initial) noexcept {
  if (!resource || !generation || desc.Dimension != D3D12_RESOURCE_DIMENSION_TEXTURE2D ||
      !camera_sized(static_cast<UINT>(desc.Width), desc.Height) || desc.MipLevels != 1 || desc.DepthOrArraySize != 1 ||
      desc.SampleDesc.Count != 1 || desc.SampleDesc.Quality || (desc.Flags & D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET) == 0 ||
      (desc.Flags & (D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL | D3D12_RESOURCE_FLAG_ALLOW_SIMULTANEOUS_ACCESS)) != 0 ||
      (desc.Format != DXGI_FORMAT_R11G11B10_FLOAT && desc.Format != DXGI_FORMAT_R8G8B8A8_UNORM &&
       desc.Format != DXGI_FORMAT_R8G8B8A8_TYPELESS && desc.Format != DXGI_FORMAT_R16G16B16A16_FLOAT))
    return false;
  const std::lock_guard lock(mutex_);
  if (!usable())
    return false;
  collect();
  if (auto* existing = source_candidate(resource))
    return existing->generation == generation;
  for (std::size_t index = 0; index < sources_.size(); ++index) {
    auto& candidate = sources_[index];
    if (candidate.native)
      continue;
    if (!device_.source_states.register_source({reinterpret_cast<std::uint64_t>(resource), generation}, initial))
      return false;
    candidate = {resource, generation, desc};
    source_handles_[index].store(reinterpret_cast<std::uint64_t>(resource), std::memory_order_relaxed);
    source_generations_[index].store(generation, std::memory_order_release);
    const auto hash = (reinterpret_cast<std::uint64_t>(resource) >> 4) * 0x9e3779b97f4a7c15ull;
    source_filter_[(hash >> 6) & 15].fetch_or(1ull << (hash & 63), std::memory_order_release);
    return true;
  }
  return false;
}
/*
 * Publishes a generation-specific retirement marker without taking the manager mutex, allowing synchronous
 * native destruction callbacks. Later collection removes the metadata.
 *
 * Last source Release may synchronously enter the app's destruction callback while this manager holds its
 * mutex. Publish a generation-specific tombstone without locking; the next collect removes metadata. No raw
 * object is read.
 *
 * Rejection-only filter bits remain set; retirement can never hide a new key.
 */
void SceneCaptureManager::unregister_source_candidate(ID3D12Resource* resource, std::uint64_t generation) noexcept {
  for (std::size_t index = 0; index < sources_.size(); ++index) {
    if (source_handles_[index].load(std::memory_order_acquire) != reinterpret_cast<std::uint64_t>(resource))
      continue;
    auto expected = generation;
    source_generations_[index].compare_exchange_strong(expected, 0, std::memory_order_acq_rel);
  }
}
/*
 * Records actual post-draw target evidence for an observed native recording, ignoring unrelated resources.
 */
void SceneCaptureManager::observe_source_draw_after(List& item,
                                                    UINT count,
                                                    ID3D12Resource* const* targets,
                                                    const std::uint64_t* generations,
                                                    bool allowed) noexcept {
  if (!count || count > 8 || !targets || !generations)
    return;
  bool candidate_present = false;
  for (UINT n = 0; n < count; ++n)
    candidate_present |= may_be_source(targets[n]);
  if (!candidate_present)
    return;
  const std::lock_guard lock(mutex_);
  if (!item.capture.admitted)
    return;
  for (UINT n = 0; n < count; ++n)
    apply_source_draw(item.capture, {reinterpret_cast<std::uint64_t>(targets[n]), generations[n]}, allowed);
}
/*
 * Adds ordered draw evidence and retains the live source once per recording. The actual native draw
 * argument, not the registry entry, authorizes its lifetime lease.
 *
 * The exact application Draw has completed but has not returned to its caller: its actual bound RTV source
 * is still a live resource argument. Keep one reference per recording, never a registry-lifetime reference.
 */
void SceneCaptureManager::apply_source_draw(CaptureRecording& item, source_state::Key key, bool allowed) noexcept {
  const auto* source = source_candidate(reinterpret_cast<ID3D12Resource*>(key.handle));
  if (!source || source->generation != key.generation)
    return;
  item.source_touched = true;
  if (!allowed) {
    item.source_effects.invalidate();
  } else {
    item.source_effects.append({key, source_state::Effect::Kind::draw});
    if (source_tracking_ && !retain_source_lease(item.source_leases, item.source_lease_count, key, source->native))
      item.source_effects.invalidate();
    ++stats_.source_draws;
  }
}
/*
 * Counts the hook condition that revoked source evidence. These counters are diagnostic only; the caller's
 * existing scoped/global invalidation still decides the safety action.
 */
void SceneCaptureManager::note_source_invalidation(std::uint32_t reasons, bool scoped) noexcept {
  const std::lock_guard lock(mutex_);
  auto& s = stats_.invalidations;
  ++(scoped ? s.scoped : s.global);
  if (reasons & list_hooks::InvalidationPassBegin)
    ++s.pass_begin;
  if (reasons & list_hooks::InvalidationPassState)
    ++s.pass_state;
  if (reasons & list_hooks::InvalidationBarrierBatch)
    ++s.barrier_batch;
  if (reasons & list_hooks::InvalidationSplitBarrier)
    ++s.split_barrier;
  if (reasons & list_hooks::InvalidationAliasOrDiscard)
    ++s.alias_or_discard;
  if (reasons & list_hooks::InvalidationUnobservedWork)
    ++s.unobserved_work;
  if (reasons & list_hooks::InvalidationObserverDisabled)
    ++s.observer_disabled;
  if (reasons & list_hooks::InvalidationResetFailed)
    ++s.reset_failed;
  if (reasons & list_hooks::InvalidationExecuteBundle)
    ++s.execute_bundle;
  if (reasons & list_hooks::InvalidationExecuteIndirect)
    ++s.execute_indirect;
  if (reasons & list_hooks::InvalidationPredication)
    ++s.predication;
}
/*
 * Invalidates all source-state evidence for an uncertain recording. Later capture must use a genuinely
 * renewed observation sequence.
 */
void SceneCaptureManager::invalidate_source_recording(List& item, bool global) noexcept {
  const std::lock_guard lock(mutex_);
  if (item.capture.admitted) {
    item.capture.source_effects.invalidate();
    item.capture.source_touched |= global;
  }
}
/*
 * Appends uncertainty for affected source targets, or invalidates the whole recording when target evidence
 * is insufficient.
 */
void SceneCaptureManager::invalidate_source_targets(List& list,
                                                    UINT count,
                                                    ID3D12Resource* const* targets,
                                                    const std::uint64_t* generations) noexcept {
  if (!count)
    return;
  const std::lock_guard lock(mutex_);
  auto& item = list.capture;
  if (!item.admitted)
    return;
  if (count > 8 || !targets || !generations) {
    item.source_touched = true;
    item.source_effects.invalidate();
    return;
  }
  for (UINT index = 0; index < count; ++index) {
    const auto* candidate = source_candidate(targets[index]);
    if (!candidate || candidate->generation != generations[index])
      continue;
    item.source_touched = true;
    item.source_effects.append(
        {{reinterpret_cast<std::uint64_t>(candidate->native), candidate->generation}, source_state::Effect::Kind::other});
  }
}
/*
 * Applies DiscardResource only to the resource named by the native call. An unrelated FSR scratch discard
 * cannot affect a different texture; a camera-source discard remains conservative until a later absolute
 * render-target transition re-establishes its state.
 */
void SceneCaptureManager::observe_source_discard(List& list, ID3D12Resource* resource) noexcept {
  const std::lock_guard lock(mutex_);
  auto& s = stats_.invalidations;
  ++s.discard_calls;
  auto& item = list.capture;
  const auto* source = source_candidate(resource);
  if (!item.admitted || !source)
    return;
  ++s.source_discards;
  item.source_touched = true;
  item.source_effects.append({{reinterpret_cast<std::uint64_t>(source->native), source->generation}, source_state::Effect::Kind::other});
}
/*
 * Appends legacy resource transitions in their recording order, invalidating ambiguous aliases or
 * unsupported state rather than guessing a layout.
 */
void SceneCaptureManager::observe_source_legacy(List& list, const D3D12_RESOURCE_BARRIER& barrier) noexcept {
  if (barrier.Type != D3D12_RESOURCE_BARRIER_TYPE_ALIASING &&
      (barrier.Type != D3D12_RESOURCE_BARRIER_TYPE_TRANSITION || !may_be_source(barrier.Transition.pResource)))
    return;
  const std::lock_guard lock(mutex_);
  auto& item = list.capture;
  if (!item.admitted)
    return;
  if (barrier.Type == D3D12_RESOURCE_BARRIER_TYPE_ALIASING) {
    ++stats_.invalidations.legacy_aliases;
    if (!barrier.Aliasing.pResourceBefore || !barrier.Aliasing.pResourceAfter) {
      item.source_touched = true;
      item.source_effects.invalidate();
    } else {
      for (auto* aliased : {barrier.Aliasing.pResourceBefore, barrier.Aliasing.pResourceAfter})
        if (const auto* candidate = source_candidate(aliased)) {
          ++stats_.invalidations.source_aliases;
          item.source_touched = true;
          item.source_effects.append(
              {{reinterpret_cast<std::uint64_t>(aliased), candidate->generation}, source_state::Effect::Kind::other});
        }
    }
    return;
  }
  const auto* source = source_candidate(barrier.Transition.pResource);
  if (!source)
    return;
  item.source_touched = true;
  if (barrier.Flags == D3D12_RESOURCE_BARRIER_FLAG_BEGIN_ONLY || barrier.Flags == D3D12_RESOURCE_BARRIER_FLAG_END_ONLY)
    ++stats_.invalidations.legacy_source_splits;
  if (barrier.Flags != D3D12_RESOURCE_BARRIER_FLAG_NONE ||
      (barrier.Transition.Subresource != 0 && barrier.Transition.Subresource != D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES)) {
    item.source_effects.append({{reinterpret_cast<std::uint64_t>(source->native), source->generation}, source_state::Effect::Kind::other});
    return;
  }
  const auto kind = barrier.Transition.StateAfter == D3D12_RESOURCE_STATE_RENDER_TARGET ? source_state::Effect::Kind::legacy_rt
                                                                                        : source_state::Effect::Kind::other;
  if (kind == source_state::Effect::Kind::legacy_rt)
    ++stats_.invalidations.legacy_rt;
  item.source_effects.append({{reinterpret_cast<std::uint64_t>(source->native), source->generation}, kind});
}
/*
 * Appends enhanced texture transition evidence for known source identities, preserving access/layout and
 * scope uncertainty.
 */
void SceneCaptureManager::observe_source_enhanced(List& list, const D3D12_TEXTURE_BARRIER& barrier) noexcept {
  if (!may_be_source(barrier.pResource))
    return;
  const std::lock_guard lock(mutex_);
  auto& item = list.capture;
  const auto* source = source_candidate(barrier.pResource);
  if (!item.admitted || !source)
    return;
  item.source_touched = true;
  if ((barrier.SyncBefore & D3D12_BARRIER_SYNC_SPLIT) || (barrier.SyncAfter & D3D12_BARRIER_SYNC_SPLIT))
    ++stats_.invalidations.enhanced_source_splits;
  if (barrier.Flags == D3D12_TEXTURE_BARRIER_FLAG_DISCARD)
    ++stats_.invalidations.enhanced_source_discards;
  const auto& range = barrier.Subresources;
  const bool whole = range.NumMipLevels == 0 ? range.IndexOrFirstMipLevel == 0 || range.IndexOrFirstMipLevel == UINT_MAX
                                             : range.IndexOrFirstMipLevel == 0 && range.NumMipLevels == 1 && range.FirstArraySlice == 0 &&
                                                   range.NumArraySlices == 1 && range.FirstPlane == 0 && range.NumPlanes == 1;
  if (!whole || barrier.Flags != D3D12_TEXTURE_BARRIER_FLAG_NONE || (barrier.SyncBefore & D3D12_BARRIER_SYNC_SPLIT) ||
      (barrier.SyncAfter & D3D12_BARRIER_SYNC_SPLIT)) {
    item.source_effects.append({{reinterpret_cast<std::uint64_t>(source->native), source->generation}, source_state::Effect::Kind::other});
    return;
  }
  const auto kind = barrier.LayoutAfter == D3D12_BARRIER_LAYOUT_RENDER_TARGET && barrier.AccessAfter == D3D12_BARRIER_ACCESS_RENDER_TARGET
                        ? source_state::Effect::Kind::enhanced_rt
                        : source_state::Effect::Kind::other;
  if (kind == source_state::Effect::Kind::enhanced_rt)
    ++stats_.invalidations.enhanced_rt;
  item.source_effects.append({{reinterpret_cast<std::uint64_t>(source->native), source->generation}, kind});
}
/*
 * Retires the previous recording and starts the next one only after a successful native Reset.
 */
void SceneCaptureManager::successful_reset(List& item) noexcept {
  const std::lock_guard lock(mutex_);
  if (item.capture.admitted) {
    retire_recording(item.capture);
    item.capture.awaiting_native_reset = false;
    collect();
  }
}
/*
 * Retires the destroyed list's recording/leases and collects work whose GPU lifetime has completed.
 */
void SceneCaptureManager::retire_list(List& item) noexcept {
  const std::lock_guard lock(mutex_);
  if (item.capture.admitted) {
    retire_recording(item.capture);
    item.capture.admitted = false;
    collect();
  }
}

/*
 * Allocates the packet's capture texture for the source description when needed, then records the
 * transition, copy and restore on its private tail list. An idle packet of another description is released
 * first; the total stays within MaximumBytes.
 *
 * Validate/count the actual committed allocation before admitting a packet.
 */
bool SceneCaptureManager::record_tail_capture(Packet& packet, const SceneCopyMatch& match, ID3D12Resource* source, bool enhanced) noexcept {
  const auto desc = source->GetDesc();
  if (desc.Dimension != D3D12_RESOURCE_DIMENSION_TEXTURE2D || desc.MipLevels != 1 || desc.DepthOrArraySize != 1 ||
      desc.SampleDesc.Count != 1 || desc.SampleDesc.Quality != 0 || (desc.Flags & D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET) == 0 ||
      (desc.Flags & D3D12_RESOURCE_FLAG_ALLOW_SIMULTANEOUS_ACCESS) != 0 || next_token_ >= MaximumCounter)
    return false;
  if (packet.gpu.state() == SceneCaptureD3D12::State::idle && !same_description(packet.description, desc)) {
    const auto bytes = packet.gpu.allocation_bytes();
    if (!packet.gpu.release_idle())
      return false;
    packet_bytes_ -= bytes;
  }
  if (packet.gpu.state() == SceneCaptureD3D12::State::empty) {
    auto allocation_desc = desc;
    allocation_desc.Alignment = 0;
    allocation_desc.Flags = D3D12_RESOURCE_FLAG_NONE;
    const auto allocation = device_.native->GetResourceAllocationInfo(0, 1, &allocation_desc);
    if (!allocation.SizeInBytes || allocation.SizeInBytes > MaximumBytes - packet_bytes_ ||
        FAILED(packet.gpu.initialize(device_.native, desc)))
      return false;
    packet_bytes_ += packet.gpu.allocation_bytes();
    packet.description = desc;
  }
  const bool recorded = enhanced ? packet.tail_list7 && packet.gpu.record_render_target_source_enhanced(packet.tail_list7, source)
                                 : packet.gpu.record_render_target_source(packet.tail_list, source);
  if (!recorded)
    return false;
  packet.match = match;
  packet.token = ++next_token_;
  packet.submitted = 0;
  packet.order = {};
  packet.capture_time_us = 0;
  packet.producer = nullptr;
  packet.in_flight = 0;
  packet.assigned = true;
  packet.leased = false;
  ++stats_.captures;
  return true;
}

/*
 * Creates or resets a packet's private tail command list only after its prior producer/consumer lifetime is
 * complete: an unassigned packet has completed its prior producer and all consumers, or its closed private
 * recording was discarded without submission.
 */
bool SceneCaptureManager::prepare_tail(Packet& packet) noexcept {
  if (packet.assigned || packet.quarantined)
    return false;
  const auto clear_tail = [&packet]() noexcept {
    if (packet.tail_list7)
      packet.tail_list7->Release();
    if (packet.tail_list)
      packet.tail_list->Release();
    if (packet.tail_allocator)
      packet.tail_allocator->Release();
    packet.tail_list7 = nullptr;
    packet.tail_list = nullptr;
    packet.tail_allocator = nullptr;
  };
  if (!packet.tail_allocator &&
      FAILED(device_.native->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&packet.tail_allocator))))
    return false;
  if (!packet.tail_list) {
    if (FAILED(device_.native->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, packet.tail_allocator, nullptr,
                                                 IID_PPV_ARGS(&packet.tail_list)))) {
      clear_tail();
      return false;
    }
    packet.tail_list->QueryInterface(IID_PPV_ARGS(&packet.tail_list7));
    if (FAILED(packet.tail_list->Close())) {
      clear_tail();
      return false;
    }
  }
  if (FAILED(packet.tail_allocator->Reset()) || FAILED(packet.tail_list->Reset(packet.tail_allocator, nullptr)))
    return false;
  return true;
}
/*
 * Captures the published image at the end of an accepted native Execute batch that drew it as a render
 * target. Recorded transitions and draws have already been replayed in submission order.
 *
 * Every admissible tail produces a frame. Packet reuse requires completed producer/consumer obligations and
 * current resource identity.
 *
 * Each accepted private copy list is submitted once and attached to the outer submission receipt. Nested
 * queue observation is bypassed in this after-phase, so the explicit receipt association supplies its
 * lifetime/fence accounting. If preparation or Close fails before submission, the unsubmitted list is
 * discarded/reset rather than executed or replayed.
 */
void SceneCaptureManager::record_queue_tail(Transaction& pending) noexcept {
  if (!usable())
    return;
  if (!list_hooks::operational()) {
    device_.source_states.invalidate_all();
    stats_.tail_status = "observer_disabled";
    return;
  }
  collect();
  stats_.tail_status = "no_candidate_draw";
  for (std::size_t source_index = 0; source_index < sources_.size(); ++source_index) {
    auto& source = sources_[source_index];
    if (!source.native || source_generations_[source_index].load(std::memory_order_acquire) != source.generation)
      continue;
    const source_state::Key key{reinterpret_cast<std::uint64_t>(source.native), source.generation};
    const auto state = device_.source_states.state(key);
    const bool observed = handoff_.observed(key.handle);
    if (state.model == source_state::Model::unknown) {
      if (observed)
        stats_.tail_status = "unknown_source_state";
      continue;
    }
    if (!state.drawn || (state.model != source_state::Model::legacy_rt && state.model != source_state::Model::enhanced_rt))
      continue;
    const auto match = handoff_.observe(key.handle);
    if (!match.matched) {
      if (observed)
        stats_.tail_status = "waiting_for_publication";
      continue;
    }
    /*
     * Only the published image matches; whatever happens to it ends this tail.
     */
    ID3D12Resource* leased_source = nullptr;
    for (std::size_t index = 0; index < pending.source_lease_count; ++index)
      if (pending.source_leases[index].key == key) {
        leased_source = pending.source_leases[index].native;
        break;
      }
    if (!leased_source) {
      stats_.tail_status = "source_lease_unavailable";
      return;
    }
    stats_.tail_status = "tail_packet_unavailable";
    for (auto& packet : packets_) {
      if (!prepare_tail(packet))
        continue;
      const bool recorded = record_tail_capture(packet, match, leased_source, state.model == source_state::Model::enhanced_rt);
      const auto closed = packet.tail_list->Close();
      if (!recorded)
        continue;
      packet.retired = true;
      if (FAILED(closed)) {
        quarantine(packet);
        stats_.tail_status = "tail_close_failed";
        return;
      }
      ++packet.in_flight;
      const auto index = static_cast<std::size_t>(&packet - packets_.data());
      pending.packets |= static_cast<std::uint16_t>(1u << index);
      pending.packet_positions[index] = ++pending.last_position;
      ID3D12CommandList* executable = packet.tail_list;
      pending.queue->ExecuteCommandLists(1, &executable);
      stats_.tail_status = "captured";
      return;
    }
    return;
  }
}

bool SceneCaptureManager::register_consumer_recording(ID3D12GraphicsCommandList* native) noexcept {
  const auto item = standalone::find_list(native);
  const std::lock_guard lock(mutex_);
  if (!item || !item->capture.admitted || !usable())
    return false;
  item->capture.consumer = true;
  return true;
}

void SceneCaptureManager::mark_input_copy(ID3D12GraphicsCommandList* native) noexcept {
  const auto item = standalone::find_list(native);
  const std::lock_guard lock(mutex_);
  if (item && item->capture.admitted && usable() && item->capture.consumer && device_.input_active)
    item->capture.input_epoch = device_.input_epoch;
}

/*
 * Starts or stops input-copy accounting under the manager lock, without revoking unrelated source tracking.
 */
void SceneCaptureManager::set_input_active(bool active) noexcept {
  const std::lock_guard lock(mutex_);
  auto& owner = device_;
  if (!owner.native || owner.input_active == active)
    return;
  owner.input_active = active;
  ++owner.input_epoch;
  owner.input_last_image = owner.input_last_completion_us = 0;
  owner.input_last_completion_value = 0;
  owner.input_pending = {};
  owner.input_completions_us = {};
  owner.input_window_cursor = 0;
  const auto total = owner.input.completed_frames;
  owner.input = {};
  owner.input.completed_frames = total;
}

/*
 * Polls consumer fence completion and accounts distinct delivered input images. A device-removed sentinel
 * cannot count as normal completion.
 */
void SceneCaptureManager::collect_input() noexcept {
  auto& owner = device_;
  if (!owner.timeline || owner.failed)
    return;
  const auto now = steady_us();
  const auto completed = owner.timeline->GetCompletedValue();
  if (completed == std::numeric_limits<std::uint64_t>::max()) {
    owner.input.latency_valid = false;
    owner.input_completions_us = {};
    return;
  }
  for (auto& sample : owner.input_pending) {
    if (!sample.fence_value || sample.fence_value > completed)
      continue;
    ++owner.input.completed_frames;
    owner.input_completions_us[owner.input_window_cursor] = now;
    owner.input_window_cursor = (owner.input_window_cursor + 1) % owner.input_completions_us.size();
    if (sample.fence_value > owner.input_last_completion_value) {
      owner.input_last_completion_value = sample.fence_value;
      owner.input.latency_valid = sample.capture_time_us && sample.capture_time_us <= now;
      owner.input.latency_ms = owner.input.latency_valid ? static_cast<float>(now - sample.capture_time_us) / 1000.0f : 0;
      owner.input_last_completion_us = now;
    }
    sample = {};
  }
}

/*
 * Collects completed input-copy samples and reports their one-second rate, observed latency and age. These
 * metrics stop before final WASM menu drawing and display presentation.
 */
SceneCaptureManager::InputStatistics SceneCaptureManager::input_statistics() noexcept {
  const std::lock_guard lock(mutex_);
  auto& owner = device_;
  if (!owner.native)
    return {};
  collect_input();
  const auto now = steady_us();
  auto result = owner.input;
  result.frames_per_second = 0;
  if (owner.failed || !owner.input_active) {
    result.latency_valid = false;
    return result;
  }
  for (const auto time : owner.input_completions_us)
    if (time && now >= time && now - time < 1000000)
      result.frames_per_second += 1;
  if (owner.input_last_completion_us && now >= owner.input_last_completion_us)
    result.age_ms = static_cast<float>(now - owner.input_last_completion_us) / 1000.0f;
  return result;
}

bool SceneCaptureManager::compatible_queue(ID3D12CommandQueue* queue) noexcept {
  if (!queue || queue->GetDesc().Type != D3D12_COMMAND_LIST_TYPE_DIRECT)
    return false;
  return same_native_device(queue, device_.native);
}

/*
 * Classifies a batch that cannot enter the queue's serialized wrapper. This path must not wait or allocate:
 * failure to acquire either metadata lock is conservative, as are unknown or incompletely observed lists.
 * Capture evidence is stable while the manager lock is held.
 */
bool SceneCaptureManager::unordered_submission_affects_capture(ID3D12CommandQueue* queue,
                                                               UINT count,
                                                               ID3D12CommandList* const* native_lists) noexcept {
  if (!native_lists || !count || count > queue_hooks::kMaximumCommandLists)
    return true;
  const std::unique_lock lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock())
    return true;
  if (!usable() || !compatible_queue(queue))
    return false;
  for (UINT index = 0; index < count; ++index) {
    auto* native = static_cast<ID3D12GraphicsCommandList*>(native_lists[index]);
    std::shared_ptr<List> item;
    if (!native || !standalone::try_find_list(native, item) || !item || !item->capture.admitted || item->capture.awaiting_native_reset)
      return true;
    const auto& capture = item->capture;
    if (capture.packets || capture.consumer || capture.source_touched)
      return true;
  }
  return false;
}

/*
 * Marks capture state uncertain around a contended batch. This is deliberately only an atomic store: the
 * helper thread must never wait for the transaction that currently owns the queue wrapper. The hook stores
 * both before and after forwarding so an owner that consumes the first marker cannot miss the escaped work.
 */
void SceneCaptureManager::defer_unordered_submission() noexcept {
  deferred_escape_.store(true, std::memory_order_release);
}

/*
 * Invalidates source state after a forwarded submission whose effects could not be ordered. No previous
 * resource state is restored: capture remains paused until later ordered transitions establish it again.
 */
void SceneCaptureManager::invalidate_source_after_escape() noexcept {
  device_.source_states.invalidate_all();
  ++stats_.escaped_invalidations;
  stats_.tail_status = "unknown_source_state";
}

/*
 * Consumes deferred uncertainty at an ordered point while the manager lock is held. Setting the flag before
 * the escaped native forward ensures an owner already completing its batch cannot publish a tail capture
 * across the concurrent work.
 */
bool SceneCaptureManager::apply_deferred_escape() noexcept {
  if (!deferred_escape_.exchange(false, std::memory_order_acq_rel))
    return false;
  invalidate_source_after_escape();
  return true;
}

/*
 * Begins an ordered submission against the device timeline while both required locks are held. Any
 * cross-queue wait references an already queued signal, never future work.
 *
 * Both mutexes held; no future transaction enters until end/refused releases submission_mutex_. Therefore
 * every Wait targets an ALREADY queued Signal.
 */
SceneCaptureManager::Submission SceneCaptureManager::begin_transaction(ID3D12CommandQueue* queue,
                                                                       std::uint16_t packets,
                                                                       bool private_work) noexcept {
  auto& owner = device_;
  if (!usable() || !compatible_queue(queue) || next_receipt_ >= MaximumCounter || owner.last_signal >= MaximumCounter) {
    fail_device();
    return {};
  }
  if (owner.last_signal && FAILED(queue->Wait(owner.timeline, owner.last_signal))) {
    fail_device();
    return {};
  }
  transaction_ = {++next_receipt_, queue, owner.last_signal + 1, packets, private_work};
  transaction_.started_us = steady_us();
  for (std::size_t index = 0; index < packets_.size(); ++index)
    if (packets & (1u << index))
      ++packets_[index].in_flight;
  transaction_owner = this;
  thread_receipt = transaction_.id;
  return {transaction_.id, owner.timeline, transaction_.value};
}

/*
 * Serializes the actual native batch, rereads each recording's capture evidence after acquiring the
 * submission lock, and replays source events in Execute order. It snapshots source leases needed by
 * post-submit tails before Reset could release them.
 *
 * Lists seen here for the first time (created before the hooks, or recorded without an observed Reset) are
 * registered as unobserved before any lock is taken; the registry lock is never held under this manager's.
 * The actual Execute arguments keep these native objects alive throughout.
 *
 * Only fully observed recordings with no owned work or source-state effects can bypass ordering. Unknown
 * recordings keep conservative invalidation.
 *
 * Re-read every recording after serialization: Reset/retirement may have changed its identity, effects or
 * leases while this submission was waiting.
 *
 * ExecuteCommandLists order can differ from recording/allocation order. Replayed lists use their last
 * occurrence in this exact batch. Private queue-tail captures follow every application list in the same
 * receipt.
 *
 * The source reference is already owned by the recording. Snapshot it before original Execute so concurrent
 * successful Reset cannot retire the only lease before the post-submit tail acquires it.
 *
 * Same native wrapper thread must complete it.
 */
std::uint64_t SceneCaptureManager::before_submission(ID3D12CommandQueue* queue,
                                                     UINT count,
                                                     ID3D12CommandList* const* native_lists) noexcept {
  if (transaction_owner || !native_lists || !count || count > queue_hooks::kMaximumCommandLists)
    return 0;
  std::array<std::shared_ptr<List>, queue_hooks::kMaximumCommandLists> items;
  bool missing = false;
  for (UINT index = 0; index < count; ++index) {
    auto* native = static_cast<ID3D12GraphicsCommandList*>(native_lists[index]);
    items[index] = native ? standalone::find_list(native) : nullptr;
    missing |= native && !items[index];
  }
  if (missing) {
    bool compatible = false;
    {
      const std::lock_guard lock(mutex_);
      compatible = usable() && compatible_queue(queue);
    }
    if (compatible)
      for (UINT index = 0; index < count; ++index)
        if (!items[index] && native_lists[index])
          items[index] = standalone::discover_list(static_cast<ID3D12GraphicsCommandList*>(native_lists[index]));
  }
  const auto unknown = [&](UINT index) {
    const auto& item = items[index];
    return !item || !item->capture.admitted || item->capture.awaiting_native_reset;
  };
  {
    const std::lock_guard lock(mutex_);
    apply_deferred_escape();
    bool known_unrelated = true;
    for (UINT index = 0; index < count && known_unrelated; ++index)
      known_unrelated =
          !unknown(index) && !items[index]->capture.packets && !items[index]->capture.consumer && !items[index]->capture.source_touched;
    if (known_unrelated)
      return 0;
  }
  std::unique_lock submission_lock(submission_mutex_);
  const std::lock_guard lock(mutex_);
  auto& owner = device_;
  std::uint16_t mask = 0;
  bool work = false, consumer = false, input_copy = false;
  bool source_work = false, unknown_lists = false;
  std::uint64_t unknown_list_count = 0;
  for (UINT index = 0; index < count; ++index) {
    if (unknown(index)) {
      unknown_lists = true;
      ++unknown_list_count;
      continue;
    }
    const auto& item = items[index]->capture;
    if (!item.packets && !item.consumer && !item.source_touched)
      continue;
    work = true;
    mask |= item.packets;
    consumer |= item.consumer;
    input_copy |= item.consumer && owner.input_active && item.input_epoch == owner.input_epoch;
    source_work |= item.source_touched;
  }
  if (unknown_lists && !work && owner.native && compatible_queue(queue)) {
    owner.source_states.invalidate_all();
    ++stats_.invalidations.unknown_batches;
    stats_.invalidations.unknown_lists += unknown_list_count;
  }
  if (!work || !owner.native || (!mask && !consumer && !source_work))
    return 0;
  const auto result = begin_transaction(queue, mask, false);
  if (result.receipt) {
    if (input_copy) {
      transaction_.input_epoch = owner.input_epoch;
      transaction_.image_id = owner.image_id;
      transaction_.capture_time_us = owner.capture_time_us;
    }
    transaction_.last_position = count;
    for (UINT index = 0; mask && index < count; ++index) {
      if (unknown(index) || !items[index]->capture.packets)
        continue;
      for (std::size_t slot = 0; slot < packets_.size(); ++slot)
        if (items[index]->capture.packets & (1u << slot))
          transaction_.packet_positions[slot] = index + 1;
    }
    transaction_.source_work = source_work;
    owner.source_states.begin_batch();
    if (unknown_lists) {
      owner.source_states.invalidate_all();
      ++stats_.invalidations.unknown_batches;
      stats_.invalidations.unknown_lists += unknown_list_count;
    } else
      for (UINT index = 0; index < count; ++index) {
        const auto& item = items[index]->capture;
        if (item.source_touched) {
          if (!owner.source_states.apply(item.source_effects))
            ++stats_.invalidations.invalid_recordings;
          if (source_tracking_)
            for (std::size_t lease_index = 0; lease_index < item.source_lease_count; ++lease_index) {
              const auto& lease = item.source_leases[lease_index];
              if (!retain_source_lease(transaction_.source_leases, transaction_.source_lease_count, lease.key, lease.native))
                owner.source_states.invalidate_all();
            }
        }
      }
    submission_lock.release();
  }
  return result.receipt;
}

/*
 * Completes the receipt, optionally records admissible tail captures, then signals and assigns timeline
 * ownership before releasing serialization. Even refused forwarded work needs a retirement signal but
 * publishes no capture contents.
 *
 * Signal even a refused/aborted receipt to retire already forwarded work; this is ordering evidence, never
 * publication of its capture contents.
 */
bool SceneCaptureManager::finish_transaction(std::uint64_t receipt, bool refused, bool fatal) noexcept {
  if (transaction_owner != this || thread_receipt != receipt || !receipt)
    return false;
  bool success = false;
  {
    const std::lock_guard lock(mutex_);
    auto& pending = transaction_;
    if (pending.id == receipt) {
      auto& owner = device_;
      const bool original_refused = refused;
      bool deferred_escape = apply_deferred_escape();
      refused |= deferred_escape;
      if (!refused && pending.source_work && source_tracking_ && capture_enabled_)
        record_queue_tail(pending);
      if (apply_deferred_escape()) {
        deferred_escape = true;
        refused = true;
      }
      success = SUCCEEDED(pending.queue->Signal(owner.timeline, pending.value));
      if (success)
        owner.last_signal = pending.value;

      if (!success || (original_refused && fatal))
        fail_device();
      else if (refused && !deferred_escape)
        invalidate_source_after_escape();
      if (success && !refused && pending.image_id) {
        if (pending.private_work) {
          owner.image_id = pending.image_id;
          owner.capture_time_us = pending.capture_time_us;
        } else if (pending.input_epoch == owner.input_epoch && owner.input_active && pending.image_id > owner.input_last_image) {
          collect_input();
          for (auto& sample : owner.input_pending)
            if (!sample.fence_value) {
              sample = {pending.value, pending.capture_time_us};
              owner.input_last_image = pending.image_id;
              break;
            }
        }
      }
      for (std::size_t index = 0; index < packets_.size(); ++index) {
        if (!(pending.packets & (1u << index)))
          continue;
        auto& packet = packets_[index];
        --packet.in_flight;
        if (success && !refused) {
          packet.producer = pending.queue;
          packet.order = {pending.value, pending.packet_positions[index]};
          packet.capture_time_us = pending.started_us;
          ++packet.submitted;
        } else {
          quarantine(packet);
        }
      }
      release_source_leases(pending.source_leases, pending.source_lease_count);
      pending = {};
      collect();
    }
  }
  transaction_owner = nullptr;
  thread_receipt = 0;
  submission_mutex_.unlock();
  return success && !refused;
}

void SceneCaptureManager::after_submission(ID3D12CommandQueue* queue, std::uint64_t receipt) noexcept {
  if (transaction_owner == this && transaction_.queue == queue)
    finish_transaction(receipt, false);
}
/*
 * Invalidates source evidence when a forwarded batch could not be observed reliably. Reentry remains
 * non-fatal so a later complete transition sequence can recover; malformed batches fail the device.
 *
 * The caller invokes this only when its non-blocking classification could not exclude capture effects.
 * Oversized or malformed batches remain fatal because referenced packets are unknown.
 */
void SceneCaptureManager::submission_refused(ID3D12CommandQueue* queue, queue_hooks::Refusal reason) noexcept {
  const bool fatal = reason != queue_hooks::Refusal::reentrant_submission;
  if (transaction_owner == this) {
    finish_transaction(thread_receipt, true, fatal);
    return;
  }
  const std::lock_guard lock(mutex_);
  if (device_.native && compatible_queue(queue)) {
    if (fatal)
      fail_device();
    else
      invalidate_source_after_escape();
  }
}

/*
 * Serializes compositor work on the same device timeline as native consumers so later uploads cannot execute
 * against an untracked output generation.
 */
SceneCaptureManager::Submission SceneCaptureManager::begin_private_submission(ID3D12CommandQueue* queue) noexcept {
  if (transaction_owner)
    return {};
  std::unique_lock submission_lock(submission_mutex_);
  const std::lock_guard lock(mutex_);
  if (!device_.native)
    return {};
  const auto result = begin_transaction(queue, 0, true);
  if (result.receipt)
    submission_lock.release();
  return result;
}
/*
 * Native consumer submissions cannot enter until this transaction completes, so their image identity
 * describes bytes they execute against, not record.
 */
bool SceneCaptureManager::end_private_submission(std::uint64_t receipt, std::uint64_t image_id, std::uint64_t capture_time_us) noexcept {
  if (transaction_owner != this || !transaction_.private_work || transaction_.id != receipt)
    return false;
  transaction_.image_id = image_id;
  transaction_.capture_time_us = capture_time_us;
  return finish_transaction(receipt, false);
}
void SceneCaptureManager::abort_private_submission(std::uint64_t receipt) noexcept {
  if (transaction_owner == this && transaction_.private_work)
    finish_transaction(receipt, true);
}

/*
 * Retires tombstoned identities and recycles packets only when their recording, producer and consumer
 * obligations allow it. Submitted uncertain work is quarantined rather than immediately reused.
 */
void SceneCaptureManager::collect() noexcept {
  collect_input();
  for (std::size_t index = 0; index < sources_.size(); ++index) {
    auto& source = sources_[index];
    if (!source.native || source_generations_[index].load(std::memory_order_acquire))
      continue;
    device_.source_states.unregister_source({reinterpret_cast<std::uint64_t>(source.native), source.generation});
    source = {};
  }
  for (auto& packet : packets_) {
    if (!packet.assigned || packet.quarantined)
      continue;
    if (packet.gpu.state() == SceneCaptureD3D12::State::consuming && packet.gpu.recycle()) {
      packet.assigned = packet.leased = false;
      continue;
    }
    if (!packet.retired || packet.in_flight || packet.gpu.state() != SceneCaptureD3D12::State::recorded)
      continue;
    if (!packet.submitted) {
      if (packet.gpu.discard_unsubmitted_retired())
        packet.assigned = false;
      else
        quarantine(packet);
    } else if (!packet.gpu.retire_serialized_recording(packet.producer)) {
      quarantine(packet);
    }
  }
}

/*
 * Returns only fence-completed captures whose handoff generation remains current, and retires obsolete
 * completed frames without publishing them.
 *
 * No external reader has seen this frame; the already-queued timeline point safely returns the packet after
 * the next collect.
 */
std::size_t SceneCaptureManager::poll_completed_frames(Frame* frames, std::size_t capacity) noexcept {
  if (!frames || !capacity)
    return 0;
  if (capacity > MaximumPackets)
    capacity = MaximumPackets;
  const std::lock_guard lock(mutex_);
  collect();
  std::size_t count = 0;
  for (auto& packet : packets_) {
    if (!packet.assigned || packet.quarantined || packet.leased || !packet.gpu.poll_ready())
      continue;
    if (!usable()) {
      quarantine(packet);
      continue;
    }
    if (!handoff_.is_current(packet.match)) {
      if (!packet.gpu.finish_consumption(device_.timeline, device_.last_signal))
        quarantine(packet);
      continue;
    }
    if (count == capacity)
      break;
    packet.leased = true;
    frames[count++] = {packet.token, packet.match, packet.gpu.ready_resource(), packet.order, packet.capture_time_us};
  }
  return count;
}
/*
 * Records the compositor's completion fence for a consumed capture, delaying packet reuse until that reader
 * finishes.
 */
bool SceneCaptureManager::finish_consumption(std::uint64_t token, ID3D12Fence* fence, std::uint64_t value) noexcept {
  const std::lock_guard lock(mutex_);
  for (auto& packet : packets_)
    if (packet.assigned && packet.leased && !packet.quarantined && packet.token == token) {
      if (!packet.gpu.finish_consumption(fence, value))
        return false;
      packet.leased = false;
      return true;
    }
  return false;
}
/*
 * Retires an unused completed frame on the existing timeline without claiming it was composed or displayed.
 */
bool SceneCaptureManager::discard_frame(std::uint64_t token) noexcept {
  const std::lock_guard lock(mutex_);
  for (auto& packet : packets_) {
    if (!packet.assigned || !packet.leased || packet.quarantined || packet.token != token)
      continue;
    if (!device_.native || !packet.gpu.finish_consumption(device_.timeline, device_.last_signal))
      return false;
    packet.leased = false;
    return true;
  }
  return false;
}

SceneCaptureManager::Statistics SceneCaptureManager::statistics() const noexcept {
  const std::lock_guard lock(mutex_);
  return stats_;
}
} /* namespace azp */
