/*
 * The DLL's single record of the simulator's D3D12 objects. d3d12_bridge.cpp creates one Resource per
 * relevant texture and one List per native direct command list, and retires them when the native object is
 * destroyed; a recycled address always gets a new record, so no stale identity is trusted.
 *
 * A List holds everything known about its current recording, in three parts with different owners:
 *   - recording state (bound render targets, readiness), written by the state-setter hooks;
 *   - render-pass scope (PassScope), written by the command-list boundary hooks;
 *   - capture evidence (CaptureRecording), guarded by the capture manager's mutex.
 * The first two are touched only by the thread recording the list: D3D12 forbids recording one list from
 * two threads at once.
 */

#pragma once
#include <d3d12.h>
#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>
#include "scene_source_state.hpp"

namespace azp::standalone {
struct Metadata {
  std::atomic<bool> alive{true};
  std::uint64_t id{};
  virtual void retire() noexcept { alive.store(false); }
  virtual ~Metadata() = default;
};
/*
 * A tracked texture. `normal_source` marks a camera-sized render target the capture manager may use as the
 * camera image.
 */
struct Resource : Metadata {
  ID3D12Resource* native{};
  D3D12_RESOURCE_DESC desc{};
  bool normal_source = false;
  void retire() noexcept override;
};
/*
 * What one render-target descriptor points at: a tracked resource and the mip level it renders to.
 */
struct View {
  std::shared_ptr<Resource> resource;
  UINT mip{};
};
/*
 * Render-pass scope of the current recording. The marked WASM upload may be replaced only outside every
 * active or suspended pass, on a recording that is not invalid. `epoch` records the hook epoch at Reset:
 * disabling capture advances that epoch, which invalidates every earlier recording.
 */
struct PassScope {
  std::uint64_t epoch = 0;
  bool active = false, suspended = false, invalid = false;
};
/*
 * One recording's reference to a camera source, taken from a live native argument.
 */
struct SourceLease {
  source_state::Key key;
  ID3D12Resource* native = nullptr;
};
/*
 * Capture evidence of the current recording: packets recorded on it, whether it reads the video output, the
 * ordered source-state effects and the source references it retains. `admitted` is set when the capture
 * manager accepts the list and cleared when the list is retired.
 */
struct CaptureRecording {
  bool admitted = false;
  bool awaiting_native_reset = false;
  std::uint16_t packets = 0;
  bool consumer = false;
  std::uint64_t input_epoch = 0;
  source_state::Recording source_effects;
  bool source_touched = false;
  std::array<SourceLease, source_state::Tracker::capacity> source_leases{};
  std::size_t source_lease_count = 0;
};
/*
 * One native direct command list. `recording` advances on each successful Reset; `ready` is true only for a
 * recording observed from its start. `targets` mirrors the currently bound render targets so each draw can
 * be attributed to a camera source.
 */
struct List : Metadata {
  ID3D12GraphicsCommandList* native{};
  std::atomic<std::uint64_t> recording{1};
  std::atomic<std::uint64_t> observation_epoch{};
  std::array<View, 8> targets{};
  UINT count{};
  bool ready = false;
  PassScope scope;
  CaptureRecording capture;
  void retire() noexcept override;
};

/*
 * The live record of a native list, or null. Uses a small per-thread cache of the lists this thread records.
 */
std::shared_ptr<List> find_list(ID3D12GraphicsCommandList*);
/*
 * Non-blocking lookup for queue submissions that must not wait behind Present or frame-generation helpers.
 * Returns false only when registry ownership could not be acquired; a true result may still contain null
 * when the list is definitely absent.
 */
bool try_find_list(ID3D12GraphicsCommandList*, std::shared_ptr<List>& result) noexcept;
/*
 * Finds or registers a direct list of the simulator's device, hooking its boundaries. A list first seen
 * mid-recording (`observed` false) is admitted only after its next successful Reset.
 */
std::shared_ptr<List> ensure_list(ID3D12GraphicsCommandList*, bool observed = false);
/*
 * Registers a list first seen at submission, as unobserved, without letting a bookkeeping failure escape.
 */
std::shared_ptr<List> discover_list(ID3D12GraphicsCommandList*) noexcept;
/*
 * Snapshot of every live list, for rare whole-registry operations; empty if it cannot be allocated.
 */
std::vector<std::shared_ptr<List>> live_lists() noexcept;
/*
 * True once the device hooks are installed and graphics observation is admitted.
 */
bool graphics_admitted() noexcept;
/*
 * True while video is requested (odd demand epoch).
 */
bool observation_enabled() noexcept;
/*
 * True when this recording started, with a real Reset, during the current demand period.
 */
bool recording_observed(const List&) noexcept;
/*
 * Maps render-target descriptors to tracked resources for the draws that follow (OMSetRenderTargets or a
 * render pass). Unknown or malformed descriptors leave those targets unattributed.
 */
void bind_render_targets(List&, UINT count, const D3D12_CPU_DESCRIPTOR_HANDLE* handles, BOOL contiguous);
/*
 * Registers a submitting DIRECT queue of the simulator's device with the capture manager.
 */
void discover_queue(ID3D12CommandQueue*) noexcept;
}  /* namespace azp::standalone */
