/*
 * Observes native draws, copies, render passes and resource barriers on tracked command lists. Each hook
 * forwards the original call exactly once and never changes its arguments; only a positively identified
 * marked WASM upload can be replaced. The hooks keep each list's render-pass scope, feed barrier and draw
 * evidence to the capture manager, and revoke capture evidence when a scope or batch is uncertain.
 */

#include "command_list_hooks.hpp"
#include <array>
#include <atomic>
#include "native_hooks.hpp"
#include "scene_runtime.hpp"

namespace azp::list_hooks {
namespace {
using standalone::HookSlot;
using standalone::List;
using standalone::OwnedWork;
using Legacy = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, UINT, const D3D12_RESOURCE_BARRIER*);
using Begin = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList4*,
                                       UINT,
                                       const D3D12_RENDER_PASS_RENDER_TARGET_DESC*,
                                       const D3D12_RENDER_PASS_DEPTH_STENCIL_DESC*,
                                       D3D12_RENDER_PASS_FLAGS);
using End = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList4*);
using Enhanced = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList7*, UINT, const D3D12_BARRIER_GROUP*);
using Draw = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, UINT, UINT, UINT, UINT);
using DrawIndexed = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, UINT, UINT, UINT, INT, UINT);
using CopyResource = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, ID3D12Resource*, ID3D12Resource*);
using CopyTexture = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*,
                                             const D3D12_TEXTURE_COPY_LOCATION*,
                                             UINT,
                                             UINT,
                                             UINT,
                                             const D3D12_TEXTURE_COPY_LOCATION*,
                                             const D3D12_BOX*);
/*
 * Legacy barrier batches are inspected completely up to this bound; larger or overflowing spans are
 * rejected in full.
 */
constexpr UINT maximum_legacy_metadata_barriers = 1u << 20;
/*
 * `enabled` is false until the hooks are installed and after a hook was found changed. Disabling also
 * advances `epoch`, which invalidates every recording that began before.
 */
std::atomic<bool> enabled{false};
std::atomic<std::uint64_t> epoch{1};
SRWLOCK control_lock = SRWLOCK_INIT;
HookSlot legacy_slot, begin_slot, end_slot, enhanced_slot, draw_slot, draw_indexed_slot, copy_texture_slot, copy_resource_slot;
/*
 * End (cell 69) of an active-pass table the runtime switched to inside BeginRenderPass.
 */
struct ActiveEnd {
  void* table = nullptr;
  HookSlot slot;
};
std::array<ActiveEnd, 8> active_ends{};
std::size_t active_end_count = 0;
void* saved_table = nullptr;
thread_local bool inside = false;
struct Lock {
  SRWLOCK& lock;
  bool exclusive;
  Lock(SRWLOCK& value, bool write) : lock(value), exclusive(write) {
    if (write)
      AcquireSRWLockExclusive(&lock);
    else
      AcquireSRWLockShared(&lock);
  }
  ~Lock() {
    if (exclusive)
      ReleaseSRWLockExclusive(&lock);
    else
      ReleaseSRWLockShared(&lock);
  }
};
/*
 * Marks this thread as inside a hook: D3D12 calls made meanwhile (AZP's own capture work, nested runtime
 * calls) forward without observation.
 */
struct Guard {
  bool previous = inside;
  Guard() { inside = true; }
  ~Guard() { inside = previous; }
};
/*
 * A value copy of a list's scope, with `invalid` including recordings begun before capture was disabled.
 * `known` is false for an untracked or retired list.
 */
struct Scope {
  bool known = false, active = false, suspended = false, invalid = false;
};
Scope scope_of(const List* item) noexcept {
  if (!item || !item->alive.load(std::memory_order_acquire))
    return {};
  const auto& s = item->scope;
  return {true, s.active, s.suspended, s.invalid || s.epoch != epoch.load()};
}
/*
 * A failed/changed hook may have missed pass calls. Never revive the previous recording's empty-pass
 * assumption when a later installation repairs slots.
 */
void disable_capture() noexcept {
  enabled.store(false);
  epoch.fetch_add(1);
}
std::uint32_t scope_invalidation(const Scope& value) noexcept {
  return (!enabled.load() ? InvalidationObserverDisabled : 0u) | ((value.suspended || value.invalid) ? InvalidationPassState : 0u);
}
bool global_uncertainty(std::uint32_t reasons) noexcept {
  return (reasons & ~(InvalidationPassBegin | InvalidationSplitBarrier | InvalidationAliasOrDiscard)) != 0;
}
/*
 * Revokes the recording's capture evidence before uncertain native work. When the uncertainty is limited to
 * the bound targets (pass begin, split barrier, alias/discard), only those sources are invalidated;
 * otherwise the whole recording is. The next capture must rebuild its proof from actual observed work.
 */
void notify_invalidation(List& item, std::uint32_t reasons) noexcept {
  if (!reasons || !item.alive.load(std::memory_order_acquire))
    return;
  const OwnedWork owned;
  const bool scoped = !global_uncertainty(reasons);
  scene_runtime::manager().note_source_invalidation(reasons, scoped);
  if (scoped && item.count) {
    std::array<ID3D12Resource*, 8> targets{};
    std::array<std::uint64_t, 8> generations{};
    UINT count = 0;
    for (UINT i = 0; i < item.count; ++i) {
      const auto& target = item.targets[i].resource;
      if (target && target->alive) {
        targets[count] = target->native;
        generations[count++] = target->id;
      }
    }
    if (count) {
      scene_runtime::manager().invalidate_source_targets(item, count, targets.data(), generations.data());
      return;
    }
  }
  scene_runtime::manager().invalidate_source_recording(item, true);
}
/*
 * Records the actual targets after a native draw. Source-state evidence is kept while idle so capture can
 * resume without guessing layouts.
 */
void record_draw(List& item, bool allowed) noexcept {
  const OwnedWork owned;
  if (!standalone::graphics_admitted() || !item.ready || (!standalone::recording_observed(item) && !item.count))
    return;
  std::array<ID3D12Resource*, 8> sources;
  std::array<std::uint64_t, 8> ids;
  UINT count = 0;
  for (UINT i = 0; i < item.count; ++i) {
    const auto& target = item.targets[i];
    if (target.resource && target.resource->alive && !target.mip) {
      sources[count] = target.resource->native;
      ids[count++] = target.resource->id;
    }
  }
  scene_runtime::manager().observe_source_draw_after(item, count, sources.data(), ids.data(), allowed);
}
/*
 * After a nonzero draw: revokes evidence for an uncertain scope, then records the draw while the same
 * recording is current.
 */
void after_draw(List& item) noexcept {
  const auto current = scope_of(&item);
  if (!current.known)
    return;
  if (const auto reasons = scope_invalidation(current)) {
    notify_invalidation(item, reasons);
    if (!item.alive.load(std::memory_order_acquire))
      return;
  }
  record_draw(item, enabled.load() && !current.active && !current.suspended && !current.invalid);
}
void STDMETHODCALLTYPE
draw(ID3D12GraphicsCommandList* list, UINT vertices, UINT instances, UINT first_vertex, UINT first_instance) noexcept {
  const auto original = draw_slot.forward<Draw>();
  if (inside) {
    original(list, vertices, instances, first_vertex, first_instance);
    return;
  }
  const Guard guard;
  const auto item = standalone::find_list(list);
  original(list, vertices, instances, first_vertex, first_instance);
  if (item && vertices && instances)
    after_draw(*item);
}
void STDMETHODCALLTYPE draw_indexed(ID3D12GraphicsCommandList* list,
                                    UINT indices,
                                    UINT instances,
                                    UINT first_index,
                                    INT vertex_offset,
                                    UINT first_instance) noexcept {
  const auto original = draw_indexed_slot.forward<DrawIndexed>();
  if (inside) {
    original(list, indices, instances, first_index, vertex_offset, first_instance);
    return;
  }
  const Guard guard;
  const auto item = standalone::find_list(list);
  original(list, indices, instances, first_index, vertex_offset, first_instance);
  if (item && indices && instances)
    after_draw(*item);
}
/*
 * Forwards the original resource copy after delivering invalid scope evidence: a copy in an uncertain scope
 * may have written a camera source.
 */
void STDMETHODCALLTYPE copy_resource(ID3D12GraphicsCommandList* list, ID3D12Resource* destination, ID3D12Resource* source) noexcept {
  const auto original = copy_resource_slot.forward<CopyResource>();
  if (inside) {
    original(list, destination, source);
    return;
  }
  const Guard guard;
  if (const auto item = standalone::find_list(list))
    notify_invalidation(*item, scope_invalidation(scope_of(item.get())));
  original(list, destination, source);
}
/*
 * Offers the copy to the marked-upload replacement, then forwards the original unless it was replaced.
 * Upload arguments are inspected even on untracked lists, so the WASM input status can tell an unsupported
 * upload recording from a missing marker; only an observed ordinary recording outside passes may replace
 * its own positively identified upload. A copy establishes its own boundary: no preceding draw is needed,
 * which does not relax the separate render-target or pass guards. The replacement copy runs inside this
 * hook's guard, so it is forwarded without observation.
 */
void STDMETHODCALLTYPE copy_texture(ID3D12GraphicsCommandList* list,
                                    const D3D12_TEXTURE_COPY_LOCATION* destination,
                                    UINT x,
                                    UINT y,
                                    UINT z,
                                    const D3D12_TEXTURE_COPY_LOCATION* source,
                                    const D3D12_BOX* box) noexcept {
  const auto original = copy_texture_slot.forward<CopyTexture>();
  if (inside) {
    original(list, destination, x, y, z, source, box);
    return;
  }
  const Guard guard;
  const auto item = standalone::find_list(list);
  const auto identity = scope_of(item.get());
  if (item)
    notify_invalidation(*item, scope_invalidation(identity));
  if (enabled.load() && standalone::observation_enabled() && standalone::graphics_admitted()) {
    const bool permitted = identity.known && !identity.active && !identity.suspended && !identity.invalid && item->alive.load();
    const OwnedWork owned;
    const char* refusal = !item                                                    ? "unregistered_command_list"
                          : !item->ready || !standalone::recording_observed(*item) ? "unobserved_recording"
                          : !permitted                                             ? "render_pass_or_invalid_recording"
                                                                                   : nullptr;
    if (scene_runtime::replace_wasm_upload(list, destination, x, y, z, source, box, refusal) && permitted)
      return;
  }
  original(list, destination, x, y, z, source, box);
}
/*
 * Observes legacy barrier metadata in order and invalidates uncertain batches before forwarding the original
 * barrier array intact, without splitting or replay.
 *
 * Invalidate after metadata delivery so a partial/uncertain batch cannot leave a seemingly complete state
 * from its last observed prefix element.
 */
void STDMETHODCALLTYPE legacy(ID3D12GraphicsCommandList* list, UINT count, const D3D12_RESOURCE_BARRIER* barriers) noexcept {
  const auto original = legacy_slot.forward<Legacy>();
  if (inside) {
    original(list, count, barriers);
    return;
  }
  const Guard guard;
  const auto item = standalone::find_list(list);
  if (!item) {
    original(list, count, barriers);
    return;
  }
  auto uncertainty = scope_invalidation(scope_of(item.get()));
  const bool metadata_complete =
      !count || (barriers && count <= maximum_legacy_metadata_barriers &&
                 reinterpret_cast<std::uintptr_t>(barriers) <= UINTPTR_MAX - std::uint64_t(count) * sizeof(*barriers));
  if (!metadata_complete)
    uncertainty |= InvalidationBarrierBatch;
  if (metadata_complete && barriers) {
    for (UINT n = 0; n < count; ++n) {
      const auto& b = barriers[n];
      if (b.Type == D3D12_RESOURCE_BARRIER_TYPE_ALIASING) {
        if (!b.Aliasing.pResourceBefore || !b.Aliasing.pResourceAfter)
          uncertainty |= InvalidationAliasOrDiscard;
      } else if (b.Type == D3D12_RESOURCE_BARRIER_TYPE_TRANSITION) {
        if (b.Flags != D3D12_RESOURCE_BARRIER_FLAG_NONE && b.Flags != D3D12_RESOURCE_BARRIER_FLAG_BEGIN_ONLY &&
            b.Flags != D3D12_RESOURCE_BARRIER_FLAG_END_ONLY)
          uncertainty |= InvalidationBarrierBatch;
        if (!b.Transition.pResource)
          uncertainty |= InvalidationBarrierBatch;
      } else if (b.Type != D3D12_RESOURCE_BARRIER_TYPE_UAV)
        uncertainty |= InvalidationBarrierBatch;
      if (b.Type != D3D12_RESOURCE_BARRIER_TYPE_TRANSITION && b.Flags != D3D12_RESOURCE_BARRIER_FLAG_NONE)
        uncertainty |= InvalidationBarrierBatch;
    }
    for (UINT n = 0; n < count && item->alive.load(std::memory_order_acquire); ++n)
      scene_runtime::manager().observe_source_legacy(*item, barriers[n]);
  }
  notify_invalidation(*item, uncertainty);
  original(list, count, barriers);
}
bool install_active_end(ID3D12GraphicsCommandList4* list) noexcept;
/*
 * Tracks native render-pass scope and target metadata, refusing suspended/local-access cases that cannot
 * admit extra work. If the runtime changes the active vtable, the actual End slot is verified and hooked
 * before returning.
 *
 * PRESERVE_LOCAL passes permit state setup between passes, but no added GPU operations. Refuse their entire
 * recording rather than inferring a resource's local-access lifetime from later barrier state alone.
 *
 * A resuming pass can continue another command list. Only its completed, nonsuspending End permits capture
 * again; no intervening copy is added.
 */
void STDMETHODCALLTYPE begin(ID3D12GraphicsCommandList4* list,
                             UINT count,
                             const D3D12_RENDER_PASS_RENDER_TARGET_DESC* targets,
                             const D3D12_RENDER_PASS_DEPTH_STENCIL_DESC* depth,
                             D3D12_RENDER_PASS_FLAGS flags) noexcept {
  const auto original = begin_slot.forward<Begin>();
  if (inside) {
    original(list, count, targets, depth, flags);
    return;
  }
  const Guard guard;
  const auto item = standalone::find_list(list);
  if (!item) {
    original(list, count, targets, depth, flags);
    return;
  }
  const auto identity = scope_of(item.get());
  if (identity.known) {
    auto& value = item->scope;
    if (value.active)
      value.invalid = true;
    if ((static_cast<UINT>(flags) & ~7u) != 0)
      value.invalid = true;
    if (count > 8 || (count && !targets))
      value.invalid = true;
    else
      for (UINT n = 0; n < count; ++n)
        if (static_cast<UINT>(targets[n].BeginningAccess.Type) > 3 || static_cast<UINT>(targets[n].EndingAccess.Type) > 3)
          value.invalid = true;
    if (depth && (static_cast<UINT>(depth->DepthBeginningAccess.Type) > 3 || static_cast<UINT>(depth->DepthEndingAccess.Type) > 3 ||
                  static_cast<UINT>(depth->StencilBeginningAccess.Type) > 3 || static_cast<UINT>(depth->StencilEndingAccess.Type) > 3))
      value.invalid = true;
    if (value.suspended && (flags & D3D12_RENDER_PASS_FLAG_RESUMING_PASS) == 0)
      value.invalid = true;
    value.active = true;
    value.suspended = (flags & D3D12_RENDER_PASS_FLAG_SUSPENDING_PASS) != 0;
  }
  if (count > 8 || (count && !targets)) {
    item->targets = {};
    item->count = 0;
  } else {
    std::array<D3D12_CPU_DESCRIPTOR_HANDLE, 8> handles{};
    for (UINT i = 0; i < count; ++i)
      handles[i] = targets[i].cpuDescriptor;
    standalone::bind_render_targets(*item, count, handles.data(), FALSE);
  }
  const auto after_begin = scope_of(item.get());
  notify_invalidation(*item, InvalidationPassBegin | scope_invalidation(identity) |
                                 ((after_begin.invalid || after_begin.suspended || (flags & D3D12_RENDER_PASS_FLAG_RESUMING_PASS) != 0)
                                      ? InvalidationPassState
                                      : 0u));
  original(list, count, targets, depth, flags);
  track_active_pass(list);
}
/*
 * Forwards the matching native End and updates the same recording's pass scope. End access can
 * discard/resolve and its targets are unbound: no image is copied here and no binding survives; only later
 * actual transitions prove layout/state.
 */
void end_impl(ID3D12GraphicsCommandList4* list, End original) noexcept {
  if (inside) {
    original(list);
    return;
  }
  const Guard guard;
  const auto item = standalone::find_list(list);
  if (!item) {
    original(list);
    return;
  }
  const auto identity = scope_of(item.get());
  if (!identity.active || identity.suspended || identity.invalid || !enabled.load())
    notify_invalidation(*item, InvalidationPassState | scope_invalidation(identity));
  original(list);
  if (!item->alive.load(std::memory_order_acquire))
    return;
  item->targets = {};
  item->count = 0;
  auto& value = item->scope;
  if (!value.active)
    value.invalid = true;
  value.active = false;
}
void STDMETHODCALLTYPE end(ID3D12GraphicsCommandList4* list) noexcept {
  end_impl(list, end_slot.forward<End>());
}
template <std::size_t Index>
void STDMETHODCALLTYPE active_end(ID3D12GraphicsCommandList4* list) noexcept {
  end_impl(list, active_ends[Index].slot.forward<End>());
}
/*
 * Validates enhanced-barrier groups and collects their texture batches without treating global/buffer
 * uncertainty as texture-state evidence.
 */
bool texture_batches(UINT count, const D3D12_BARRIER_GROUP* groups, UINT max_groups, UINT max_barriers) noexcept {
  if (!count || count > max_groups || !groups)
    return false;
  UINT total = 0;
  for (UINT n = 0; n < count; ++n) {
    if (groups[n].NumBarriers > max_barriers - total)
      return false;
    total += groups[n].NumBarriers;
    switch (groups[n].Type) {
      case D3D12_BARRIER_TYPE_TEXTURE:
        if (groups[n].NumBarriers && !groups[n].pTextureBarriers)
          return false;
        break;
      case D3D12_BARRIER_TYPE_BUFFER:
        if (groups[n].NumBarriers && !groups[n].pBufferBarriers)
          return false;
        break;
      case D3D12_BARRIER_TYPE_GLOBAL:
        if (groups[n].NumBarriers && !groups[n].pGlobalBarriers)
          return false;
        break;
      default:
        return false;
    }
  }
  return true;
}
/*
 * Observes enhanced texture transitions and scope uncertainty in order (at most 64 groups and 4096
 * barriers) before forwarding the original barrier groups unchanged.
 */
void STDMETHODCALLTYPE enhanced(ID3D12GraphicsCommandList7* list, UINT count, const D3D12_BARRIER_GROUP* groups) noexcept {
  const auto original = enhanced_slot.forward<Enhanced>();
  if (inside) {
    original(list, count, groups);
    return;
  }
  const Guard guard;
  const auto item = standalone::find_list(list);
  if (!item) {
    original(list, count, groups);
    return;
  }
  auto uncertainty = scope_invalidation(scope_of(item.get()));
  const bool metadata_complete = texture_batches(count, groups, 64, 4096);
  if (count && !metadata_complete)
    uncertainty |= InvalidationBarrierBatch;
  if (metadata_complete) {
    for (UINT g = 0; g < count; ++g) {
      if (groups[g].Type != D3D12_BARRIER_TYPE_TEXTURE)
        continue;
      for (UINT n = 0; n < groups[g].NumBarriers; ++n) {
        const auto& b = groups[g].pTextureBarriers[n];
        if (b.Flags != D3D12_TEXTURE_BARRIER_FLAG_NONE && b.Flags != D3D12_TEXTURE_BARRIER_FLAG_DISCARD)
          uncertainty |= InvalidationBarrierBatch;
        if (!b.pResource)
          uncertainty |= InvalidationBarrierBatch;
      }
    }
    for (UINT g = 0; g < count; ++g) {
      if (groups[g].Type != D3D12_BARRIER_TYPE_TEXTURE)
        continue;
      for (UINT n = 0; n < groups[g].NumBarriers && item->alive.load(std::memory_order_acquire); ++n)
        scene_runtime::manager().observe_source_enhanced(*item, groups[g].pTextureBarriers[n]);
    }
  }
  notify_invalidation(*item, uncertainty);
  original(list, count, groups);
}
/*
 * Reads the object's current vtable pointer without trusting it: an unreadable object fails instead of
 * faulting. The table must leave room for cell 80.
 */
bool read_table(const void* object, void*& table) noexcept {
  SIZE_T bytes = 0;
  return object && reinterpret_cast<std::uintptr_t>(object) % 8 == 0 &&
         ReadProcessMemory(GetCurrentProcess(), object, &table, sizeof(table), &bytes) && bytes == sizeof(table) && table &&
         reinterpret_cast<std::uintptr_t>(table) <= UINTPTR_MAX - 81 * 8;
}
/*
 * Resolves the live active-pass End cell, reusing only an exact known table or installing a separately
 * retained forwarding target. Known, installed tables need one read and no mutation; lock promotion repeats
 * every check, since SRW locks cannot be upgraded.
 */
bool install_active_end(ID3D12GraphicsCommandList4* list) noexcept {
  constexpr std::array<End, 8> wrappers{&active_end<0>, &active_end<1>, &active_end<2>, &active_end<3>,
                                        &active_end<4>, &active_end<5>, &active_end<6>, &active_end<7>};
  void* table = nullptr;
  {
    const Lock read_lock(control_lock, false);
    if (!enabled.load() || !read_table(list, table))
      return false;
    if (table == saved_table && end_slot.installed())
      return end_slot.intact();
    for (std::size_t n = 0; n < active_end_count; ++n)
      if (active_ends[n].table == table)
        return active_ends[n].slot.intact();
  }
  const Lock lock(control_lock, true);
  if (!enabled.load() || !read_table(list, table))
    return false;
  if (table == saved_table)
    return end_slot.intact();
  for (std::size_t n = 0; n < active_end_count; ++n)
    if (active_ends[n].table == table)
      return active_ends[n].slot.install(list, 69, reinterpret_cast<void*>(wrappers[n]));
  if (active_end_count == active_ends.size())
    return false;
  auto& entry = active_ends[active_end_count];
  entry.table = table;
  const bool installed = entry.slot.install(list, 69, reinterpret_cast<void*>(wrappers[active_end_count]));
  if (entry.slot.installed())
    ++active_end_count;
  return installed;
}
} /* namespace */

/*
 * The first call records the shared table; every call installs or re-verifies the eight hooks and the known
 * active-pass End hooks, then enables capture.
 */
bool install(ID3D12GraphicsCommandList* list) noexcept {
  if (inside || !list)
    return false;
  const Lock control(control_lock, true);
  ID3D12GraphicsCommandList7* extended = nullptr;
  if (FAILED(list->QueryInterface(IID_PPV_ARGS(&extended))) || !extended)
    return false;
  const bool same = static_cast<ID3D12GraphicsCommandList*>(extended) == list;
  extended->Release();
  if (!same || list->GetType() != D3D12_COMMAND_LIST_TYPE_DIRECT)
    return false;
  void* table = nullptr;
  if (!read_table(list, table) || (saved_table && table != saved_table))
    return false;
  saved_table = table;
  const struct {
    HookSlot& slot;
    unsigned index;
    void* wrapper;
  } hooks[] = {{legacy_slot, 26, reinterpret_cast<void*>(&legacy)},
               {begin_slot, 68, reinterpret_cast<void*>(&begin)},
               {end_slot, 69, reinterpret_cast<void*>(&end)},
               {enhanced_slot, 80, reinterpret_cast<void*>(&enhanced)},
               {draw_slot, 12, reinterpret_cast<void*>(&draw)},
               {draw_indexed_slot, 13, reinterpret_cast<void*>(&draw_indexed)},
               {copy_texture_slot, 16, reinterpret_cast<void*>(&copy_texture)},
               {copy_resource_slot, 17, reinterpret_cast<void*>(&copy_resource)}};
  for (const auto& hook : hooks)
    if (!hook.slot.install(list, hook.index, hook.wrapper)) {
      disable_capture();
      return false;
    }
  for (std::size_t n = 0; n < active_end_count; ++n)
    if (!active_ends[n].slot.intact()) {
      disable_capture();
      return false;
    }
  enabled.store(true);
  return true;
}
void track_active_pass(ID3D12GraphicsCommandList4* list) noexcept {
  if (!install_active_end(list))
    disable_capture();
}
void begin_recording(standalone::List& item) noexcept {
  const auto current = epoch.load();
  item.scope = {current, false, false, !enabled.load()};
}
void invalidate_recording(standalone::List& item, std::uint32_t reasons) noexcept {
  if (!item.alive.load(std::memory_order_acquire))
    return;
  item.scope.invalid = true;
  const Guard guard;
  notify_invalidation(item, reasons ? reasons : InvalidationUnobservedWork);
}
bool operational() noexcept {
  return enabled.load();
}
} /* namespace azp::list_hooks */
