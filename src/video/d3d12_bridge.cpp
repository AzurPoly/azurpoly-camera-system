/*
 * Connects native D3D12 activity to the camera image pipeline. It hooks resource creation, render-target
 * views, command-list creation/Reset/Close and the render-target setters on the simulator's device, and
 * keeps the single record of each tracked object (d3d12_objects.hpp):
 *
 *   - resource and command-list incarnations, so no recycled address is trusted;
 *   - the render targets bound on each recording, so draws can be attributed;
 *   - the camera source candidates handed to the capture manager (scene_capture_manager.cpp).
 *
 * The command-list boundary hooks (command_list_hooks.cpp) and the queue hook (queue_hooks.cpp) read these
 * records. Every hook forwards the original call exactly once and never changes application arguments.
 */

#include "d3d12_bridge.hpp"
#include <dxgi1_6.h>
#include <algorithm>
#include <memory>
#include <mutex>
#include <tuple>
#include <unordered_map>
#include <utility>
#include "command_list_hooks.hpp"
#include "d3d12_objects.hpp"
#include "native_hooks.hpp"
#include "queue_hooks.hpp"
#include "resource_guards.hpp"

namespace azp::standalone {
namespace {
namespace runtime = scene_runtime;
constexpr GUID LifetimeId{0x986208c6, 0x68b9, 0x450d, {0x86, 0x0c, 0xac, 0x85, 0x60, 0x07, 0x88, 0x01}};
class Lifetime final : public IUnknown {
 public:
  explicit Lifetime(std::shared_ptr<Metadata> p) : value_(std::move(p)) {}
  HRESULT STDMETHODCALLTYPE QueryInterface(REFIID iid, void** out) override {
    if (!out)
      return E_POINTER;
    *out = nullptr;
    if (iid != __uuidof(IUnknown))
      return E_NOINTERFACE;
    *out = static_cast<IUnknown*>(this);
    AddRef();
    return S_OK;
  }
  ULONG STDMETHODCALLTYPE AddRef() override { return ++references_; }
  ULONG STDMETHODCALLTYPE Release() override {
    const auto n = --references_;
    if (!n) {
      value_->retire();
      delete this;
    }
    return n;
  }

 private:
  std::atomic<ULONG> references_{1};
  std::shared_ptr<Metadata> value_;
};
/*
 * Every tracked object. `mutex` guards the maps and is a leaf lock: nothing that takes another lock runs
 * while it is held, so the capture manager may take it under its own lock.
 */
struct Registry {
  std::recursive_mutex mutex;
  /*
   * Only demand transitions and publication of a real Reset/new-list proof take this lock. Ordinary setters
   * and metadata callbacks never take it.
   */
  std::mutex observation_mutex;
  std::atomic<std::uint64_t> observation_epoch{1};  /* Odd: observing; even: idle. */
  ID3D12Device* device{};
  std::uint64_t next_id = 0;
  UINT rtv_stride{};
  std::atomic<bool> ready{};
  std::atomic<std::uint64_t> failures{};
  const char* error = "not_started";
  std::unordered_map<ID3D12Resource*, std::shared_ptr<Resource>> resources;
  std::unordered_map<ID3D12GraphicsCommandList*, std::shared_ptr<List>> lists;
  std::unordered_map<SIZE_T, View> rtvs;
};
Registry& registry() {
  static auto* r = new Registry;
  return *r;
}
/*
 * Replaces one tracked render-target descriptor. The map is bounded; an unknown or dropped descriptor simply
 * leaves its draws unattributed. No descriptor or native resource memory is read here.
 */
void replace_view(Registry& r, SIZE_T handle, const View& view) {
  const auto old = r.rtvs.find(handle);
  if (view.resource && (old != r.rtvs.end() || r.rtvs.size() < 65536))
    r.rtvs[handle] = view;
  else if (old != r.rtvs.end())
    r.rtvs.erase(old);
}
bool attach(ID3D12Object* object, const std::shared_ptr<Metadata>& metadata) {
  auto* lifetime = new Lifetime(metadata);
  const auto hr = object->SetPrivateDataInterface(LifetimeId, lifetime);
  lifetime->Release();
  return SUCCEEDED(hr);
}
void error(const char* value) noexcept {
  auto& r = registry();
  ++r.failures;
  const std::lock_guard lock(r.mutex);
  r.error = value;
}
/*
 * Contains C++ failures from an observation callback and records an operational error without unwinding
 * through the simulator's D3D12 call.
 */
template <class F>
void observe_safely(F&& action) noexcept {
  try {
    action();
  } catch (...) {
    registry().ready = false;
    error("native_observation_allocation_failed");
  }
}
/*
 * Keep creation/lifetime metadata for camera-sized render textures only. Among them, a supported format
 * makes a source candidate (normal_source below); the camera publishes which one holds its image.
 */
bool relevant(const D3D12_RESOURCE_DESC& d) noexcept {
  return d.Dimension == D3D12_RESOURCE_DIMENSION_TEXTURE2D && camera_sized(static_cast<UINT>(d.Width), d.Height) &&
         d.DepthOrArraySize == 1 && d.SampleDesc.Count == 1 && (d.Flags & D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET) != 0;
}
/*
 * Checks that an object belongs to the retained native device, comparing COM identity when an alternate
 * interface is involved.
 *
 * The registry retains this exact interface for its entire lifetime. Equal pointers already prove identity;
 * alternate interfaces still use IUnknown.
 */
bool same_device(ID3D12Device* device) noexcept {
  IUnknown *a{}, *b{};
  auto* expected = registry().device;
  if (!device || !expected)
    return false;
  if (device == expected)
    return true;
  const bool equal =
      SUCCEEDED(device->QueryInterface(IID_PPV_ARGS(&a))) && SUCCEEDED(expected->QueryInterface(IID_PPV_ARGS(&b))) && a && a == b;
  if (a)
    a->Release();
  if (b)
    b->Release();
  return equal;
}
/*
 * Registers relevant same-device textures with their description and incarnation, then enrolls eligible
 * camera sources. Destruction tracking invalidates the identity instead of trusting a recycled address.
 */
void observe_resource(ID3D12Device* device, IUnknown* object, source_state::Model initial) {
  auto& r = registry();
  if (!r.ready || !object || !same_device(device))
    return;
  ID3D12Resource* native{};
  if (FAILED(object->QueryInterface(IID_PPV_ARGS(&native))))
    return;
  const auto desc = native->GetDesc();
  if (relevant(desc)) {
    std::shared_ptr<Resource> item;
    {
      const std::lock_guard lock(r.mutex);
      auto found = r.resources.find(native);
      if (found != r.resources.end() && found->second->alive) {
        native->Release();
        return;
      }
      if (r.resources.size() < 16384 && r.next_id != UINT64_MAX) {
        item = std::make_shared<Resource>();
        item->native = native;
        item->desc = desc;
        item->normal_source =
            camera_sized(static_cast<UINT>(desc.Width), desc.Height) && desc.MipLevels == 1 && desc.DepthOrArraySize == 1 &&
            desc.SampleDesc.Count == 1 && !desc.SampleDesc.Quality && (desc.Flags & D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET) != 0 &&
            (desc.Flags & (D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL | D3D12_RESOURCE_FLAG_ALLOW_SIMULTANEOUS_ACCESS)) == 0 &&
            (desc.Format == DXGI_FORMAT_R11G11B10_FLOAT || desc.Format == DXGI_FORMAT_R8G8B8A8_UNORM ||
             desc.Format == DXGI_FORMAT_R8G8B8A8_TYPELESS || desc.Format == DXGI_FORMAT_R16G16B16A16_FLOAT);
        item->id = ++r.next_id;
        r.resources[native] = item;
      }
    }
    if (item && scene_handoff().register_resource(reinterpret_cast<std::uint64_t>(native), item->id)) {
      if (item->normal_source)
        runtime::manager().register_source_candidate(native, item->id, desc, initial);
      if (!attach(native, item))
        error("resource_lifetime_notification_failed");
    } else if (item) {
      item->retire();
      error("resource_registry_full");
    }
  }
  native->Release();
}
std::shared_ptr<Resource> resource(ID3D12Resource* p) {
  auto& r = registry();
  const std::lock_guard lock(r.mutex);
  const auto it = r.resources.find(p);
  return it != r.resources.end() && it->second->alive ? it->second : nullptr;
}
/*
 * A bounded weak cache avoids keeping metadata, descriptor heaps or resources alive after registry cleanup.
 * The retained object identity and recording must still match; retirement and address reuse never borrow
 * another list's proof. Recording calls on one D3D12 list remain caller-serialized as before.
 */
struct KnownListCache {
  struct Entry {
    ID3D12GraphicsCommandList* native{};
    std::uint64_t id{}, recording{};
    std::weak_ptr<List> item;
  };
  std::array<Entry, 8> entries{};
  unsigned next{};
  std::shared_ptr<List> find(ID3D12GraphicsCommandList* native) noexcept {
    for (auto& entry : entries)
      if (entry.native == native) {
        auto item = entry.item.lock();
        if (item && item->alive.load(std::memory_order_acquire) && item->native == native && item->id == entry.id &&
            item->recording.load(std::memory_order_acquire) == entry.recording)
          return item;
        entry = {};
      }
    return {};
  }
  void remember(const std::shared_ptr<List>& item) noexcept {
    if (item)
      entries[next++ % entries.size()] = {item->native, item->id, item->recording.load(std::memory_order_acquire), item};
  }
};
thread_local KnownListCache known_lists;
}  /* namespace */

void Resource::retire() noexcept {
  alive.store(false, std::memory_order_release);
  runtime::manager().unregister_source_candidate(native, id);
  scene_handoff().unregister_resource(reinterpret_cast<std::uint64_t>(native));
}
void List::retire() noexcept {
  alive.store(false, std::memory_order_release);
  runtime::manager().retire_list(*this);
}
bool graphics_admitted() noexcept {
  return registry().ready.load(std::memory_order_acquire);
}
bool observation_enabled() noexcept {
  return (registry().observation_epoch.load(std::memory_order_acquire) & 1u) != 0;
}
bool recording_observed(const List& list) noexcept {
  const auto epoch = registry().observation_epoch.load(std::memory_order_acquire);
  return (epoch & 1u) && list.observation_epoch.load(std::memory_order_acquire) == epoch;
}
/*
 * Checks the recording-local cache first; it only remembers live lists and revalidates identity and
 * recording, so retirement and address reuse never borrow another list's record.
 */
std::shared_ptr<List> find_list(ID3D12GraphicsCommandList* p) {
  if (auto item = known_lists.find(p))
    return item;
  auto& r = registry();
  const std::lock_guard lock(r.mutex);
  const auto it = r.lists.find(p);
  auto item = it != r.lists.end() && it->second->alive ? it->second : nullptr;
  known_lists.remember(item);
  return item;
}
/*
 * Reads the same identity registry without waiting. The caller receives a retained record only after its
 * native pointer, liveness and current recording identity have passed the normal cache/registry checks.
 */
bool try_find_list(ID3D12GraphicsCommandList* p, std::shared_ptr<List>& result) noexcept {
  result.reset();
  if (!p)
    return true;
  if (auto item = known_lists.find(p)) {
    result = std::move(item);
    return true;
  }
  try {
    auto& r = registry();
    const std::unique_lock lock(r.mutex, std::try_to_lock);
    if (!lock.owns_lock())
      return false;
    const auto it = r.lists.find(p);
    result = it != r.lists.end() && it->second->alive.load(std::memory_order_acquire) ? it->second : nullptr;
    known_lists.remember(result);
    return true;
  } catch (...) {
    result.reset();
    return false;
  }
}
/*
 * Registers a real native direct command list, hooks its boundaries, admits it to capture and records
 * whether creation/Reset was fully observed. A list discovered mid-recording cannot inherit a clean initial
 * state: its scope is invalid until an observed Reset.
 */
std::shared_ptr<List> ensure_list(ID3D12GraphicsCommandList* native, bool observed) {
  if (auto existing = find_list(native))
    return existing;
  auto& r = registry();
  const std::lock_guard observation_lock(r.observation_mutex);
  if (!r.ready || native->GetType() != D3D12_COMMAND_LIST_TYPE_DIRECT || !same_native_device(native, r.device))
    return {};
  std::shared_ptr<List> item;
  {
    const std::lock_guard lock(r.mutex);
    if (r.lists.size() >= 4096 || r.next_id == UINT64_MAX) {
      error("command_list_capacity");
      return {};
    }
    item = std::make_shared<List>();
    item->native = native;
    item->id = ++r.next_id;
    item->ready = observed;
    item->observation_epoch = observed ? r.observation_epoch.load(std::memory_order_acquire) : 0;
    r.lists[native] = item;
  }
  const bool hooked = list_hooks::install(native);
  const bool admitted = runtime::manager().admit_list(*item, observed);
  if (!hooked || !admitted || !attach(native, item)) {
    item->retire();
    error("native_list_registration_failed");
    return {};
  }
  if (observed)
    list_hooks::begin_recording(*item);
  else
    list_hooks::invalidate_recording(*item, list_hooks::InvalidationResetFailed);
  return item;
}
std::shared_ptr<List> discover_list(ID3D12GraphicsCommandList* native) noexcept {
  const OwnedWork guard;
  std::shared_ptr<List> item;
  observe_safely([&] { item = ensure_list(native); });
  return item;
}
std::vector<std::shared_ptr<List>> live_lists() noexcept {
  std::vector<std::shared_ptr<List>> result;
  try {
    auto& r = registry();
    const std::lock_guard lock(r.mutex);
    result.reserve(r.lists.size());
    for (const auto& entry : r.lists)
      if (entry.second->alive)
        result.push_back(entry.second);
  } catch (...) {
    result.clear();
  }
  return result;
}
/*
 * Maps the bound render-target descriptors to tracked resources before the call is forwarded, while the
 * caller's CPU handles are still valid. Draws that follow are attributed to these targets.
 */
void bind_render_targets(List& l, UINT count, const D3D12_CPU_DESCRIPTOR_HANDLE* handles, BOOL contiguous) {
  auto& r = registry();
  const std::lock_guard lock(r.mutex);
  l.targets = {};
  l.count = 0;
  if (count > 8 || (count && !handles) || (contiguous && count && handles[0].ptr > SIZE_MAX - SIZE_T{count - 1} * r.rtv_stride))
    return;
  l.count = count;
  for (UINT i = 0; i < count; ++i) {
    const auto handle = contiguous ? handles[0].ptr + SIZE_T{i} * r.rtv_stride : handles[i].ptr;
    if (!handle)
      return;
    const auto it = r.rtvs.find(handle);
    if (it != r.rtvs.end())
      l.targets[i] = it->second;
  }
}
/*
 * Admits a submitting direct queue only when it belongs to the retained native device, then hooks its
 * ordered submissions.
 */
void discover_queue(ID3D12CommandQueue* queue) noexcept {
  if (owned_depth || !registry().ready || queue->GetDesc().Type != D3D12_COMMAND_LIST_TYPE_DIRECT)
    return;
  const OwnedWork guard;
  if (!same_native_device(queue, registry().device)) {
    error("queue_device_mismatch");
    return;
  }
  if (!runtime::init_queue(queue))
    error("queue_registration_failed");
}

namespace {
std::array<HookSlot, 10> creations;
constexpr std::array<unsigned, 10> CreationSlots{27, 29, 30, 53, 55, 69, 70, 76, 77, 78};
constexpr std::array<unsigned, 10> StateArgs{3, 3, 1, 3, 1, 3, 3, 3, 3, 1};
source_state::Model model(D3D12_RESOURCE_STATES s) {
  return s == D3D12_RESOURCE_STATE_RENDER_TARGET ? source_state::Model::legacy_rt : source_state::Model::unknown;
}
source_state::Model model(D3D12_BARRIER_LAYOUT s) {
  return s == D3D12_BARRIER_LAYOUT_RENDER_TARGET ? source_state::Model::enhanced_rt : source_state::Model::unknown;
}
template <unsigned I, class Signature>
struct Creation;
template <unsigned I, class C, class... Args>
struct Creation<I, HRESULT (STDMETHODCALLTYPE C::*)(Args...)> {
  using F = HRESULT(STDMETHODCALLTYPE*)(C*, Args...);
  /*
   * Forwards the native resource-creation call and observes its returned object only on success, containing
   * bookkeeping failures outside the original API result.
   */
  static HRESULT STDMETHODCALLTYPE call(C* self, Args... args) noexcept {
    const bool observe = owned_depth == 0;
    const OwnedWork guard;
    const auto hr = creations[I].forward<F>()(self, args...);
    if (observe && SUCCEEDED(hr)) {
      const auto tuple = std::forward_as_tuple(args...);
      auto** out = std::get<sizeof...(Args) - 1>(tuple);
      if (out && *out)
        observe_safely([&] {
          observe_resource(reinterpret_cast<ID3D12Device*>(self), static_cast<IUnknown*>(*out), model(std::get<StateArgs[I]>(tuple)));
        });
    }
    return hr;
  }
};
const std::array<void*, 10> CreationWrappers{
    reinterpret_cast<void*>(&Creation<0, decltype(&ID3D12Device::CreateCommittedResource)>::call),
    reinterpret_cast<void*>(&Creation<1, decltype(&ID3D12Device::CreatePlacedResource)>::call),
    reinterpret_cast<void*>(&Creation<2, decltype(&ID3D12Device::CreateReservedResource)>::call),
    reinterpret_cast<void*>(&Creation<3, decltype(&ID3D12Device4::CreateCommittedResource1)>::call),
    reinterpret_cast<void*>(&Creation<4, decltype(&ID3D12Device4::CreateReservedResource1)>::call),
    reinterpret_cast<void*>(&Creation<5, decltype(&ID3D12Device8::CreateCommittedResource2)>::call),
    reinterpret_cast<void*>(&Creation<6, decltype(&ID3D12Device8::CreatePlacedResource1)>::call),
    reinterpret_cast<void*>(&Creation<7, decltype(&ID3D12Device10::CreateCommittedResource3)>::call),
    reinterpret_cast<void*>(&Creation<8, decltype(&ID3D12Device10::CreatePlacedResource2)>::call),
    reinterpret_cast<void*>(&Creation<9, decltype(&ID3D12Device10::CreateReservedResource2)>::call)};

HookSlot rtv_creation, descriptor_copy, descriptor_copy_simple, create_list, create_list1;
/*
 * Forwards render-target-view creation and records the descriptor-to-resource relationship for the retained
 * native device.
 */
void STDMETHODCALLTYPE rtv_create(ID3D12Device* device,
                                  ID3D12Resource* native,
                                  const D3D12_RENDER_TARGET_VIEW_DESC* desc,
                                  D3D12_CPU_DESCRIPTOR_HANDLE handle) noexcept {
  using F = void(STDMETHODCALLTYPE*)(ID3D12Device*, ID3D12Resource*, const D3D12_RENDER_TARGET_VIEW_DESC*, D3D12_CPU_DESCRIPTOR_HANDLE);
  rtv_creation.forward<F>()(device, native, desc, handle);
  if (owned_depth || !registry().ready || !same_device(device))
    return;
  const OwnedWork guard;
  observe_safely([&] {
    if (native && !resource(native))
      observe_resource(device, native, source_state::Model::unknown);
    auto item = resource(native);
    auto& r = registry();
    View view;
    if (item && (!desc || desc->ViewDimension == D3D12_RTV_DIMENSION_TEXTURE2D)) {
      view = {item, desc ? desc->Texture2D.MipSlice : 0};
    }
    const std::lock_guard lock(r.mutex);
    replace_view(r, handle.ptr, view);
  });
}
/*
 * Mirrors a verified render-target descriptor-range copy in the registry. Unknown source descriptors clear
 * destination knowledge rather than inheriting stale bindings.
 */
void copy_descriptors(UINT count, D3D12_CPU_DESCRIPTOR_HANDLE dest, D3D12_CPU_DESCRIPTOR_HANDLE src) {
  auto& r = registry();
  const std::lock_guard lock(r.mutex);
  if (count > 65536) {
    r.rtvs.clear();
    error("descriptor_copy_capacity");
    return;
  }
  for (UINT i = 0; i < count; ++i) {
    const auto found = r.rtvs.find(src.ptr);
    replace_view(r, dest.ptr, found != r.rtvs.end() ? found->second : View{});
    src.ptr += r.rtv_stride;
    dest.ptr += r.rtv_stride;
  }
}
void STDMETHODCALLTYPE descriptors_simple(ID3D12Device* device,
                                          UINT count,
                                          D3D12_CPU_DESCRIPTOR_HANDLE dest,
                                          D3D12_CPU_DESCRIPTOR_HANDLE src,
                                          D3D12_DESCRIPTOR_HEAP_TYPE type) noexcept {
  using F =
      void(STDMETHODCALLTYPE*)(ID3D12Device*, UINT, D3D12_CPU_DESCRIPTOR_HANDLE, D3D12_CPU_DESCRIPTOR_HANDLE, D3D12_DESCRIPTOR_HEAP_TYPE);
  descriptor_copy_simple.forward<F>()(device, count, dest, src, type);
  if (!owned_depth && registry().ready && type == D3D12_DESCRIPTOR_HEAP_TYPE_RTV && same_device(device))
    observe_safely([&] { copy_descriptors(count, dest, src); });
}
/*
 * Forwards descriptor copies and walks the source/destination ranges with their actual lengths, updating
 * only the corresponding tracked metadata.
 */
void STDMETHODCALLTYPE descriptors(ID3D12Device* device,
                                   UINT nd,
                                   const D3D12_CPU_DESCRIPTOR_HANDLE* dest,
                                   const UINT* ds,
                                   UINT ns,
                                   const D3D12_CPU_DESCRIPTOR_HANDLE* src,
                                   const UINT* ss,
                                   D3D12_DESCRIPTOR_HEAP_TYPE type) noexcept {
  using F = void(STDMETHODCALLTYPE*)(ID3D12Device*, UINT, const D3D12_CPU_DESCRIPTOR_HANDLE*, const UINT*, UINT,
                                     const D3D12_CPU_DESCRIPTOR_HANDLE*, const UINT*, D3D12_DESCRIPTOR_HEAP_TYPE);
  descriptor_copy.forward<F>()(device, nd, dest, ds, ns, src, ss, type);
  if (owned_depth || !registry().ready || type != D3D12_DESCRIPTOR_HEAP_TYPE_RTV || !same_device(device))
    return;
  observe_safely([&] {
    auto& r = registry();
    if (nd > 4096 || ns > 4096 || !dest || !src) {
      const std::lock_guard lock(r.mutex);
      r.rtvs.clear();
      error("descriptor_ranges_refused");
      return;
    }
    const UINT stride = r.rtv_stride;
    UINT di = 0, si = 0, dp = 0, sp = 0, total = 0;
    while (di < nd && si < ns) {
      const UINT dcount = ds ? ds[di] : 1, scount = ss ? ss[si] : 1;
      if (dp == dcount) {
        ++di;
        dp = 0;
        continue;
      }
      if (sp == scount) {
        ++si;
        sp = 0;
        continue;
      }
      const UINT n = std::min(dcount - dp, scount - sp);
      if (n > 65536 - total) {
        const std::lock_guard lock(r.mutex);
        r.rtvs.clear();
        error("descriptor_ranges_overflow");
        return;
      }
      copy_descriptors(n, {dest[di].ptr + SIZE_T{dp} * stride}, {src[si].ptr + SIZE_T{sp} * stride});
      dp += n;
      sp += n;
      total += n;
    }
  });
}
/*
 * Forwards command-list creation and registers its native interface only after successful construction.
 */
HRESULT STDMETHODCALLTYPE list_create(ID3D12Device* device,
                                      UINT node,
                                      D3D12_COMMAND_LIST_TYPE type,
                                      ID3D12CommandAllocator* allocator,
                                      ID3D12PipelineState* pso,
                                      REFIID iid,
                                      void** out) noexcept {
  using F = HRESULT(STDMETHODCALLTYPE*)(ID3D12Device*, UINT, D3D12_COMMAND_LIST_TYPE, ID3D12CommandAllocator*, ID3D12PipelineState*, REFIID,
                                        void**);
  const bool observe = !owned_depth;
  const OwnedWork guard;
  const auto hr = create_list.forward<F>()(device, node, type, allocator, pso, iid, out);
  if (observe && SUCCEEDED(hr) && out && *out && type == D3D12_COMMAND_LIST_TYPE_DIRECT) {
    ID3D12GraphicsCommandList* native{};
    if (SUCCEEDED(static_cast<IUnknown*>(*out)->QueryInterface(IID_PPV_ARGS(&native)))) {
      observe_safely([&] { ensure_list(native, true); });
      native->Release();
    }
  }
  return hr;
}
/*
 * Registers a successfully created closed list without claiming an observed recording. A later native Reset
 * must establish recording state.
 *
 * CreateCommandList1 creates a closed list; actual Reset admits it.
 */
HRESULT STDMETHODCALLTYPE list_create1(ID3D12Device4* device,
                                       UINT node,
                                       D3D12_COMMAND_LIST_TYPE type,
                                       D3D12_COMMAND_LIST_FLAGS flags,
                                       REFIID iid,
                                       void** out) noexcept {
  using F = HRESULT(STDMETHODCALLTYPE*)(ID3D12Device4*, UINT, D3D12_COMMAND_LIST_TYPE, D3D12_COMMAND_LIST_FLAGS, REFIID, void**);
  const bool observe = !owned_depth;
  const OwnedWork guard;
  const auto hr = create_list1.forward<F>()(device, node, type, flags, iid, out);
  if (observe && SUCCEEDED(hr) && out && *out && type == D3D12_COMMAND_LIST_TYPE_DIRECT) {
    ID3D12GraphicsCommandList* native{};
    if (SUCCEEDED(static_cast<IUnknown*>(*out)->QueryInterface(IID_PPV_ARGS(&native)))) {
      observe_safely([&] { ensure_list(native); });
      native->Release();
    }
  }
  return hr;
}
HookSlot list_reset, list_close;
/*
 * Close ends the observed recording: later work on this list is attributed only after a successful Reset.
 */
HRESULT STDMETHODCALLTYPE close(ID3D12GraphicsCommandList* native) noexcept {
  using F = HRESULT(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*);
  if (!owned_depth && registry().ready) {
    const OwnedWork guard;
    observe_safely([&] {
      if (auto item = find_list(native))
        item->ready = false;
    });
  }
  return list_close.forward<F>()(native);
}
/*
 * Forwards Reset and establishes a new recording generation only after success, using one consistent demand
 * epoch. Failure invalidates state rather than reviving old bindings.
 *
 * Demand cannot change between qualifying this native Reset and publishing its recording proof. Source state
 * remains continuously observed separately.
 */
HRESULT STDMETHODCALLTYPE reset(ID3D12GraphicsCommandList* native, ID3D12CommandAllocator* allocator, ID3D12PipelineState* pso) noexcept {
  using F = HRESULT(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, ID3D12CommandAllocator*, ID3D12PipelineState*);
  if (owned_depth)
    return list_reset.forward<F>()(native, allocator, pso);
  const OwnedWork guard;
  const auto observation_epoch = registry().observation_epoch.load(std::memory_order_acquire);
  std::shared_ptr<List> item;
  observe_safely([&] { item = ensure_list(native); });
  const auto hr = list_reset.forward<F>()(native, allocator, pso);
  if (item) {
    const std::lock_guard observation_lock(registry().observation_mutex);
    item->observation_epoch = observation_epoch == registry().observation_epoch.load(std::memory_order_acquire) ? observation_epoch : 0;
    item->targets = {};
    item->count = 0;
    item->ready = hr == S_OK && item->recording < UINT64_MAX;
    if (item->ready) {
      ++item->recording;
      list_hooks::begin_recording(*item);
      runtime::manager().successful_reset(*item);
    } else {
      list_hooks::invalidate_recording(*item, list_hooks::InvalidationResetFailed);
    }
  }
  return hr;
}
struct ClearState {
  /*
   * Mirrors native ClearState by unbinding targets. This is still the same recording: boundary/capture
   * invalidations, render-pass scope and readiness are kept; only an actual Reset renews them.
   */
  static void apply(List& l, ID3D12PipelineState*) {
    l.targets = {};
    l.count = 0;
  }
};
struct Targets {
  static void before(List& l, UINT count, const D3D12_CPU_DESCRIPTOR_HANDLE* handles, BOOL contiguous, const D3D12_CPU_DESCRIPTOR_HANDLE*) {
    bind_render_targets(l, count, handles, contiguous);
  }
  static void apply(List&, UINT, const D3D12_CPU_DESCRIPTOR_HANDLE*, BOOL, const D3D12_CPU_DESCRIPTOR_HANDLE*) {}
};
struct Unsupported {
  template <class... Args>
  static void apply(List& l, Args...) {
    list_hooks::invalidate_recording(l);
  }
};
struct Predication {
  /*
   * NULL disables conditional execution; D3D11On12 emits it before an ordinary capture copy. It neither
   * executes work nor changes resource states. Never revive a recording invalidated by an earlier real
   * predicate.
   */
  static void apply(List& list, ID3D12Resource* buffer, UINT64, D3D12_PREDICATION_OP) {
    if (buffer)
      Unsupported::apply(list);
  }
};
/*
 * Every signature is derived from the pinned Windows COM declaration. These hooks observe, forward exactly
 * once, and never alter application inputs.
 */
template <unsigned Slot, class Signature, class Action>
struct StateHook;
template <unsigned Slot, class C, class... Args, class Action>
struct StateHook<Slot, void (STDMETHODCALLTYPE C::*)(Args...), Action> {
  using F = void(STDMETHODCALLTYPE*)(C*, Args...);
  static inline HookSlot slot;
  /*
   * The native active-pass table has distinct cells even where the target functions are identical. Retain a
   * second, pinned cell for that exact table; Each table forwards its own immutable original, including if
   * another hook changes a cell between the admission read and the installation CAS.
   */
  static inline HookSlot active_slot;
  /*
   * Wraps native state setters with one reentry guard across forwarding and observation.
   *
   * A runtime implementation can re-enter a patched setter while forwarding the caller's operation. Keep one
   * guard across forwarding and observation, just as Reset does, so only the outer call updates the tracked
   * state.
   */
  static void invoke(F forward, C* native, Args... args) noexcept {
    if (owned_depth || !registry().ready) {
      forward(native, args...);
      return;
    }
    const OwnedWork guard;
    if constexpr (requires(List& l) { Action::before(l, args...); }) {
      observe_safely([&] {
        if (auto item = find_list(reinterpret_cast<ID3D12GraphicsCommandList*>(native)))
          Action::before(*item, args...);
      });
    }
    forward(native, args...);
    if (!registry().ready)
      return;
    observe_safely([&] {
      if (auto item = ensure_list(reinterpret_cast<ID3D12GraphicsCommandList*>(native)))
        Action::apply(*item, args...);
    });
  }
  /*
   * Wrappers for the base table and the active-pass table; each forwards to its own table's original.
   */
  static void STDMETHODCALLTYPE call(C* native, Args... args) noexcept { invoke(slot.forward<F>(), native, args...); }
  static void STDMETHODCALLTYPE active_call(C* native, Args... args) noexcept { invoke(active_slot.forward<F>(), native, args...); }
  /*
   * Installs the wrapper in the base table, or in the active-pass table of the bootstrap list. The active
   * cell must forward to exactly the same original as the base cell; a changed hook chain refuses graphics
   * admission while installed wrappers keep their captured originals.
   */
  static bool install(ID3D12GraphicsCommandList* list, bool active = false) {
    if (!active)
      return slot.install(list, Slot, reinterpret_cast<void*>(&call));
    auto** address = *reinterpret_cast<void***>(list) + Slot;
    if (address == slot.address())
      return *address == reinterpret_cast<void*>(&call);
    if (!active_slot.installed() && (!slot.installed() || *address != slot.forward<void*>()))
      return false;
    return active_slot.install(list, Slot, reinterpret_cast<void*>(&active_call)) && active_slot.forward<void*>() == slot.forward<void*>();
  }
};
#define STATE(Slot, Method, Action) StateHook<Slot, decltype(&ID3D12GraphicsCommandList::Method), Action>
/*
 * Installs Reset/Close and the state hooks that attribute draws to render targets or revoke a recording
 * whose work cannot be observed. The runtime uses a distinct vtable while a render pass is active; the same
 * setters are hooked there too.
 */
bool hook_state(ID3D12GraphicsCommandList* list, bool active = false) {
  bool ok = true;
  if (!active) {
    ok = list_reset.install(list, 10, reinterpret_cast<void*>(&reset));
    ok &= list_close.install(list, 9, reinterpret_cast<void*>(&close));
  }
  ok &= STATE(11, ClearState, ClearState)::install(list, active);
  ok &= STATE(46, OMSetRenderTargets, Targets)::install(list, active);
  ok &= STATE(27, ExecuteBundle, Unsupported)::install(list, active);
  ok &= STATE(59, ExecuteIndirect, Unsupported)::install(list, active);
  ok &= STATE(55, SetPredication, Predication)::install(list, active);
  ok &= STATE(51, DiscardResource, Unsupported)::install(list, active);
  return ok;
}
#undef STATE
bool same_interface(ID3D12Device* device, REFIID iid) {
  IUnknown* p{};
  const bool ok = SUCCEEDED(device->QueryInterface(iid, reinterpret_cast<void**>(&p))) && p == device;
  if (p)
    p->Release();
  return ok;
}
}  /* namespace */

/*
 * Establishes the retained native device and installs resource, recording and queue observations using
 * private bootstrap objects. Actual active-pass interfaces and original endpoints are verified before
 * application recordings are admitted.
 *
 * Hook below cooperating proxies: translated descriptors, unwrapped queue submissions, and Close after
 * downstream add-on callbacks.
 *
 * Bootstrap objects, modules and the device remain pinned. No runtime unload.
 *
 * Discover the runtime's real active-pass table on this owned, empty list, before admitting application
 * recordings. No simulator resource is bound. The state setters and End are hooked in both tables; native
 * Begin/End and all pass/suspend/alias capture guards belong to command_list_hooks.cpp. A different
 * forwarding implementation is refused, never guessed.
 */
bool initialize_graphics(IUnknown* reported) noexcept {
  const OwnedWork guard;
  ID3D12Device* device{};
  if (!resolve_native_device(reported, &device)) {
    error("native_device_resolution_failed");
    return false;
  }
  const auto release = [](ID3D12Device* p) { p->Release(); };
  const std::unique_ptr<ID3D12Device, decltype(release)> reference(device, release);
  auto& r = registry();
  if (r.device)
    return r.device == device && r.ready;
  if (!device || !same_interface(device, __uuidof(ID3D12Device10))) {
    error("native_device10_required");
    return false;
  }
  r.device = device;
  device->AddRef();
  if (!scene_handoff().register_device() || !runtime::init_device(device)) {
    error("native_device_registration_failed");
    return false;
  }
  r.rtv_stride = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
  ID3D12CommandAllocator* allocator{};
  ID3D12GraphicsCommandList* list{};
  ID3D12CommandQueue* queue{};
  D3D12_COMMAND_QUEUE_DESC q{};
  q.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
  if (FAILED(device->CreateCommandAllocator(q.Type, IID_PPV_ARGS(&allocator))) ||
      FAILED(device->CreateCommandList(0, q.Type, allocator, nullptr, IID_PPV_ARGS(&list))) ||
      FAILED(device->CreateCommandQueue(&q, IID_PPV_ARGS(&queue)))) {
    error("bootstrap_objects_failed");
    return false;
  }
  bool ok = hook_state(list);
  const bool base = list_hooks::install(list);
  ok &= base;
  ID3D12GraphicsCommandList4* pass_list{};
  const bool same_pass_interface = SUCCEEDED(list->QueryInterface(IID_PPV_ARGS(&pass_list))) && pass_list == list;
  if (base && same_pass_interface) {
    pass_list->BeginRenderPass(0, nullptr, nullptr, D3D12_RENDER_PASS_FLAG_NONE);
    list_hooks::track_active_pass(pass_list);
    ok &= hook_state(list, true);
    pass_list->EndRenderPass();
  } else {
    ok = false;
  }
  if (pass_list)
    pass_list->Release();
  list->Close();
  ok &= runtime::init_queue(queue);
  for (unsigned i = 0; i < creations.size(); ++i)
    ok &= creations[i].install(device, CreationSlots[i], CreationWrappers[i]);
  ok &= rtv_creation.install(device, 20, reinterpret_cast<void*>(&rtv_create));
  ok &= descriptor_copy.install(device, 23, reinterpret_cast<void*>(&descriptors));
  ok &= descriptor_copy_simple.install(device, 24, reinterpret_cast<void*>(&descriptors_simple));
  ok &= create_list.install(device, 12, reinterpret_cast<void*>(&list_create));
  ok &= create_list1.install(device, 51, reinterpret_cast<void*>(&list_create1));
  r.ready = ok;
  r.error = ok ? "native_graphics_ready" : "native_hook_installation_failed";
  if (!ok)
    ++r.failures;
  return ok;
}
/*
 * Establishes the retained native device and installs resource, recording and queue observations using
 * private bootstrap objects. Actual active-pass interfaces and original endpoints are verified before
 * application recordings are admitted.
 */
bool initialize_graphics() noexcept {
  const OwnedWork guard;
  ID3D12Device* reported{};
  if (FAILED(D3D12CreateDevice(nullptr, D3D_FEATURE_LEVEL_12_0, IID_PPV_ARGS(&reported)))) {
    error("d3d12_device_unavailable");
    return false;
  }
  const bool ok = initialize_graphics(reported);
  reported->Release();
  return ok;
}
bool graphics_ready() noexcept {
  return registry().ready.load(std::memory_order_acquire);
}
/*
 * Enables or suppresses new private capture work without losing continuous source-state or pending GPU
 * lifetime obligations. A recording that started while idle counts as unobserved until its next Reset.
 *
 * Suppress new capture insertion without discarding the continuous layout model or previously recorded
 * replay/consumer obligations.
 */
void set_graphics_observation_demand(bool enabled) noexcept {
  auto& r = registry();
  if (observation_enabled() == enabled)
    return;
  const std::lock_guard lock(r.observation_mutex);
  const auto previous = r.observation_epoch.load(std::memory_order_acquire);
  if (!previous || bool(previous & 1u) == enabled)
    return;
  runtime::manager().set_capture_enabled(enabled);
  r.observation_epoch.store(previous == UINT64_MAX ? 0 : previous + 1, std::memory_order_release);
}
GraphicsStatus graphics_status() noexcept {
  auto& r = registry();
  const std::lock_guard lock(r.mutex);
  return {r.ready, r.failures, r.error};
}
/*
 * Drops registry entries whose native objects have been destroyed. Retirement already invalidated their
 * identities; this only bounds the maps. Called periodically by the DLL control loop.
 */
void collect_retired_objects() noexcept {
  observe_safely([&] {
    auto& r = registry();
    const std::lock_guard lock(r.mutex);
    std::erase_if(r.resources, [](const auto& p) { return !p.second->alive; });
    std::erase_if(r.lists, [](const auto& p) { return !p.second->alive; });
    std::erase_if(r.rtvs, [](const auto& p) { return !p.second.resource || !p.second.resource->alive; });
  });
}
}  /* namespace azp::standalone */
