/*
 * Installs guarded hooks at verified engine callback slots. It preserves original functions and memory
 * protection, uses atomic slot replacement, and exposes callbacks at the phases required for camera, scenery
 * and night-vision updates.
 */

#include "observer_hook.hpp"
#include <intrin.h>
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <limits>

extern "C" {
alignas(8) std::uintptr_t azp_engine_hook_original = 0;
void azp_engine_hook_thunk();
alignas(8) std::uintptr_t azp_camera_pose_hook_original = 0;
void azp_camera_pose_hook_thunk();
}

namespace {
using namespace azp::engine_hook;

SRWLOCK control_lock = SRWLOCK_INIT;
std::atomic<Observer> saved_observer{nullptr};
void** saved_slot = nullptr;
void** pending_protection_slot = nullptr;
DWORD pending_protection = 0;
bool consumed = false;
bool removed = false;
ImageDataSlotProof saved_image_data{};
bool has_image_data = false;
bool installed_copy_on_write = false;
thread_local bool observing = false;

std::atomic<Observer> service_observer{nullptr};
std::atomic<void*> service_original{nullptr};
void** service_slot = nullptr;
bool service_consumed = false;
bool service_removed = false;
ImageDataSlotProof service_image_data{};
bool service_has_image_data = false;
bool service_copy_on_write = false;
thread_local bool service_observing = false;

std::atomic<void*> terrain_original{nullptr};
std::atomic<TerrainFilter> terrain_filter{nullptr};
void** terrain_slot = nullptr;
bool terrain_consumed = false, terrain_removed = false, terrain_copy_on_write = false;

std::atomic<void*> terrain_update_original{nullptr};
std::atomic<TerrainUpdateObserver> terrain_update_observer{nullptr};
void** terrain_update_slot = nullptr;
bool terrain_update_consumed = false, terrain_update_removed = false, terrain_update_copy_on_write = false;
thread_local bool terrain_updating = false;

std::atomic<void*> tile_submit_original{nullptr};
std::atomic<TileSubmitObserver> tile_submit_observer{nullptr};
void** tile_submit_slot = nullptr;
bool tile_submit_consumed = false, tile_submit_removed = false, tile_submit_copy_on_write = false;

std::atomic<CameraUpdateObserver> camera_observer{nullptr};
void** camera_slot = nullptr;
bool camera_consumed = false, camera_removed = false, camera_copy_on_write = false;
thread_local bool camera_updating = false;

std::atomic<ViewParametersObserver> view_parameters_observer{nullptr};
std::atomic<void*> view_parameters_original{nullptr};
void** view_parameters_slot = nullptr;
bool view_parameters_consumed = false, view_parameters_removed = false, view_parameters_copy_on_write = false;
thread_local bool view_parameters_updating = false;

/*
 * Calls the original view-parameter callback first, then invokes AZP with the live callback context. This
 * ordering lets the optional override follow native cycle updates.
 *
 * The verified flight-world method consumes RCX/RDX only, and its caller uses no return value. Let the
 * engine finish applying its scene cycles before our observer changes the AZP parameters. The original runs
 * for every world/view, including when the optional trial is off or our observer refuses the context.
 */
__declspec(noinline) void view_parameters_thunk(void* world, void* context) {
  const auto return_address = reinterpret_cast<std::uintptr_t>(_ReturnAddress());
  const auto original = reinterpret_cast<void (*)(void*, void*)>(view_parameters_original.load(std::memory_order_acquire));
  original(world, context);
  const auto observer = view_parameters_observer.load(std::memory_order_acquire);
  if (observer && !view_parameters_updating) {
    view_parameters_updating = true;
    observer(world, context, return_address);
    view_parameters_updating = false;
  }
}

thread_local bool tile_submitting = false;
/*
 * Forwards tile submission, then invokes the registered observer while that callback's native arguments
 * remain live.
 */
void tile_submit_thunk(void* self, void* node, float priority, void* traversal, unsigned char coarse, unsigned char mode) {
  const auto original = reinterpret_cast<TileSubmit>(tile_submit_original.load(std::memory_order_acquire));
  original(self, node, priority, traversal, coarse, mode);
  const auto observer = tile_submit_observer.load(std::memory_order_acquire);
  if (observer && !tile_submitting) {
    tile_submitting = true;
    observer(self, node, coarse);
    tile_submitting = false;
  }
}

/*
 * Forwards the native terrain update and then captures its current view through the registered observer.
 */
void terrain_update_thunk(void* self, const void* update) {
  const auto original = reinterpret_cast<void (*)(void*, const void*)>(terrain_update_original.load(std::memory_order_acquire));
  original(self, update);
  const auto observer = terrain_update_observer.load(std::memory_order_acquire);
  if (observer && !terrain_updating) {
    terrain_updating = true;
    observer(self, update);
    terrain_updating = false;
  }
}

unsigned char terrain_classify_thunk(void* self, const void* node, const void* context, float* priority) {
  const auto original = reinterpret_cast<TerrainClassify>(terrain_original.load(std::memory_order_acquire));
  const auto result = original(self, node, context, priority);
  const auto filter = terrain_filter.load(std::memory_order_acquire);
  return filter ? filter(self, node, context, priority, result, original) : result;
}

/*
 * Invokes the registered preload observer at the service-update boundary and forwards the original call in
 * the reviewed order.
 */
void service_update_thunk(void* self) {
  const auto original = reinterpret_cast<void (*)(void*)>(service_original.load(std::memory_order_acquire));
  const auto observer = service_observer.load(std::memory_order_acquire);
  if (observer != nullptr && !service_observing) {
    service_observing = true;
    observer(self);
    service_observing = false;
  }
  original(self);
}

struct Lock {
  Lock() noexcept { AcquireSRWLockExclusive(&control_lock); }
  ~Lock() { ReleaseSRWLockExclusive(&control_lock); }
};

/*
 * Accepts only readable committed-page protection forms and rejects guard/no-access flags before examining a
 * hook location.
 */
bool readable_protection(DWORD protection) noexcept {
  if ((protection & (PAGE_GUARD | PAGE_NOACCESS)) != 0)
    return false;
  switch (protection & 0xff) {
    case PAGE_READONLY:
    case PAGE_READWRITE:
    case PAGE_WRITECOPY:
    case PAGE_EXECUTE_READ:
    case PAGE_EXECUTE_READWRITE:
    case PAGE_EXECUTE_WRITECOPY:
      return true;
    default:
      return false;
  }
}

/*
 * Reads a bounded main-image header field only after verifying its current memory region.
 *
 * Header-only read: no raw dereference, no protection changes, and no section content reads. All metadata
 * must fit within the first 64 KiB of this image.
 */
bool image_header(HMODULE image, std::uint32_t rva, void* output, std::size_t size) noexcept {
  constexpr std::uint32_t header_limit = 65536;
  const auto base = reinterpret_cast<std::uintptr_t>(image);
  if (rva > header_limit || size > header_limit - rva || base > std::numeric_limits<std::uintptr_t>::max() - rva)
    return false;
  const auto address = base + rva;
  MEMORY_BASIC_INFORMATION region{};
  if (VirtualQuery(reinterpret_cast<void*>(address), &region, sizeof(region)) != sizeof(region) || region.State != MEM_COMMIT ||
      region.Type != MEM_IMAGE || region.AllocationBase != image || !readable_protection(region.Protect))
    return false;
  const auto first = reinterpret_cast<std::uintptr_t>(region.BaseAddress);
  if (region.RegionSize > std::numeric_limits<std::uintptr_t>::max() - first || address < first || address > first + region.RegionSize ||
      size > first + region.RegionSize - address)
    return false;
  SIZE_T copied = 0;
  return ReadProcessMemory(GetCurrentProcess(), reinterpret_cast<void*>(address), output, size, &copied) != FALSE && copied == size;
}

/*
 * Proves that a hook slot belongs to the expected main-image section by checking live PE metadata and
 * section bounds: a read-only data section, or with `code` a read-only executable section.
 */
bool verified_image_data(void** slot, const MEMORY_BASIC_INFORMATION& region, const ImageDataSlotProof* proof, bool code = false) noexcept {
  if (proof == nullptr || proof->main_image == nullptr || proof->main_image != GetModuleHandleW(nullptr) || region.Type != MEM_IMAGE ||
      region.AllocationBase != proof->main_image || proof->image_size == 0 || proof->image_size > 0x80000000u || proof->section_size == 0 ||
      proof->section_rva > proof->image_size || proof->section_size > proof->image_size - proof->section_rva)
    return false;
  const auto base = reinterpret_cast<std::uintptr_t>(proof->main_image);
  const auto address = reinterpret_cast<std::uintptr_t>(slot);
  if (base > std::numeric_limits<std::uintptr_t>::max() - proof->image_size || address < base || address - base < proof->section_rva ||
      address - base - proof->section_rva > proof->section_size ||
      sizeof(void*) > proof->section_size - (address - base - proof->section_rva))
    return false;
  IMAGE_DOS_HEADER dos{};
  if (!image_header(proof->main_image, 0, &dos, sizeof(dos)) || dos.e_magic != IMAGE_DOS_SIGNATURE || dos.e_lfanew < 64 ||
      dos.e_lfanew > 65536 - 24)
    return false;
  struct NtPrefix {
    DWORD signature;
    IMAGE_FILE_HEADER file;
  } nt{};
  static_assert(sizeof(nt) == 24);
  const auto nt_rva = static_cast<std::uint32_t>(dos.e_lfanew);
  if (!image_header(proof->main_image, nt_rva, &nt, sizeof(nt)) || nt.signature != IMAGE_NT_SIGNATURE ||
      nt.file.Machine != IMAGE_FILE_MACHINE_AMD64 || nt.file.NumberOfSections == 0 || nt.file.NumberOfSections > 96 ||
      nt.file.SizeOfOptionalHeader < offsetof(IMAGE_OPTIONAL_HEADER64, DataDirectory) || nt.file.SizeOfOptionalHeader > 4096)
    return false;
  IMAGE_OPTIONAL_HEADER64 optional{};
  if (!image_header(proof->main_image, nt_rva + sizeof(nt), &optional, offsetof(IMAGE_OPTIONAL_HEADER64, DataDirectory)) ||
      optional.Magic != IMAGE_NT_OPTIONAL_HDR64_MAGIC || optional.SizeOfImage != proof->image_size || optional.SizeOfHeaders > 65536 ||
      optional.SizeOfHeaders > optional.SizeOfImage)
    return false;
  const auto sections_rva = nt_rva + static_cast<std::uint32_t>(sizeof(nt)) + nt.file.SizeOfOptionalHeader;
  const auto sections_size = nt.file.NumberOfSections * static_cast<std::uint32_t>(sizeof(IMAGE_SECTION_HEADER));
  if (sections_rva > optional.SizeOfHeaders || sections_size > optional.SizeOfHeaders - sections_rva)
    return false;
  IMAGE_SECTION_HEADER sections[96]{};
  if (!image_header(proof->main_image, sections_rva, sections, sections_size))
    return false;
  bool matched = false;
  for (unsigned i = 0; i < nt.file.NumberOfSections; ++i) {
    const auto& section = sections[i];
    const auto extent = section.Misc.VirtualSize != 0 ? section.Misc.VirtualSize : section.SizeOfRawData;
    if (extent == 0)
      continue;
    if (section.VirtualAddress < optional.SizeOfHeaders || section.VirtualAddress > optional.SizeOfImage ||
        extent > optional.SizeOfImage - section.VirtualAddress)
      return false;
    for (unsigned j = 0; j < i; ++j) {
      const auto previous_extent = sections[j].Misc.VirtualSize != 0 ? sections[j].Misc.VirtualSize : sections[j].SizeOfRawData;
      if (previous_extent != 0 && section.VirtualAddress < sections[j].VirtualAddress + previous_extent &&
          sections[j].VirtualAddress < section.VirtualAddress + extent)
        return false;
    }
    if (section.VirtualAddress == proof->section_rva && extent == proof->section_size) {
      const DWORD required = IMAGE_SCN_MEM_READ | (code ? IMAGE_SCN_MEM_EXECUTE : 0);
      const DWORD refused = IMAGE_SCN_MEM_WRITE | IMAGE_SCN_MEM_DISCARDABLE | (code ? 0 : IMAGE_SCN_MEM_EXECUTE);
      if ((section.Characteristics & required) != required || (section.Characteristics & refused) != 0)
        return false;
      matched = true;
    }
  }
  return matched;
}

/*
 * Validates slot range, alignment and protection before patching. The narrowly reviewed
 * executable-copy-on-write main-image exception does not apply to arbitrary executable memory.
 *
 * Executable pages remain refused by default. The one explicit exception is a main-image read-only data
 * section already mapped executable copy-on-write.
 *
 * Preserve executable permission.
 */
bool slot_memory(void** slot, const ImageDataSlotProof* proof, DWORD& writable_protection, bool owned_copy_on_write = false) noexcept {
  MEMORY_BASIC_INFORMATION region{};
  if (VirtualQuery(slot, &region, sizeof(region)) != sizeof(region) || region.State != MEM_COMMIT ||
      (region.Protect & (PAGE_GUARD | PAGE_NOACCESS)) != 0)
    return false;
  const auto protection = region.Protect & 0xff;
  if (protection == PAGE_EXECUTE_WRITECOPY || (owned_copy_on_write && protection == PAGE_EXECUTE_READWRITE)) {
    if (!verified_image_data(slot, region, proof))
      return false;
    writable_protection = region.Protect;
  } else if (protection == PAGE_READONLY || protection == PAGE_READWRITE || protection == PAGE_WRITECOPY) {
    writable_protection = PAGE_READWRITE;
  } else {
    return false;
  }
  const auto first = reinterpret_cast<std::uintptr_t>(region.BaseAddress);
  const auto address = reinterpret_cast<std::uintptr_t>(slot);
  return region.RegionSize <= std::numeric_limits<std::uintptr_t>::max() - first && address >= first &&
         address <= first + region.RegionSize && sizeof(void*) <= first + region.RegionSize - address;
}

bool code_pointer(const void* pointer) noexcept {
  MEMORY_BASIC_INFORMATION region{};
  if (pointer == nullptr || VirtualQuery(pointer, &region, sizeof(region)) != sizeof(region) || region.State != MEM_COMMIT ||
      (region.Protect & (PAGE_GUARD | PAGE_NOACCESS)) != 0)
    return false;
  const auto protection = region.Protect & 0xff;
  return protection == PAGE_EXECUTE || protection == PAGE_EXECUTE_READ || protection == PAGE_EXECUTE_READWRITE ||
         protection == PAGE_EXECUTE_WRITECOPY;
}

void* slot_value(void** slot) noexcept {
  return std::atomic_ref<void*>(*slot).load(std::memory_order_acquire);
}

BOOL change_protection(void* address, SIZE_T size, DWORD protection, PDWORD prior) noexcept {
  return VirtualProtect(address, size, protection, prior);
}

DWORD preserve_cfg_targets(DWORD protection) noexcept {
  const auto access = protection & 0xff;
  return access == PAGE_EXECUTE_WRITECOPY || access == PAGE_EXECUTE_READWRITE ? protection | PAGE_TARGETS_NO_UPDATE : protection;
}

/*
 * Repairs the saved page-protection obligation before another installation/removal attempt. The original
 * protection remains authoritative until restoration succeeds.
 */
bool restore_pending_protection(DWORD& error) noexcept {
  if (pending_protection_slot == nullptr)
    return true;
  DWORD discarded = 0;
  if (!change_protection(pending_protection_slot, sizeof(void*), preserve_cfg_targets(pending_protection), &discarded)) {
    error = GetLastError();
    return false;
  }
  pending_protection_slot = nullptr;
  pending_protection = 0;
  return true;
}

/*
 * Recognizes a page that this hook owns after Windows copy-on-write promotion. This permits repair of the
 * same slot without authorizing unrelated private pages.
 *
 * A previous verified image write can promote the whole page, including the other hook's slot. Only that
 * exact page gains admission; slot_memory still checks the new slot's own image/section proof before any
 * write.
 */
bool owned_copy_on_write_page(void** slot) noexcept {
  SYSTEM_INFO info{};
  GetSystemInfo(&info);
  if (info.dwPageSize == 0)
    return false;
  const auto page = reinterpret_cast<std::uintptr_t>(slot) / info.dwPageSize;
  return (consumed && installed_copy_on_write && page == reinterpret_cast<std::uintptr_t>(saved_slot) / info.dwPageSize) ||
         (service_consumed && service_copy_on_write && page == reinterpret_cast<std::uintptr_t>(service_slot) / info.dwPageSize) ||
         (terrain_consumed && terrain_copy_on_write && page == reinterpret_cast<std::uintptr_t>(terrain_slot) / info.dwPageSize) ||
         (terrain_update_consumed && terrain_update_copy_on_write &&
          page == reinterpret_cast<std::uintptr_t>(terrain_update_slot) / info.dwPageSize) ||
         (tile_submit_consumed && tile_submit_copy_on_write &&
          page == reinterpret_cast<std::uintptr_t>(tile_submit_slot) / info.dwPageSize) ||
         (camera_consumed && camera_copy_on_write && page == reinterpret_cast<std::uintptr_t>(camera_slot) / info.dwPageSize) ||
         (view_parameters_consumed && view_parameters_copy_on_write &&
          page == reinterpret_cast<std::uintptr_t>(view_parameters_slot) / info.dwPageSize);
}

/*
 * Publishes the replacement with atomic compare/exchange under temporary write permission. Original
 * protection and forwarding state survive both compare failures and restoration failures.
 *
 * Retain the original protection even if restoration fails after a CAS mismatch. No later exchange may
 * replace this recovery obligation.
 *
 * The thunk may run even if restoration fails.
 */
Result exchange_slot(void** slot,
                     void* expected,
                     void* replacement,
                     DWORD writable_protection,
                     Status applied,
                     Status mismatch,
                     bool& hook_consumed,
                     bool& hook_removed,
                     bool& hook_copy_on_write) noexcept {
  DWORD prior = 0;
  if (!change_protection(slot, sizeof(void*), preserve_cfg_targets(writable_protection), &prior))
    return {Status::protection_change_failed, GetLastError()};
  pending_protection_slot = slot;
  pending_protection = prior;
  const auto observed = InterlockedCompareExchangePointer(reinterpret_cast<void* volatile*>(slot), replacement, expected);
  const bool changed = observed == expected;
  if (changed && applied == Status::installed) {
    const bool inherited_copy = owned_copy_on_write_page(slot);
    hook_consumed = true;
    hook_copy_on_write = (writable_protection & 0xff) == PAGE_EXECUTE_WRITECOPY || inherited_copy;
  }
  if (changed && applied == Status::removed)
    hook_removed = true;
  DWORD error = ERROR_SUCCESS;
  if (!restore_pending_protection(error))
    return {Status::protection_restore_failed, error, changed, false};
  return {changed ? applied : mismatch, ERROR_SUCCESS, changed};
}

bool call_redirect_consumed = false;

/*
 * Allocates a 14-byte `jmp [rip+0]` stub to `destination` in free memory within 2 GiB of `origin`, so a
 * rel32 call there can reach it. Searches below `origin`, then above it, one allocation granule at a time,
 * skipping occupied regions. The stub is executable and read-only once written, and is never freed.
 */
void* near_stub(std::uintptr_t origin, const void* destination) noexcept {
  SYSTEM_INFO info{};
  GetSystemInfo(&info);
  const std::uintptr_t granule = info.dwAllocationGranularity ? info.dwAllocationGranularity : 0x10000;
  constexpr std::uintptr_t reach = 0x7ff00000;
  const auto low = origin > reach + granule ? origin - reach : granule;
  const auto high = origin < std::numeric_limits<std::uintptr_t>::max() - reach ? origin + reach : origin;
  for (const bool downward : {true, false}) {
    auto address = origin / granule * granule;
    for (unsigned step = 0; step < 65536; ++step) {
      address = downward ? address - granule : address + granule;
      if (address < low || address > high)
        break;
      MEMORY_BASIC_INFORMATION region{};
      if (VirtualQuery(reinterpret_cast<void*>(address), &region, sizeof(region)) != sizeof(region))
        break;
      const auto begin = reinterpret_cast<std::uintptr_t>(region.BaseAddress);
      if (region.State != MEM_FREE) {
        address = downward ? std::min(address, reinterpret_cast<std::uintptr_t>(region.AllocationBase)) / granule * granule
                           : (begin + region.RegionSize + granule - 1) / granule * granule - granule;
        continue;
      }
      auto* stub = static_cast<unsigned char*>(VirtualAlloc(reinterpret_cast<void*>(address), 16, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE));
      if (!stub)
        continue;
      const unsigned char jump[6] = {0xff, 0x25, 0, 0, 0, 0};
      std::memcpy(stub, jump, sizeof(jump));
      std::memcpy(stub + sizeof(jump), &destination, sizeof(destination));
      DWORD prior = 0;
      if (!VirtualProtect(stub, 16, PAGE_EXECUTE_READ, &prior) || !FlushInstructionCache(GetCurrentProcess(), stub, 16)) {
        VirtualFree(stub, 0, MEM_RELEASE);
        return nullptr;
      }
      return stub;
    }
  }
  return nullptr;
}
}  /* namespace */

extern "C" void azp_engine_hook_observe(void* original_rcx) noexcept {
  const auto observer = saved_observer.load(std::memory_order_acquire);
  if (observer == nullptr || observing)
    return;
  observing = true;
  observer(original_rcx);
  observing = false;
}

extern "C" void azp_camera_pose_hook_observe(void* camera, void* node) noexcept {
  const auto observer = camera_observer.load(std::memory_order_acquire);
  if (!observer || camera_updating)
    return;
  camera_updating = true;
  observer(camera, node);
  camera_updating = false;
}

namespace azp::engine_hook {

/*
 * Installs the view-parameter callback observer only after checking the exact slot, original code and page
 * ownership. Original forwarding state is retained before atomic replacement, including on partial
 * installation failure.
 */
Result install_view_parameters(void** slot,
                               void* expected_original,
                               ViewParametersObserver observer,
                               const ImageDataSlotProof* image_data) noexcept {
  const Lock lock;
  DWORD error = ERROR_SUCCESS;
  if (!restore_pending_protection(error))
    return {Status::protection_restore_failed, error, false, false};
  if (view_parameters_consumed)
    return {Status::installation_consumed};
  if (!slot || reinterpret_cast<std::uintptr_t>(slot) % alignof(void*) || !expected_original || !observer ||
      expected_original == reinterpret_cast<void*>(&view_parameters_thunk))
    return {Status::invalid_argument};
  DWORD writable_protection = 0;
  if (!slot_memory(slot, image_data, writable_protection, owned_copy_on_write_page(slot)))
    return {Status::invalid_slot_memory};
  if (!code_pointer(expected_original) || !code_pointer(reinterpret_cast<const void*>(observer)))
    return {Status::invalid_code_pointer};
  if (slot_value(slot) != expected_original)
    return {Status::original_mismatch};
  view_parameters_original.store(expected_original, std::memory_order_release);
  view_parameters_observer.store(observer, std::memory_order_release);
  view_parameters_slot = slot;
  return exchange_slot(slot, expected_original, reinterpret_cast<void*>(&view_parameters_thunk), writable_protection, Status::installed,
                       Status::original_mismatch, view_parameters_consumed, view_parameters_removed, view_parameters_copy_on_write);
}

/*
 * Checks the manager-update slot, preserves its original endpoint and installs the register-preserving
 * thunk. Forwarding state is published before any engine thread can enter the replacement.
 *
 * Published before the interlocked slot exchange, and immutable from the first successful exchange until the
 * caller establishes unload quiescence.
 */
Result install(void** slot, void* expected_original, Observer observer, const ImageDataSlotProof* image_data) noexcept {
  const Lock lock;
  DWORD error = ERROR_SUCCESS;
  if (!restore_pending_protection(error))
    return {Status::protection_restore_failed, error, false, false};
  if (consumed)
    return {Status::installation_consumed};
  if (slot == nullptr || reinterpret_cast<std::uintptr_t>(slot) % alignof(void*) != 0 || expected_original == nullptr ||
      expected_original == reinterpret_cast<void*>(&azp_engine_hook_thunk) ||
      expected_original == reinterpret_cast<void*>(&service_update_thunk))
    return {Status::invalid_argument};
  DWORD writable_protection = 0;
  if (!slot_memory(slot, image_data, writable_protection, owned_copy_on_write_page(slot)))
    return {Status::invalid_slot_memory};
  if (!code_pointer(expected_original) || (observer != nullptr && !code_pointer(reinterpret_cast<const void*>(observer))))
    return {Status::invalid_code_pointer};
  if (slot_value(slot) != expected_original)
    return {Status::original_mismatch};

  azp_engine_hook_original = reinterpret_cast<std::uintptr_t>(expected_original);
  saved_observer.store(observer, std::memory_order_release);
  saved_slot = slot;
  saved_image_data = image_data != nullptr ? *image_data : ImageDataSlotProof{};
  has_image_data = image_data != nullptr;
  return exchange_slot(slot, expected_original, reinterpret_cast<void*>(&azp_engine_hook_thunk), writable_protection, Status::installed,
                       Status::original_mismatch, consumed, removed, installed_copy_on_write);
}

/*
 * Restores the original slot only if it still contains this hook and page-protection repair succeeds.
 */
Result remove() noexcept {
  const Lock lock;
  DWORD error = ERROR_SUCCESS;
  if (!restore_pending_protection(error))
    return {Status::protection_restore_failed, error, false, false};
  if (!consumed || removed)
    return {Status::not_installed};
  DWORD writable_protection = 0;
  if (!slot_memory(saved_slot, has_image_data ? &saved_image_data : nullptr, writable_protection, installed_copy_on_write))
    return {Status::invalid_slot_memory};
  if (slot_value(saved_slot) != reinterpret_cast<void*>(&azp_engine_hook_thunk))
    return {Status::slot_changed};
  return exchange_slot(saved_slot, reinterpret_cast<void*>(&azp_engine_hook_thunk), reinterpret_cast<void*>(azp_engine_hook_original),
                       writable_protection, Status::removed, Status::slot_changed, consumed, removed, installed_copy_on_write);
}

/*
 * Installs the streaming service callback observer only after checking the exact slot, original code and
 * page ownership. Original forwarding state is retained before atomic replacement, including on partial
 * installation failure.
 */
Result install_service_update(void** slot, void* expected_original, Observer observer, const ImageDataSlotProof* image_data) noexcept {
  const Lock lock;
  DWORD error = ERROR_SUCCESS;
  if (!restore_pending_protection(error))
    return {Status::protection_restore_failed, error, false, false};
  if (service_consumed)
    return {Status::installation_consumed};
  if (slot == nullptr || reinterpret_cast<std::uintptr_t>(slot) % alignof(void*) != 0 || expected_original == nullptr ||
      expected_original == reinterpret_cast<void*>(&azp_engine_hook_thunk) ||
      expected_original == reinterpret_cast<void*>(&service_update_thunk))
    return {Status::invalid_argument};
  DWORD writable_protection = 0;
  if (!slot_memory(slot, image_data, writable_protection, owned_copy_on_write_page(slot)))
    return {Status::invalid_slot_memory};
  if (!code_pointer(expected_original) || (observer != nullptr && !code_pointer(reinterpret_cast<const void*>(observer))))
    return {Status::invalid_code_pointer};
  if (slot_value(slot) != expected_original)
    return {Status::original_mismatch};

  service_original.store(expected_original, std::memory_order_release);
  service_observer.store(observer, std::memory_order_release);
  service_slot = slot;
  service_image_data = image_data != nullptr ? *image_data : ImageDataSlotProof{};
  service_has_image_data = image_data != nullptr;
  return exchange_slot(slot, expected_original, reinterpret_cast<void*>(&service_update_thunk), writable_protection, Status::installed,
                       Status::original_mismatch, service_consumed, service_removed, service_copy_on_write);
}

/*
 * Disables and restores the owned service-update hook without overwriting a slot changed by another owner.
 */
Result remove_service_update() noexcept {
  const Lock lock;
  DWORD error = ERROR_SUCCESS;
  if (!restore_pending_protection(error))
    return {Status::protection_restore_failed, error, false, false};
  if (!service_consumed || service_removed)
    return {Status::not_installed};
  DWORD writable_protection = 0;
  if (!slot_memory(service_slot, service_has_image_data ? &service_image_data : nullptr, writable_protection, service_copy_on_write))
    return {Status::invalid_slot_memory};
  if (slot_value(service_slot) != reinterpret_cast<void*>(&service_update_thunk))
    return {Status::slot_changed};
  return exchange_slot(service_slot, reinterpret_cast<void*>(&service_update_thunk), service_original.load(std::memory_order_acquire),
                       writable_protection, Status::removed, Status::slot_changed, service_consumed, service_removed,
                       service_copy_on_write);
}

Result restore_protection() noexcept {
  const Lock lock;
  DWORD error = ERROR_SUCCESS;
  if (!restore_pending_protection(error))
    return {Status::protection_restore_failed, error, false, false};
  return {Status::protection_restored};
}

/*
 * Installs the terrain classifier observer only after checking the exact slot, original code and page
 * ownership. Original forwarding state is retained before atomic replacement, including on partial
 * installation failure.
 */
Result install_terrain_filter(void** slot, void* expected_original, TerrainFilter filter, const ImageDataSlotProof* image_data) noexcept {
  const Lock lock;
  DWORD error = ERROR_SUCCESS;
  if (!restore_pending_protection(error))
    return {Status::protection_restore_failed, error, false, false};
  if (terrain_consumed)
    return {Status::installation_consumed};
  if (!slot || reinterpret_cast<std::uintptr_t>(slot) % alignof(void*) || !expected_original || !filter ||
      expected_original == reinterpret_cast<void*>(&terrain_classify_thunk))
    return {Status::invalid_argument};
  DWORD writable_protection = 0;
  if (!slot_memory(slot, image_data, writable_protection, owned_copy_on_write_page(slot)))
    return {Status::invalid_slot_memory};
  if (!code_pointer(expected_original) || !code_pointer(reinterpret_cast<const void*>(filter)))
    return {Status::invalid_code_pointer};
  if (slot_value(slot) != expected_original)
    return {Status::original_mismatch};
  terrain_original.store(expected_original, std::memory_order_release);
  terrain_filter.store(filter, std::memory_order_release);
  terrain_slot = slot;
  return exchange_slot(slot, expected_original, reinterpret_cast<void*>(&terrain_classify_thunk), writable_protection, Status::installed,
                       Status::original_mismatch, terrain_consumed, terrain_removed, terrain_copy_on_write);
}

/*
 * Installs the terrain-view update observer only after checking the exact slot, original code and page
 * ownership. Original forwarding state is retained before atomic replacement, including on partial
 * installation failure.
 */
Result install_terrain_update(void** slot,
                              void* expected_original,
                              TerrainUpdateObserver observer,
                              const ImageDataSlotProof* image_data) noexcept {
  const Lock lock;
  DWORD error = ERROR_SUCCESS;
  if (!restore_pending_protection(error))
    return {Status::protection_restore_failed, error, false, false};
  if (terrain_update_consumed)
    return {Status::installation_consumed};
  if (!slot || reinterpret_cast<std::uintptr_t>(slot) % alignof(void*) || !expected_original || !observer ||
      expected_original == reinterpret_cast<void*>(&terrain_update_thunk))
    return {Status::invalid_argument};
  DWORD writable_protection = 0;
  if (!slot_memory(slot, image_data, writable_protection, owned_copy_on_write_page(slot)))
    return {Status::invalid_slot_memory};
  if (!code_pointer(expected_original) || !code_pointer(reinterpret_cast<const void*>(observer)))
    return {Status::invalid_code_pointer};
  if (slot_value(slot) != expected_original)
    return {Status::original_mismatch};
  terrain_update_original.store(expected_original, std::memory_order_release);
  terrain_update_observer.store(observer, std::memory_order_release);
  terrain_update_slot = slot;
  return exchange_slot(slot, expected_original, reinterpret_cast<void*>(&terrain_update_thunk), writable_protection, Status::installed,
                       Status::original_mismatch, terrain_update_consumed, terrain_update_removed, terrain_update_copy_on_write);
}

/*
 * Installs the vegetation tile submission observer only after checking the exact slot, original code and
 * page ownership. Original forwarding state is retained before atomic replacement, including on partial
 * installation failure.
 */
Result install_tile_submit(void** slot,
                           void* expected_original,
                           TileSubmitObserver observer,
                           const ImageDataSlotProof* image_data) noexcept {
  const Lock lock;
  DWORD error = ERROR_SUCCESS;
  if (!restore_pending_protection(error))
    return {Status::protection_restore_failed, error, false, false};
  if (tile_submit_consumed)
    return {Status::installation_consumed};
  if (!slot || reinterpret_cast<std::uintptr_t>(slot) % alignof(void*) || !expected_original || !observer ||
      expected_original == reinterpret_cast<void*>(&tile_submit_thunk))
    return {Status::invalid_argument};
  DWORD writable_protection = 0;
  if (!slot_memory(slot, image_data, writable_protection, owned_copy_on_write_page(slot)))
    return {Status::invalid_slot_memory};
  if (!code_pointer(expected_original) || !code_pointer(reinterpret_cast<const void*>(observer)))
    return {Status::invalid_code_pointer};
  if (slot_value(slot) != expected_original)
    return {Status::original_mismatch};
  tile_submit_original.store(expected_original, std::memory_order_release);
  tile_submit_observer.store(observer, std::memory_order_release);
  tile_submit_slot = slot;
  return exchange_slot(slot, expected_original, reinterpret_cast<void*>(&tile_submit_thunk), writable_protection, Status::installed,
                       Status::original_mismatch, tile_submit_consumed, tile_submit_removed, tile_submit_copy_on_write);
}

/*
 * Installs the camera scene-node calculation observer only after checking the exact slot, original code and
 * page ownership. Original forwarding state is retained before atomic replacement, including on partial
 * installation failure.
 */
Result install_camera_update(void** slot,
                             void* expected_original,
                             CameraUpdateObserver observer,
                             const ImageDataSlotProof* image_data) noexcept {
  const Lock lock;
  DWORD error = ERROR_SUCCESS;
  if (!restore_pending_protection(error))
    return {Status::protection_restore_failed, error, false, false};
  if (camera_consumed)
    return {Status::installation_consumed};
  if (!slot || reinterpret_cast<std::uintptr_t>(slot) % alignof(void*) || !expected_original || !observer ||
      expected_original == reinterpret_cast<void*>(&azp_camera_pose_hook_thunk))
    return {Status::invalid_argument};
  DWORD writable_protection = 0;
  if (!slot_memory(slot, image_data, writable_protection, owned_copy_on_write_page(slot)))
    return {Status::invalid_slot_memory};
  if (!code_pointer(expected_original) || !code_pointer(reinterpret_cast<const void*>(observer)))
    return {Status::invalid_code_pointer};
  if (slot_value(slot) != expected_original)
    return {Status::original_mismatch};
  azp_camera_pose_hook_original = reinterpret_cast<std::uintptr_t>(expected_original);
  camera_observer.store(observer, std::memory_order_release);
  camera_slot = slot;
  return exchange_slot(slot, expected_original, reinterpret_cast<void*>(&azp_camera_pose_hook_thunk), writable_protection,
                       Status::installed, Status::original_mismatch, camera_consumed, camera_removed, camera_copy_on_write);
}

/*
 * Checks the call against the proof and its current target, publishes the stub to the replacement, then
 * exchanges only the call's aligned 32-bit displacement. The page is made writable for that one exchange,
 * without changing its control-flow-guard targets, and its prior protection is put back.
 */
Result install_call_redirect(void* call, void* expected_target, void* replacement, const ImageDataSlotProof* code) noexcept {
  const Lock lock;
  DWORD error = ERROR_SUCCESS;
  if (!restore_pending_protection(error))
    return {Status::protection_restore_failed, error, false, false};
  if (call_redirect_consumed)
    return {Status::installation_consumed};
  auto* bytes = static_cast<unsigned char*>(call);
  auto* field = reinterpret_cast<volatile LONG*>(bytes + 1);
  if (!call || reinterpret_cast<std::uintptr_t>(field) % sizeof(LONG) || !expected_target || !replacement)
    return {Status::invalid_argument};
  MEMORY_BASIC_INFORMATION region{};
  if (VirtualQuery(call, &region, sizeof(region)) != sizeof(region) || region.State != MEM_COMMIT || !code_pointer(call) ||
      !verified_image_data(reinterpret_cast<void**>(bytes + 1), region, code, true) ||
      reinterpret_cast<std::uintptr_t>(bytes + 5) > reinterpret_cast<std::uintptr_t>(region.BaseAddress) + region.RegionSize)
    return {Status::invalid_slot_memory};
  if (!code_pointer(expected_target) || !code_pointer(replacement))
    return {Status::invalid_code_pointer};
  const LONG original = *field;
  const auto next = reinterpret_cast<std::intptr_t>(bytes + 5);
  if (bytes[0] != 0xe8 || next + original != reinterpret_cast<std::intptr_t>(expected_target))
    return {Status::original_mismatch};
  const auto* stub = near_stub(reinterpret_cast<std::uintptr_t>(call), replacement);
  const auto distance = stub ? reinterpret_cast<std::intptr_t>(stub) - next : 0;
  if (!stub || distance < std::numeric_limits<LONG>::min() || distance > std::numeric_limits<LONG>::max())
    return {Status::invalid_code_pointer};
  DWORD prior = 0;
  if (!change_protection(const_cast<LONG*>(field), sizeof(LONG), PAGE_EXECUTE_READWRITE | PAGE_TARGETS_NO_UPDATE, &prior))
    return {Status::protection_change_failed, GetLastError()};
  const bool changed = InterlockedCompareExchange(field, static_cast<LONG>(distance), original) == original;
  call_redirect_consumed = changed;
  DWORD discarded = 0;
  const bool restored = change_protection(const_cast<LONG*>(field), sizeof(LONG), prior | PAGE_TARGETS_NO_UPDATE, &discarded) != FALSE;
  const DWORD restore_error = restored ? ERROR_SUCCESS : GetLastError();
  FlushInstructionCache(GetCurrentProcess(), call, 5);
  if (!restored)
    return {Status::protection_restore_failed, restore_error, changed, false};
  return {changed ? Status::installed : Status::original_mismatch, ERROR_SUCCESS, changed};
}

/*
 * Converts hook result codes to stable operational text.
 */
const char* status_name(Status status) noexcept {
  switch (status) {
    case Status::installed:
      return "installed";
    case Status::removed:
      return "removed";
    case Status::invalid_argument:
      return "invalid_argument";
    case Status::invalid_slot_memory:
      return "invalid_slot_memory";
    case Status::invalid_code_pointer:
      return "invalid_code_pointer";
    case Status::original_mismatch:
      return "original_mismatch";
    case Status::installation_consumed:
      return "installation_consumed";
    case Status::not_installed:
      return "not_installed";
    case Status::slot_changed:
      return "slot_changed";
    case Status::protection_change_failed:
      return "protection_change_failed";
    case Status::protection_restore_failed:
      return "protection_restore_failed";
    case Status::protection_restored:
      return "protection_restored";
  }
  return "unknown";
}
}  /* namespace azp::engine_hook */
