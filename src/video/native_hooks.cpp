/*
 * Implements HookSlot: validates the vtable cell, the original function and the wrapper, pins their modules
 * and exchanges the cell atomically under temporary write access.
 */

#include "native_hooks.hpp"
#include <cstdint>

namespace azp::standalone {
namespace {
/*
 * Reads one aligned pointer through ReadProcessMemory, so an invalid address fails instead of faulting.
 */
bool read_pointer(const void* address, void*& value) noexcept {
  const auto location = reinterpret_cast<std::uintptr_t>(address);
  SIZE_T bytes = 0;
  return location && location % alignof(void*) == 0 && ReadProcessMemory(GetCurrentProcess(), address, &value, sizeof(value), &bytes) &&
         bytes == sizeof(value);
}
/*
 * Pins the loaded image containing `p`. A vtable cell must be a readable, non-executable image page; a
 * function must be executable image code.
 */
bool pin(const void* p, bool executable) noexcept {
  MEMORY_BASIC_INFORMATION m{};
  if (!p || !VirtualQuery(p, &m, sizeof(m)) || m.State != MEM_COMMIT || m.Type != MEM_IMAGE || (m.Protect & (PAGE_GUARD | PAGE_NOACCESS)))
    return false;
  const DWORD access = m.Protect & 255;
  const bool expected = executable ? access == PAGE_EXECUTE_READ || access == PAGE_EXECUTE_READWRITE || access == PAGE_EXECUTE_WRITECOPY
                                   : access == PAGE_READONLY || access == PAGE_READWRITE || access == PAGE_WRITECOPY;
  HMODULE module{};
  return expected &&
         GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN, reinterpret_cast<LPCWSTR>(p), &module);
}
}  /* namespace */

bool HookSlot::repair_protection() noexcept {
  if (!pending_protection_)
    return true;
  DWORD discarded{};
  if (!VirtualProtect(pending_protection_, sizeof(void*), pending_access_, &discarded))
    return false;
  pending_protection_ = nullptr;
  return true;
}

bool HookSlot::intact() const noexcept {
  return address_ && *reinterpret_cast<void* volatile*>(address_) == wrapper_;
}

/*
 * The original is stored before the exchange, and the address is kept even when protection restoration
 * fails, so the wrapper stays forwardable and the protection can be repaired later.
 */
bool HookSlot::install(void* object, unsigned index, void* wrapper) noexcept {
  if (!object || !wrapper || !repair_protection())
    return false;
  void* table = nullptr;
  if (!read_pointer(object, table) || !table ||
      reinterpret_cast<std::uintptr_t>(table) > UINTPTR_MAX - (std::uintptr_t{index} + 1) * sizeof(void*))
    return false;
  auto** cell = static_cast<void**>(table) + index;
  if (address_)
    return cell == address_ && wrapper == wrapper_ && intact();
  void* prior = nullptr;
  if (!read_pointer(cell, prior) || !prior || prior == wrapper || !pin(cell, false) || !pin(prior, true) || !pin(wrapper, true))
    return false;
  original_.store(prior, std::memory_order_release);
  wrapper_ = wrapper;
  DWORD old{};
  if (!VirtualProtect(cell, sizeof(void*), PAGE_READWRITE, &old))
    return false;
  void* observed = InterlockedCompareExchangePointer(reinterpret_cast<void* volatile*>(cell), wrapper, prior);
  DWORD discarded{};
  if (!VirtualProtect(cell, sizeof(void*), old, &discarded)) {
    pending_protection_ = cell;
    pending_access_ = old;
  }
  const bool restored = repair_protection();
  if (observed == prior)
    address_ = cell;
  return observed == prior && restored;
}
}  /* namespace azp::standalone */
