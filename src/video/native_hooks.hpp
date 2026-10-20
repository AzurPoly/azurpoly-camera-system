/*
 * The one way this DLL patches a native D3D12 COM method: HookSlot replaces a single vtable cell with a
 * wrapper that observes the call and forwards to the original. The device, command-list and queue hooks all
 * use it. OwnedWork marks D3D12 calls made by AZP itself so the hooks do not observe them as simulator work.
 */

#pragma once
#include <windows.h>
#include <atomic>

namespace azp::standalone {
inline thread_local unsigned owned_depth = 0;
struct OwnedWork {
  OwnedWork() noexcept { ++owned_depth; }
  ~OwnedWork() { --owned_depth; }
};

/*
 * One patched cell of a native COM vtable. The cell, the original function and the wrapper must all lie in
 * loaded images, which are pinned so none of them can be unloaded; no executable memory is allocated and no
 * table is cloned. The original is published before the compare-exchange and kept for the life of the
 * process, so an installed wrapper can always forward, even after a partial failure. Write access to the
 * cell is granted only for the exchange; a failed protection restore is retried before the next install.
 * Callers serialize install() for one slot.
 */
class HookSlot {
 public:
  /*
   * Installs `wrapper` in cell `index` of `object`'s vtable. Once installed, the slot belongs to that cell:
   * a later call succeeds only for an object with the same table whose cell still holds the wrapper. A cell
   * changed by another module is refused, never overwritten.
   */
  bool install(void* object, unsigned index, void* wrapper) noexcept;
  /*
   * True while the installed cell still holds the wrapper.
   */
  bool intact() const noexcept;
  bool installed() const noexcept { return address_ != nullptr; }
  void** address() const noexcept { return address_; }
  template <class F>
  F forward() const noexcept {
    return reinterpret_cast<F>(original_.load(std::memory_order_acquire));
  }

 private:
  bool repair_protection() noexcept;
  std::atomic<void*> original_{};
  void* wrapper_{};
  void** address_{};
  void** pending_protection_{};
  DWORD pending_access_{};
};
}  /* namespace azp::standalone */
