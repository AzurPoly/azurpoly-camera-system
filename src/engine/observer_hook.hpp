/*
 * Defines engine-hook arguments, callbacks and installation results. Each hook has a reviewed calling
 * convention and forwarding order; an installed hook does not remove the need to validate the current
 * receiver before mutation.
 */

#pragma once
#include <windows.h>
#include <cstdint>

namespace azp::engine_hook {

using Observer = void (*)(void* original_rcx) noexcept;

enum class Status {
  installed,
  removed,
  invalid_argument,
  invalid_slot_memory,
  invalid_code_pointer,
  original_mismatch,
  installation_consumed,
  not_installed,
  slot_changed,
  protection_change_failed,
  protection_restore_failed,
  protection_restored,
};

struct Result {
  Status status;
  DWORD windows_error = ERROR_SUCCESS;
  /*
   * True only when this operation's compare/exchange changed the slot.
   */
  bool pointer_changed = false;
  /*
   * True when protection was unchanged or VirtualProtect accepted restoration of the prior protection.
   * Windows may report PAGE_EXECUTE_READWRITE for a private copy after a PAGE_EXECUTE_WRITECOPY write. A
   * failed restoration is reported even if the pointer exchange succeeded.
   */
  bool protection_restored = true;
};

/*
 * Optional admission for an image data page mapped PAGE_EXECUTE_WRITECOPY. The implementation verifies these
 * values against bounded PE headers in the current main image. This never admits executable/writable PE
 * sections, other modules, or anonymous executable memory. The caller still owns image lifetime. A
 * successful verified copy-on-write install admits the same image page's PAGE_EXECUTE_READWRITE promotion
 * for removal or the second hook, with a fresh PE data-section proof for that second slot.
 */
struct ImageDataSlotProof {
  HMODULE main_image = nullptr;
  std::uint32_t image_size = 0;
  std::uint32_t section_rva = 0;
  std::uint32_t section_size = 0;
};

/*
 * One successful installation per linked module instance, including an install whose final protection
 * restoration fails. The caller supplies the exact live slot and original, and owns all module/slot/observer
 * lifetime guarantees. A null observer is a no-op. Saved observer/original never change after a successful
 * exchange; enable requests inside the observer's own state.
 */
Result install(void** slot, void* expected_original, Observer observer = nullptr, const ImageDataSlotProof* image_data = nullptr) noexcept;

/*
 * Exchanges only our thunk for the original. Does not wait for in-flight calls, erase callback state, or
 * permit another installation. A foreign replacement is left untouched; removal can be retried if that owner
 * restores our thunk.
 */
Result remove() noexcept;

/*
 * Independent typed observer for CInGameStreamingMgr_G::update (void(void*)). It observes before calling the
 * original exactly once. Its installation, callback and recursion guard are independent from the manager/ASM
 * hook. The same one-install and caller-owned lifetime rules apply to this hook.
 */
Result install_service_update(void** slot,
                              void* expected_original,
                              Observer observer = nullptr,
                              const ImageDataSlotProof* image_data = nullptr) noexcept;
Result remove_service_update() noexcept;

/*
 * Runs the native classifier first, then lets the filter adjust its byte result. Arguments belong to the
 * native invocation and must not be retained by the filter.
 */
using TerrainClassify = unsigned char (*)(void* resource, const void* node, const void* context, float* priority);
using TerrainFilter = unsigned char (*)(void* resource,
                                        const void* node,
                                        const void* context,
                                        float* priority,
                                        unsigned char result,
                                        TerrainClassify original) noexcept;
Result install_terrain_filter(void** slot, void* expected_original, TerrainFilter filter, const ImageDataSlotProof* image_data) noexcept;

/*
 * The terrain update is void(void*, const void*). Its arguments remain borrowed through this callback, which
 * runs after the original update has returned.
 */
using TerrainUpdateObserver = void (*)(void* resource, const void* update) noexcept;
Result install_terrain_update(void** slot,
                              void* expected_original,
                              TerrainUpdateObserver observer,
                              const ImageDataSlotProof* image_data) noexcept;

/*
 * Runs after the native tile submission, in the same streaming traversal. All six arguments keep their
 * native ABI, including priority in XMM2.
 */
using TileSubmit = void (*)(void*, void*, float, void*, unsigned char, unsigned char);
using TileSubmitObserver = void (*)(void* world, void* node, unsigned char coarse) noexcept;
Result install_tile_submit(void** slot,
                           void* expected_original,
                           TileSubmitObserver observer,
                           const ImageDataSlotProof* image_data) noexcept;

/*
 * Before Camera_Z's calculation. The assembly thunk preserves all incoming arguments and tail-calls the
 * original; no return-type guess is made.
 */
using CameraUpdateObserver = void (*)(void* camera, void* node) noexcept;
Result install_camera_update(void** slot,
                             void* expected_original,
                             CameraUpdateObserver observer,
                             const ImageDataSlotProof* image_data) noexcept;

/*
 * Per-view world callback, after the renderer resets/applies view parameters. The caller must verify this
 * slot's native implementation and render call site. The verified void(world, context) method runs first;
 * the observer runs after its scene-cycle application. The native caller address identifies the phase.
 */
using ViewParametersObserver = void (*)(void* world, void* context, std::uintptr_t return_address) noexcept;
Result install_view_parameters(void** slot,
                               void* expected_original,
                               ViewParametersObserver observer,
                               const ImageDataSlotProof* image_data) noexcept;

/*
 * Redirects one verified E8 call in the main image's code to `replacement`, which must have the callee's
 * exact ABI. The call must lie in the read-only executable section `code` describes and still reach
 * `expected_target`. A jump stub within reach of the call is allocated; then only the call's aligned 32-bit
 * displacement changes, in one atomic exchange, so a thread running that code reaches either the original or
 * the replacement. One installation per process; the redirect and stub stay until exit (pin the module).
 */
Result install_call_redirect(void* call, void* expected_target, void* replacement, const ImageDataSlotProof* code) noexcept;

/*
 * Retries a failed page-protection restoration without exchanging the pointer. Both hooks share this
 * recovery obligation. Install/remove first retry it and refuse to exchange any pointer while it remains
 * unresolved. Keep the slot allocation alive.
 */
Result restore_protection() noexcept;

const char* status_name(Status status) noexcept;

}  /* namespace azp::engine_hook */
