/*
 * Declares the command-list boundary hooks: barriers, render passes, draws and copies. They maintain each
 * tracked list's render-pass scope and hand every source-state observation and marked WASM upload to the
 * capture manager and the video runtime.
 */

#pragma once
#include <d3d12.h>
#include <cstdint>
#include "d3d12_objects.hpp"

namespace azp::list_hooks {
/*
 * Why a recording's capture evidence was revoked.
 */
enum InvalidationFlags : std::uint32_t {
  InvalidationPassBegin = 1,
  InvalidationPassState = 2,
  InvalidationBarrierBatch = 4,
  InvalidationSplitBarrier = 8,
  InvalidationAliasOrDiscard = 16,
  InvalidationUnobservedWork = 32,
  InvalidationObserverDisabled = 64,
  InvalidationResetFailed = 128,
};
/*
 * Installs, or re-verifies, the boundary hooks on the one shared command-list table: QI7 must return this
 * identical interface pointer before its extended cells are inspected, then eight cells (12/13/16/17/26/68/
 * 69/80) are patched in place. No expanded-interface slot is guessed and no list reference is held. A hook
 * changed by another module disables capture rather than being overwritten. Unknown lists forward
 * unchanged.
 */
bool install(ID3D12GraphicsCommandList*) noexcept;
/*
 * Called inside a render pass: hooks End in the active-pass table the runtime switched this list to, for at
 * most eight such tables. A failure disables capture.
 */
void track_active_pass(ID3D12GraphicsCommandList4*) noexcept;
/*
 * Starts a new recording's scope; call only after an actual successful native Reset. While capture is
 * disabled the new recording stays invalid.
 */
void begin_recording(standalone::List&) noexcept;
/*
 * Call from bundle/indirect/otherwise unobserved native work or a failed Reset. Revokes this recording's
 * capture permission until the next successful Reset and invalidates its capture evidence. Does not
 * execute that work or dereference resources.
 */
void invalidate_recording(standalone::List&, std::uint32_t reason_flags = InvalidationUnobservedWork) noexcept;
/*
 * False once a changed or failed hook has disabled capture; existing recordings stay invalid until Reset.
 */
bool operational() noexcept;
}  /* namespace azp::list_hooks */
