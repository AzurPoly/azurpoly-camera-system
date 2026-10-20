/*
 * Declares graphics initialization, observation demand and bridge status for the DLL worker. The
 * implementation owns native hooks and resource tracking; callers do not obtain unrestricted access to
 * simulator GPU objects.
 */

#pragma once
#include "scene_runtime.hpp"

namespace azp::standalone {
struct GraphicsStatus {
  bool ready{};
  std::uint64_t hook_failures{};
  const char* error = "not_started";
};
bool initialize_graphics() noexcept;
/*
 * Resolve a reported device's optional COM proxy chain before native hooks or owned GPU work. Isolated
 * validation supplies a hardware/WARP device; this never calls the simulator or telemetry.
 */
bool initialize_graphics(IUnknown*) noexcept;
bool graphics_ready() noexcept;
/*
 * True while the WASM requests video. Idle keeps native resource/descriptor lifetime and retirement of
 * previously recorded/submitted work; source models stay continuously observed. A recording started while
 * idle needs a real successful Reset to count as observed.
 */
void set_graphics_observation_demand(bool enabled) noexcept;
GraphicsStatus graphics_status() noexcept;
/*
 * Bounds the native-object registry by dropping entries whose objects were destroyed.
 */
void collect_retired_objects() noexcept;
}  /* namespace azp::standalone */
