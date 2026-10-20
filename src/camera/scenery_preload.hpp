/*
 * Declares target-preload demand, tuning inputs and operational status. The implementation services
 * registrations in the engine's own callbacks and uses a copied, short-lived view volume for terrain and
 * vegetation decisions.
 */

#pragma once
#include <cstdint>

namespace azp::native_camera {
struct WorldPreloadStatus {
  std::uint32_t active = 0;
  std::uint64_t requests = 0, releases = 0;
  char message[160]{};
};

/*
 * Experimental, session-only. Commands are consumed by the engine streaming update.
 */
void set_world_preload_enabled(bool enabled) noexcept;
/*
 * Positive LVAR multipliers; 1 preserves the FOV-dependent policy. Area scales the extra loading footprint,
 * not the rendered camera or global scenery LOD.
 */
void set_world_preload_scalars(double density, double area) noexcept;
/*
 * Numeric identity only, supplied inside the validated camera observer phase. A negative index withdraws the
 * view; no engine pointer crosses this API.
 */
void set_world_preload_view(int view_index) noexcept;
WorldPreloadStatus world_preload_status() noexcept;
}  /* namespace azp::native_camera */
