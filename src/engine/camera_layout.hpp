/*
 * Defines the engine layout offsets supplied by the verified camera contract. Keeping these fields together
 * lets inspectors follow the same reviewed object model instead of scattering unvalidated offsets through
 * camera logic.
 */

#pragma once
#include <cstdint>

namespace azp::native_camera {

/*
 * Image-relative identities only. Member offsets, native signatures and lifetime rules remain a separate
 * validated ABI contract; supplying RVAs proves none of those obligations. Runtime callers must pass their
 * resolved image layout.
 */
struct CameraImageLayout {
  std::uint32_t manager_owner_global = 0;
  std::uint32_t renderer_global = 0;
  std::uint32_t manager_vtable = 0;
  std::uint32_t aircraft_worlds_global = 0;
  std::uint32_t aircraft_facade_vtable = 0;
  std::uint32_t aircraft_facade_method = 0;
  std::uint32_t aircraft_controller_vtable = 0;
  std::uint32_t aircraft_controller_method = 0;
  std::uint32_t aircraft_selected_vtable = 0;
  std::uint32_t aircraft_selected_method = 0;
  std::uint32_t aircraft_key_component_vtable = 0;
  std::uint32_t scene_node_vtable = 0;
  std::uint32_t scene_model_vtable = 0;
  std::uint32_t activation_disable_mask = 0;
  std::uint32_t view_flag_clear_override = 0;
  std::uint32_t view_flag_set_override = 0;
};

namespace camera_layout_detail {
/*
 * Shared structural ceiling, not proof of a particular image's section bounds. Readers and metadata
 * inspectors retain their stricter actual-image checks.
 */
constexpr bool image_rva(std::uint32_t rva, std::uint32_t bytes, std::uint32_t alignment = 1) noexcept {
  return rva && rva < 0x80000000u && bytes && bytes <= 0x80000000u - rva && alignment && rva % alignment == 0;
}
}  /* namespace camera_layout_detail */
}  /* namespace azp::native_camera */
