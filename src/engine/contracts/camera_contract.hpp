/*
 * Defines the resolved camera contract and its proof records. It connects discovered function addresses,
 * reviewed layout offsets and callback locations to the camera subsystem without hard-coding a previous
 * executable's base address.
 */

#pragma once
#include <cstdint>
#include <string>
#include "../camera_layout.hpp"
#include "../image_inventory.hpp"

namespace azp::native_camera {

struct CameraFunctions {
  std::uint32_t initialize_descriptor = 0;
  std::uint32_t activate_entry = 0;
  std::uint32_t create_entry = 0;
  std::uint32_t erase_entry = 0;
  std::uint32_t set_position = 0;
  std::uint32_t set_up = 0;
  std::uint32_t set_target = 0;
  std::uint32_t set_fov = 0;
  std::uint32_t update_view = 0;
  std::uint32_t refresh_output = 0;
  std::uint32_t manager_update = 0;
};

/*
 * Published only after the entire instruction/data contract resolves. The runtime owns this value for the
 * loaded image's lifetime and never updates it after installing its observer. Discovery does not create or
 * own engine objects.
 */
struct CameraContract {
  CameraFunctions functions;
  CameraImageLayout layout;
};

struct CameraContractResolution {
  bool valid = false;
  std::string error;
  CameraContract contract;
  std::uint64_t scanned_bytes = 0;
  std::uint32_t matched_ranges = 0;
};

/*
 * Read-only discovery and verification. Resolves moved code/data while keeping instruction semantics, field
 * offsets and cross-reference relationships pinned to the reviewed camera ABI. No version/whole-image hash
 * whitelist is used. Ambiguous, incomplete or changed contracts publish no callable addresses.
 */
CameraContractResolution resolve_camera_contract(discovery::ImageReader& reader,
                                                 const discovery::Inventory& image,
                                                 std::uint64_t loaded_image_base);

}  /* namespace azp::native_camera */
