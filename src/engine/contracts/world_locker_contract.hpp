/*
 * Declares the scenery-preload contract resolver. A successful result supplies verified operations and
 * layouts to camera/scenery_preload.cpp; it does not itself create registrations or request scenery. The
 * "world locker" name is historical and kept because the generated model and its evidence use it.
 */

#pragma once
#include <array>
#include "code_contract.hpp"

namespace azp::native_camera {
struct WorldLockerContract {
  bool valid = false;
  std::string error;
  std::uint32_t update = 0, vtable = 0, add88 = 0, add78 = 0, remove = 0, convert = 0, current_world = 0;
  std::uint32_t terrain_classify = 0, terrain_vtable = 0, terrain_update = 0, terrain_bounds = 0, terrain_intersects = 0;
  std::uint32_t tile_submit = 0, world_vtable = 0, node_ready = 0, node_bounds = 0;
  std::uint32_t vegetation_update = 0, vegetation_level = 0, vegetation_quality = 0;
  std::uint32_t vegetation_density_low = 0, vegetation_density_high = 0;
  std::vector<CodeFingerprint> fingerprints;
};
WorldLockerContract resolve_world_locker_contract(discovery::ImageReader& reader,
                                                  const discovery::Inventory& image,
                                                  std::uint64_t image_base);
}  /* namespace azp::native_camera */
