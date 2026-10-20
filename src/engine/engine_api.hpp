/*
 * Everything version-specific that the DLL uses inside the running FlightSimulator2024.exe, found once.
 *
 * engine_api() parses the executable image, then locates and verifies each part through its engine contract
 * (see contracts/). The result is kept for the process lifetime. Nothing here installs a hook or calls an
 * engine function: the camera, attachment, night-vision, scenery-preload and zoom-detail modules install their
 * own hooks from these results.
 *
 * Required parts gate the camera. The optional scenery-preload and zoom-detail parts only disable their own
 * feature when missing; video continues.
 */

#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include "contracts/camera_attachment_contract.hpp"
#include "contracts/camera_contract.hpp"
#include "contracts/zoom_detail_contract.hpp"
#include "contracts/world_locker_contract.hpp"
#include "night_vision.hpp"
#include "view.hpp"

namespace azp::native_camera {

struct EnginePart {
  const char* name = "";
  bool required = false;
  bool valid = false;
  std::string detail;
};

struct EngineApi {
  /*
   * True when every required part was verified. `error` names the first required failure.
   */
  bool valid = false;
  std::string error;
  std::uintptr_t base = 0;
  discovery::Inventory image;
  CameraContract camera;
  CameraAttachmentContract attachment;
  ActivationMaskInventory activation_mask;
  NightVisionContract night_vision;
  WorldLockerContract preload;
  ZoomDetailContract zoom_detail;
  /*
   * One line per part, in resolution order, for the bridge log.
   */
  std::vector<EnginePart> parts;
};

/*
 * Resolves on the first call, which can take several seconds, and returns the same result afterwards. Call
 * only from the DLL control thread, never from an engine or render callback.
 */
const EngineApi& engine_api() noexcept;
bool engine_api_resolved() noexcept;
/*
 * "camera ok; attachment ok; night vision unavailable: ..." for the bridge log.
 */
std::string engine_api_summary(const EngineApi& api);

}  /* namespace azp::native_camera */
