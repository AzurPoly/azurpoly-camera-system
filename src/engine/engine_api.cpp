/*
 * Resolves, once and in a fixed order, every part of the running executable that the DLL depends on:
 *
 *   1. the process and its verified PE image;
 *   2. the camera contract (manager, view pool, entry table, native camera calls)      - required;
 *   3. the render-gate disable mask used to close the owned view                          - required;
 *   4. the rigid-attachment contract (node parenting and camera-update callback)          - required;
 *   5. the night-vision contract (post-processing gates and parameter transfer)           - required: the
 *      camera image is the view's post-processed final image, in TV and IR;
 *   6. the scenery-preload ("world locker") contract                                      - optional;
 *   7. the zoom-detail contract (LOD selection and mesh fade distance factor)              - optional.
 *
 * Parts 3 to 5 build on the camera contract and are skipped when it fails. Scenery preload and zoom detail are
 * independent of the camera and are always attempted on a verified image.
 */

#include "engine_api.hpp"
#include <cwchar>
#include <exception>
#include <mutex>
#include "memory.hpp"

namespace azp::native_camera {
namespace {

struct Resolved {
  std::mutex mutex;
  bool done = false;
  EngineApi api;
};
/*
 * Process-lifetime storage: installed hooks keep using these results until exit.
 */
Resolved& resolved() {
  static auto* const instance = new Resolved;
  return *instance;
}

bool is_simulator_process() {
  wchar_t path[32768]{};
  const auto length = GetModuleFileNameW(nullptr, path, 32768);
  const auto* basename = std::wcsrchr(path, L'\\');
  return length && length < 32768 && _wcsicmp(basename ? basename + 1 : path, L"FlightSimulator2024.exe") == 0;
}

/*
 * Records one part and, for a failed required part, the first error that explains why the camera cannot
 * start.
 */
void record(EngineApi& api, const char* name, bool required, bool valid, const std::string& detail) {
  api.parts.push_back({name, required, valid, valid ? std::string() : detail});
  if (required && !valid && api.error.empty())
    api.error = detail;
}

void resolve(EngineApi& api) {
  if (!is_simulator_process()) {
    record(api, "process", true, false, "Native camera access requires the FlightSimulator2024.exe process.");
    return;
  }
  api.image = parse_verified_main_image();
  api.base = reinterpret_cast<std::uintptr_t>(GetModuleHandleW(nullptr));
  record(api, "image", true, api.image.valid_image, api.image.error);
  if (!api.image.valid_image)
    return;
  LocalImageReader reader(reinterpret_cast<HMODULE>(api.base), api.image.image_size);

  const auto camera = resolve_camera_contract(reader, api.image, api.base);
  api.camera = camera.contract;
  record(api, "camera", true, camera.valid, camera.error);
  if (camera.valid) {
    api.activation_mask = inspect_activation_disable_mask(reader, api.image, api.camera.layout);
    record(api, "render-gate mask", true, api.activation_mask.valid,
           "Native activation-mask verification refused: " + api.activation_mask.error);

    api.attachment = resolve_camera_attachment_contract(reader, api.image, api.base, api.camera);
    record(api, "attachment", true, api.attachment.valid, api.attachment.error);

    api.night_vision = resolve_night_vision_contract(reader, api.image, api.base);
    record(api, "night vision", true, api.night_vision.valid, api.night_vision.error);
  }

  try {
    api.preload = resolve_world_locker_contract(reader, api.image, api.base);
  } catch (...) {
    api.preload = {};
    api.preload.error = "Preload refused: engine contract resolution failed.";
  }
  record(api, "scenery preload", false, api.preload.valid, api.preload.error);

  try {
    api.zoom_detail = resolve_zoom_detail_contract(reader, api.image, api.base);
  } catch (...) {
    api.zoom_detail = {};
    api.zoom_detail.error = "Zoom detail unavailable: engine contract resolution failed.";
  }
  record(api, "zoom detail", false, api.zoom_detail.valid, api.zoom_detail.error);
}

}  /* namespace */

const EngineApi& engine_api() noexcept {
  auto& state = resolved();
  const std::lock_guard lock(state.mutex);
  if (!state.done) {
    state.done = true;
    try {
      resolve(state.api);
    } catch (const std::exception& error) {
      record(state.api, "resolution", true, false, error.what());
    } catch (...) {
      record(state.api, "resolution", true, false, "Native engine resolution failed.");
    }
    state.api.valid = state.api.error.empty();
  }
  return state.api;
}

bool engine_api_resolved() noexcept {
  auto& state = resolved();
  const std::lock_guard lock(state.mutex);
  return state.done;
}

std::string engine_api_summary(const EngineApi& api) {
  std::string text = "Engine:";
  for (const auto& part : api.parts) {
    text += ' ';
    text += part.name;
    text += part.valid ? " ok;" : part.required ? " FAILED (" + part.detail + ");" : " unavailable (" + part.detail + ");";
  }
  return text;
}

}  /* namespace azp::native_camera */
