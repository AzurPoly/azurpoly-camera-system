/*
 * Implements zoom detail (zoom_detail.hpp). The engine's per-piece LOD selection for the primary viewpoint
 * list is redirected to select_lod (engine_hook::install_call_redirect), which calls the original selection
 * with the AZP entry alone or without it. The AZP view's mesh fade factor is scaled by the camera observer.
 *
 * select_lod runs on the engine's worker threads, for every model piece, every frame. It reads only its
 * arguments and the published AZP view, and calls only the original selection. apply_zoom_fade runs in the
 * camera observer, on the owned view the observer has just verified and updated, and writes only that view's
 * fade factor.
 */

#include "zoom_detail.hpp"
#include <windows.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <mutex>
#include "../engine/engine_api.hpp"
#include "../engine/memory.hpp"
#include "../engine/observer_hook.hpp"

namespace azp::native_camera {
namespace {
/*
 * One entry of the engine's viewpoint list: world position, unit forward and the FOV scale
 * sqrt(Objects LoD factor) / tan(FOV / 2). The selection reads the entries from `items` to `items + count`
 * and keeps the nearest one; it stores nothing about them.
 */
struct Viewpoint {
  double position[3];
  double forward[3];
  float scale;
  std::uint32_t padding;
};
static_assert(sizeof(Viewpoint) == 0x38);
struct ViewpointList {
  const Viewpoint* items;
  std::int32_t count;
};
/*
 * piece, sphere (world center then radius, m), viewpoint list, size out (fraction of the image height), and
 * two flags the native size calculation uses; returns the LOD index or -1 (hidden).
 */
using LodSelect = int (*)(void*, const double*, const ViewpointList*, float*, bool, bool);

/*
 * The AZP entry of the list is the one at the published camera position whose direction matches the
 * published look direction. The distance absorbs a frame of aircraft motion; the direction separates it from
 * the pilot's eye, 2.3 m away. The engine lists one viewpoint per view of the world.
 */
constexpr double kMatchDistance = 30;
constexpr double kMatchCosine = 0.99985;  /* 1 degree */
constexpr std::uint64_t kViewLifetimeMs = 250;
constexpr int kMaximumViewpoints = 16;
constexpr double kZoomOneTanHalf = 0.2679491924311227;  /* tan(15 degrees) */

struct State {
  std::mutex install_mutex, publish_mutex, message_mutex;
  bool tried = false;
  std::atomic<bool> enabled{false}, contract_ready{false};
  std::atomic<LodSelect> original{nullptr};
  ZoomDetailContract contract;  /* Set once, before contract_ready. */
  /*
   * The AZP view, published by one writer at a time under a sequence lock: position, forward, tan(FOV / 2).
   */
  std::atomic<std::uint32_t> sequence{0};
  std::array<std::atomic<double>, 7> view{};
  std::atomic<std::uint64_t> published_ms{0};
  std::string message = "Zoom detail: not started.";
};
State& state() {
  static auto* const value = new State;
  return *value;
}
void set_message(const std::string& text) {
  auto& s = state();
  const std::lock_guard lock(s.message_mutex);
  s.message = text;
}

struct View {
  Vector3 position{}, forward{};
  double tan_half = 0;
};
/*
 * Reads a complete, recent publication; a torn or stale one leaves the engine's list.
 */
bool published_view(View& view) noexcept {
  auto& s = state();
  const auto before = s.sequence.load(std::memory_order_acquire);
  std::array<double, 7> values{};
  for (std::size_t i = 0; i < values.size(); ++i)
    values[i] = s.view[i].load(std::memory_order_relaxed);
  const auto published = s.published_ms.load(std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_acquire);
  if ((before & 1) || s.sequence.load(std::memory_order_relaxed) != before || GetTickCount64() - published > kViewLifetimeMs)
    return false;
  view = {{values[0], values[1], values[2]}, {values[3], values[4], values[5]}, values[6]};
  return true;
}

/*
 * The redirected selection. With the AZP entry found in a list of several views, a piece whose sphere can
 * appear in the AZP image is judged from that entry alone, any other piece from the remaining entries;
 * otherwise the engine's list is kept.
 */
int select_lod(void* piece, const double* sphere, const ViewpointList* viewpoints, float* size, bool first, bool second) noexcept {
  auto& s = state();
  const auto original = s.original.load(std::memory_order_acquire);
  View view;
  if (!s.enabled.load(std::memory_order_relaxed) || !sphere || !viewpoints || !viewpoints->items || viewpoints->count < 2 ||
      viewpoints->count > kMaximumViewpoints || !published_view(view))
    return original(piece, sphere, viewpoints, size, first, second);
  int azp = -1;
  double best = kMatchCosine;
  for (int i = 0; i < viewpoints->count; ++i) {
    const auto& item = viewpoints->items[i];
    const Vector3 position{item.position[0], item.position[1], item.position[2]};
    const auto cosine = item.forward[0] * view.forward[0] + item.forward[1] * view.forward[1] + item.forward[2] * view.forward[2];
    if (body_math::distance(position, view.position) <= kMatchDistance && cosine >= best) {
      best = cosine;
      azp = i;
    }
  }
  if (azp < 0)
    return original(piece, sphere, viewpoints, size, first, second);
  const auto& camera = viewpoints->items[azp];
  const bool inside = std::isfinite(sphere[3]) && sphere[3] >= 0 &&
                      zoom_detail::inside_image({camera.position[0], camera.position[1], camera.position[2]},
                                                {camera.forward[0], camera.forward[1], camera.forward[2]}, view.tan_half,
                                                {sphere[0], sphere[1], sphere[2]}, sphere[3]);
  std::array<Viewpoint, kMaximumViewpoints> kept;
  ViewpointList list{kept.data(), 0};
  for (int i = 0; i < viewpoints->count; ++i)
    if ((i == azp) == inside)
      kept[list.count++] = viewpoints->items[i];
  return original(piece, sphere, &list, size, first, second);
}
}  /* namespace */

void set_zoom_detail_enabled(bool enabled) noexcept {
  auto& s = state();
  s.enabled.store(enabled, std::memory_order_relaxed);
  if (!enabled || s.contract_ready.load(std::memory_order_acquire))
    return;
  try {
    const std::lock_guard lock(s.install_mutex);
    if (s.tried)
      return;
    s.tried = true;
    const auto& api = engine_api();
    if (!api.zoom_detail.valid) {
      set_message(api.zoom_detail.error);
      return;
    }
    HMODULE pinned = nullptr;
    if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN, reinterpret_cast<LPCWSTR>(&select_lod),
                            &pinned)) {
      set_message("Zoom detail refused: module pin failed.");
      return;
    }
    s.contract = api.zoom_detail;
    s.contract_ready.store(true, std::memory_order_release);
    s.original.store(reinterpret_cast<LodSelect>(api.base + s.contract.lod_select), std::memory_order_release);
    const engine_hook::ImageDataSlotProof code{reinterpret_cast<HMODULE>(api.base), api.image.image_size, s.contract.call_section_rva,
                                               s.contract.call_section_size};
    const auto result = engine_hook::install_call_redirect(reinterpret_cast<void*>(api.base + s.contract.primary_call),
                                                           reinterpret_cast<void*>(api.base + s.contract.lod_select),
                                                           reinterpret_cast<void*>(&select_lod), &code);
    if (!result.pointer_changed)
      set_message(std::string("Zoom detail: fade only, LOD redirect refused: ") + engine_hook::status_name(result.status) + ".");
    else
      set_message(result.protection_restored ? "Zoom detail ready: LOD viewpoints and fade follow the AZP image."
                                             : "Zoom detail ready; the call page protection was not restored.");
  } catch (...) {
    set_message("Zoom detail refused: installation failed.");
  }
}

void publish_zoom_detail_view(const Vector3& position, const Vector3& forward, double fov) noexcept {
  auto& s = state();
  const auto tan_half = std::tan(fov * 0.5);
  if (!s.contract_ready.load(std::memory_order_relaxed) || !mount_detail::finite(position) || !mount_detail::finite(forward) ||
      !std::isfinite(tan_half) || tan_half <= 0)
    return;
  const std::unique_lock lock(s.publish_mutex, std::try_to_lock);
  if (!lock.owns_lock())
    return;
  const auto sequence = s.sequence.load(std::memory_order_relaxed);
  s.sequence.store(sequence + 1, std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);
  const std::array<double, 7> values{position[0], position[1], position[2], forward[0], forward[1], forward[2], tan_half};
  for (std::size_t i = 0; i < values.size(); ++i)
    s.view[i].store(values[i], std::memory_order_relaxed);
  s.published_ms.store(GetTickCount64(), std::memory_order_relaxed);
  s.sequence.store(sequence + 2, std::memory_order_release);
}

/*
 * The native view update (camera contract update_view) has just written the view's fade factor: 1, or a
 * cinematic camera's FOV ratio, divided by the quality profile's multiplier. The renderer copies it to the
 * view's render context each frame and multiplies each mesh's camera distance by it; the next native update
 * restores it. The compare/exchange leaves a concurrent engine write in place.
 */
void apply_zoom_fade(std::uint64_t view_address, double fov) noexcept {
  auto& s = state();
  const auto scale = static_cast<float>(std::tan(fov * 0.5) / kZoomOneTanHalf);
  if (!s.contract_ready.load(std::memory_order_acquire) || !(scale > 1e-4f && scale < 1) || !view_address || (view_address & 7))
    return;
  const auto field = view_address + s.contract.view_fade_scale;
  if (!writable_private_span(field, 4))
    return;
  auto* target = reinterpret_cast<volatile LONG*>(field);
  const auto native = *target;
  const auto factor = std::bit_cast<float>(native);
  if (std::isfinite(factor) && factor > 0)
    InterlockedCompareExchange(target, std::bit_cast<LONG>(factor * scale), native);
}

std::string zoom_detail_message() {
  auto& s = state();
  const std::lock_guard lock(s.message_mutex);
  return s.message;
}

}  /* namespace azp::native_camera */
