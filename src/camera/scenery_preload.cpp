/*
 * Requests scenery around the camera target without moving the camera. Verified engine callbacks manage
 * target registrations, terrain detail and vegetation thresholds. FOV-dependent area/detail rules and the
 * two LVAR multipliers affect only AZP's extra requests, leaving native scenery demand intact.
 */

#include "scenery_preload.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cwchar>
#include <mutex>
#include "../engine/engine_api.hpp"
#include "../engine/memory.hpp"
#include "../engine/observer_hook.hpp"
#include "simconnect_link.hpp"

namespace azp::native_camera {
namespace {
using WeakWorld = std::array<std::uint64_t, 2>;
using AddZone = void (*)(void*, const void*, const double*, const double*, float, float, bool, const void*);
using RemoveOwner = void (*)(void*, const void*);
using ConvertPosition = bool (*)(double*, const double*, const int*, const unsigned*);
using CurrentWorld = const void* (*)();
constexpr double maximum_object_detail_multiplier = 64.0;
constexpr double base_density_multiplier = 0.5;
constexpr float full_detail_fov = static_cast<float>(2.0 * body_math::radians);

/*
 * FOV is in radians. A wide view spreads the gauge's pixels over more scenery, so only a fraction of AZP's
 * extra detail is useful. The FOV weight saturates at 2 degrees; the global density multiplier scales the
 * extra demand at every zoom.
 */
double preload_detail_weight(float fov) noexcept {
  if (fov <= full_detail_fov)
    return base_density_multiplier;
  return base_density_multiplier * std::clamp(std::tan(full_detail_fov * 0.5) / std::tan(fov * 0.5), 1.0 / 16.0, 1.0);
}

/*
 * Maps published FOV to the shared zoom anchors and interpolates the extra loading area between them. Zoom 1
 * disables preload; zooms 2 through 6 decrease the area multiplier from 2 to 0.1.
 *
 * Match CAMERA_FIELDS_OF_VIEW_DEG in Rafale_CameraOverlay.h without changing the WASM packet. Zoom 1
 * disables preload. Zooms 2..6 use 2 - 0.475 * (zoom - 2). Interpolate between FOV anchors if a publisher
 * supplies intermediate values.
 */
double preload_area_weight(float fov) noexcept {
  constexpr std::array<double, 6> fovs{30, 15, 6.5, 2.4, 0.75, 0.2};
  constexpr std::array<double, 6> areas{0, 2, 1.525, 1.05, 0.575, 0.1};
  if (fov >= static_cast<float>(fovs[0] * body_math::radians))
    return 0;
  for (unsigned i = 1; i < fovs.size(); ++i) {
    const auto upper = static_cast<float>(fovs[i - 1] * body_math::radians);
    const auto lower = static_cast<float>(fovs[i] * body_math::radians);
    if (fov >= lower)
      return std::lerp(areas[i - 1], areas[i], static_cast<double>((upper - fov) / (upper - lower)));
  }
  return areas.back();
}

struct ServiceTick {
  std::uintptr_t address = 0;
  WeakWorld world{};
  std::uint64_t next_ms = 0, last_seen = 0;
  std::array<double, 3> position{};
  float fov = 0;
  double density_scalar = 1, area_scalar = 1;
  bool active = false;
};
struct TerrainView {
  /*
   * Copied world-space planes and scalars, never borrowed engine storage.
   */
  std::array<double, 24> planes{};
  double far_distance = 0;
  double detail_weight = 1;
  float fov = 0, reference_fov = 0;
  int index = -1;
  std::uintptr_t resource = 0;  /* Equality only; never dereferenced from this snapshot. */
  std::uint64_t time_ms = 0;
};
struct State {
  std::mutex install_mutex, status_mutex, terrain_mutex;
  std::atomic<bool> enabled{false}, installed{false}, ready{false}, faulted{false};
  bool tried = false;
  std::uintptr_t base = 0;
  discovery::Inventory image;
  WorldLockerContract contract;
  WorldPreloadStatus status;
  /*
   * Observer-thread metadata; addresses are never called or dereferenced from this cache.
   */
  std::array<ServiceTick, 8> services{};
  std::array<std::atomic<std::uintptr_t>, 8> terrain_resources{};
  std::atomic<unsigned> point_level_reduction{4};
  std::atomic<double> density_scalar{1}, area_scalar{1};
  std::atomic<std::uint64_t> terrain_requests{0};
  std::atomic<unsigned> terrain_level{0};
  std::atomic<int> view_index{-1};
  TerrainView terrain_view;
  std::atomic<std::uint64_t> terrain_revision{0};
  std::uint64_t next_prune_ms = 0;
  std::atomic_flag observing = ATOMIC_FLAG_INIT;
};
State& state() {
  static State* const value = new State;
  return *value;
}
void publish(const char* message, int operation = 0) noexcept {
  auto& s = state();
  std::lock_guard lock(s.status_mutex);
  std::snprintf(s.status.message, sizeof(s.status.message), "%s", message);
  if (operation > 0)
    ++s.status.requests;
  if (operation < 0)
    ++s.status.releases;
}
template <class T>
bool field(LocalMemoryReader& reader, std::uint64_t address, std::uint64_t offset, T& value) noexcept {
  return address && address <= UINT64_MAX - offset && reader.read(address + offset, &value, sizeof(value));
}
bool live_world(LocalMemoryReader& reader, const WeakWorld& key) noexcept {
  std::uint32_t generation = 0;
  std::uintptr_t object = 0;
  return key[0] && field(reader, key[0], 0x1c, generation) && generation == static_cast<std::uint32_t>(key[1]) &&
         field(reader, key[0], 0, object) && object;
}

/*
 * Copies the verified owned view's projection and six planes for later terrain/vegetation callbacks. It
 * scales opposite side-plane pairs by the square root of the requested area while preserving the camera apex
 * and depth bounds; no engine view pointer crosses to workers.
 *
 * The native update keeps its receiver and view context alive through this callback. Inspect the actual
 * view, independently of group ordering.
 *
 * Clear the copied view immediately: terrain and vegetation must not reuse the previous zoom's detail while
 * the streaming service releases its owner.
 *
 * The copied frustum stores opposite side planes in pairs 0/1 and 2/3; planes 4/5 bound depth. Scale each
 * pair's opening around its centre line. Width and height use sqrt(area), so area=0.5 halves the
 * cross-section. All four plane coefficients must change together to keep the camera apex.
 */
void terrain_updated(void* resource, const void* update) noexcept {
  auto& s = state();
  const auto wanted = s.view_index.load(std::memory_order_acquire);
  if (wanted < 0 || !s.enabled.load() || !s.ready.load(std::memory_order_acquire) || s.faulted.load())
    return;
  const auto target = get_world_preload_target();
  if (!target.valid)
    return;

  ScopedLocalMemoryQueryCache query(LocalMemoryQueryMode::private_pages);
  LocalMemoryReader reader;
  std::uintptr_t context = 0, camera = 0, type = 0;
  std::array<int, 2> ids{}, again{};
  unsigned flags = 0;
  TerrainView view;
  if (!field(reader, reinterpret_cast<std::uintptr_t>(update), 8, context) || !field(reader, context, 0x74c, ids) || ids[0] != wanted ||
      ids[0] == ids[1] || !field(reader, context, 0x748, flags) || (flags & 4) ||
      !field(reader, reinterpret_cast<std::uintptr_t>(resource), 0, type) || type != s.base + s.contract.terrain_vtable ||
      !field(reader, reinterpret_cast<std::uintptr_t>(resource), 0x21f58, view.reference_fov) || !field(reader, context, 0xc18, camera) ||
      !field(reader, camera, 0x650, view.fov) || !field(reader, context, 0x2a0 + 0xc0, view.planes) ||
      !field(reader, context, 0x2a0 + 0x3d0, view.far_distance) || !field(reader, context, 0x74c, again) || again != ids || !query.finish())
    return;
  if (!std::isfinite(view.reference_fov) || view.reference_fov <= 0 || view.reference_fov >= body_math::pi || !std::isfinite(view.fov) ||
      view.fov < 0.001f || view.fov >= body_math::pi || !std::isfinite(view.far_distance) || view.far_distance <= 0 ||
      !std::all_of(view.planes.begin(), view.planes.end(), [](double x) { return std::isfinite(x); }))
    return;
  const auto base_area = preload_area_weight(view.fov);
  if (base_area == 0) {
    std::lock_guard lock(s.terrain_mutex);
    s.terrain_view = {};
    s.terrain_revision.fetch_add(1, std::memory_order_release);
    return;
  }
  for (unsigned i = 0; i < 6; ++i) {
    const auto* p = view.planes.data() + i * 4;
    const auto norm = std::hypot(p[0], p[1], p[2]);
    if (!std::isfinite(norm) || norm <= 0)
      return;
  }
  view.detail_weight = preload_detail_weight(view.fov) * s.density_scalar.load(std::memory_order_relaxed);
  const auto area = base_area * s.area_scalar.load(std::memory_order_relaxed);
  if (area != 1) {
    const auto width_scale = std::sqrt(area);
    for (unsigned pair = 0; pair < 4; pair += 2) {
      auto* a = view.planes.data() + pair * 4;
      auto* b = a + 4;
      const auto a_norm = std::hypot(a[0], a[1], a[2]);
      const auto b_norm = std::hypot(b[0], b[1], b[2]);
      for (unsigned component = 0; component < 4; ++component) {
        const auto first = a[component] / a_norm, second = b[component] / b_norm;
        const auto centre = (first + second) * 0.5 * width_scale;
        const auto side = (first - second) * 0.5;
        a[component] = centre + side;
        b[component] = centre - side;
      }
      for (auto* plane : {a, b}) {
        const auto norm = std::hypot(plane[0], plane[1], plane[2]);
        if (!std::isfinite(norm) || norm <= 0)
          return;
        for (unsigned component = 0; component < 4; ++component)
          plane[component] /= norm;
      }
    }
    if (!std::all_of(view.planes.begin(), view.planes.end(), [](double x) { return std::isfinite(x); }))
      return;
  }
  view.index = wanted;
  view.resource = reinterpret_cast<std::uintptr_t>(resource);
  view.time_ms = GetTickCount64();
  std::lock_guard lock(s.terrain_mutex);
  if (s.view_index.load(std::memory_order_acquire) != wanted || !s.enabled.load())
    return;
  s.terrain_view = view;
  s.terrain_revision.fetch_add(1, std::memory_order_release);
}

/*
 * Calls the native classifier and adds only AZP's admissible detail demand within the fresh copied loading
 * volume. FOV/density adjust extra projection and subdivision demand; native accepted requests and existing
 * tiles are never demoted.
 *
 * These immutable fields belong to the live classifier invocation. The engine holds the resource while
 * calling its virtual function; no pointer is retained here.
 *
 * Both the reference-view correction and the target point's extra admissions use AZP's scaled preload
 * volume. A point locker alone has no view frustum. Workers retain copied planes only; no engine container
 * or view is borrowed.
 *
 * Test complete tile bounds, not their centres, so tiles crossing the image edges remain eligible. Outside
 * this volume, preserve the native result without adding AZP detail. Existing scenery and other views keep
 * their own requests.
 *
 * Only fields read by the classifier and its reviewed callees are copied. In particular, no engine array
 * pointer or group storage is retained.
 *
 * Keep the engine's base LOD. FOV and the density LVAR scale only AZP's additional projection demand; the
 * loading footprint is independent.
 *
 * Loading requests are the union of the native view and AZP demands.
 *
 * Point lockers have no view frustum. Limit only AZP's extra admissions so their finest subdivisions cannot
 * bypass the wide-angle detail reduction. Native requests and existing tiles retain the original
 * classification.
 *
 * Result 1 only maintains existing tile resources. Result 2 admits a loading request within the original
 * point criterion's distance and maximum level.
 */
unsigned char classify_terrain(void* resource,
                               const void* node,
                               const void* context,
                               float* priority,
                               unsigned char result,
                               engine_hook::TerrainClassify original) noexcept {
  auto& s = state();
  if (result >= 2 || !s.enabled.load(std::memory_order_relaxed) || !s.ready.load(std::memory_order_acquire) || s.faulted.load())
    return result;
  const auto address = reinterpret_cast<std::uintptr_t>(resource);
  bool owned = false;
  for (const auto& current : s.terrain_resources)
    owned |= current.load(std::memory_order_acquire) == address;
  int level = 0, maximum = 0, frusta = 0;
  std::memcpy(&level, node, sizeof(level));
  std::memcpy(&maximum, static_cast<const unsigned char*>(resource) + 0x1c, sizeof(maximum));
  std::memcpy(&frusta, static_cast<const unsigned char*>(resource) + 0x16c0, sizeof(frusta));
  if (level < 2 || maximum < 2 || maximum > 26 || level > maximum)
    return result;

  if (!context || (!owned && (frusta <= 0 || !priority || !original)) || (owned && (frusta != 0 || result != 1)))
    return result;

  alignas(16) static thread_local std::array<unsigned char, 0x21f60> scratch{};
  static thread_local TerrainView view;
  static thread_local std::uint64_t revision = 0;
  const auto current = s.terrain_revision.load(std::memory_order_acquire);
  if (current != revision) {
    std::lock_guard lock(s.terrain_mutex);
    view = s.terrain_view;
    revision = s.terrain_revision.load(std::memory_order_relaxed);
    std::memcpy(scratch.data() + 0x6c0, view.planes.data(), sizeof(view.planes));
    std::memcpy(scratch.data() + 0x9d0, &view.far_distance, sizeof(view.far_distance));
    const int count = 1, capacity = 4;
    std::memcpy(scratch.data() + 0x16c0, &count, sizeof(count));
    std::memcpy(scratch.data() + 0x16c4, &capacity, sizeof(capacity));
  }
  if (!current || (!owned && view.resource != address) || view.index != s.view_index.load(std::memory_order_acquire) ||
      GetTickCount64() - view.time_ms > 250)
    return result;

  alignas(16) std::array<double, 16> bounds{};
  reinterpret_cast<void (*)(const void*, void*)>(s.base + s.contract.terrain_bounds)(context, bounds.data());
  if (!reinterpret_cast<bool (*)(const void*, const void*)>(s.base + s.contract.terrain_intersects)(scratch.data(), bounds.data()))
    return result;

  if (!owned) {
    const auto* source = static_cast<const unsigned char*>(resource);
    std::memcpy(scratch.data() + 8, source + 8, 24);
    std::memcpy(scratch.data() + 0x230, source + 0x230, 4);
    std::memcpy(scratch.data() + 0x5fc, source + 0x5fc, 4);
    scratch[0x21ea8] = source[0x21ea8];
    std::memcpy(scratch.data() + 0x21eb0, source + 0x21eb0, 0x38);
    std::memcpy(scratch.data() + 0x21f00, source + 0x21f00, 0x30);
    std::memcpy(scratch.data() + 0x21f38, source + 0x21f38, 0x10);
    float base_factor = 0, zoom_factor = 0, reference_fov = 0;
    std::memcpy(&base_factor, source + 0x21f40, sizeof(float));
    std::memcpy(&zoom_factor, source + 0x21f44, sizeof(float));
    std::memcpy(&reference_fov, source + 0x21f58, sizeof(float));
    if (!std::isfinite(reference_fov) || reference_fov <= view.fov || reference_fov >= body_math::pi || !std::isfinite(base_factor) ||
        base_factor <= 0 || !std::isfinite(zoom_factor) || zoom_factor <= 0)
      return result;
    const auto projection_zoom = std::tan(reference_fov * 0.5) / std::tan(view.fov * 0.5);
    const auto detail_zoom = 1.0 + view.detail_weight * (projection_zoom - 1.0);
    const auto detail_fov =
        view.detail_weight == 1.0 ? view.fov : static_cast<float>(2.0 * std::atan(std::tan(reference_fov * 0.5) / detail_zoom));
    zoom_factor = static_cast<float>(zoom_factor * detail_zoom);
    if (!std::isfinite(zoom_factor) || zoom_factor <= base_factor)
      return result;
    std::memcpy(scratch.data() + 0x21f44, &zoom_factor, sizeof(float));
    std::memcpy(scratch.data() + 0x21f58, &detail_fov, sizeof(float));
    scratch[0x21f5c] = 1;
    float zoom_priority = 0;
    const auto zoom_result = original(scratch.data(), node, context, &zoom_priority);
    if (zoom_result >= 2 && zoom_result <= 3 && std::isfinite(zoom_priority)) {
      *priority = zoom_priority;

      return zoom_result;
    }
    return result;
  }
  const auto detail_maximum = std::max(2, maximum - static_cast<int>(s.point_level_reduction.load(std::memory_order_relaxed)));
  if (level > detail_maximum)
    return result;

  s.terrain_requests.fetch_add(1, std::memory_order_relaxed);
  auto highest = s.terrain_level.load(std::memory_order_relaxed);
  while (highest < static_cast<unsigned>(level) && !s.terrain_level.compare_exchange_weak(highest, level, std::memory_order_relaxed)) {
  }
  return 2;
}

/*
 * Adjusts vegetation fade thresholds for eligible tiles intersecting the fresh AZP loading volume. Values
 * are rebuilt from current native settings and the weighted projection factor so repeated callbacks cannot
 * accumulate an ever-growing correction.
 *
 * The world owns this node and its vegetation resource during submission. Restrict writes to the level whose
 * native submission refreshes the fade; finer nodes can share the same resource without refreshing it
 * themselves.
 *
 * This is a copy of AZP's view, not a retained engine view or tile pointer.
 *
 * The node getter already returns double-precision world bounds. Unlike classifier bounds they must not pass
 * through the compact-bounds converter.
 *
 * Rebuild from the current native settings, never from previously enlarged thresholds. Keep the true
 * distance: individual tree LOD already uses FOV.
 */
void tile_submitted(void* world, void* node, unsigned char coarse) noexcept {
  auto& s = state();
  if (coarse || !s.enabled.load(std::memory_order_relaxed) || !s.ready.load(std::memory_order_acquire) || s.faulted.load())
    return;

  const auto* w = static_cast<const unsigned char*>(world);
  const auto* n = static_cast<const unsigned char*>(node);
  std::uintptr_t type = 0;
  int level = 0, selected_level = 0, quality = 0;
  std::memcpy(&type, w, sizeof(type));
  std::memcpy(&level, n, sizeof(level));
  std::memcpy(&selected_level, reinterpret_cast<const void*>(s.base + s.contract.vegetation_level), sizeof(selected_level));
  std::memcpy(&quality, reinterpret_cast<const void*>(s.base + s.contract.vegetation_quality), sizeof(quality));
  if (type != s.base + s.contract.world_vtable || level < 1 || level > 20 || level != selected_level || !w[0x46b] ||
      (w[0x480] && quality == -1))
    return;
  const unsigned char* tile = nullptr;
  unsigned char* vegetation = nullptr;
  std::memcpy(&tile, n + 0x1c0, sizeof(tile));
  std::memcpy(&vegetation, n + 0x1d0, sizeof(vegetation));
  if (!tile || !vegetation)
    return;
  int tile_kind = 0, resource_level = 0;
  std::memcpy(&tile_kind, tile + 0x188, sizeof(tile_kind));
  std::memcpy(&resource_level, vegetation + 0x10, sizeof(resource_level));
  if ((!w[0x482] && tile_kind != 1) || resource_level != level ||
      !reinterpret_cast<bool (*)(const void*)>(s.base + s.contract.node_ready)(node))
    return;

  alignas(16) static thread_local std::array<unsigned char, 0x16c8> frustum{};
  static thread_local TerrainView view;
  static thread_local std::uint64_t revision = 0;
  const auto current = s.terrain_revision.load(std::memory_order_acquire);
  if (current != revision) {
    std::lock_guard lock(s.terrain_mutex);
    view = s.terrain_view;
    revision = s.terrain_revision.load(std::memory_order_relaxed);
    std::memcpy(frustum.data() + 0x6c0, view.planes.data(), sizeof(view.planes));
    std::memcpy(frustum.data() + 0x9d0, &view.far_distance, sizeof(view.far_distance));
    const int count = 1, capacity = 4;
    std::memcpy(frustum.data() + 0x16c0, &count, sizeof(count));
    std::memcpy(frustum.data() + 0x16c4, &capacity, sizeof(capacity));
  }
  if (!current || view.index != s.view_index.load(std::memory_order_acquire) || GetTickCount64() - view.time_ms > 250 ||
      view.reference_fov <= view.fov || !get_world_preload_target().valid)
    return;
  std::uintptr_t criterion = 0;
  std::memcpy(&criterion, w + 0xa88, sizeof(criterion));
  if (!criterion)
    std::memcpy(&criterion, w + 0xa98, sizeof(criterion));
  if (criterion != view.resource)
    return;
  const auto projection_zoom = std::tan(view.reference_fov * 0.5) / std::tan(view.fov * 0.5);
  const auto zoom = 1.0 + view.detail_weight * (projection_zoom - 1.0);
  if (!std::isfinite(zoom) || zoom <= 1)
    return;
  const auto bounds = reinterpret_cast<const void* (*)(const void*)>(s.base + s.contract.node_bounds)(node);
  if (!bounds || !reinterpret_cast<bool (*)(const void*, const void*)>(s.base + s.contract.terrain_intersects)(frustum.data(), bounds))
    return;

  float distance = 0;
  std::memcpy(&distance, vegetation + 0x3e4, sizeof(distance));
  if (!std::isfinite(distance) || distance < 0)
    return;
  reinterpret_cast<void (*)(void*, float)>(s.base + s.contract.vegetation_update)(vegetation, distance);
  std::array<float, 4> fade{};
  std::memcpy(fade.data(), vegetation + 0x3e0, sizeof(fade));
  if (!std::all_of(fade.begin(), fade.end(), [](float x) { return std::isfinite(x); }) || fade[0] < 0 || fade[3] <= fade[0])
    return;
  const auto start = static_cast<float>(fade[0] * zoom), end = static_cast<float>(fade[3] * zoom);
  if (!std::isfinite(start) || !std::isfinite(end) || end <= start)
    return;
  fade[0] = start;
  fade[2] = std::max(fade[2], std::clamp((end - distance) / (end - start), 0.0f, 1.0f));
  fade[3] = end;
  float low = 0, high = 0;
  std::memcpy(&low, reinterpret_cast<const void*>(s.base + s.contract.vegetation_density_low), sizeof(low));
  std::memcpy(&high, reinterpret_cast<const void*>(s.base + s.contract.vegetation_density_high), sizeof(high));
  if (!std::isfinite(low) || !std::isfinite(high) || low < 0 || high < low || high > 1)
    return;
  const float density = fade[2] < low ? low : fade[2] < high ? high : 1.0f;
  std::memcpy(vegetation + 0x3e0, fade.data(), sizeof(fade));
  std::memcpy(vegetation + 0x33c, &density, sizeof(density));
}

/*
 * Checks the live streaming service, its world identity and registration groups within one bounded memory
 * transaction before the callback can update target demand.
 *
 * The receiver belongs to the running native update. Only copied fields survive this inspection.
 */
bool inspect(void* receiver,
             WeakWorld& world,
             bool& owned88,
             bool& owned78,
             unsigned char& flags,
             std::uintptr_t* terrain_resource = nullptr) noexcept {
  auto& s = state();
  const auto address = reinterpret_cast<std::uintptr_t>(receiver);
  ScopedLocalMemoryQueryCache query(LocalMemoryQueryMode::private_pages);
  LocalMemoryReader reader;
  std::array<unsigned char, 0xb0> header{}, again{};
  if (!reader.read(address, header.data(), header.size()))
    return false;
  std::uintptr_t vtable = 0;
  std::memcpy(&vtable, header.data(), sizeof(vtable));
  std::memcpy(world.data(), header.data() + 0x50, sizeof(world));
  if (vtable != s.base + s.contract.vtable || !live_world(reader, world))
    return false;
  flags = header[0xa8];
  owned88 = owned78 = false;
  std::uintptr_t terrain = 0;
  for (unsigned group = 0; group < 2; ++group) {
    const unsigned count_offset = group ? 0x8c : 0x6c, pointer_offset = group ? 0x90 : 0x70, stride = group ? 0x78 : 0x88;
    int count = 0;
    std::uintptr_t entries = 0;
    std::memcpy(&count, header.data() + count_offset, sizeof(count));
    std::memcpy(&entries, header.data() + pointer_offset, sizeof(entries));
    if (count < 0 || count > 128 || (count && (!entries || entries > UINTPTR_MAX - 128 * stride)))
      return false;
    for (int index = 0; index < count; ++index) {
      int owner_count = 0;
      std::uintptr_t owners = 0;
      const auto entry = entries + index * stride;
      if (!field(reader, entry, 0x5c, owner_count) || !field(reader, entry, 0x60, owners) || owner_count < 0 || owner_count > 256 ||
          (owner_count && (!owners || owners > UINTPTR_MAX - 256 * sizeof(std::uintptr_t))))
        return false;
      std::array<std::uintptr_t, 256> values{};
      if (owner_count && !reader.read(owners, values.data(), owner_count * sizeof(std::uintptr_t)))
        return false;
      const bool owned =
          std::find(values.begin(), values.begin() + owner_count, reinterpret_cast<std::uintptr_t>(&s)) != values.begin() + owner_count;
      (group ? owned78 : owned88) |= owned;
      if (!group && owned && owner_count == 1) {
        std::uintptr_t resource = 0, type = 0;
        unsigned char active = 0;
        if (!field(reader, entry, 0x68, active) || !field(reader, entry, 0x70, resource))
          return false;
        if (active && resource && field(reader, resource, 0, type) && type == s.base + s.contract.terrain_vtable)
          terrain = resource;
      }
    }
  }
  if (!reader.read(address, again.data(), again.size()) || again != header || !live_world(reader, world) || !query.finish())
    return false;
  if (terrain_resource)
    *terrain_resource = terrain;
  return true;
}

bool same_receiver(void* receiver, const WeakWorld& expected) noexcept {
  WeakWorld current{};
  bool a = false, b = false;
  unsigned char flags = 0;
  return inspect(receiver, current, a, b, flags) && current == expected;
}

/*
 * Rechecks the retained verified function-byte ranges before using native preload operations. Installation
 * supplies the full image/boundary proof; each call must still match its reviewed code.
 *
 * Installation proves the main image, function bounds and relocation-free ranges. Each call checks their
 * current bytes without walking the image's relocation directory again.
 */
bool valid_code() {
  auto& s = state();
  if (!s.contract.valid || s.contract.fingerprints.empty() || s.contract.fingerprints.size() > 32)
    return false;
  ScopedLocalMemoryQueryCache query(LocalMemoryQueryMode::private_pages);
  LocalImageReader reader(reinterpret_cast<HMODULE>(s.base), s.image.image_size, LocalImageQueryMode::pages);
  std::array<std::uint8_t, 8192> bytes{};
  std::uint32_t total = 0;
  for (const auto& expected : s.contract.fingerprints) {
    if (!expected.hash || !expected.size || expected.size > bytes.size() || expected.size > 65536 - total ||
        !reader.read(expected.rva, bytes.data(), expected.size))
      return false;
    total += expected.size;
    std::uint64_t hash = 14695981039346656037ull;
    for (std::uint32_t i = 0; i < expected.size; ++i) {
      hash ^= bytes[i];
      hash *= 1099511628211ull;
    }
    if (hash != expected.hash)
      return false;
  }
  return query.finish();
}

/*
 * Services target registration from the engine's streaming callback. It releases disabled/stale demand,
 * validates current target and code, then updates terrain/model groups only when needed and within the
 * update rate limit. Registrations remain bound to their verified world/service identities.
 *
 * Disabling still visits known services so their own callback removes the registration.
 *
 * Old identities may be evicted only after their weak world has expired.
 *
 * Each terrain subdivision halves the tile width. Round towards coarser extra admissions; existing
 * engine-selected detail is never demoted.
 *
 * Engine debug-string inline storage, zero length/flags outside the NUL-terminated label.
 *
 * Model LOD uses 1/tan(FOV/2). Scale the extra demand with the same budget as terrain and vegetation,
 * without changing the rendered FOV or target zone.
 */
void observe(void* receiver) noexcept {
  auto& s = state();
  if (!s.ready.load(std::memory_order_acquire) || s.observing.test_and_set(std::memory_order_acquire))
    return;
  struct Exit {
    bool refresh_status = false;
    ~Exit() {
      auto& value = state();
      if (refresh_status) {
        std::lock_guard lock(value.status_mutex);
        value.status.active =
            static_cast<std::uint32_t>(std::count_if(value.services.begin(), value.services.end(), [](const auto& x) { return x.active; }));
      }
      value.observing.clear(std::memory_order_release);
    }
  } exit;
  try {
    const auto now = GetTickCount64();
    const auto address = reinterpret_cast<std::uintptr_t>(receiver);
    auto tick = std::find_if(s.services.begin(), s.services.end(), [&](const auto& x) { return x.address == address; });
    if (tick != s.services.end() && now < tick->next_ms)
      return;
    if (!s.enabled.load() && tick == s.services.end())
      return;
    exit.refresh_status = true;
    if (now >= s.next_prune_ms) {
      s.next_prune_ms = now + 1000;
      ScopedLocalMemoryQueryCache query(LocalMemoryQueryMode::private_pages);
      LocalMemoryReader reader;
      std::array<bool, 8> expired{};
      for (std::size_t i = 0; i < s.services.size(); ++i)
        expired[i] = s.services[i].address && !live_world(reader, s.services[i].world);
      if (query.finish())
        for (std::size_t i = 0; i < s.services.size(); ++i)
          if (expired[i]) {
            s.terrain_resources[i].store(0, std::memory_order_release);
            s.services[i] = {};
          }
      tick = std::find_if(s.services.begin(), s.services.end(), [&](const auto& x) { return x.address == address; });
    }
    WeakWorld world{};
    bool owned88 = false, owned78 = false;
    unsigned char flags = 0;
    if (!inspect(receiver, world, owned88, owned78, flags)) {
      if (tick != s.services.end())
        s.terrain_resources[tick - s.services.begin()].store(0, std::memory_order_release);
      publish("Preload: service identity unavailable.");
      return;
    }
    if (tick == s.services.end()) {
      tick = std::find_if(s.services.begin(), s.services.end(), [](const auto& x) { return !x.address; });
      if (tick == s.services.end()) {
        ScopedLocalMemoryQueryCache query(LocalMemoryQueryMode::private_pages);
        LocalMemoryReader reader;
        tick = std::find_if(s.services.begin(), s.services.end(), [&](const auto& x) { return !live_world(reader, x.world); });
        if (!query.finish())
          tick = s.services.end();
      }
      if (tick == s.services.end()) {
        publish("Preload refused: too many active worlds.");
        return;
      }
      *tick = {};
      tick->address = address;
    }
    if (tick->world != world) {
      s.terrain_resources[tick - s.services.begin()].store(0, std::memory_order_release);
      *tick = {};
      tick->address = address;
      tick->world = world;
    }
    tick->last_seen = now;
    tick->next_ms = now + 1000;
    tick->active = owned88 || owned78;

    const auto target = get_world_preload_target();
    const auto camera = get_ima_camera();
    const auto fov = static_cast<float>(camera.fov * body_math::radians);
    const bool valid_fov = std::isfinite(fov) && fov > 0 && fov < body_math::pi;
    const auto base_area = valid_fov ? preload_area_weight(fov) : 0.0;
    bool wanted = s.enabled.load() && !s.faulted.load() && target.valid && camera.valid && camera.has_target && valid_fov && base_area > 0;
    WeakWorld selected{};
    if (wanted) {
      if (!valid_code()) {
        s.faulted.store(true);
        publish("Preload refused: engine code changed.");
        return;
      }
      const auto selected_address = reinterpret_cast<std::uintptr_t>(reinterpret_cast<CurrentWorld>(s.base + s.contract.current_world)());
      ScopedLocalMemoryQueryCache query(LocalMemoryQueryMode::private_pages);
      LocalMemoryReader reader;
      wanted = field(reader, selected_address, 0, selected) && live_world(reader, selected) && selected == world && query.finish();
    }
    if (wanted && (flags & 0x0e)) {
      wanted = false;
      publish("Preload: the engine is suspending independent requests.");
    }
    const auto density_scalar = s.density_scalar.load(std::memory_order_relaxed);
    const auto area_scalar = s.area_scalar.load(std::memory_order_relaxed);
    const auto effective_area = base_area * area_scalar;
    const auto detail_weight = wanted ? preload_detail_weight(fov) * density_scalar : 0.0;
    const auto object_detail_multiplier = 1.0 + detail_weight * (maximum_object_detail_multiplier - 1.0);
    const auto object_fov = static_cast<float>(2 * std::atan(std::tan(fov * 0.5) / object_detail_multiplier));
    if (wanted && (!std::isfinite(object_detail_multiplier) || !std::isfinite(object_fov) || object_fov <= 0)) {
      publish("Preload: density scalar exceeds the native projection range.");
      return;
    }
    if (wanted) {
      const auto reduction = std::clamp(std::ceil(-std::log2(detail_weight)), 0.0, 4.0);
      s.point_level_reduction.store(static_cast<unsigned>(reduction), std::memory_order_relaxed);
    }
    auto& terrain_resource = s.terrain_resources[tick - s.services.begin()];
    if (!wanted)
      terrain_resource.store(0, std::memory_order_release);
    std::array<double, 3> position{};
    if (wanted) {
      const double geographic[]{target.latitude, target.longitude, target.altitude_m};
      const int referential = 2;
      const unsigned object = 0;
      if (!same_receiver(receiver, world) ||
          !reinterpret_cast<ConvertPosition>(s.base + s.contract.convert)(position.data(), geographic, &referential, &object) ||
          !std::all_of(position.begin(), position.end(), [](double x) { return std::isfinite(x); }) ||
          std::hypot(position[0], position[1], position[2]) < 6300000 || std::hypot(position[0], position[1], position[2]) > 6500000) {
        s.faulted.store(true);
        publish("Preload refused: invalid WORLD conversion.");
        return;
      }
      const auto movement = std::hypot(position[0] - tick->position[0], position[1] - tick->position[1], position[2] - tick->position[2]);
      if (owned88 && owned78 && movement < 50 && tick->fov == fov && tick->density_scalar == density_scalar) {
        std::uintptr_t resource = 0;
        WeakWorld confirmed{};
        if (inspect(receiver, confirmed, owned88, owned78, flags, &resource) && confirmed == world)
          terrain_resource.store(resource, std::memory_order_release);
        else
          terrain_resource.store(0, std::memory_order_release);
        if (tick->area_scalar != area_scalar) {
          tick->area_scalar = area_scalar;
          char message[160]{};
          std::snprintf(message, sizeof(message),
                        "Target registered | density x%.3g | area x%.3g (effective x%.3g) | detail %.0f%% | objects x%.1f.", density_scalar,
                        area_scalar, effective_area, detail_weight * 100.0, object_detail_multiplier);
          publish(message);
        }
        return;
      }
    }
    terrain_resource.store(0, std::memory_order_release);
    if (owned88 || owned78) {
      if (!valid_code() || !same_receiver(receiver, world)) {
        s.faulted.store(true);
        publish("Preload: release refused because its context changed.");
        return;
      }
      reinterpret_cast<RemoveOwner>(s.base + s.contract.remove)(receiver, &s);
      WeakWorld after{};
      if (!inspect(receiver, after, owned88, owned78, flags) || after != world || owned88 || owned78) {
        s.faulted.store(true);
        publish("Preload: release could not be confirmed.");
        return;
      }
      tick->active = false;
      publish("Preload: request released.", -1);
    }
    if (!wanted) {
      if (s.enabled.load() && !target.valid)
        publish("Preload: waiting for a valid IMA target.");
      else if (s.enabled.load() && camera.valid && !valid_fov)
        publish("Preload: invalid camera FOV.");
      else if (s.enabled.load() && camera.valid && valid_fov && base_area == 0)
        publish("Preload: disabled at zoom 1.");
      return;
    }
    if (!valid_code() || !same_receiver(receiver, world)) {
      s.faulted.store(true);
      publish("Preload refused: context changed before registration.");
      return;
    }
    const double direction[]{0, 0, 1};
    std::array<char, 0x108> label{};
    std::memcpy(label.data(), "AZP Camera target preload", sizeof("AZP Camera target preload"));
    reinterpret_cast<AddZone>(s.base + s.contract.add88)(receiver, &s, position.data(), direction, fov, 0.01f, true, label.data());
    if (!same_receiver(receiver, world)) {
      s.faulted.store(true);
      publish("Preload: context changed after initial registration.");
      return;
    }
    reinterpret_cast<AddZone>(s.base + s.contract.add78)(receiver, &s, position.data(), direction, object_fov, 0.01f, true, label.data());
    WeakWorld after{};
    std::uintptr_t resource = 0;
    if (!inspect(receiver, after, owned88, owned78, flags, &resource) || after != world || !owned88 || !owned78) {
      s.faulted.store(true);
      publish("Preload: partial registration; release queued for the next update.");
      return;
    }
    tick->position = position;
    tick->fov = fov;
    tick->density_scalar = density_scalar;
    tick->area_scalar = area_scalar;
    tick->active = true;
    terrain_resource.store(resource, std::memory_order_release);
    char message[160]{};
    std::snprintf(message, sizeof(message),
                  "Target registered | density x%.3g | area x%.3g (effective x%.3g) | detail %.0f%% | objects x%.1f | terrain %s.",
                  density_scalar, area_scalar, effective_area, detail_weight * 100.0, object_detail_multiplier,
                  resource ? "enhanced" : "pending");
    publish(message, 1);
  } catch (...) {
    s.faulted.store(true);
    publish("Preload interrupted by an internal exception.");
  }
}
}  /* namespace */

void set_world_preload_view(int view_index) noexcept {
  state().view_index.store(view_index, std::memory_order_release);
}

void set_world_preload_scalars(double density, double area) noexcept {
  auto& s = state();
  s.density_scalar.store(std::isfinite(density) && density > 0 ? density : 1, std::memory_order_relaxed);
  s.area_scalar.store(std::isfinite(area) && area > 0 ? area : 1, std::memory_order_relaxed);
}

/*
 * Publishes preload demand and, on first use, installs the verified callbacks from the resolved engine
 * contract (engine/engine_api.cpp). Disabling invalidates the copied view immediately; each live service
 * callback subsequently removes its own registration.
 */
void set_world_preload_enabled(bool enabled) noexcept {
  auto& s = state();
  const bool previous = s.enabled.exchange(enabled);
  if (!enabled)
    for (auto& resource : s.terrain_resources)
      resource.store(0, std::memory_order_release);
  if (enabled && !previous) {
    s.terrain_requests.store(0, std::memory_order_relaxed);
    s.terrain_level.store(0, std::memory_order_relaxed);
    std::lock_guard lock(s.status_mutex);
  }
  if (s.installed.load(std::memory_order_acquire) && !s.ready.load(std::memory_order_acquire)) {
    const auto restored = engine_hook::restore_protection();
    if (restored.protection_restored)
      s.ready.store(true, std::memory_order_release);
  }
  if (!enabled || s.ready.load()) {
    if (enabled && !previous)
      s.faulted.store(false);
    return;
  }
  try {
    std::lock_guard lock(s.install_mutex);
    if (s.tried)
      return;
    s.tried = true;
    const auto& api = engine_api();
    if (!api.preload.valid) {
      publish(api.preload.error.c_str());
      return;
    }
    s.image = api.image;
    s.base = api.base;
    s.contract = api.preload;
    const auto terrain_slot = s.contract.terrain_vtable + 0x20;
    const auto terrain_section = std::find_if(s.image.sections.begin(), s.image.sections.end(), [&](const auto& x) {
      return x.rva <= terrain_slot && terrain_slot - x.rva < x.size && sizeof(void*) <= x.size - (terrain_slot - x.rva);
    });
    if (terrain_section == s.image.sections.end()) {
      publish("Preload refused: terrain vtable outside the engine image.");
      return;
    }
    const engine_hook::ImageDataSlotProof terrain_proof{reinterpret_cast<HMODULE>(s.base), s.image.image_size, terrain_section->rva,
                                                        terrain_section->size};
    const auto terrain_hook = engine_hook::install_terrain_filter(reinterpret_cast<void**>(s.base + terrain_slot),
                                                                  reinterpret_cast<void*>(s.base + s.contract.terrain_classify),
                                                                  classify_terrain, &terrain_proof);
    if (!terrain_hook.pointer_changed || !terrain_hook.protection_restored) {
      publish("Preload refused: terrain criterion installation failed.");
      return;
    }
    const auto update_slot = s.contract.terrain_vtable + 8;
    const auto terrain_update_hook =
        engine_hook::install_terrain_update(reinterpret_cast<void**>(s.base + update_slot),
                                            reinterpret_cast<void*>(s.base + s.contract.terrain_update), terrain_updated, &terrain_proof);
    if (!terrain_update_hook.pointer_changed || !terrain_update_hook.protection_restored) {
      publish("Preload refused: terrain zoom observer installation failed.");
      return;
    }
    const auto tile_slot = s.contract.world_vtable + 0x1a0;
    const auto tile_section = std::find_if(s.image.sections.begin(), s.image.sections.end(), [&](const auto& x) {
      return x.rva <= tile_slot && tile_slot - x.rva < x.size && sizeof(void*) <= x.size - (tile_slot - x.rva);
    });
    if (tile_section == s.image.sections.end()) {
      publish("Preload refused: world vtable outside the engine image.");
      return;
    }
    const engine_hook::ImageDataSlotProof tile_proof{reinterpret_cast<HMODULE>(s.base), s.image.image_size, tile_section->rva,
                                                     tile_section->size};
    const auto tile_hook =
        engine_hook::install_tile_submit(reinterpret_cast<void**>(s.base + tile_slot),
                                         reinterpret_cast<void*>(s.base + s.contract.tile_submit), tile_submitted, &tile_proof);
    if (!tile_hook.pointer_changed || !tile_hook.protection_restored) {
      publish("Preload refused: vegetation observer installation failed.");
      return;
    }
    const auto slot = s.contract.vtable + 0x78;
    const auto section = std::find_if(s.image.sections.begin(), s.image.sections.end(), [&](const auto& x) {
      return x.rva <= slot && slot - x.rva < x.size && sizeof(void*) <= x.size - (slot - x.rva);
    });
    if (section == s.image.sections.end()) {
      publish("Preload refused: vtable outside the engine image.");
      return;
    }
    const engine_hook::ImageDataSlotProof proof{reinterpret_cast<HMODULE>(s.base), s.image.image_size, section->rva, section->size};
    const auto hook = engine_hook::install_service_update(reinterpret_cast<void**>(s.base + slot),
                                                          reinterpret_cast<void*>(s.base + s.contract.update), observe, &proof);
    s.installed.store(hook.pointer_changed, std::memory_order_release);
    if (!hook.pointer_changed || !hook.protection_restored) {
      publish("Preload refused: engine observer installation failed.");
      return;
    }
    s.ready.store(true, std::memory_order_release);
    publish("Preload: observer ready, waiting for the target.");
  } catch (...) {
    publish("Preload refused: engine contract resolution failed.");
  }
}
WorldPreloadStatus world_preload_status() noexcept {
  auto& s = state();
  const std::lock_guard lock(s.status_mutex);
  return s.status;
}
}  /* namespace azp::native_camera */
