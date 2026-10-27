/*
 * Attaches the camera node to the aircraft scene hierarchy so its local mount moves with the rendered
 * aircraft. A native update callback reapplies the WASM look direction using the current aircraft transform,
 * avoiding positional lag from asynchronous telemetry.
 */

#include "camera_attachment.hpp"
#include <algorithm>
#include <atomic>
#include <mutex>
#include "../engine/memory.hpp"
#include "../engine/observer_hook.hpp"
#include "simconnect_link.hpp"
#include "zoom_detail.hpp"

namespace azp::native_camera {
namespace {
using NodeHandle = std::array<std::uint64_t, 2>;
struct Attachment {
  std::uint64_t camera = 0, node = 0, parent = 0;
  NodeHandle node_handle{}, parent_handle{}, world_handle{};
  Vector3 mount_m{};
  bool offset_valid = false;
};
struct AttachmentRuntime {
  std::mutex install_mutex, mutex;
  std::atomic<std::uint64_t> selected_camera{0};
  Attachment current;
  std::uint64_t base = 0;
  CameraContract camera;
  CameraAttachmentContract contract;
  bool installed = false, ready = false;
};
/*
 * The engine retains the hook until process exit; the module is pinned.
 */
AttachmentRuntime& state() {
  static auto* value = new AttachmentRuntime;
  return *value;
}
template <typename T>
bool read(LocalMemoryReader& reader, std::uint64_t node, unsigned offset, T& value) {
  return node && node <= UINT64_MAX - offset - sizeof(T) && reader.read(node + offset, &value, sizeof(value));
}
std::uint64_t resolve_node(LocalMemoryReader& reader, const NodeHandle& handle) {
  auto& s = state();
  std::uint32_t generation = 0;
  std::uint64_t node = 0, vtable = 0;
  if (!read(reader, handle[0], 28, generation) || generation != static_cast<std::uint32_t>(handle[1]) ||
      !read(reader, handle[0], 0, node) || !read(reader, node, 0, vtable) || vtable != s.base + s.camera.layout.scene_node_vtable)
    return 0;
  return node;
}
bool node_handle(LocalMemoryReader& reader, std::uint64_t node, NodeHandle& handle) {
  return read(reader, node, 16, handle) && resolve_node(reader, handle) == node;
}
/*
 * Checks the recorded child/parent link against the current native node fields. A matching address without
 * the expected link is insufficient for restoration.
 *
 * Verify both ends of the sibling links before asking the engine to change them.
 */
bool linked_node(LocalMemoryReader& reader, std::uint64_t node, std::uint64_t parent) {
  std::uint64_t actual = 0, next = 0, previous = 0, link = 0;
  if (!read(reader, node, 368, actual) || actual != parent || !read(reader, node, 280, next) || !read(reader, node, 376, previous))
    return false;
  if (previous) {
    if (!read(reader, previous, 280, link) || link != node || !read(reader, previous, 368, link) || link != parent)
      return false;
  } else if (parent && (!read(reader, parent, 272, link) || link != node)) {
    return false;
  }
  return !next || (read(reader, next, 376, link) && link == node && read(reader, next, 368, link) && link == parent);
}
/*
 * Restores the saved parent only while the camera is still linked exactly as AZP attached it. Engine
 * teardown or independent reparenting invalidates that restoration authority.
 *
 * World/aircraft teardown can already have removed this link. Do not restore an old world after the engine
 * has reparented the node itself.
 *
 * Caller holds the attachment mutex and supplies the freshly owned camera node.
 */
bool restore_parent(const engine_camera::OwnedViewSnapshot& view, std::string& error) {
  auto& s = state();
  if (!s.current.node)
    return true;
  if (view.node_address != s.current.node || view.camera_address != s.current.camera) {
    error = "Attached camera identity differs from the owned view.";
    return false;
  }
  LocalMemoryReader reader;
  if (resolve_node(reader, s.current.node_handle) != view.node_address) {
    error = "Attached camera generation changed.";
    return false;
  }
  std::uint64_t parent = 0;
  if (!read(reader, view.node_address, 368, parent) || !linked_node(reader, view.node_address, parent)) {
    error = "Attached camera links are unavailable.";
    return false;
  }
  if (parent == s.current.parent) {
    if (resolve_node(reader, s.current.parent_handle) != parent) {
      error = "Aircraft parent generation changed; camera remains closed.";
      return false;
    }
    const auto world = resolve_node(reader, s.current.world_handle);
    if (!world) {
      error = "Original world parent unavailable; camera remains closed.";
      return false;
    }
    s.selected_camera.store(0, std::memory_order_release);
    reinterpret_cast<void (*)(void*, bool, bool)>(s.base + s.contract.detach)(reinterpret_cast<void*>(view.node_address), false, true);
    reinterpret_cast<void (*)(void*, const NodeHandle*, bool)>(s.base + s.contract.attach)(reinterpret_cast<void*>(world),
                                                                                           &s.current.node_handle, true);
    reader.reset_budget();
    if (!linked_node(reader, view.node_address, world)) {
      error = "Camera world-parent restoration did not complete.";
      return false;
    }
  }
  s.selected_camera.store(0, std::memory_order_release);
  s.current = {};
  return true;
}

/*
 * During the native node traversal, reads the current camera/aircraft ancestry and fresh WASM guidance. It
 * calculates the look direction from that same live transform before the original camera calculation,
 * keeping translation rigidly attached.
 *
 * These nodes are arguments/ancestors of this native traversal, not a pose saved by the asynchronous
 * telemetry worker or a previous manager tick.
 *
 * The observer closes the render gate while publication is unavailable. Do not substitute a forward aim in
 * the gap before that observer runs.
 *
 * The original calculation immediately reads this same node matrix and subtracts its position. Aircraft
 * translation cannot leave the aim behind.
 *
 * The same node position and look direction, with the WASM FOV, are published for zoom detail: they are what
 * the engine's LOD viewpoint for this camera holds.
 *
 * The assembly thunk still forwards the native invocation exactly once.
 */
void camera_updating(void* camera, void* node) noexcept {
  auto& s = state();
  if (s.selected_camera.load(std::memory_order_acquire) != reinterpret_cast<std::uint64_t>(camera))
    return;
  try {
    Attachment current;
    {
      const std::unique_lock lock(s.mutex, std::try_to_lock);
      if (!lock.owns_lock() || s.current.camera != reinterpret_cast<std::uint64_t>(camera) ||
          s.current.node != reinterpret_cast<std::uint64_t>(node))
        return;
      current = s.current;
    }
    LocalMemoryReader reader;
    ScopedLocalMemoryQueryCache queries(LocalMemoryQueryMode::private_pages);
    std::uint64_t parent = 0, attached = 0, camera_vtable = 0, matrix = 0, parent_matrix = 0;
    if (resolve_node(reader, current.node_handle) != current.node || resolve_node(reader, current.parent_handle) != current.parent ||
        !read(reader, current.node, 368, parent) || parent != current.parent || !read(reader, current.node, 256, attached) ||
        attached != current.camera || !read(reader, current.camera, 0, camera_vtable) || camera_vtable != s.base + s.contract.vtable ||
        !read(reader, current.node, 296, matrix) || !read(reader, current.parent, 296, parent_matrix))
      return;
    Vector3 position{};
    std::array<Vector3, 4> rows{}, again{};
    if (!read(reader, matrix, 96, position))
      return;
    for (unsigned i = 0; i < rows.size(); ++i)
      if (!read(reader, parent_matrix, i * 32, rows[i]))
        return;
    for (unsigned i = 0; i < rows.size(); ++i)
      if (!read(reader, parent_matrix, i * 32, again[i]) || rows[i] != again[i])
        return;
    BodyPose body{{rows[3][0], rows[3][1], rows[3][2]},
                  {-rows[0][0], -rows[0][1], -rows[0][2]},
                  {rows[1][0], rows[1][1], rows[1][2]},
                  {rows[2][0], rows[2][1], rows[2][2]}};
    if (!valid_body_pose(body) || !mount_detail::finite(position) || !queries.finish())
      return;
    const auto ima = get_ima_camera();
    Vector3 direction = body.forward, up = body.up;
    if (!ima.valid || !ima.local_mount)
      return;
    if (ima.has_target) {
      Vector3 location{};
      if (!body_math::geodetic(body.origin, location))
        return;
      const auto aim = body_math::body({location[0], location[1], location[2], ima.pitch, ima.bank, ima.heading}, 0);
      if (!valid_body_pose(aim))
        return;
      direction = aim.forward;
      up = aim.up;
    }
    Vector3 target{};
    for (unsigned i = 0; i < 3; ++i)
      target[i] = position[i] + direction[i];
    publish_zoom_detail_view(position, direction, ima.fov * body_math::radians);
    using SetVector = void (*)(void*, const double*);
    reinterpret_cast<SetVector>(s.base + s.camera.functions.set_up)(camera, up.data());
    reinterpret_cast<SetVector>(s.base + s.camera.functions.set_target)(camera, target.data());
  } catch (...) {
  }
}
}  /* namespace */

/*
 * Resolves the attachment contract, verifies the hook slot and pins the module before installing the update
 * callback. A partially installed hook retains protection-repair obligations.
 */
bool initialize_camera_attachment(const CameraAttachmentContract& contract,
                                  const discovery::Inventory& image,
                                  std::uint64_t base,
                                  const CameraContract& camera,
                                  std::string& error) {
  auto& s = state();
  const std::lock_guard lock(s.install_mutex);
  if (s.installed) {
    s.ready = engine_hook::restore_protection().protection_restored;
    if (!s.ready)
      error = "Camera update hook page protection is not restored.";
    return s.ready;
  }
  if (!contract.valid) {
    error = contract.error;
    return false;
  }
  const auto slot = contract.vtable + 216;
  const auto section = std::find_if(image.sections.begin(), image.sections.end(), [slot](const auto& item) {
    return (item.flags & 0x40000000u) && !(item.flags & 0xa2000000u) && slot >= item.rva && slot - item.rva <= item.size &&
           sizeof(void*) <= item.size - (slot - item.rva);
  });
  if (section == image.sections.end()) {
    error = "Camera update slot is outside read-only image data.";
    return false;
  }
  HMODULE pinned = nullptr;
  if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN,
                          reinterpret_cast<LPCWSTR>(&camera_updating), &pinned)) {
    error = "Camera attachment module pin failed.";
    return false;
  }
  s.base = base;
  s.camera = camera;
  s.contract = contract;
  const engine_hook::ImageDataSlotProof proof{reinterpret_cast<HMODULE>(base), image.image_size, section->rva, section->size};
  const auto hook = engine_hook::install_camera_update(reinterpret_cast<void**>(base + slot),
                                                       reinterpret_cast<void*>(base + contract.update), camera_updating, &proof);
  s.installed = hook.pointer_changed;
  s.ready = s.installed && hook.protection_restored;
  if (!s.ready)
    error = std::string("Camera update hook: ") + engine_hook::status_name(hook.status);
  return s.ready;
}

/*
 * Validates the current nodes, restores any prior owned attachment, then establishes the aircraft parent and
 * local mount. Local position is rewritten only when the mount changes; the hierarchy carries ordinary
 * aircraft motion.
 *
 * Only a mount change writes position. Thereafter the aircraft hierarchy transports this offset; it is never
 * chased with sampled world positions.
 */
bool prepare_camera_attachment(const engine_camera::OwnedViewSnapshot& view,
                               const AircraftScenePose& aircraft,
                               const ImaCameraSample& ima,
                               bool& attached,
                               std::string& error) {
  attached = false;
  auto& s = state();
  const std::lock_guard lock(s.mutex);
  if (!ima.valid || !ima.local_mount) {
    error = "Waiting for a valid WASM mount.";
    return false;
  }
  if (!s.ready || !view.ready || !aircraft.complete || !aircraft.node || !mount_detail::finite(ima.mount_m)) {
    error = "Native attachment is not ready.";
    return false;
  }
  LocalMemoryReader reader;
  std::uint64_t current_parent = 0;
  if (s.current.node && (s.current.node != view.node_address || s.current.parent != aircraft.node ||
                         !read(reader, view.node_address, 368, current_parent) || current_parent != s.current.parent)) {
    if (!restore_parent(view, error))
      return false;
    reader.reset_budget();
  }
  if (s.current.node) {
    if (resolve_node(reader, s.current.node_handle) != view.node_address ||
        resolve_node(reader, s.current.parent_handle) != aircraft.node || !linked_node(reader, view.node_address, aircraft.node)) {
      error = "Native attachment identity or parent changed.";
      return false;
    }
    if (s.current.offset_valid && s.current.mount_m == ima.mount_m) {
      attached = true;
      return true;
    }
  } else {
    Attachment next;
    next.node = view.node_address;
    next.camera = view.camera_address;
    next.parent = aircraft.node;
    std::uint64_t old_parent = 0, child = 0, vtable = 0;
    if (!node_handle(reader, next.node, next.node_handle) || !node_handle(reader, next.parent, next.parent_handle) ||
        !read(reader, next.node, 368, old_parent) || !old_parent || old_parent == next.parent ||
        !node_handle(reader, old_parent, next.world_handle) || !linked_node(reader, next.node, old_parent) ||
        !read(reader, next.node, 272, child) || child || !read(reader, next.camera, 0, vtable) || vtable != s.base + s.contract.vtable) {
      error = "Native attachment requires an owned leaf camera and valid world/aircraft parents.";
      return false;
    }
    s.current = next;
    reinterpret_cast<void (*)(void*, bool, bool)>(s.base + s.contract.detach)(reinterpret_cast<void*>(next.node), false, true);
    reinterpret_cast<void (*)(void*, const NodeHandle*, bool)>(s.base + s.contract.attach)(reinterpret_cast<void*>(next.parent),
                                                                                           &next.node_handle, true);
    reader.reset_budget();
    if (!linked_node(reader, next.node, next.parent)) {
      error = "Native attachment did not establish the requested parent.";
      return false;
    }
  }
  auto position = mount_detail::rotate(aircraft.pose, ima.mount_m);
  for (unsigned i = 0; i < 3; ++i)
    position[i] += aircraft.pose.origin[i];
  reinterpret_cast<void (*)(void*, const double*)>(s.base + s.camera.functions.set_position)(reinterpret_cast<void*>(view.node_address),
                                                                                             position.data());
  reader.reset_budget();
  Vector3 local{};
  if (!read(reader, view.node_address, 112, local) || !mount_detail::finite(local) || std::abs(local[0] + ima.mount_m[0]) > 0.01 ||
      std::abs(local[1] - ima.mount_m[1]) > 0.01 || std::abs(local[2] - ima.mount_m[2]) > 0.01) {
    error = "Native camera local offset did not match the WASM mount.";
    return false;
  }
  s.current.mount_m = ima.mount_m;
  s.current.offset_valid = true;
  s.selected_camera.store(view.camera_address, std::memory_order_release);
  attached = true;
  return true;
}

void forget_camera_attachment() {
  auto& s = state();
  const std::lock_guard lock(s.mutex);
  s.selected_camera.store(0, std::memory_order_release);
  s.current = {};
}

bool release_camera_attachment(const engine_camera::OwnedViewSnapshot& view, std::string& error) {
  auto& s = state();
  const std::lock_guard lock(s.mutex);
  return restore_parent(view, error);
}
}  /* namespace azp::native_camera */
