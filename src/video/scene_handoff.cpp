/*
 * Matches the camera-published final-image identity to a live D3D12 resource. Session epochs and resource
 * generations invalidate stale publications, ensuring that capture follows this camera's current image
 * rather than a reused address.
 */

#include "scene_handoff.hpp"
#include <limits>
#include <new>

namespace azp {

void SceneHandoff::clear_handles() noexcept {
  candidate_handle_.store(0, std::memory_order_release);
  observed_handle_.store(0, std::memory_order_release);
}

/*
 * Advances the handoff generation and invalidates dependent identities on overflow instead of letting an old
 * token become current again.
 */
bool SceneHandoff::advance(std::uint64_t& counter) {
  if (exhausted_ || counter == std::numeric_limits<std::uint64_t>::max()) {
    exhausted_ = true;
    active_ = false;
    publication_ = {};
    clear_handles();
    return false;
  }
  ++counter;
  return true;
}

/*
 * Invalidates in-progress inspection tickets when native resource lifetime changes, while preserving the
 * already published identity where allowed.
 *
 * Unrelated uploads/streaming allocations cannot change the identity of our already registered resource.
 * Still invalidate every open inspection ticket so an address freed/reused while its graph is being read is
 * refused.
 */
bool SceneHandoff::lifecycle_event(std::uint64_t changed_handle) {
  if (!changed_handle || observed(changed_handle)) {
    clear_handles();
    publication_ = {};
  }
  return advance(lifecycle_epoch_);
}

/*
 * Registers the simulator's device and resets dependent publications: resources seen before it cannot
 * satisfy capture tickets.
 */
bool SceneHandoff::register_device() {
  const std::lock_guard lock(mutex_);
  if (!lifecycle_event())
    return false;
  resources_.clear();
  device_registered_ = true;
  return true;
}

/*
 * Registers a resource incarnation. Duplicate/reused addresses revoke prior incarnation evidence before
 * becoming candidates again.
 *
 * A duplicate registration must not leave the previous incarnation usable.
 */
bool SceneHandoff::register_resource(std::uint64_t handle, std::uint64_t id) {
  const std::lock_guard lock(mutex_);
  if (!lifecycle_event(handle))
    return false;
  if (!device_registered_ || handle == 0)
    return false;
  if (id == 0) {
    resources_.erase(handle);
    return false;
  }
  const auto existing = resources_.find(handle);
  if (existing == resources_.end() && resources_.size() >= maximum_resources)
    return false;
  if (!advance(resource_counter_))
    return false;
  try {
    resources_.insert_or_assign(handle, Resource{id, resource_counter_});
  } catch (const std::bad_alloc&) {
    resources_.erase(handle);
    return false;
  }
  return true;
}

void SceneHandoff::unregister_resource(std::uint64_t handle) {
  const std::lock_guard lock(mutex_);
  lifecycle_event(handle);
  resources_.erase(handle);
}

/*
 * Starts an authorized scene epoch bound to the expected manager and owned entry, revoking earlier image
 * publications.
 */
std::uint64_t SceneHandoff::begin_scene() {
  const std::lock_guard lock(mutex_);
  clear_handles();
  publication_ = {};
  owner_bound_ = false;
  owner_ = {};
  owned_entry_ = 0;
  active_ = advance(scene_epoch_);
  return active_ ? scene_epoch_ : 0;
}

/*
 * Withdraws scene publication and advances identity state so old tickets cannot deliver pixels after a stop.
 */
void SceneHandoff::stop_scene() {
  const std::lock_guard lock(mutex_);
  clear_handles();
  active_ = false;
  owner_bound_ = false;
  publication_ = {};
  advance(scene_epoch_);
}

/*
 * A refresh is not a resource/owner lifecycle event. Keep the last complete publication usable while the new
 * ticket proves its own independently checked graph.
 */
SceneCaptureTicket SceneHandoff::begin_capture() {
  const std::lock_guard lock(mutex_);
  if (!active_ || !advance(capture_sequence_))
    return {};
  return {scene_epoch_, lifecycle_epoch_, capture_sequence_};
}

/*
 * Revokes the current output identity without asserting that the native camera was destroyed.
 */
void SceneHandoff::invalidate_publication() {
  const std::lock_guard lock(mutex_);
  publication_ = {};
  clear_handles();
  advance(capture_sequence_);
  minimum_capture_sequence_ = capture_sequence_;
}

/*
 * Accepts a completed camera inspection only if its scene epoch, ownership, ticket and resource incarnation
 * remain current. Older concurrent completion cannot overwrite or revoke a newer accepted publication.
 *
 * A new manager/entry requires a new authorized scene epoch.
 */
bool SceneHandoff::publish(SceneCaptureTicket ticket,
                           SceneManagerIdentity manager,
                           std::uint64_t entry_id,
                           std::uint64_t handle,
                           bool night_vision) {
  const std::lock_guard lock(mutex_);
  if (!active_ || exhausted_ || ticket.scene_epoch != scene_epoch_ || ticket.lifecycle_epoch != lifecycle_epoch_ || ticket.sequence == 0 ||
      ticket.sequence != capture_sequence_ || (publication_.valid && publication_.sequence == ticket.sequence))
    return false;
  const auto refuse_latest = [&] {
    publication_ = {};
    candidate_handle_.store(0, std::memory_order_release);
    return false;
  };
  if (manager.identity == 0 || manager.generation == 0 || !entry_id || !handle)
    return refuse_latest();
  if (owner_bound_ && (owner_ != manager || owned_entry_ != entry_id)) {
    active_ = false;
    clear_handles();
    advance(scene_epoch_);
    return refuse_latest();
  }
  const auto found = resources_.find(handle);
  if (!device_registered_ || found == resources_.end())
    return refuse_latest();
  owner_bound_ = true;
  owner_ = manager;
  owned_entry_ = entry_id;
  publication_ = {true, night_vision, scene_epoch_, ticket.sequence, manager, entry_id, handle, {found->second.id, found->second.generation}};
  candidate_handle_.store(handle, std::memory_order_release);
  observed_handle_.store(handle, std::memory_order_release);
  return true;
}

/*
 * Matches the published output to a live device/resource incarnation and returns its evidence only when the
 * whole identity agrees.
 */
SceneCopyMatch SceneHandoff::match(std::uint64_t handle) const {
  if (!active_ || exhausted_ || !publication_.valid || publication_.scene_epoch != scene_epoch_ || !device_registered_ || handle == 0 ||
      handle != publication_.handle)
    return {};
  const auto resource = resources_.find(handle);
  if (resource == resources_.end())
    return {};
  const SceneResourceIdentity identity{resource->second.id, resource->second.generation};
  if (publication_.resource != identity)
    return {};
  return {true, scene_epoch_, publication_.sequence, publication_.manager, publication_.entry_id, identity, publication_.night_vision};
}

SceneCopyMatch SceneHandoff::observe(std::uint64_t handle) const {
  if (!handle || candidate_handle_.load(std::memory_order_acquire) != handle)
    return {};
  const std::lock_guard lock(mutex_);
  return match(handle);
}

bool SceneHandoff::observed(std::uint64_t handle) const noexcept {
  return handle && observed_handle_.load(std::memory_order_acquire) == handle;
}

/*
 * Rechecks a captured handoff ticket against current scene/resource generations before its pixels are
 * consumed.
 */
bool SceneHandoff::is_current(const SceneCopyMatch& value) const {
  const std::lock_guard lock(mutex_);
  if (!value.matched || !publication_.valid)
    return false;
  const auto current = match(publication_.handle);
  return current.matched && value.scene_epoch == current.scene_epoch && value.manager == current.manager &&
         value.entry_id == current.entry_id && value.resource == current.resource && value.night_vision == current.night_vision &&
         value.capture_sequence != 0 && value.capture_sequence >= minimum_capture_sequence_ &&
         value.capture_sequence <= current.capture_sequence;
}

SceneHandoff& scene_handoff() {
  static auto* const instance = new SceneHandoff;
  return *instance;
}

}  /* namespace azp */
