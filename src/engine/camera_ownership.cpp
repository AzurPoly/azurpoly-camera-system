/*
 * Serializes requests to create or disable the owned camera and tracks every returned native entry ID.
 * Creation/removal callbacks run only during an authorized engine update; uncertain cleanup retains
 * ownership so a live entry is never silently forgotten.
 */

#include "camera_ownership.hpp"
#include <algorithm>

namespace azp::engine_camera {
namespace {

struct ProcessingGuard {
  std::atomic_flag& flag;
  ~ProcessingGuard() { flag.clear(std::memory_order_release); }
};

bool zero_dword(const DescriptorStorage& descriptor, std::size_t offset) noexcept {
  return std::all_of(descriptor.bytes.begin() + offset, descriptor.bytes.begin() + offset + 4,
                     [](std::uint8_t value) { return value == 0; });
}

}  /* namespace */

/*
 * Packs the independent camera mode into a descriptor only after its inline-name fields match the expected
 * initialized layout. The attached scene node, not a built-in pose key, supplies placement.
 *
 * Verify the engine initializer's two empty inline names before packing mode 2. The opaque built-in pose key
 * is zero: our attached scene node owns placement.
 */
bool pack_independent_pose(DescriptorStorage& descriptor) noexcept {
  if (descriptor.bytes[0] != 0 || !zero_dword(descriptor, 32) || descriptor.bytes[64] != 0 || !zero_dword(descriptor, 192))
    return false;
  std::fill_n(descriptor.bytes.begin() + 40, 4, 0);
  descriptor.bytes[40] = 2;
  descriptor.bytes[44] = 1;
  std::fill_n(descriptor.bytes.begin() + 48, 16, 0);
  return true;
}

void CameraOwnership::request_enable() noexcept {
  const std::lock_guard lock(mailbox_mutex_);
  mailbox_ = {Command::enable};
}

void CameraOwnership::request_disable() noexcept {
  const std::lock_guard lock(mailbox_mutex_);
  mailbox_ = {Command::disable};
}

Snapshot CameraOwnership::snapshot() const noexcept {
  const std::lock_guard lock(mailbox_mutex_);
  auto result = published_;
  result.request_pending = mailbox_.command != Command::none;
  return result;
}

/*
 * Cancels queued creation only when no native ID has materialized and no update is in progress. It cannot
 * discard ownership of an allocated view.
 */
EmptyCameraCancel CameraOwnership::cancel_uncreated_request() noexcept {
  if (processing_.test_and_set(std::memory_order_acquire))
    return EmptyCameraCancel::busy;
  const ProcessingGuard guard{processing_};
  const std::lock_guard lock(mailbox_mutex_);
  if (owns_entry() || owner_ != ManagerToken{})
    return EmptyCameraCancel::owned;
  mailbox_ = {};
  desired_enabled_ = false;
  creation_pending_ = false;
  active_ = false;
  failure_ = Failure::none;
  published_ = {};
  return EmptyCameraCancel::cancelled;
}

void CameraOwnership::publish(State state, Blocked blocked) noexcept {
  const std::lock_guard lock(mailbox_mutex_);
  published_ = {state, failure_, blocked, owner_, id_, false, creation_pending_};
}

/*
 * Removes the owned entry and retains its ID when absence cannot be confirmed, so partial cleanup remains
 * visible to the next authorized update. A failed confirmation may still have removed it; a confirmed-absent
 * or zero ID is never erased again.
 */
void CameraOwnership::cleanup(const EngineCallbacks& engine) noexcept {
  active_ = false;
  if (id_ != 0 && engine.erase(engine.context, owner_, id_))
    id_ = 0;
  if (!owns_entry())
    owner_ = {};
}

/*
 * Initializes and validates the descriptor before creation. A failed native initializer or packing check
 * stops the creation sequence.
 */
bool CameraOwnership::prepare(const EngineCallbacks& engine, DescriptorStorage& descriptor) noexcept {
  if (!engine.initialize(engine.context, descriptor)) {
    failure_ = Failure::initializer_failed;
    return false;
  }
  if (!pack_independent_pose(descriptor)) {
    failure_ = Failure::initializer_contract;
    return false;
  }
  return true;
}

void CameraOwnership::creation_failed(const EngineCallbacks& engine, Failure failure) noexcept {
  failure_ = failure;
  if (owns_entry())
    cleanup(engine);
  publish(owns_entry() ? State::cleanup_pending : State::failed);
}

/*
 * Consumes one queued lifecycle request while excluding reentry. It validates the manager, cleans up prior
 * ownership when authorized, and retains every new ID even if a later creation stage fails.
 *
 * Re-enabling the active camera is idempotent.
 *
 * Consume once, before any callback can reenter.
 */
bool CameraOwnership::process_update(ManagerToken manager, const EngineCallbacks& engine) noexcept {
  if (processing_.test_and_set(std::memory_order_acquire))
    return false;
  const ProcessingGuard guard{processing_};
  Request request;
  {
    const std::lock_guard lock(mailbox_mutex_);
    request = mailbox_;
    mailbox_ = {};
  }
  if (request.command == Command::enable) {
    desired_enabled_ = true;
    creation_pending_ = true;
    failure_ = Failure::none;
    if (active_ && owns_entry())
      creation_pending_ = false;
  } else if (request.command == Command::disable) {
    desired_enabled_ = false;
    creation_pending_ = false;
    failure_ = Failure::none;
  }

  if (!owns_entry() && !creation_pending_) {
    publish(failure_ == Failure::none ? State::disabled : State::failed);
    return true;
  }
  if (!manager.valid()) {
    publish(State::blocked, Blocked::invalid_manager);
    return true;
  }
  if (owns_entry() && manager != owner_) {
    publish(State::blocked, Blocked::manager_mismatch);
    return true;
  }
  if (engine.initialize == nullptr || engine.create == nullptr || engine.erase == nullptr) {
    publish(State::blocked, Blocked::missing_callbacks);
    return true;
  }
  if (owns_entry()) {
    if (desired_enabled_ && active_ && !creation_pending_ && failure_ == Failure::none && owns_entry()) {
      publish(State::active);
      return true;
    }
    cleanup(engine);
    if (owns_entry()) {
      publish(State::cleanup_pending);
      return true;
    }
  }
  if (!desired_enabled_ || !creation_pending_) {
    publish(failure_ == Failure::none ? State::disabled : State::failed);
    return true;
  }

  creation_pending_ = false;
  DescriptorStorage descriptor{};
  if (!prepare(engine, descriptor)) {
    creation_failed(engine, failure_);
    return true;
  }
  const auto first_id = engine.create(engine.context, manager, descriptor);
  if (first_id == 0) {
    creation_failed(engine, Failure::first_create_failed);
    return true;
  }
  owner_ = manager;
  id_ = first_id;
  active_ = true;
  publish(State::active);
  return true;
}

/*
 * Forgets owned entries only when the caller confirms that their manager lifetime ended. The acknowledgement
 * must match the retained manager identity and cannot race an active update.
 */
bool CameraOwnership::acknowledge_manager_destroyed(ManagerToken manager) noexcept {
  if (processing_.test_and_set(std::memory_order_acquire))
    return false;
  const ProcessingGuard guard{processing_};
  if (!owns_entry() || manager != owner_)
    return false;
  id_ = 0;
  owner_ = {};
  desired_enabled_ = false;
  creation_pending_ = false;
  active_ = false;
  failure_ = Failure::manager_destroyed;
  {
    const std::lock_guard lock(mailbox_mutex_);
    mailbox_ = {};
  }
  publish(State::failed);
  return true;
}

}  /* namespace azp::engine_camera */
