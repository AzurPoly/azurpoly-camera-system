/*
 * Implements the camera lifecycle decisions: the phase of each update and the trackers for session changes,
 * temporary loss, resize recovery and retirement. Their decisions distinguish waiting from confirmed
 * destruction, so a transient failure cannot erase a live native camera or repeat an uncertain allocation.
 */

#include "lifecycle.hpp"

namespace azp::native_camera {
bool SessionTransition::session_changed(const Ownership& ownership, std::uint64_t started, std::uint64_t current) noexcept {
  return ownership.owned_id && started != current;
}

/*
 * Starts a transition bound to the current manager and owned entry. Invalid ownership cannot become a
 * resumable transition; no owned entry is immediately ready.
 */
void SessionTransition::begin(const Ownership& ownership, const ViewDimensions& dimensions) noexcept {
  awaiting_ownership_ = false;
  holding_ = true;
  owner_ = ownership.owner;
  id_ = ownership.owned_id;
  dimensions_ = dimensions;
  state_ = Decision::wait;
  if (ownership.request_pending || ownership.creation_pending)
    state_ = Decision::refused;
  else if (!id_)
    state_ = Decision::ready;
  else if (!valid_ownership(ownership))
    state_ = Decision::refused;
}

void SessionTransition::defer_ownership() noexcept {
  holding_ = true;
  owner_ = {};
  id_ = 0;
  dimensions_ = {};
  awaiting_ownership_ = true;
  state_ = Decision::wait;
}

void SessionTransition::begin_published(const Ownership& ownership, const AllocationEvidence& evidence) noexcept {
  if (valid_ownership(ownership) && !evidence.matches(ownership))
    defer_ownership();
  else
    begin(ownership, evidence.dimensions);
}

/*
 * Compares a fresh inspection with the retained identity and expected allocation. Only a complete match can
 * advance the transition toward readiness; an open gate must be closed first.
 */
SessionTransition::Decision SessionTransition::inspect(engine_camera::ManagerToken manager,
                                                       const Ownership& ownership,
                                                       const engine_camera::OwnedViewSnapshot& view) noexcept {
  if (state_ == Decision::refused)
    return state_;
  if (awaiting_ownership_)
    return Decision::wait;
  if (!matches(ownership) || manager != owner_)
    return state_ = Decision::refused;
  if (!view.complete || !view.ready || view.status == engine_camera::OwnedViewStatus::pending)
    return state_ = Decision::wait;
  if (view.status != engine_camera::OwnedViewStatus::ready || view.mode != 2 || !view.resource_present || !view.resource_address ||
      !view.view_address || !view.node_address || !view.camera_address || view.view_index < 0 || view.view_index >= 8 ||
      view.dimensions != dimensions_ || view.output_dimensions != dimensions_[0])
    return state_ = Decision::refused;
  const bool open = (view.flags[0] & 1u) == 0;
  return state_ = open ? Decision::close : Decision::ready;
}

bool SessionTransition::matches(const Ownership& ownership) const noexcept {
  return valid_ownership(ownership) && ownership.owner == owner_ && ownership.owned_id == id_;
}

void SessionTransition::refuse() noexcept {
  awaiting_ownership_ = false;
  state_ = Decision::refused;
}

void SessionTransition::consume() noexcept {
  holding_ = false;
  awaiting_ownership_ = false;
}

bool SessionTransition::valid_ownership(const Ownership& ownership) noexcept {
  return ownership.state == engine_camera::State::active && ownership.owner.valid() && ownership.owned_id && !ownership.request_pending &&
         !ownership.creation_pending && ownership.failure == engine_camera::Failure::none &&
         ownership.blocked == engine_camera::Blocked::none;
}

/*
 * Dimension drift uses the retained-camera path. It must never trigger erase and recreation of camera entries
 * that the renderer may still reference.
 */
bool retryable_stop(StopReason reason) noexcept {
  return reason == StopReason::inspection_unavailable || reason == StopReason::owned_entry_absent;
}

/*
 * A failed native creation/initial resize may have partially allocated output. Later inspection failures
 * must neither replace that cause nor rearm retry.
 */
bool latched_stop(StopReason reason) noexcept {
  return reason == StopReason::identity_refused || reason == StopReason::creation_failed;
}

bool temporary_pose_unavailable(const char* reason) noexcept {
  return reason && (!std::strcmp(reason, "telemetry_busy") || !std::strcmp(reason, "aircraft_telemetry_stale") ||
                    !std::strcmp(reason, "camera_telemetry_stale") || !std::strcmp(reason, "not_initialized") ||
                    !std::strcmp(reason, "aircraft_session_changed"));
}

void CameraRecovery::start() noexcept {
  requested_ = true;
  pending_ = false;
  attempts_ = 0;
  reason_ = StopReason::none;
  healthy_since_ = last_progress_ = 0;
}

void CameraRecovery::stop() noexcept {
  requested_ = false;
  pending_ = false;
  reason_ = StopReason::explicit_stop;
  ++sequence_;
}

/*
 * Starts or updates one recovery episode while preserving a latched cause and its original retry deadline.
 * Repeated unavailable samples do not postpone recovery indefinitely.
 *
 * Only explicit Start/Stop transitions can clear a latched refusal. Preserve its original reason even when a
 * later failure is independently terminal.
 *
 * A pending failure is one recovery episode. Repeated unavailable samples may report a different
 * field/region, but must not keep postponing its original retry deadline. A fresh guarded update still owns
 * every retry.
 */
void CameraRecovery::failed(StopReason reason, std::uint64_t now) noexcept {
  if (latched_stop(reason_) && reason != reason_)
    return;
  if (pending_ && reason_ == reason)
    return;
  reason_ = reason;
  stopped_at_ = now;
  healthy_since_ = last_progress_ = 0;
  pending_ = requested_ && retryable_stop(reason) && attempts_ < maximum_retries;
  ++sequence_;
}

bool CameraRecovery::retry(std::uint64_t now, const engine_camera::Snapshot& ownership, bool fresh_pose) noexcept {
  if (!requested_ || !pending_ || !fresh_pose || now < stopped_at_ || now - stopped_at_ < retry_delay_ms || ownership.owned_id ||
      ownership.request_pending || ownership.creation_pending || ownership.state != engine_camera::State::disabled ||
      ownership.failure != engine_camera::Failure::none || ownership.blocked != engine_camera::Blocked::none)
    return false;
  pending_ = false;
  ++attempts_;
  return true;
}

/*
 * Bump the control-loop snapshot so "Camera dimensions restored" is logged.
 */
void CameraRecovery::resumed_retained_resolution() noexcept {
  if (requested_ && !pending_ && reason_ == StopReason::resolution_changed) {
    reason_ = StopReason::none;
    ++sequence_;
  }
}

/*
 * Uses resumed capture progress to clear an eligible transient stop, without clearing failures that require
 * an explicit start/stop decision.
 */
void CameraRecovery::capture_progress(std::uint64_t now) noexcept {
  if (!requested_ || pending_ || !retryable_stop(reason_))
    return;
  if (!last_progress_ || now < last_progress_ || now - last_progress_ > 2000)
    healthy_since_ = now;
  last_progress_ = now;
  if (now >= healthy_since_ && now - healthy_since_ >= 10000) {
    attempts_ = 0;
    reason_ = StopReason::none;
  }
}

/*
 * Parks an initial creation request when public pose is not ready and records the appropriate recovery wait.
 * It does not manufacture an aircraft transform to complete startup.
 */
bool defer_initial_pose_failure(engine_camera::CameraOwnership& controller,
                                engine_camera::ManagerToken manager,
                                CameraRecovery& recovery,
                                bool temporary_pose,
                                unsigned created,
                                std::uint64_t inspected_revision,
                                std::uint64_t current_revision,
                                std::uint64_t now) noexcept {
  const auto ownership = controller.snapshot();
  if (!temporary_pose || created != 0 || inspected_revision != current_revision || !recovery.requested() ||
      recovery.attempts() >= CameraRecovery::maximum_retries ||
      (recovery.reason() != StopReason::none && !retryable_stop(recovery.reason())) || ownership.state != engine_camera::State::failed ||
      ownership.failure != engine_camera::Failure::initializer_failed || ownership.blocked != engine_camera::Blocked::none ||
      ownership.owner != engine_camera::ManagerToken{} || ownership.owned_id || ownership.request_pending || ownership.creation_pending)
    return false;
  controller.request_disable();
  if (!controller.process_update(manager, {}))
    return false;
  const auto clean = controller.snapshot();
  if (clean.state != engine_camera::State::disabled || clean.failure != engine_camera::Failure::none ||
      clean.blocked != engine_camera::Blocked::none || clean.owned_id || clean.request_pending || clean.creation_pending)
    return false;
  recovery.failed(StopReason::inspection_unavailable, now);
  return recovery.pending();
}

/*
 * Tracks a temporary readiness gap for a specific manager and entry. A new identity resets the wait; elapsed
 * time alone never authorizes deletion, and a different manager or entry cannot inherit the previous wait,
 * even if native addresses recur.
 */
bool ViewReadinessWait::observe(std::uint64_t now,
                                const engine_camera::Snapshot& ownership,
                                bool established,
                                const engine_camera::OwnedViewSnapshot& view) noexcept {
  const bool valid_ownership = established && ownership.state == engine_camera::State::active && ownership.owner.valid() &&
                               ownership.owned_id && !ownership.request_pending && !ownership.creation_pending &&
                               ownership.failure == engine_camera::Failure::none && ownership.blocked == engine_camera::Blocked::none;
  const bool stable_pending = view.complete && !view.ready && view.status == engine_camera::OwnedViewStatus::pending;
  const bool transient_read =
      !view.complete && !view.ready &&
      (view.status == engine_camera::OwnedViewStatus::read_failed || view.status == engine_camera::OwnedViewStatus::changed ||
       view.status == engine_camera::OwnedViewStatus::pool_changed || view.status == engine_camera::OwnedViewStatus::not_inspected);
  const bool ready = view.complete && view.ready && view.status == engine_camera::OwnedViewStatus::ready;
  const bool valid = valid_ownership && (stable_pending || transient_read || ready);
  const bool pending = stable_pending || transient_read;
  if (!valid || !pending) {
    clear();
    return false;
  }
  if (!waiting_) {
    waiting_ = true;
    owner_ = ownership.owner;
    id_ = ownership.owned_id;
    since_ = now;
  }
  return ownership.owner == owner_ && ownership.owned_id == id_ && now >= since_;
}

void ViewReadinessWait::clear() noexcept {
  waiting_ = false;
  owner_ = {};
  id_ = 0;
  since_ = 0;
}

/*
 * Requires a currently valid owned view and a closed gate observed across engine updates before allowing
 * retirement. Same-update closure is not sufficient evidence for removal.
 */
ViewRetirement::Action ViewRetirement::observe(engine_camera::ManagerToken owner,
                                               engine_camera::EntryId id,
                                               std::uint64_t update,
                                               const engine_camera::OwnedViewSnapshot& view) noexcept {
  if (!owner.valid() || !id)
    return Action::wait;
  const bool tracked = closed_.owner == owner && closed_.id == id;
  if (!view.complete || !view.ready || view.status != engine_camera::OwnedViewStatus::ready || !view.view_address || !view.node_address ||
      !view.camera_address || view.view_index < 0 || view.view_index >= 8) {
    if (tracked)
      closed_ = {};
    return Action::wait;
  }
  if ((view.flags[0] & 1u) == 0) {
    if (tracked)
      closed_ = {};
    return Action::close_gate;
  }
  if (!tracked) {
    if (!closed_.id)
      closed_ = {owner, id, update};
    return Action::wait;
  }
  if (update < closed_.update) {
    closed_ = {};
    return Action::wait;
  }
  return update > closed_.update ? Action::erase : Action::wait;
}

void ViewRetirement::forget(engine_camera::ManagerToken owner, engine_camera::EntryId id) noexcept {
  if (closed_.owner == owner && closed_.id == id)
    closed_ = {};
}

/*
 * Binds a resize-recovery episode to the current manager and entry. A mismatched episode cannot inherit an
 * earlier resize authorization.
 */
bool ViewResizeRecovery::begin(engine_camera::ManagerToken owner, engine_camera::EntryId id) noexcept {
  if (failed_)
    return false;
  if (!owner.valid() || !id || (pending_ && !matches(owner, id))) {
    mark_failed();
    return false;
  }
  if (pending_)
    return true;
  owner_ = owner;
  id_ = id;
  pending_ = true;
  return true;
}

/*
 * Checks the fresh view chain, dimensions and closed gate before issuing one resize authorization. A native
 * attempt consumes it: the caller must finish on complete success or mark_failed on ANY refused/partial
 * mutation.
 */
ViewResizeRecovery::Action ViewResizeRecovery::observe(engine_camera::ManagerToken owner,
                                                       engine_camera::EntryId id,
                                                       std::uint64_t update,
                                                       const engine_camera::OwnedViewSnapshot& view) noexcept {
  if (failed_)
    return Action::blocked;
  if (!pending_)
    return Action::wait;
  if (!owner.valid() || !id || !matches(owner, id) || !update || update < last_update_) {
    mark_failed();
    return Action::blocked;
  }
  last_update_ = update;
  using Status = engine_camera::OwnedViewStatus;
  const bool temporary = (view.status == Status::pending && view.complete && !view.ready) ||
                         (!view.complete && !view.ready &&
                          (view.status == Status::not_inspected || view.status == Status::read_failed || view.status == Status::changed ||
                           view.status == Status::pool_changed));
  if (!temporary && !ready_chain(view)) {
    mark_failed();
    return Action::blocked;
  }
  if (temporary) {
    closed_seen_ = false;
    return Action::wait;
  }

  if (resize_issued_)
    return Action::wait;
  if (!(view.flags[0] & 1u)) {
    closed_seen_ = false;
    return Action::close_gate;
  }
  if (!closed_seen_) {
    closed_seen_ = true;
    closed_update_ = update;
    return Action::wait;
  }
  if (update == closed_update_)
    return Action::wait;
  resize_issued_ = true;
  return Action::resize;
}

/*
 * Completes recovery only when the post-resize identities and dimensions still match the authorized episode.
 */
bool ViewResizeRecovery::finish(engine_camera::ManagerToken owner, engine_camera::EntryId id) noexcept {
  if (!pending_ || failed_)
    return false;
  if (!owner.valid() || !id || !matches(owner, id)) {
    mark_failed();
    return false;
  }
  if (!resize_issued_)
    return false;
  clear();
  return true;
}

void ViewResizeRecovery::mark_failed() noexcept {
  failed_ = true;
  closed_seen_ = resize_issued_ = false;
}

bool ViewResizeRecovery::ready_chain(const engine_camera::OwnedViewSnapshot& view) noexcept {
  return view.complete && view.ready && view.status == engine_camera::OwnedViewStatus::ready && !view.read_failures &&
         (!view.error || !*view.error) && view.view_address && !(view.view_address & 7u) && view.node_address &&
         !(view.node_address & 7u) && view.camera_address && !(view.camera_address & 7u) && view.view_index >= 0 && view.view_index < 8 &&
         std::isfinite(view.fov) && view.fov > 0;
}

/*
 * The first applicable phase wins: a session hold owns the camera, a parked or telemetry-less start has
 * nothing to do natively, and a close-only update tries the lighter path before the full one.
 */
UpdatePhase select_update_phase(const UpdateFacts& facts) noexcept {
  if (facts.session_hold)
    return UpdatePhase::session_transition;
  if (facts.suspended && !facts.has_camera)
    return UpdatePhase::parked;
  if (facts.start_requested && !facts.has_camera && !facts.start_telemetry_valid)
    return UpdatePhase::waiting_for_telemetry;
  if (facts.close_gate_only)
    return UpdatePhase::close_gate;
  return UpdatePhase::full;
}

/*
 * Failures and recoveries take precedence over rendering; an unreadable view waits before it is refused.
 */
CameraPhase select_camera_phase(const CameraFacts& facts) noexcept {
  if (facts.initial_resize_failed)
    return CameraPhase::initial_resize_failed;
  if (facts.resolution_pause)
    return CameraPhase::resolution_recovery;
  if (facts.warmup_waiting)
    return CameraPhase::warmup;
  if (facts.ready_to_render)
    return CameraPhase::render;
  if (facts.view_wait)
    return CameraPhase::view_wait;
  return CameraPhase::invalid;
}

/*
 * Chooses whether inspection is due, allowing immediate checks for lifecycle or mount changes while
 * throttling a genuinely idle view.
 */
InspectionDecision InspectionThrottle::decide(const InspectionState& state) noexcept {
  const bool idle = state.established && state.suspended && state.gate_closed && state.view_ready && !state.lifecycle_pending &&
                    !state.start_pending && !state.warmup_pending && !state.recovery_pending && !state.mount_changed;
  if (idle) {
    idled_ = true;
    return InspectionDecision::idle;
  }
  const bool required = idled_ || state.mount_changed;
  idled_ = false;
  return required ? InspectionDecision::required : InspectionDecision::periodic;
}

}  /* namespace azp::native_camera */
