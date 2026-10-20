/*
 * The camera lifecycle, as the engine observer in camera.cpp walks it on every manager update:
 *
 *   no camera --start--> creating --> warmup (gate closed, one engine update) --> initial resize
 *     --> ready --> streaming (gate open) <--> paused (gate closed while video is not wanted)
 *
 *   Side paths keep the entry ID and a closed gate: resolution recovery (graphics settings changed), view
 *   wait (the view is briefly unreadable) and session transition (aircraft or flight changed). Removal
 *   happens only after a validated closed view was seen on two distinct manager updates.
 *
 * Each update first picks its UpdatePhase, then, for an existing camera, its CameraPhase. The observer walks
 * them as one ordered branch chain; select_update_phase/select_camera_phase state the same orders. Those, and
 * the small trackers below that each hold one piece of evidence (session transition, recovery, readiness
 * wait, retirement, resize recovery, inspection throttle), are pure: they store decisions and identity
 * tokens only, and tests/camera_lifecycle_test.cpp exercises them offline. Engine reads and mutations stay
 * in camera.cpp.
 */

#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>
#include "../engine/camera_ownership.hpp"
#include "../engine/view.hpp"

namespace azp::native_camera {
/*
 * Keeps the owned camera parked across an aircraft/flight session change. The retained entry is closed,
 * revalidated against the identity and dimensions recorded when the session changed, and only then allowed
 * to resume; the session change itself never erases or recreates the camera. Only numeric identities and
 * dimension expectations survive between inspections.
 */
class SessionTransition {
 public:
  using Ownership = engine_camera::Snapshot;
  struct AllocationEvidence {
    engine_camera::ManagerToken owner{};
    engine_camera::EntryId id = 0;
    ViewDimensions dimensions{};
    bool matches(const Ownership& ownership) const noexcept { return owner == ownership.owner && id == ownership.owned_id; }
  };
  enum class Decision { wait, close, ready, refused };
  static bool session_changed(const Ownership& ownership, std::uint64_t started, std::uint64_t current) noexcept;
  void begin(const Ownership& ownership, const ViewDimensions& dimensions) noexcept;
  /*
   * The controller is still processing. Its published empty snapshot cannot prove absence until
   * cancel_uncreated_request or a fresh owned entry settles.
   */
  void defer_ownership() noexcept;
  /*
   * CameraOwnership publishes ownership before the observer can publish allocation expectations. Never bind
   * a new entry to an earlier entry's dimensions during that gap; the observer will publish matching evidence
   * before retrying.
   */
  void begin_published(const Ownership& ownership, const AllocationEvidence& evidence) noexcept;
  Decision inspect(engine_camera::ManagerToken manager, const Ownership& ownership, const engine_camera::OwnedViewSnapshot& view) noexcept;
  bool matches(const Ownership& ownership) const noexcept;
  bool can_resume(const Ownership& ownership) const noexcept { return ready() && matches(ownership); }
  void refuse() noexcept;
  void consume() noexcept;
  bool holding() const noexcept { return holding_; }
  bool awaiting_ownership() const noexcept { return holding() && awaiting_ownership_; }
  bool pending() const noexcept { return holding() && (state_ == Decision::wait || state_ == Decision::close); }
  bool ready() const noexcept { return holding() && state_ == Decision::ready; }
  bool failed() const noexcept { return holding() && state_ == Decision::refused; }

 private:
  static bool valid_ownership(const Ownership& ownership) noexcept;
  bool holding_ = false;
  engine_camera::ManagerToken owner_{};
  engine_camera::EntryId id_ = 0;
  ViewDimensions dimensions_{};
  bool awaiting_ownership_ = false;
  Decision state_ = Decision::wait;
};
}  /* namespace azp::native_camera */

namespace azp::native_camera {

enum class StopReason {
  none,
  explicit_stop,
  inspection_unavailable,
  owned_entry_absent,
  resolution_changed,
  capture_stalled,
  identity_refused,
  pose_invalid,
  creation_failed,
  exception
};

bool retryable_stop(StopReason reason) noexcept;

bool latched_stop(StopReason reason) noexcept;

bool temporary_pose_unavailable(const char* reason) noexcept;

/*
 * Observer policy, serialized by the camera request mutex. No native calls. Retry only after the existing
 * controller has confirmed every old ID absent, and a fresh pose plus fresh manager validation is available
 * to the caller.
 */
class CameraRecovery {
 public:
  static constexpr unsigned maximum_retries = 3;
  static constexpr std::uint64_t retry_delay_ms = 2000;
  void start() noexcept;
  void stop() noexcept;
  void failed(StopReason reason, std::uint64_t now) noexcept;
  bool retry(std::uint64_t now, const engine_camera::Snapshot& ownership, bool fresh_pose) noexcept;
  void resumed_retained_resolution() noexcept;
  bool requested() const noexcept { return requested_; }
  void capture_progress(std::uint64_t now) noexcept;
  bool pending() const noexcept { return pending_; }
  unsigned attempts() const noexcept { return attempts_; }
  std::uint64_t sequence() const noexcept { return sequence_; }
  StopReason reason() const noexcept { return reason_; }

 private:
  bool requested_ = false;
  bool pending_ = false;
  unsigned attempts_ = 0;
  std::uint64_t sequence_ = 0;
  std::uint64_t stopped_at_ = 0;
  std::uint64_t healthy_since_ = 0, last_progress_ = 0;
  StopReason reason_ = StopReason::none;
};

/*
 * Called under the camera request mutex, after the first initializer refused a temporary pose inspection. The
 * enable was consumed, but no native descriptor initializer/create ran. Clear only this empty controller
 * failure; supplying no callbacks makes native cleanup/creation impossible here. The next retry still needs
 * the ordinary fresh manager, pose, pool and request guards.
 */
bool defer_initial_pose_failure(engine_camera::CameraOwnership& controller,
                                engine_camera::ManagerToken manager,
                                CameraRecovery& recovery,
                                bool temporary_pose,
                                unsigned created,
                                std::uint64_t inspected_revision,
                                std::uint64_t current_revision,
                                std::uint64_t now) noexcept;

}  /* namespace azp::native_camera */

namespace azp::native_camera {

/*
 * Established views may briefly be pending or fail a consistency read during allocation. Waiting grants no
 * permission to use the failed snapshot. The caller must classify the inspection as transient; no borrowed
 * pointers survive.
 */
class ViewReadinessWait {
 public:
  bool observe(std::uint64_t now,
               const engine_camera::Snapshot& ownership,
               bool established,
               const engine_camera::OwnedViewSnapshot& view) noexcept;
  void clear() noexcept;

 private:
  bool waiting_ = false;
  engine_camera::ManagerToken owner_{};
  engine_camera::EntryId id_ = 0;
  std::uint64_t since_ = 0;
};

}  /* namespace azp::native_camera */

namespace azp::native_camera {

/*
 * No borrowed pointers are retained. A complete ready chain and a closed gate must be observed on distinct
 * manager updates before calling native removal. This is a CPU lifecycle precondition, not a GPU fence: the
 * engine's normal deferred view release remains responsible for renderer retirement.
 */
class ViewRetirement {
 public:
  enum class Action { wait, close_gate, erase };

  Action observe(engine_camera::ManagerToken owner,
                 engine_camera::EntryId id,
                 std::uint64_t update,
                 const engine_camera::OwnedViewSnapshot& view) noexcept;

  void forget(engine_camera::ManagerToken owner, engine_camera::EntryId id) noexcept;
  void clear() noexcept { closed_ = {}; }

 private:
  /*
   * The entry whose closed gate was first observed, and on which manager update.
   */
  struct Closed {
    engine_camera::ManagerToken owner{};
    engine_camera::EntryId id = 0;
    std::uint64_t update = 0;
  };
  Closed closed_{};
};

}  /* namespace azp::native_camera */

namespace azp::native_camera {

/*
 * Serialized by the caller in the manager observer phase. Retains only opaque owner/entry identities and
 * update counters, never borrowed view pointers. Closed gates across CPU updates do NOT prove GPU or native
 * output lifetime; the caller must establish that separately before acting on resize.
 */
class ViewResizeRecovery {
 public:
  enum class Action { wait, close_gate, resize, blocked };

  bool begin(engine_camera::ManagerToken owner, engine_camera::EntryId id) noexcept;

  Action observe(engine_camera::ManagerToken owner,
                 engine_camera::EntryId id,
                 std::uint64_t update,
                 const engine_camera::OwnedViewSnapshot& view) noexcept;

  bool finish(engine_camera::ManagerToken owner, engine_camera::EntryId id) noexcept;
  /*
   * Caller inspected a closed ready view but could not prove the existing Bitmap still matches the image size.
   * Release the one-shot authorization so a later update may restore instead of sitting in wait after an
   * unused resize token.
   */
  void release_resize_authorization() noexcept;
  bool pending() const noexcept { return pending_; }
  bool failed() const noexcept { return failed_; }
  void clear() noexcept { *this = {}; }  /* Explicit lifecycle reset, never automatic retry. */
  void mark_failed() noexcept;

 private:
  bool matches(engine_camera::ManagerToken owner, engine_camera::EntryId id) const noexcept { return owner == owner_ && id == id_; }
  static bool ready_chain(const engine_camera::OwnedViewSnapshot& view) noexcept;

  engine_camera::ManagerToken owner_{};
  engine_camera::EntryId id_ = 0;
  std::uint64_t last_update_ = 0, closed_update_ = 0;
  bool pending_ = false, failed_ = false, closed_seen_ = false, resize_issued_ = false;
};

}  /* namespace azp::native_camera */

namespace azp::native_camera {

/*
 * What one manager update does, chosen before any native access.
 */
enum class UpdatePhase {
  session_transition,    /* An aircraft/flight change holds the retained camera closed until revalidated. */
  parked,                /* No camera and video not wanted: queued creation waits. */
  waiting_for_telemetry, /* A start is queued but the public aircraft telemetry is not valid yet. */
  close_gate,            /* An established ready camera only needs its gate closed (lighter inspection). */
  full,                  /* Verify the manager, then create, resize, render or retire. */
};
struct UpdateFacts {
  bool session_hold = false;
  bool has_camera = false;
  /*
   * Video is not wanted: gates must close, and creation waits. An existing camera still needs the validated
   * path to close its gate or be cleaned up.
   */
  bool suspended = false;
  bool start_requested = false;
  bool start_telemetry_valid = false;
  bool close_gate_only = false;
};
UpdatePhase select_update_phase(const UpdateFacts& facts) noexcept;

/*
 * What a full update does with the existing camera, after its view was inspected.
 */
enum class CameraPhase {
  initial_resize_failed, /* Hold if the aircraft changed meanwhile; otherwise latch the failure and remove. */
  resolution_recovery,   /* Graphics settings changed: keep the ID, close the gate, restore dimensions. */
  warmup,                /* A new view waits, gate closed, for one engine update before its first resize. */
  render,                /* Ready: validate pose and attachment, then drive the gate. */
  view_wait,             /* The view is briefly unreadable: keep the ID with the gate closed. */
  invalid,               /* Hold if the aircraft changed meanwhile; otherwise refuse and request removal. */
};
struct CameraFacts {
  bool initial_resize_failed = false;
  bool resolution_pause = false;
  bool warmup_waiting = false;
  bool ready_to_render = false;
  bool view_wait = false;
};
CameraPhase select_camera_phase(const CameraFacts& facts) noexcept;

struct InspectionState {
  bool established = false;
  bool suspended = false;
  bool gate_closed = false;
  bool view_ready = false;
  bool lifecycle_pending = false;
  bool start_pending = false;
  bool warmup_pending = false;
  bool recovery_pending = false;
  bool mount_changed = false;
};

enum class InspectionDecision { periodic, idle, required };

/*
 * This gate stores no native address or memory proof. An idle observer performs no native operation. Leaving
 * idle requires a fresh complete inspection before the existing caller may use any manager, view or camera
 * pointer again.
 */
class InspectionThrottle {
 public:
  InspectionDecision decide(const InspectionState& state) noexcept;

 private:
  bool idled_ = false;
};

}  /* namespace azp::native_camera */
