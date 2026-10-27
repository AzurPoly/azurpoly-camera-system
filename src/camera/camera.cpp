/*
 * Coordinates the native camera from start request to image publication. The control thread queues intent; a
 * verified engine observer checks ownership, updates the aircraft attachment, controls render gates and
 * handles recovery. Only a freshly validated final-image resource identity is handed to the rendering
 * pipeline.
 */

#include "camera.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cwchar>
#include <mutex>
#include <optional>
#include <stdexcept>
#include "../engine/aircraft_scene_pose.hpp"
#include "../engine/engine_api.hpp"
#include "../engine/memory.hpp"
#include "../engine/night_vision.hpp"
#include "../engine/observer_hook.hpp"
#include "../engine/view.hpp"
#include "../shared/configuration.hpp"
#include "../video/scene_handoff.hpp"
#include "aircraft_inventory.hpp"
#include "camera_attachment.hpp"
#include "lifecycle.hpp"
#include "scenery_preload.hpp"
#include "simconnect_link.hpp"
#include "zoom_detail.hpp"

namespace azp::native_camera {
namespace {
namespace ec = engine_camera;

struct Runtime {
  std::mutex mutex;
  std::mutex start_mutex;
  CameraStatus published;
  ec::CameraOwnership ownership;
  std::atomic<bool> hooked{false};
  std::atomic<bool> enabled{false};
  std::atomic<bool> suspended{false};
  std::atomic<bool> requested_night_vision{false};
  /*
   * IR gain written by the NV callback, set by the control loop's automatic gain.
   */
  std::atomic<float> night_vision_gain{kNightVisionDefaultIntensity};
  bool selected_night_vision = false;  /* Frozen for each complete observer inspection. */
  /*
   * Protected by mutex; never used directly by a native engine call.
   */
  SessionTransition session_transition;
  SessionTransition::AllocationEvidence published_allocation{};
  std::uint64_t session_transition_token = 0;
  std::uint64_t started_session_epoch = 0;
  bool retained_restart_requested = false;
  bool requested_start = false;
  std::uint64_t requested_start_revision = 0;
  CameraRecovery recovery;
  std::string stop_detail;
  bool protection_ready = false;
  std::atomic_flag observing = ATOMIC_FLAG_INIT;
  std::uintptr_t base = 0;
  discovery::Inventory image;
  CameraContract contract;
  NightVision night_vision;
  /*
   * View address of the last owned view the camera observer found ready, or 0. The NV callback uses it only
   * to ignore other views' callbacks cheaply; it never authorizes a write.
   */
  std::atomic<std::uint64_t> night_vision_view{0};
  /*
   * Cumulative time spent in the two engine callbacks (manager update, NV render), in microseconds.
   */
  std::atomic<std::uint64_t> observer_us{0}, night_vision_us{0};
  /*
   * Native render limit in frames per second (0 = every simulator frame) and, on the observer thread, the
   * steady-clock time in microseconds at which the view is next due to render.
   */
  std::atomic<std::uint32_t> render_rate{0};
  std::uint64_t next_render_us = 0;
  /*
   * Observer-thread fields. No borrowed camera/output pointers cross into UI.
   */
  std::uint64_t updates = 0;
  std::uint64_t created_total = 0;
  InspectionThrottle inspection_gate;
  /*
   * The entry whose gate this observer last drove, and the last gate state it requested.
   */
  ec::EntryId scheduled_id = 0;
  bool gate = false;
  /*
   * The entry resized to the camera image size, and the dimensions it must keep.
   */
  ec::EntryId resized_id = 0;
  ViewDimensions resized_dimensions{};
  std::array<float, 2> far_planes{};
  ViewResizeWarmup resize_warmup;
  ViewResizeRecovery resize_recovery;
  ViewReadinessWait view_wait;
  ViewRetirement retirement;
  ULONGLONG last_inspection = 0;
  std::uint64_t manager = 0;
  std::uint64_t control = 0;
  std::uint64_t owned_control = 0;
  std::uint64_t renderer = 0;
  ec::ManagerToken token{};
  unsigned creations = 0;
  bool lifecycle_touched = false;
  bool creation_valid = false;
  bool creation_pose_unavailable = false;
  bool pose_busy = false;
  bool inspection_changed = false;
  StopReason inspection_stop = StopReason::identity_refused;
  MountedPose mounted_pose{};
  AircraftScenePose aircraft_scene{};
  ImaCameraSample ima_camera{};
  const char* stage_error = "";
  std::string manager_memory_error;
  std::string inspection_error;
  std::string creation_error;
  std::string message;
};

/*
 * Intentionally process-lifetime storage: the installed observer pins this DLL. Neither callbacks nor their
 * mutexes may be destroyed during runtime unloading.
 */
Runtime& state() {
  static Runtime* const runtime = new Runtime;
  return *runtime;
}

/*
 * Runs a read-only inspection inside a short-lived memory-query cache, then validates its endpoint. A
 * successful inner result is discarded if the cached memory evidence changed.
 *
 * Cache metadata only while a pure inspection builds an unpublished result. No private call occurs until the
 * cache is detached and all endpoint queries agree. Each helper retains its original bounded reads and full
 * observation reread.
 */
template <typename Operation>
auto inspected(Runtime& runtime, Operation&& operation, std::string* failure_detail = nullptr) {
  ScopedLocalMemoryQueryCache queries(LocalMemoryQueryMode::private_pages);
  auto result = operation();
  if (!queries.finish()) {
    runtime.inspection_changed = true;
    runtime.inspection_error = "Memory-region metadata changed during inspection; no result was accepted: " +
                               describe_local_memory_query_failure(queries.failure());
    runtime.stage_error = runtime.inspection_error.c_str();
    if (failure_detail)
      *failure_detail = runtime.inspection_error;
    return decltype(result){};
  }
  return result;
}

/*
 * Adds the duration of one engine callback to a cumulative counter, in microseconds.
 */
class CallbackTimer {
 public:
  explicit CallbackTimer(std::atomic<std::uint64_t>& total) noexcept : total_(total) { QueryPerformanceCounter(&start_); }
  ~CallbackTimer() {
    static const auto frequency = [] {
      LARGE_INTEGER value{};
      QueryPerformanceFrequency(&value);
      return value.QuadPart;
    }();
    LARGE_INTEGER end{};
    QueryPerformanceCounter(&end);
    if (frequency > 0 && end.QuadPart > start_.QuadPart)
      total_.fetch_add(static_cast<std::uint64_t>((end.QuadPart - start_.QuadPart) * 1000000 / frequency), std::memory_order_relaxed);
  }
  CallbackTimer(const CallbackTimer&) = delete;
  CallbackTimer& operator=(const CallbackTimer&) = delete;

 private:
  std::atomic<std::uint64_t>& total_;
  LARGE_INTEGER start_{};
};

template <typename Function>
Function function(Runtime& runtime, std::uint32_t rva) noexcept {
  return reinterpret_cast<Function>(runtime.base + rva);
}

template <typename T>
bool word(LocalMemoryReader& reader, std::uint64_t address, T& output) noexcept {
  return reader.read(address, &output, sizeof(output));
}

/*
 * Establishes the current native manager/control identity from the resolved image and checks the
 * memory-query transaction before exposing it to the observer.
 *
 * Called only from this manager's update thunk, immediately before its original. Cached pointer, weak handle
 * and vptr must all agree, then be reread. An enclosing transaction may borrow this pure stage. Its caller
 * must finish the shared cache before using the provisional Runtime identity for any call.
 */
bool manager_context(Runtime& runtime,
                     void* current_manager,
                     ScopedLocalMemoryQueryCache* shared = nullptr,
                     ManagerInspection* outcome = nullptr) {
  const auto refused = [&](ManagerInspection result) {
    if (outcome)
      *outcome = result;
    return false;
  };
  std::optional<ScopedLocalMemoryQueryCache> queries;
  if (shared) {
    if (!shared->is_current())
      return refused({ManagerInspectionStatus::unavailable, "Manager inspection transaction is no longer current."});
  } else {
    queries.emplace(LocalMemoryQueryMode::private_pages);
  }
  LocalMemoryReader reader(512);
  LocalImageReader image(reinterpret_cast<HMODULE>(runtime.base), runtime.image.image_size, LocalImageQueryMode::pages);
  const auto result = inspect_manager_identity(image, reader, runtime.base, reinterpret_cast<std::uintptr_t>(current_manager),
                                               runtime.owned_control, runtime.contract.layout);
  if (!result)
    return refused(result);
  if (queries && !queries->finish()) {
    runtime.manager_memory_error =
        "Manager memory-region metadata changed during inspection: " + describe_local_memory_query_failure(queries->failure());
    return refused({ManagerInspectionStatus::unavailable, runtime.manager_memory_error.c_str()});
  }
  runtime.manager = result.manager;
  runtime.renderer = result.renderer;
  runtime.control = result.control;
  runtime.token = {result.manager, result.generation};
  if (outcome)
    *outcome = result;
  return true;
}

/*
 * Validates the public flight sample, resolves the rendered aircraft node and reads its current transform.
 * It then combines this transform with fresh WASM mount/aim data; asynchronous telemetry never becomes the
 * attachment position.
 *
 * The visible aircraft and its camera offsets must use the same scene pose. SimConnect is asynchronous; a
 * fresh packet is still not the rendered model transform during a taxi turn. Retain its
 * session/freshness/plausibility guards, but never use it as the mount transform or interpolate toward it.
 */
bool capture_pose(Runtime& runtime) {
  runtime.pose_busy = false;
  runtime.mounted_pose = {};
  runtime.aircraft_scene = {};
  runtime.ima_camera = {};
  std::string memory_detail;
  auto body = sample_body_pose(GetTickCount64());
  if (!body.valid && temporary_pose_unavailable(body.error)) {
    runtime.pose_busy = true;
    runtime.message = std::string("Aircraft telemetry temporarily unavailable; render gates remain closed: ") + body.error;
    return false;
  }
  if (!body.valid) {
    runtime.pose_busy = true;
    runtime.message = std::string("Aircraft telemetry unavailable: ") + body.error;
    return false;
  }
  LocalMemoryReader objects;
  LocalImageReader image(reinterpret_cast<HMODULE>(runtime.base), runtime.image.image_size, LocalImageQueryMode::pages);
  std::uint64_t user = 0;
  const auto aircraft = inspected(
      runtime,
      [&] {
        return discovery::inspect_aircraft_metadata(image, objects, runtime.image, runtime.base,
                                                    runtime.contract.layout.aircraft_facade_vtable, true, true, nullptr, &user,
                                                    runtime.contract.layout);
      },
      &memory_detail);
  if (!aircraft.valid || !aircraft.available || !user) {
    runtime.pose_busy = true;
    runtime.message = "Aircraft scene mount is waiting for a stable active aircraft: " + aircraft.stage + ". " + aircraft.error;
    if (!memory_detail.empty())
      runtime.message += " " + memory_detail;
    return false;
  }
  objects.reset_budget();
  const auto scene =
      inspected(runtime, [&] { return inspect_aircraft_scene_pose(objects, user, runtime.base, runtime.contract.layout); }, &memory_detail);
  if (!scene.complete || !validate_aircraft_scene_pose(scene.pose, GetTickCount64())) {
    runtime.pose_busy = true;
    runtime.message = std::string("Aircraft scene mount is waiting for a consistent model pose: ") +
                      (scene.complete ? "public_pose_mismatch" : scene.error);
    if (!memory_detail.empty())
      runtime.message += " " + memory_detail;
    return false;
  }
  runtime.aircraft_scene = scene;
  runtime.ima_camera = get_ima_camera();
  if (!runtime.ima_camera.valid || !runtime.ima_camera.local_mount) {
    runtime.pose_busy = true;
    runtime.message = "Waiting for a fresh WASM camera publication.";
    return false;
  }
  if (!make_ima_camera_pose(scene.pose, runtime.ima_camera, runtime.mounted_pose)) {
    runtime.message = "IMA camera orientation unavailable.";
    return false;
  }
  return true;
}
/*
 * Checks the owned ID table, view pool and full view chain in sequence. No camera/output pointer is accepted
 * from an incomplete stage.
 */
ec::OwnedViewSnapshot inspect_entry(Runtime& runtime, std::uint64_t id) {
  LocalMemoryReader reader;
  const auto entries = inspected(runtime, [&] { return ec::inspect_owned_entries(reader, runtime.manager, id); });
  if (!entries.complete || !entries.entry.found) {
    ec::OwnedViewSnapshot failed;
    failed.error = entries.complete ? "The created entry ID is absent from the manager table." : entries.error;
    return failed;
  }
  reader.reset_budget();
  const auto pool = inspected(runtime, [&] { return ec::inspect_view_pool(reader, runtime.renderer); });
  reader.reset_budget();
  return inspected(runtime, [&] { return ec::inspect_owned_view(reader, entries.entry.address, id, pool); });
}

/*
 * The entry-table, pool and view-chain reads of inspect_entry, inside the caller's memory-query transaction:
 * one validation at the end instead of one per step. Nothing may be called or written before the caller
 * finishes that transaction.
 */
ec::OwnedViewSnapshot inspect_entry_reads(Runtime& runtime, std::uint64_t id) {
  LocalMemoryReader reader;
  const auto entries = ec::inspect_owned_entries(reader, runtime.manager, id);
  if (!entries.complete || !entries.entry.found) {
    ec::OwnedViewSnapshot failed;
    failed.error = entries.complete ? "The created entry ID is absent from the manager table." : entries.error;
    return failed;
  }
  reader.reset_budget();
  const auto pool = ec::inspect_view_pool(reader, runtime.renderer);
  reader.reset_budget();
  return ec::inspect_owned_view(reader, entries.entry.address, id, pool);
}

/*
 * Publishes the view's output only for a completed inspection, with the TV/IR mode it was made for. With
 * post-processing enabled, the engine renders the scene into its own intermediate targets and only its final
 * passes write this resource, so it holds the final image: clouds, effects and display transform included.
 * It is published only once the observer has cleared the bypass flag on this owned view.
 *
 * A zero resource revokes only the current image, never the native view.
 */
bool publish_outputs(Runtime& runtime, SceneCaptureTicket ticket, ec::EntryId id, const ec::OwnedViewSnapshot& view, CameraStatus& report) {
  report.night_vision = runtime.selected_night_vision;
  const auto resource = view.flags[0] & kViewBypassPostProcessing ? 0 : view.resource_address;
  return scene_handoff().publish(ticket, {runtime.token.identity, runtime.token.generation}, id, resource, report.night_vision);
}

/*
 * Inspects the retained ownership set and its image dimensions as one read-only transaction. Ready results
 * become publishable only after endpoint validation; pending output and size drift retain their distinct
 * recovery paths.
 *
 * Borrowed results remain provisional and may never publish a handoff here.
 *
 * Malformed identity must not be downgraded to a transient failure. Every new retry still repeats the full
 * native guards.
 *
 * Primary-view resizing can overwrite mode2 size fields while its Bitmap remains pane-sized, and a renderer
 * reset can reallocate that Bitmap at the primary size. Retain the ownership; closed-gate recovery restores
 * the fields, and rebuilds the Bitmap only when fresh output dimensions prove the engine replaced it.
 *
 * Pool, entry table and owned view share one read-only region-query scope. Every field/trace is still
 * reread, and every queried region is revalidated before publishing pointers or calling the engine. Nothing
 * survives this observer.
 */
void inspect_camera(Runtime& runtime,
                    ec::EntryId id,
                    CameraStatus& report,
                    ec::OwnedViewSnapshot& view,
                    bool publish = true,
                    ScopedLocalMemoryQueryCache* shared = nullptr) {
  view = {};
  if (shared && (publish || !shared->is_current()))
    return;
  runtime.inspection_stop = StopReason::none;
  runtime.stage_error = "Owned view validation failed; removal requested.";
  const auto refuse = [&](StopReason reason, const char* detail) {
    if (runtime.inspection_stop == StopReason::none || reason == StopReason::identity_refused) {
      runtime.inspection_stop = reason;
      runtime.stage_error = detail;
    }
  };
  const auto ticket = publish ? scene_handoff().begin_capture() : SceneCaptureTicket{};
  std::optional<ScopedLocalMemoryQueryCache> queries;
  if (!shared)
    queries.emplace(LocalMemoryQueryMode::private_pages);
  LocalMemoryReader reader;
  const auto pool = ec::inspect_view_pool(reader, runtime.renderer);
  report.free_views = pool.valid ? pool.free_count : 0;
  if (!pool.valid) {
    refuse((runtime.inspection_changed && pool.status == ec::ViewPoolStatus::not_inspected) ||
                   pool.status == ec::ViewPoolStatus::read_failed || pool.status == ec::ViewPoolStatus::changed
               ? StopReason::inspection_unavailable
               : StopReason::identity_refused,
           "The current view pool could not be validated.");
    return;
  }
  reader.reset_budget();
  const auto entries = ec::inspect_owned_entries(reader, runtime.manager, id);
  if (!entries.complete) {
    refuse((runtime.inspection_changed && !entries.read_bytes) || entries.read_failures || std::strstr(entries.error, "changed")
               ? StopReason::inspection_unavailable
               : StopReason::identity_refused,
           entries.error);
    return;
  }
  if (!entries.entry.found) {
    refuse(StopReason::owned_entry_absent, "The owned camera entry is absent from the verified manager table.");
  } else {
    reader.reset_budget();
    view = ec::inspect_owned_view(reader, entries.entry.address, id, pool);
    report.ready = view.complete && view.ready;
    if (!report.ready) {
      refuse((runtime.inspection_changed && view.status == ec::OwnedViewStatus::not_inspected) ||
                     view.status == ec::OwnedViewStatus::read_failed || view.status == ec::OwnedViewStatus::changed ||
                     view.status == ec::OwnedViewStatus::pool_changed || view.status == ec::OwnedViewStatus::pending
                 ? StopReason::inspection_unavailable
                 : StopReason::identity_refused,
             *view.error ? view.error : "The owned view is not ready.");
    }
    report.resource_present = report.ready && view.resource_present;
    if (report.ready) {
      report.dimensions = view.dimensions;
      report.flags = view.flags;
      if (runtime.resized_id != id || runtime.resized_dimensions != view.dimensions) {
        report.ready = report.resource_present = false;
        refuse(StopReason::resolution_changed, "Owned-view dimensions changed; retaining the camera for closed-gate dimension recovery.");
      }
    }
  }
  if (queries && !queries->finish()) {
    runtime.inspection_changed = true;
    runtime.inspection_stop = StopReason::inspection_unavailable;
    runtime.stage_error = "Memory-region metadata changed during camera inspection; no result was accepted.";
    view = {};
    report.ready = report.resource_present = false;
    report.dimensions = {};
    report.flags = {};
    return;
  }
  if (publish && report.ready)
    publish_outputs(runtime, ticket, id, view, report);
}

/*
 * Independently revalidates the entry chain needed to close render gates. It can park a camera when full
 * image inspection is unavailable, but it cannot refresh image tickets, open a gate or authorize
 * destruction.
 *
 * Captured return is AL: false means native lookup refused. No open call is available here. Any refusal,
 * including after a partial close, falls back to the normal full inspection without trusting the cached gate
 * booleans.
 *
 * Retain the last complete camera/output status; do not publish a new resource ticket or imply that this
 * close-only inspection refreshed them.
 *
 * Only a closing pulse reaches this path. It cannot supply an OwnedViewSnapshot to pose, resize or handoff
 * code. Keep the complete table/pool and association guards, but do not touch Camera/material/output data
 * that false activation neither reads nor writes. Each cache ends before the first private call.
 */
bool close_owned_view(Runtime& runtime, void* manager, const ec::Snapshot& ownership, CameraStatus& report) {
  ec::OwnedViewCloseSnapshot view{};
  {
    ScopedLocalMemoryQueryCache queries(LocalMemoryQueryMode::private_pages);
    if (!manager_context(runtime, manager, &queries) || runtime.token != ownership.owner)
      return false;
    LocalMemoryReader reader;
    const auto pool = ec::inspect_view_pool(reader, runtime.renderer);
    if (!pool.valid)
      return false;
    reader.reset_budget();
    const auto entries = ec::inspect_owned_entries(reader, runtime.manager, ownership.owned_id);
    if (!entries.complete || !entries.entry.found)
      return false;
    reader.reset_budget();
    view = ec::inspect_owned_view_for_close(reader, entries.entry.address, ownership.owned_id, pool);
    if (!view.complete || view.dimensions != runtime.resized_dimensions)
      return false;
    if (!queries.finish())
      return false;
  }
  const auto current = runtime.ownership.snapshot();
  if (current.state != ec::State::active || current.owner != ownership.owner || current.owned_id != ownership.owned_id ||
      current.request_pending || current.creation_pending || current.failure != ec::Failure::none || current.blocked != ec::Blocked::none)
    return false;
  {
    const std::lock_guard lock(runtime.mutex);
    if (runtime.requested_start || runtime.recovery.pending() || runtime.published.view_waiting || runtime.published.pose_waiting)
      return false;
  }
  const bool closed = ([&] {
    const auto activate = function<bool (*)(void*, std::uint64_t, bool)>(runtime, runtime.contract.functions.activate_entry);
    if ((view.flags[0] & 1u) == 0 && !activate(reinterpret_cast<void*>(runtime.manager), ownership.owned_id, false))
      return false;
    ScopedLocalMemoryQueryCache queries(LocalMemoryQueryMode::private_pages);
    LocalMemoryReader reader(64);
    auto expected = view.flags;
    expected[0] |= 1u;
    std::array<std::uint64_t, 2> first{}, second{};
    if (!word(reader, view.view_address + 48, first) || first != expected || !word(reader, view.view_address + 48, second) ||
        second != first)
      return false;
    return queries.finish();
  }());
  if (!closed)
    return false;
  runtime.gate = false;
  {
    const std::lock_guard lock(runtime.mutex);
    report = runtime.published;
  }
  report.ownership = ownership;
  report.flags = view.flags;
  report.flags[0] |= 1u;
  return true;
}
/*
 * With current ownership proven, disables the view's AA and enables its post-processing, which gives the
 * camera its final image in TV and IR, and reinspects the view after each mutation. TV/IR changes then need
 * no flag change.
 *
 * Enabling post-processing changes render topology. Close only the existing owned gate, then recheck its
 * identity before changing flags; never recreate the camera.
 *
 * This comparison is used after closure and after the flag write.
 */
bool prepare_owned_view_render(Runtime& runtime, ec::EntryId id, ec::OwnedViewSnapshot& view) {
  LocalImageReader image(reinterpret_cast<HMODULE>(runtime.base), runtime.image.image_size, LocalImageQueryMode::pages);
  const auto result = disable_owned_view_aa(view, image, runtime.contract.layout);
  if (!result.complete) {
    runtime.stage_error = result.error;
    return false;
  }
  if (result.write_attempted) {
    const auto confirmed = inspect_entry(runtime, id);
    auto expected_flags = view.flags;
    expected_flags[0] &= ~kViewAaFlag;
    if (!confirmed.complete || !confirmed.ready || confirmed.mode != 2 || confirmed.view_address != view.view_address ||
        confirmed.node_address != view.node_address || confirmed.camera_address != view.camera_address ||
        confirmed.resource_address != view.resource_address || confirmed.dimensions != view.dimensions ||
        confirmed.output_dimensions != view.output_dimensions || confirmed.flags != expected_flags) {
      runtime.stage_error = "Owned camera identity changed while disabling its AA; render gate stays closed.";
      return false;
    }
    view = confirmed;
  }
  if (runtime.night_vision.post_processing_flags(view.flags[0]) == view.flags[0])
    return true;

  const auto before = view;
  auto expected_flags = before.flags;
  expected_flags[0] |= 1u;
  if (!(view.flags[0] & 1u) && !function<bool (*)(void*, std::uint64_t, bool)>(runtime, runtime.contract.functions.activate_entry)(
                                   reinterpret_cast<void*>(runtime.manager), id, false)) {
    runtime.stage_error = "NV post-processing transition could not close its owned view.";
    return false;
  }
  runtime.gate = false;
  const auto matches = [&](const ec::OwnedViewSnapshot& confirmed) {
    return confirmed.complete && confirmed.ready && confirmed.mode == 2 && confirmed.view_address == before.view_address &&
           confirmed.node_address == before.node_address && confirmed.camera_address == before.camera_address &&
           confirmed.resource_address == before.resource_address && confirmed.dimensions == before.dimensions &&
           confirmed.output_dimensions == before.output_dimensions && confirmed.flags == expected_flags;
  };
  view = inspect_entry(runtime, id);
  if (!matches(view) || !runtime.night_vision.enable_post_processing(view, runtime.contract.layout)) {
    runtime.stage_error = "NV post-processing configuration refused; owned view retained closed.";
    return false;
  }
  expected_flags[0] = runtime.night_vision.post_processing_flags(expected_flags[0]);
  view = inspect_entry(runtime, id);
  if (!matches(view)) {
    runtime.stage_error = "Owned camera identity changed during the NV post-processing transition.";
    return false;
  }
  return true;
}

/*
 * Runs the IR parameter update in the verified engine callback, once per rendered view and frame. It
 * excludes reentry and matches the current manager, session and owned view before writing parameters;
 * failures remain inside the callback boundary.
 *
 * Night vision must never unwind through an engine callback.
 *
 * The same world callback also renders the pilot's view. A context that is not the last validated AZP view's
 * is ignored before any inspection: that address only rejects. For our view, the manager identity, entry
 * table, pool and view chain are reinspected in one memory-query transaction, and the write still needs
 * NightVision::apply's fresh checks. The camera observer remains responsible for creation, pose and gates.
 */
void night_vision_observer(void* world, void* context, std::uintptr_t return_address) noexcept {
  auto& runtime = state();
  const CallbackTimer timer(runtime.night_vision_us);
  const bool requested = runtime.requested_night_vision.load(std::memory_order_acquire);
  if (!requested || !runtime.enabled.load(std::memory_order_acquire) || runtime.suspended.load())
    return;
  const auto owned_view = runtime.night_vision_view.load(std::memory_order_acquire);
  if (!owned_view || reinterpret_cast<std::uintptr_t>(context) != NightVision::render_context(owned_view))
    return;
  if (runtime.observing.test_and_set(std::memory_order_acquire)) {
    return;
  }
  struct Guard {
    Runtime& runtime;
    ~Guard() { runtime.observing.clear(std::memory_order_release); }
  } guard{runtime};
  try {
    if (!runtime.selected_night_vision || !runtime.night_vision.available())
      return;
    const auto ownership = runtime.ownership.snapshot();
    if (ownership.state != ec::State::active || !ownership.owner.valid() || !ownership.owned_id || ownership.request_pending ||
        ownership.creation_pending || ownership.failure != ec::Failure::none || ownership.blocked != ec::Blocked::none ||
        ownership.owned_id != runtime.scheduled_id || !runtime.gate || ownership.owner != runtime.token ||
        runtime.started_session_epoch != get_aircraft_session_epoch()) {
      return;
    }
    ec::OwnedViewSnapshot view;
    {
      ScopedLocalMemoryQueryCache queries(LocalMemoryQueryMode::private_pages);
      LocalMemoryReader objects(512);
      LocalImageReader image(reinterpret_cast<HMODULE>(runtime.base), runtime.image.image_size, LocalImageQueryMode::pages);
      const auto identity =
          inspect_manager_identity(image, objects, runtime.base, runtime.manager, runtime.owned_control, runtime.contract.layout);
      if (!identity || identity.manager != ownership.owner.identity || identity.generation != ownership.owner.generation ||
          identity.renderer != runtime.renderer)
        return;
      view = inspect_entry_reads(runtime, ownership.owned_id);
      if (!queries.finish())
        return;
    }
    runtime.night_vision.apply(view, world, context, return_address, runtime.night_vision_gain.load(std::memory_order_relaxed));
  } catch (...) {
  }
}

void apply_pose(Runtime& runtime, const ec::OwnedViewSnapshot& view, const MountedPose& pose, bool attached = false) noexcept {
  using SetVector = void (*)(void*, const double*);
  if (!attached)
    function<SetVector>(runtime, runtime.contract.functions.set_position)(reinterpret_cast<void*>(view.node_address), pose.position.data());
  function<SetVector>(runtime, runtime.contract.functions.set_up)(reinterpret_cast<void*>(view.camera_address), pose.up.data());
  function<SetVector>(runtime, runtime.contract.functions.set_target)(reinterpret_cast<void*>(view.camera_address), pose.target.data());
  function<void (*)(void*, float)>(runtime, runtime.contract.functions.set_fov)(reinterpret_cast<void*>(view.camera_address), pose.fov);
  function<void (*)(void*)>(runtime, runtime.contract.functions.update_view)(reinterpret_cast<void*>(view.view_address));
  apply_zoom_fade(view.view_address, pose.fov);
}

/*
 * Drives the owned view's render gate. Native activation results update the observer's gate evidence;
 * cached booleans alone cannot authorize a call.
 *
 * The verified false path ORs the separately verified {1,0} mask into P48/56. A gate is always closed before
 * it is (re)opened. On a new camera, explicitly close it even when setup already left bit0 set; no original
 * update runs between calls.
 */
void apply_gate(Runtime& runtime, ec::EntryId id, bool desired, const CameraStatus& inspected, bool initialize_gate) {
  using Activate = void (*)(void*, std::uint64_t, bool);
  const auto activate = function<Activate>(runtime, runtime.contract.functions.activate_entry);
  const bool observed_active = (inspected.flags[0] & 1u) == 0;
  if (initialize_gate || (!desired && (runtime.gate || observed_active)))
    activate(reinterpret_cast<void*>(runtime.manager), id, false);
  if (desired && (initialize_gate || !runtime.gate || !observed_active))
    activate(reinterpret_cast<void*>(runtime.manager), id, true);
  runtime.gate = desired;
}

/*
 * Native render pacing (EXE frame rate limit). Without a limit the owned view renders on every simulator frame. With a
 * limit of N frames per second it renders on a fixed schedule advanced by 1/N s per rendered frame, so N
 * frames per second are rendered even when the simulator rate is not a multiple of N; after a gap longer than
 * one interval the schedule restarts from now. Between due frames the caller keeps the render gate closed and
 * the engine skips the view.
 */
bool render_due(Runtime& runtime) noexcept {
  const auto rate = runtime.render_rate.load(std::memory_order_relaxed);
  if (!rate) {
    runtime.next_render_us = 0;
    return true;
  }
  const auto now = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
  const std::uint64_t interval = (1000000u + rate - 1) / rate;
  const auto due = runtime.next_render_us;
  if (due && now < due)
    return false;
  runtime.next_render_us = due && now - due < interval ? due + interval : now + interval;
  return true;
}

/*
 * Keeps the camera parked across an aircraft/flight session change until ownership, allocation and a fresh
 * pose agree. It consumes the queued transition only after successful revalidation, avoiding an
 * erase/recreate cycle for temporary waits.
 *
 * Serialized with Start requests while the controller separately excludes in-flight creation. This only
 * cancels unmaterialized mailbox work; it cannot erase, create or forget an owned native camera.
 *
 * A prior ready acknowledgement never authorizes resume after a failed fresh manager inspection. Missing
 * telemetry keeps this hold active.
 *
 * Runs only in the verified observer phase. Reuses the existing gate operation; a session change never calls
 * the native erase, create, resize or pose setters.
 */
void service_session_transition(Runtime& runtime, void* manager, CameraStatus& report) {
  SessionTransition transition;
  std::uint64_t token, start_revision;
  bool resume_requested;
  {
    const std::lock_guard lock(runtime.mutex);
    if (!runtime.session_transition.holding()) {
      report.ownership = runtime.ownership.snapshot();
      return;
    }
    if (runtime.session_transition.awaiting_ownership()) {
      const auto cancelled = runtime.ownership.cancel_uncreated_request();
      report.ownership = runtime.ownership.snapshot();
      if (cancelled == ec::EmptyCameraCancel::busy) {
        report.message = "Waiting for the current camera operation before the aircraft session change.";
        return;
      }
      runtime.session_transition.begin_published(report.ownership, runtime.published_allocation);
      if (runtime.session_transition.awaiting_ownership()) {
        report.message = "Waiting for the current camera allocation evidence before the aircraft session change.";
        return;
      }
      if (cancelled == ec::EmptyCameraCancel::cancelled) {
        report.message = "Pending camera startup cleared; ready for the new aircraft session.";
        return;
      }
    }
    transition = runtime.session_transition;
    token = runtime.session_transition_token;
    resume_requested = runtime.retained_restart_requested;
    start_revision = runtime.requested_start_revision;
  }
  const auto ownership = runtime.ownership.snapshot();
  report.ownership = ownership;
  bool validated = false;
  if (!transition.failed() && manager_context(runtime, manager)) {
    ec::OwnedViewSnapshot view{};
    inspect_camera(runtime, ownership.owned_id, report, view, false);
    const auto action = transition.inspect(runtime.token, ownership, view);
    if (action == SessionTransition::Decision::close) {
      const auto before = view;
      if (!manager_context(runtime, manager) || runtime.token != ownership.owner)
        transition.refuse();
      else {
        apply_gate(runtime, ownership.owned_id, false, report, true);
        inspect_camera(runtime, ownership.owned_id, report, view, false);
        const bool unchanged = view.complete && view.ready && view.view_address == before.view_address &&
                               view.node_address == before.node_address && view.camera_address == before.camera_address &&
                               view.resource_address == before.resource_address && (view.flags[0] & 1u);
        if (unchanged)
          transition.inspect(runtime.token, runtime.ownership.snapshot(), view);
        else
          transition.refuse();
      }
    }
    validated = transition.ready();
    if (validated)
      runtime.gate = false;
  }
  const auto resume_epoch = get_aircraft_session_epoch();
  const bool pose_ready = validated && resume_requested && aircraft_is_rafale() && capture_pose(runtime);
  report.pose_waiting = resume_requested && validated && !pose_ready;
  report.message = transition.failed() ? "Aircraft change paused: retained camera identity could not be validated. Restart MSFS to resume."
                   : !validated        ? "Waiting for the complete retained camera view before resuming."
                   : resume_requested && !pose_ready ? "Retained camera is closed; waiting for fresh aircraft pose."
                                                     : "Camera retained with its gate closed; ready for the new aircraft session.";
  const std::lock_guard lock(runtime.mutex);
  if (token == runtime.session_transition_token) {
    runtime.session_transition = transition;
    if (pose_ready && runtime.retained_restart_requested && start_revision == runtime.requested_start_revision &&
        runtime.recovery.requested() && resume_epoch == get_aircraft_session_epoch()) {
      runtime.session_transition.consume();
      runtime.retained_restart_requested = false;
      runtime.started_session_epoch = resume_epoch;
      scene_handoff().begin_scene();
    }
  }
}
/*
 * Prepares a native creation descriptor from a verified source view and fresh aircraft/WASM pose. It borrows
 * reviewed projection limits only and refuses initialization when those inputs are unavailable.
 *
 * Only a real creation request reaches this callback, after prior cleanup. Stop requests cannot call the
 * pose getters or descriptor initializer.
 *
 * Borrow only the verified main view far planes, never its pose or near plane.
 */
bool initialize(void* opaque, ec::DescriptorStorage& descriptor) noexcept {
  auto& runtime = *static_cast<Runtime*>(opaque);
  runtime.lifecycle_touched = true;
  if (runtime.creations == 0) {
    runtime.creation_pose_unavailable = false;
    runtime.far_planes = {};
    try {
      runtime.message.clear();
      runtime.stage_error = "";
      LocalMemoryReader reader;
      const auto pool = inspected(runtime, [&] { return ec::inspect_view_pool(reader, runtime.renderer); });
      runtime.creation_valid = pool.valid && pool.free_count >= 1;
      if (runtime.creation_valid) {
        const auto source = inspected(runtime, [&] { return inspect_source_view(reader, pool); });
        std::array<float, 2> planes{}, again{};
        runtime.creation_valid = source.complete && source.camera_address <= UINTPTR_MAX - 1532 &&
                                 reader.read(source.camera_address + 1524, planes.data(), sizeof(planes)) &&
                                 reader.read(source.camera_address + 1524, again.data(), sizeof(again)) && planes == again;
        for (const auto value : planes)
          runtime.creation_valid = runtime.creation_valid && std::isfinite(value) && value >= 1000 && value <= 10000000;
        if (runtime.creation_valid)
          runtime.far_planes = planes;
        else
          runtime.stage_error = "The main view rendering range could not be read consistently.";
      }
      if (!runtime.creation_valid) {
        if (!*runtime.stage_error)
          runtime.stage_error = pool.valid ? "No free engine view is available for the camera." : "The view pool could not be validated.";
      } else if (!(runtime.creation_valid = capture_pose(runtime))) {
        runtime.creation_error =
            runtime.message.empty() ? "A verified aircraft body pose is required before creating mounted views." : runtime.message;
        runtime.stage_error = runtime.creation_error.c_str();
        runtime.creation_pose_unavailable = runtime.pose_busy;
      }
    } catch (...) {
      runtime.creation_valid = false;
    }
  }
  if (!runtime.creation_valid)
    return false;
  function<void (*)(void*)>(runtime, runtime.contract.functions.initialize_descriptor)(descriptor.bytes.data());
  return true;
}

/*
 * Calls the proven native allocator, immediately retains any returned ID, then closes and validates the new
 * view. Output resizing waits for the required native warmup update; partial initialization cannot make
 * ownership disappear.
 *
 * Always return a created nonzero ID, even if validation fails. The controller must retain ownership so the
 * next failed stage can remove it.
 *
 * Setup may already have queued its initial primary-size allocation. Close this new owned gate before
 * pose/resolution work, while no original update has run. The existing engine mismatch path replaces its own
 * outputs.
 *
 * Retain the requested dimensions, but do not resize yet. The original manager gets one closed-gate update
 * to populate its primary-size cache.
 */
ec::EntryId create(void* opaque, ec::ManagerToken token, const ec::DescriptorStorage& descriptor) noexcept {
  auto& runtime = *static_cast<Runtime*>(opaque);
  runtime.lifecycle_touched = true;
  if (runtime.creations >= 1) {
    runtime.creation_valid = false;
    runtime.stage_error = "The owned-view creation limit was reached.";
    return 0;
  }
  if (!runtime.creation_valid || !manager_context(runtime, reinterpret_cast<void*>(token.identity)) || token != runtime.token) {
    runtime.stage_error = "Manager identity changed before camera creation.";
    return 0;
  }
  LocalMemoryReader reader;
  const auto pool = inspected(runtime, [&] { return ec::inspect_view_pool(reader, runtime.renderer); });
  if (!pool.valid || pool.free_count < 1) {
    runtime.creation_valid = false;
    runtime.stage_error = "Available view capacity changed before camera creation.";
    return 0;
  }
  const auto id = function<std::uint64_t (*)(void*, const void*)>(runtime, runtime.contract.functions.create_entry)(
      reinterpret_cast<void*>(token.identity), descriptor.bytes.data());
  if (!id) {
    runtime.stage_error = "The native creation call returned no owned ID.";
    return 0;
  }
  runtime.owned_control = runtime.control;
  ++runtime.creations;
  ++runtime.created_total;
  auto view = inspect_entry(runtime, id);
  runtime.creation_valid = view.complete && view.ready;
  if (!runtime.creation_valid)
    runtime.stage_error = view.complete ? "Entry setup was pending; rolling back the new camera."
                          : *view.error ? view.error
                                        : "The new entry failed owned-view validation.";
  if (runtime.creation_valid) {
    function<void (*)(void*, std::uint64_t, bool)>(runtime, runtime.contract.functions.activate_entry)(
        reinterpret_cast<void*>(runtime.manager), id, false);
    view = inspect_entry(runtime, id);
    runtime.creation_valid = view.complete && view.ready && (view.flags[0] & 1u);
    if (!runtime.creation_valid) {
      runtime.stage_error = "The new owned view could not be validated with its render gate closed.";
      return id;
    }
    if (!prepare_owned_view_render(runtime, id, view)) {
      runtime.creation_valid = false;
      return id;
    }
    apply_pose(runtime, view, runtime.mounted_pose);
    ViewDimensions desired{};
    if (!plan_view_resize(view.dimensions, native_camera_side.load(std::memory_order_relaxed), desired)) {
      runtime.creation_valid = false;
      runtime.stage_error = "The owned view dimensions cannot be validated for the camera resolution.";
      return id;
    }
    runtime.resized_dimensions = desired;
  }
  return id;
}

/*
 * Reinspects a closed owned entry and chooses an output (re)allocating resize, used at creation and after the
 * engine replaced the output, or proven in-place dimension restoration.
 * Far-plane and output changes are accepted only through the guarded view helpers.
 */
bool resize_closed_entry(Runtime& runtime, ec::EntryId id, bool initialize_output = true) {
  const auto view = inspect_entry(runtime, id);
  if (!view.complete || !view.ready || !(view.flags[0] & 1u)) {
    runtime.stage_error = "The owned view could not be validated with its render gate closed.";
    return false;
  }
  const auto desired = runtime.resized_dimensions;
  const ViewResizeCallbacks resize_callbacks{
      &runtime,
      [](void* opaque, std::uint64_t address) noexcept {
        auto& current = *static_cast<Runtime*>(opaque);
        function<void (*)(void*)>(current, current.contract.functions.update_view)(reinterpret_cast<void*>(address));
        return true;
      },
      [](void* opaque, std::uint64_t address) noexcept -> std::uint64_t {
        auto& current = *static_cast<Runtime*>(opaque);
        return reinterpret_cast<std::uintptr_t>(
            function<void* (*)(void*)>(current, current.contract.functions.refresh_output)(reinterpret_cast<void*>(address)));
      }};
  if (const auto* error = set_owned_view_far_planes(view, runtime.far_planes, resize_callbacks)) {
    runtime.stage_error = error;
    return false;
  }
  const auto resized = initialize_output ? resize_owned_view(view, desired, resize_callbacks)
                                         : restore_owned_view_dimensions(view, desired, resize_callbacks);
  if (!resized.complete) {
    runtime.stage_error = view_resize_status_name(resized.status);
    return false;
  }
  const auto confirmed = inspect_entry(runtime, id);
  const bool valid =
      confirmed.complete && confirmed.ready && confirmed.view_address == view.view_address && confirmed.node_address == view.node_address &&
      confirmed.camera_address == view.camera_address && confirmed.dimensions == desired && (confirmed.flags[0] & 1u) &&
      (initialize_output || (confirmed.mode == 2 && confirmed.resource_present && confirmed.output_dimensions == desired[0] &&
                             confirmed.resource_address == view.resource_address));
  if (!valid) {
    runtime.stage_error = "The resized owned-view chain did not remain stable; its render gate stays closed.";
    return false;
  }
  runtime.resized_id = id;
  return true;
}

/*
 * Removes an owned entry only after current identity, a ready chain and retirement evidence permit it.
 * Attachment is released in the same guarded lifecycle; an unavailable chain never counts as successful
 * removal.
 *
 * An ID-table match alone does not authorize native removal. Its ready path indexes the view pool; its
 * pending path skips normal view detachment while still destroying entry-held references. Never enter either
 * with an unavailable chain, or erase on the same update that closes an active gate.
 */
bool erase(void* opaque, ec::ManagerToken token, ec::EntryId id) noexcept {
  auto& runtime = *static_cast<Runtime*>(opaque);
  runtime.lifecycle_touched = true;
  if (!manager_context(runtime, reinterpret_cast<void*>(token.identity)) || token != runtime.token)
    return false;
  LocalMemoryReader reader;
  auto entries = inspected(runtime, [&] { return ec::inspect_owned_entries(reader, token.identity, id); });
  if (!entries.complete) {
    runtime.retirement.forget(token, id);
    return false;
  }
  if (!entries.entry.found) {
    runtime.retirement.forget(token, id);
    return true;
  }
  auto view = inspect_entry(runtime, id);
  const auto action = runtime.retirement.observe(token, id, runtime.updates, view);
  if (action == ViewRetirement::Action::close_gate) {
    function<void (*)(void*, std::uint64_t, bool)>(runtime, runtime.contract.functions.activate_entry)(
        reinterpret_cast<void*>(token.identity), id, false);
    return false;
  }
  if (action != ViewRetirement::Action::erase)
    return false;
  if (!release_camera_attachment(view, runtime.message))
    return false;
  function<void (*)(void*, std::uint64_t)>(runtime, runtime.contract.functions.erase_entry)(reinterpret_cast<void*>(token.identity), id);
  reader.reset_budget();
  entries = inspected(runtime, [&] { return ec::inspect_owned_entries(reader, token.identity, id); });
  const bool absent = entries.complete && !entries.entry.found;
  if (absent)
    runtime.retirement.forget(token, id);
  return absent;
}

/*
 * Records a failure only for the request revision that produced it. A late inspection cannot overwrite a
 * newer start/stop decision or replace a latched failure's original cause.
 *
 * An inspection belongs to the request it sampled. Do not overwrite recovery state that a newer explicit
 * Start/Stop has already replaced.
 *
 * Preserve the matching detail while a terminal identity/startup failure is retained. Only an explicit
 * Start/Stop may leave that state.
 */
void record_stop(Runtime& runtime,
                 StopReason reason,
                 const char* detail,
                 std::uint64_t now,
                 std::optional<std::uint64_t> expected_start_revision = std::nullopt) {
  const std::lock_guard lock(runtime.mutex);
  if (expected_start_revision && runtime.requested_start_revision != *expected_start_revision)
    return;
  if (latched_stop(runtime.recovery.reason()) && reason != runtime.recovery.reason())
    return;
  if (runtime.recovery.reason() == reason && runtime.stop_detail == detail && (runtime.recovery.pending() || !retryable_stop(reason)))
    return;
  runtime.recovery.failed(reason, now);
  runtime.stop_detail = detail;
}

/*
 * Detects a new flight session, withdraws the old image publication and places the retained camera into a
 * transition hold. Native revalidation must complete before that ownership can resume.
 *
 * Recheck at destructive fallback boundaries too: the public epoch may change while a full native inspection
 * is running. A flight change parks the ownership.
 */
bool hold_changed_session(Runtime& runtime, const ec::Snapshot& ownership) {
  const auto epoch = get_aircraft_session_epoch();
  const std::lock_guard lock(runtime.mutex);
  if (!SessionTransition::session_changed(ownership, runtime.started_session_epoch, epoch))
    return false;
  runtime.suspended.store(true);
  scene_handoff().stop_scene();
  runtime.requested_start = false;
  ++runtime.requested_start_revision;
  runtime.retained_restart_requested = false;
  if (!runtime.session_transition.holding()) {
    runtime.session_transition.begin(ownership, runtime.resized_dimensions);
    if (runtime.session_transition_token != UINT64_MAX)
      ++runtime.session_transition_token;
    else
      runtime.session_transition.refuse();
  }
  return true;
}
/*
 * Applies queued camera intent at the verified manager-update boundary. A reentry guard permits one
 * observation at a time. Idle updates avoid native object access; resume, lifecycle changes and active
 * continuous rendering require fresh validation rather than relying on the idle throttle.
 *
 * The update first checks the current session and manager, then handles retained-camera transitions or
 * queued lifecycle work. Read-only inspection may share a short-lived memory cache, but endpoint validation
 * must finish before publication or any native call. A failed inspection grants no permission to mutate its
 * borrowed objects.
 *
 * Initial creation retains every returned ID, closes its gate and waits for a native warmup update before
 * resizing. Partial creation/resize failures stay latched because another allocation attempt could duplicate
 * an uncertain result. Missing startup telemetry parks the request instead of repeatedly walking native
 * aircraft graphs.
 *
 * For an established camera, dimension drift is restored only when its existing bitmap already matches the
 * requested size. The camera is retained during AA/upscaler changes; pending outputs wait with closed gates.
 * Neither elapsed time nor image loss alone proves that an entry can be erased.
 *
 * A ready camera receives the validated current aircraft attachment and WASM aim before its render gate
 * opens. Missing guidance retains the closed camera and any completed image subject to normal expiry; no
 * stale transform or speculative forward view is applied. Pending entries receive no pose, resize or
 * activation call through unavailable pointers.
 *
 * The final snapshot is published only after the guarded update completes and still belongs to the sampled
 * request revision. C++ failures are contained at the engine boundary; this does not claim to catch access
 * violations or validate an unknown calling convention.
 */
void observer(void* manager) noexcept {
  auto& runtime = state();
  const CallbackTimer timer(runtime.observer_us);
  if (!runtime.enabled.load(std::memory_order_acquire) || runtime.observing.test_and_set(std::memory_order_acquire))
    return;
  struct Guard {
    Runtime& runtime;
    ~Guard() { runtime.observing.clear(std::memory_order_release); }
  } guard{runtime};
  try {
    ++runtime.updates;
    set_world_preload_view(-1);
    const auto before = runtime.ownership.snapshot();
    const auto now = GetTickCount64();
    const bool night_vision = runtime.requested_night_vision.load(std::memory_order_acquire);
    if (night_vision != runtime.selected_night_vision) {
      scene_handoff().invalidate_publication();
      runtime.selected_night_vision = night_vision;
    }
    bool requested_start = false;
    bool recovery_pending = false;
    bool view_ready = false;
    bool session_hold = false;
    const auto session_epoch = get_aircraft_session_epoch();
    std::uint64_t start_revision = 0;
    {
      const std::lock_guard lock(runtime.mutex);
      if (SessionTransition::session_changed(before, runtime.started_session_epoch, session_epoch) &&
          !runtime.session_transition.holding()) {
        runtime.suspended.store(true);
        scene_handoff().stop_scene();
        runtime.requested_start = false;
        ++runtime.requested_start_revision;
        runtime.retained_restart_requested = false;
        runtime.session_transition.begin(before, runtime.resized_dimensions);
        if (runtime.session_transition_token != UINT64_MAX)
          ++runtime.session_transition_token;
        else
          runtime.session_transition.refuse();
      }
      session_hold = runtime.session_transition.holding() && (before.owned_id || runtime.session_transition.awaiting_ownership());
      requested_start = runtime.requested_start;
      start_revision = runtime.requested_start_revision;
      recovery_pending = runtime.recovery.pending() || runtime.published.view_waiting || runtime.published.pose_waiting ||
                         runtime.resize_recovery.pending() || runtime.resize_recovery.failed();
      view_ready = runtime.published.ready;
    }
    /*
     * The camera renders continuously while it is not paused.
     */
    const bool scheduled = before.state == ec::State::active && runtime.scheduled_id == before.owned_id;
    const bool suspended = runtime.suspended.load();
    bool desired = scheduled && !suspended;
    const bool gate_change = scheduled && desired != runtime.gate;
    const InspectionState inspection_state{scheduled && before.owner.valid() && before.owned_id && runtime.resized_id == before.owned_id &&
                                               before.failure == ec::Failure::none && before.blocked == ec::Blocked::none,
                                           suspended,
                                           !runtime.gate,
                                           view_ready,
                                           before.request_pending || before.creation_pending,
                                           requested_start,
                                           runtime.resize_warmup.pending(),
                                           recovery_pending || session_hold,
                                           false};
    const auto inspection = runtime.inspection_gate.decide(inspection_state);
    if (inspection == InspectionDecision::idle)
      return;
    const bool continuous_view = scheduled && !suspended;
    if (!continuous_view && inspection != InspectionDecision::required && !before.request_pending && !gate_change &&
        !runtime.resize_warmup.pending() && now - runtime.last_inspection < 250)
      return;
    runtime.last_inspection = now;
    runtime.inspection_changed = false;
    CameraStatus report;
    report.night_vision = runtime.selected_night_vision;
    bool body_pose_failed = false;
    report.accepting_requests = true;
    const bool close_only = inspection_state.established && view_ready && gate_change && !desired && (runtime.gate) &&
                            !before.request_pending && !before.creation_pending && !requested_start && !runtime.resize_warmup.pending() &&
                            !runtime.resize_recovery.pending() && !runtime.resize_recovery.failed() && !recovery_pending;
    const bool fuse_inspection = inspection_state.established && view_ready && !before.request_pending && !before.creation_pending &&
                                 !requested_start && !runtime.resize_warmup.pending() && !runtime.resize_recovery.pending() &&
                                 !runtime.resize_recovery.failed() && !recovery_pending && !session_hold;
    bool prepared = false;
    ec::OwnedViewSnapshot prepared_view{};
    SceneCaptureTicket prepared_ticket{};
    std::uint32_t prepared_free_views = 0;
    ManagerInspection manager_inspection;
    const auto inspect_manager = [&] {
      if (fuse_inspection) {
        prepared_ticket = scene_handoff().begin_capture();
        ScopedLocalMemoryQueryCache queries(LocalMemoryQueryMode::private_pages);
        const bool manager_valid = manager_context(runtime, manager, &queries, &manager_inspection);
        if (manager_valid && runtime.token == before.owner)
          inspect_camera(runtime, before.owned_id, report, prepared_view, false, &queries);
        const bool stable = queries.finish();
        if (manager_valid && stable) {
          prepared = runtime.token == before.owner;
          prepared_free_views = report.free_views;
          return true;
        }
        prepared_view = {};
        report.ready = report.resource_present = false;
        report.dimensions = {};
        report.flags = {};
      }
      return manager_context(runtime, manager, nullptr, &manager_inspection);
    };
    const auto start_body = requested_start ? sample_body_pose(now) : BodyPoseSnapshot{};
    if (session_hold) {
      service_session_transition(runtime, manager, report);
    } else if (suspended && !before.owned_id) {
      report.ownership = before;
      report.message = "Initial camera creation parked until render demand resumes.";
    } else if (requested_start && !before.owned_id && !start_body.valid) {
      report.ownership = before;
      report.message = std::string("Waiting for read-only aircraft telemetry: ") + start_body.error;
    } else if (close_only && close_owned_view(runtime, manager, before, report)) {
      /* The gate was closed with the lighter close-only inspection; nothing else is due. */
    } else if (!inspect_manager()) {
      scene_handoff().stop_scene();
      const bool temporary = manager_inspection.temporary();
      record_stop(runtime, temporary ? StopReason::inspection_unavailable : StopReason::identity_refused, manager_inspection.detail, now,
                  start_revision);
      {
        const std::lock_guard lock(runtime.mutex);
        if (requested_start && runtime.requested_start && runtime.requested_start_revision == start_revision) {
          runtime.requested_start = false;
          ++runtime.requested_start_revision;
        }
      }
      if (temporary)
        runtime.ownership.request_disable();
      report.message =
          std::string(temporary ? "Camera manager inspection temporarily unavailable: " : "Camera manager identity refused: ") +
          manager_inspection.detail;
      report.ownership = runtime.ownership.snapshot();
    } else {
      LocalMemoryReader reader;
      ec::ViewPoolSnapshot pool;
      if (before.state != ec::State::active || before.request_pending || requested_start || runtime.resize_warmup.pending())
        pool = ([&] { return inspected(runtime, [&] { return ec::inspect_view_pool(reader, runtime.renderer); }); }());
      report.free_views = pool.valid ? pool.free_count : 0;
      runtime.creations = 0;
      runtime.creation_pose_unavailable = false;
      runtime.lifecycle_touched = false;
      runtime.creation_valid = pool.valid && pool.free_count >= 1;
      ec::EngineCallbacks callbacks{&runtime, initialize, create, erase};
      {
        runtime.ownership.process_update(runtime.token, callbacks);
      }
      auto ownership = runtime.ownership.snapshot();
      if (!requested_start && !ownership.owned_id) {
        const auto retry_body = sample_body_pose(GetTickCount64());
        const bool fresh_pose = retry_body.valid;
        const std::lock_guard lock(runtime.mutex);
        if (!runtime.requested_start && runtime.requested_start_revision == start_revision && !runtime.suspended.load() &&
            runtime.recovery.retry(GetTickCount64(), ownership, fresh_pose)) {
          runtime.requested_start = requested_start = true;
          start_revision = ++runtime.requested_start_revision;
        }
      }
      bool waiting_for_body = false;
      bool resolution_pause = false;
      bool attachment_failed = false;
      bool camera_attached = false;
      if (requested_start && !ownership.owned_id) {
        waiting_for_body = !capture_pose(runtime);
        bool create_requested = false;
        if (!waiting_for_body) {
          const std::lock_guard lock(runtime.mutex);
          if (runtime.requested_start && runtime.requested_start_revision == start_revision && !runtime.suspended.load()) {
            scene_handoff().begin_scene();
            runtime.ownership.request_enable();
            runtime.requested_start = false;
            create_requested = true;
          }
        }
        if (create_requested) {
          {
            runtime.ownership.process_update(runtime.token, callbacks);
          }
          ownership = runtime.ownership.snapshot();
          if (ownership.state != ec::State::active) {
            bool deferred = false;
            {
              const std::lock_guard lock(runtime.mutex);
              deferred = defer_initial_pose_failure(runtime.ownership, runtime.token, runtime.recovery, runtime.creation_pose_unavailable,
                                                    runtime.creations, start_revision, runtime.requested_start_revision, now);
              if (deferred)
                runtime.stop_detail = runtime.creation_error;
            }
            if (deferred) {
              ownership = runtime.ownership.snapshot();
              waiting_for_body = report.pose_waiting = true;
            } else {
              record_stop(runtime, StopReason::creation_failed,
                          *runtime.stage_error ? runtime.stage_error : "Owned camera creation did not complete.", now, start_revision);
            }
          }
        }
      }
      if (ownership.state == ec::State::active) {
        if (runtime.creations != 0) {
          reader.reset_budget();
          pool = ([&] { return inspected(runtime, [&] { return ec::inspect_view_pool(reader, runtime.renderer); }); }());
          report.free_views = pool.valid ? pool.free_count : 0;
          runtime.creation_valid = runtime.creation_valid && runtime.resize_warmup.begin(ownership.owned_id, runtime.updates);
          runtime.gate = false;
        }
        const bool closed_warmup = runtime.creations != 0 && runtime.creation_valid && runtime.resize_warmup.pending();
        bool initial_resize_failed = false;
        if (!closed_warmup && runtime.resize_warmup.pending()) {
          runtime.stage_error = "The initial closed-gate resize phase could not be validated.";
          runtime.creation_valid =
              runtime.resize_warmup.may_resize(ownership.owned_id, runtime.updates) && resize_closed_entry(runtime, ownership.owned_id);
          if (runtime.creation_valid)
            runtime.resize_warmup.finish(ownership.owned_id, runtime.updates);
          else
            initial_resize_failed = true;
        }
        ec::OwnedViewSnapshot view{};
        if (!closed_warmup && !initial_resize_failed) {
          if (prepared && !runtime.lifecycle_touched && ownership.owner == before.owner && ownership.owned_id == before.owned_id &&
              ownership.failure == ec::Failure::none && ownership.blocked == ec::Blocked::none && !ownership.request_pending &&
              !ownership.creation_pending) {
            view = prepared_view;
            report.free_views = prepared_free_views;
            if (report.ready)
              publish_outputs(runtime, prepared_ticket, ownership.owned_id, view, report);
          } else {
            inspect_camera(runtime, ownership.owned_id, report, view);
          }
          runtime.night_vision_view.store(report.ready ? view.view_address : 0, std::memory_order_release);
        }
        const bool wait_for_view = runtime.view_wait.observe(
            now, ownership,
            !runtime.resize_warmup.pending() && runtime.scheduled_id == ownership.owned_id && runtime.resized_id == ownership.owned_id &&
                runtime.inspection_stop == StopReason::inspection_unavailable,
            view);
        resolution_pause = runtime.resize_recovery.pending() || runtime.resize_recovery.failed() ||
                           (runtime.inspection_stop == StopReason::resolution_changed && runtime.scheduled_id == ownership.owned_id &&
                            runtime.resized_id == ownership.owned_id);
        if (initial_resize_failed && hold_changed_session(runtime, ownership)) {
          report.view_waiting = true;
          runtime.message = "Aircraft changed during initial resizing; retaining the camera ID for fresh validation.";
        } else if (initial_resize_failed) {
          runtime.message = std::string("Initial camera resize failed: ") + runtime.stage_error;
          record_stop(runtime, StopReason::creation_failed, runtime.message.c_str(), now, start_revision);
          scene_handoff().stop_scene();
          runtime.ownership.request_disable();
          {
            runtime.ownership.process_update(runtime.token, callbacks);
          }
          ownership = runtime.ownership.snapshot();
        } else if (resolution_pause) {
          if (!runtime.resize_recovery.pending() && !runtime.resize_recovery.failed()) {
            runtime.resize_recovery.begin(ownership.owner, ownership.owned_id);
            record_stop(runtime, StopReason::resolution_changed,
                        "Primary dimensions changed; camera ID retained and output allocation prohibited.", now);
          }
          const auto action = runtime.resize_recovery.observe(ownership.owner, ownership.owned_id, runtime.updates, view);
          if (view.complete && view.ready && view.status == ec::OwnedViewStatus::ready) {
            if ((view.flags[0] & 1u) == 0)
              function<void (*)(void*, std::uint64_t, bool)>(runtime, runtime.contract.functions.activate_entry)(
                  reinterpret_cast<void*>(runtime.manager), ownership.owned_id, false);
            runtime.gate = false;
          }
          runtime.message = "Graphics settings changed; camera ID retained while its render gate closes.";
          bool restored_now = false;
          if (action == ViewResizeRecovery::Action::resize) {
            /*
             * An AA/upscaler change keeps the output bitmap at the camera resolution: restore the fields in place.
             * A renderer reset (2D/VR switch) has already replaced it with one of the primary size, which capture
             * never recognizes: rebuild it at the camera resolution through the closed-gate initial resize.
             */
            const auto output = view.output_dimensions;
            const bool outputs_unchanged = view.mode == 2 && view.resource_present && output == runtime.resized_dimensions[0];
            if (resize_closed_entry(runtime, ownership.owned_id, !outputs_unchanged) &&
                runtime.resize_recovery.finish(ownership.owner, ownership.owned_id)) {
              {
                const std::lock_guard lock(runtime.mutex);
                runtime.recovery.resumed_retained_resolution();
                runtime.stop_detail.clear();
              }
              runtime.message = outputs_unchanged
                                    ? "Camera dimensions restored without replacing entries or output textures; waiting for fresh frames."
                                    : "Camera output rebuilt at its resolution after the simulator reallocated it at " +
                                          std::to_string(output[0]) + " x " + std::to_string(output[1]) + "; waiting for fresh frames.";
              inspect_camera(runtime, ownership.owned_id, report, view);
              restored_now = report.ready;
            } else {
              runtime.resize_recovery.mark_failed();
            }
          }
          if (runtime.resize_recovery.failed()) {
            scene_handoff().stop_scene();
            runtime.message = "Camera paused after graphics change: its output could not be safely restored. Restart MSFS to resume.";
          }
          if (restored_now)
            report.view_waiting = false;
          else {
            report.ready = report.resource_present = false;
            report.view_waiting = true;
          }
        } else if (closed_warmup) {
          /* A new view waits, gate closed, for one original engine update before its initial resize. */
        } else if (!runtime.resize_warmup.pending() && report.ready && (runtime.creations == 0 || runtime.creation_valid)) {
          const bool new_camera = runtime.scheduled_id != ownership.owned_id;
          desired = !runtime.suspended.load();
          const bool needs_pose = desired;
          const bool pose_ready = !needs_pose || capture_pose(runtime);
          bool aa_ready = true;
          if (pose_ready && needs_pose) {
            if (prepare_owned_view_render(runtime, ownership.owned_id, view))
              report.flags = view.flags;
            else
              aa_ready = false;
          }
          bool attached = false;
          const bool attachment_ready =
              !pose_ready || !aa_ready || !needs_pose ||
              prepare_camera_attachment(view, runtime.aircraft_scene, runtime.ima_camera, attached, runtime.message);
          attachment_failed = !attachment_ready;
          camera_attached = attached;
          if (pose_ready && aa_ready && attachment_ready) {
            if (needs_pose)
              apply_pose(runtime, view, runtime.mounted_pose, attached);
            /* A paced-closed gate still belongs to a streaming camera: keep its preload registered. */
            apply_gate(runtime, ownership.owned_id, desired && render_due(runtime), report, new_camera);
            runtime.scheduled_id = ownership.owned_id;
            if (desired && !runtime.suspended.load())
              set_world_preload_view(view.view_index);
          } else if (!aa_ready || !attachment_ready) {
            apply_gate(runtime, ownership.owned_id, false, report, true);
            runtime.scheduled_id = ownership.owned_id;
            report.view_waiting = true;
            runtime.message = attachment_ready ? std::string("Camera render configuration unavailable; retaining the closed camera: ") +
                                                     runtime.stage_error
                                               : "Camera attachment unavailable; view closed: " + runtime.message;
          } else {
            body_pose_failed = true;
            apply_gate(runtime, ownership.owned_id, false, report, true);
            if (runtime.pose_busy) {
              runtime.scheduled_id = ownership.owned_id;
              report.pose_waiting = true;
            } else if (hold_changed_session(runtime, ownership)) {
              report.pose_waiting = true;
              runtime.message = "Aircraft changed during pose inspection; retaining the closed camera.";
            } else {
              record_stop(runtime, StopReason::pose_invalid, runtime.message.c_str(), now);
              scene_handoff().stop_scene();
              runtime.ownership.request_disable();
              {
                runtime.ownership.process_update(runtime.token, callbacks);
              }
              ownership = runtime.ownership.snapshot();
              runtime.stage_error = "Aircraft body pose became unavailable; the owned camera gate was closed and removal requested.";
            }
          }
        } else if (wait_for_view) {
          if (report.ready) {
            if (runtime.gate || (report.flags[0] & 1u) == 0)
              function<void (*)(void*, std::uint64_t, bool)>(runtime, runtime.contract.functions.activate_entry)(
                  reinterpret_cast<void*>(runtime.manager), ownership.owned_id, false);
            runtime.gate = false;
          }
          report.view_waiting = true;
        } else if (hold_changed_session(runtime, ownership)) {
          report.view_waiting = true;
          runtime.message = "Aircraft changed during view inspection; retaining the camera ID for fresh validation.";
        } else {
          record_stop(runtime, runtime.inspection_stop == StopReason::none ? StopReason::identity_refused : runtime.inspection_stop,
                      *runtime.stage_error ? runtime.stage_error : "Owned view validation failed; removal requested.", now);
          scene_handoff().stop_scene();
          runtime.ownership.request_disable();
          {
            runtime.ownership.process_update(runtime.token, callbacks);
          }
          ownership = runtime.ownership.snapshot();
          runtime.message = *runtime.stage_error ? runtime.stage_error : "Owned view validation failed; removal requested.";
        }
      }
      if (!ownership.owned_id) {
        forget_camera_attachment();
        scene_handoff().stop_scene();
        runtime.night_vision_view.store(0, std::memory_order_release);
        runtime.owned_control = 0;
        runtime.scheduled_id = 0;
        runtime.resized_id = 0;
        runtime.resized_dimensions = {};
        runtime.resize_warmup.clear();
        runtime.resize_recovery.clear();
        runtime.view_wait.clear();
        runtime.retirement.clear();
        runtime.gate = false;
      }
      report.ownership = ownership;
      if (ownership.state != ec::State::active) {
        report.ready = false;
        report.resource_present = false;
        report.dimensions = {};
        report.flags = {};
      }
      if (resolution_pause || attachment_failed)
        report.message = runtime.message;
      else if (body_pose_failed || waiting_for_body)
        report.message = runtime.message.empty() ? runtime.stage_error : runtime.message;
      else if (report.view_waiting)
        report.message = "Owned view inspection unavailable; camera ID retained while waiting for fresh validation.";
      else if (ownership.state == ec::State::active && runtime.resize_warmup.pending())
        report.message = "The new camera view is closed for its initial engine update; final resizing is pending.";
      else if (ownership.state == ec::State::active)
        report.message = camera_attached ? "The camera is attached to the aircraft node; aiming follows the native camera calculation."
                                         : "The camera view renders continuously with a freshly validated aircraft mount.";
      else if (ownership.state == ec::State::cleanup_pending)
        report.message = "Removal waits for a freshly validated, closed view; the owned ID remains retained until confirmed absent.";
      else if (ownership.state == ec::State::disabled)
        report.message = "Camera stopped; the owned ID is confirmed absent.";
      else if (!runtime.message.empty())
        report.message = runtime.message;
      else if (*runtime.stage_error)
        report.message = runtime.stage_error;
      else
        report.message = pool.valid && pool.free_count >= 1 ? "Camera creation failed; inspect readiness and the owned ID."
                                                            : "Camera creation refused: a free engine view is required.";
    }
    report.created_total = runtime.created_total;
    report.resolution = report.ownership.owned_id && runtime.resized_id == report.ownership.owned_id
                            ? static_cast<std::uint32_t>(runtime.resized_dimensions[0][0])
                            : 0;
    report.night_vision_message = runtime.selected_night_vision ? runtime.night_vision.message() : "TV";
    {
      const std::lock_guard lock(runtime.mutex);
      runtime.published_allocation = {report.ownership.owner, report.ownership.owned_id, runtime.resized_dimensions};
      report.session_transition_token = runtime.session_transition_token;
      report.session_transition_pending = runtime.session_transition.pending();
      report.session_transition_ready = runtime.session_transition.ready();
      report.session_transition_failed = runtime.session_transition.failed();
      report.accepting_requests = runtime.recovery.requested();
      report.recovery_pending = runtime.recovery.pending();
      report.restart_pending = runtime.requested_start;
      report.stop_sequence = runtime.recovery.sequence();
      report.stop_reason = runtime.recovery.reason();
      runtime.published = std::move(report);
    }
  } catch (...) {
    scene_handoff().stop_scene();
    runtime.ownership.request_disable();
    try {
      record_stop(runtime, StopReason::exception, "Observer exception; owned cleanup requested without automatic retry.", GetTickCount64());
    } catch (...) {
    }
  }
}
}  /* namespace */

/*
 * On first use, takes the resolved engine contracts (engine/engine_api.cpp) and installs the guarded
 * observers, then queues a start with the current session. Contract failure is reported instead of falling
 * back to old addresses; repeated starts use the existing ownership/recovery state.
 *
 * This build declares the slot as read-only, non-executable image data, while the Xbox loader maps its page
 * executable/write-copy. The hook rechecks the bounds and preserves execute permission through Windows'
 * copy-on-write promotion. Anonymous/executable PE sections are refused.
 */
void start_camera(bool reuse_camera) noexcept {
  auto& runtime = state();
  const std::lock_guard start_lock(runtime.start_mutex);
  {
    const std::lock_guard lock(runtime.mutex);
    runtime.published.accepting_requests = false;
  }
  try {
    if (!runtime.hooked.load(std::memory_order_acquire)) {
      const auto& api = engine_api();
      if (!api.valid) {
        const std::lock_guard lock(runtime.mutex);
        runtime.published.message = "Camera compatibility check failed: " + api.error;
        return;
      }
      runtime.image = api.image;
      runtime.base = api.base;
      runtime.contract = api.camera;
      runtime.night_vision.initialize(api.night_vision, api.base, api.image.image_size, night_vision_observer);
      if (!runtime.night_vision.available())
        throw std::runtime_error("Camera compatibility check failed: " + runtime.night_vision.message());
      std::string attachment_error;
      if (!initialize_camera_attachment(api.attachment, api.image, api.base, api.camera, attachment_error))
        throw std::runtime_error(attachment_error);
      LocalImageReader reader(reinterpret_cast<HMODULE>(runtime.base), runtime.image.image_size);
      std::uint64_t original = 0;
      const auto update_slot = runtime.contract.layout.manager_vtable + 15 * 8;
      const auto update = runtime.contract.functions.manager_update;
      if (!reader.read(update_slot, &original, sizeof(original)) || original != runtime.base + update)
        throw std::runtime_error("Manager update slot does not match the resolved camera contract.");
      HMODULE pinned = nullptr;
      if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN, reinterpret_cast<LPCWSTR>(&observer),
                              &pinned))
        throw std::runtime_error("Pinning the observer module failed.");
      const auto section = std::find_if(runtime.image.sections.begin(), runtime.image.sections.end(), [update_slot](const auto& item) {
        return (item.flags & 0x40000000u) && !(item.flags & 0xa2000000u) && update_slot >= item.rva &&
               update_slot - item.rva <= item.size && sizeof(void*) <= item.size - (update_slot - item.rva);
      });
      if (section == runtime.image.sections.end())
        throw std::runtime_error("Camera compatibility check failed: manager update slot moved outside static image data.");
      const engine_hook::ImageDataSlotProof image_data{reinterpret_cast<HMODULE>(runtime.base), runtime.image.image_size, section->rva,
                                                       section->size};
      const auto hook = engine_hook::install(reinterpret_cast<void**>(runtime.base + update_slot),
                                             reinterpret_cast<void*>(runtime.base + update), observer, &image_data);
      runtime.hooked.store(hook.pointer_changed, std::memory_order_release);
      runtime.protection_ready = hook.protection_restored;
      if (!hook.pointer_changed || !hook.protection_restored) {
        const std::lock_guard lock(runtime.mutex);
        runtime.published.message = std::string("Update observer refused: ") + engine_hook::status_name(hook.status);
        return;
      }
    }
    if (!runtime.protection_ready) {
      const auto restored = engine_hook::restore_protection();
      runtime.protection_ready = restored.protection_restored;
      if (!runtime.protection_ready)
        throw std::runtime_error("Update-slot page protection has not been restored; native requests remain disabled.");
    }
    if (!start_simconnect_link())
      throw std::runtime_error("The read-only aircraft telemetry provider could not be started.");
    if (!reuse_camera || !sample_body_pose(GetTickCount64()).valid)
      reset_body_pose_calibration();
    const std::lock_guard lock(runtime.mutex);
    const auto ownership = runtime.ownership.snapshot();
    const bool retained = runtime.session_transition.can_resume(ownership);
    if (runtime.session_transition.holding() && ownership.owned_id && !retained) {
      runtime.published.message = "Retained transition is not ready; no creation or removal requested.";
      return;
    }
    if (retained)
      runtime.retained_restart_requested = true;
    else {
      runtime.ownership.request_disable();
      runtime.session_transition.consume();
      runtime.started_session_epoch = get_aircraft_session_epoch();
    }
    runtime.recovery.start();
    runtime.stop_detail.clear();
    runtime.requested_start = !retained;
    ++runtime.requested_start_revision;
    runtime.enabled.store(true, std::memory_order_release);
    runtime.published.accepting_requests = true;
    runtime.published.message = "Camera start queued; waiting for a verified aircraft body pose.";
  } catch (const std::exception& error) {
    const std::lock_guard lock(runtime.mutex);
    runtime.published.message = error.what();
  } catch (...) {
    const std::lock_guard lock(runtime.mutex);
    runtime.published.message = "Native camera initialization failed.";
  }
}

/*
 * Queues a tokenized session transition and withdraws current image demand. Observer-owned fields are not
 * read from this control thread; published allocation evidence, which is mailbox-protected and survives
 * temporarily unavailable snapshots, describes what must be revalidated.
 */
std::uint64_t begin_session_transition() noexcept {
  auto& runtime = state();
  const std::lock_guard start_lock(runtime.start_mutex);
  const std::lock_guard lock(runtime.mutex);
  if (runtime.session_transition_token == UINT64_MAX)
    return 0;
  runtime.suspended.store(true);
  scene_handoff().stop_scene();
  runtime.requested_start = false;
  ++runtime.requested_start_revision;
  runtime.retained_restart_requested = false;
  runtime.recovery.stop();
  const auto cancelled = runtime.ownership.cancel_uncreated_request();
  if (cancelled == ec::EmptyCameraCancel::busy)
    runtime.session_transition.defer_ownership();
  else
    runtime.session_transition.begin_published(runtime.ownership.snapshot(), runtime.published_allocation);
  const auto token = ++runtime.session_transition_token;
  runtime.published.session_transition_token = token;
  runtime.published.session_transition_pending = runtime.session_transition.pending();
  runtime.published.session_transition_ready = runtime.session_transition.ready();
  runtime.published.session_transition_failed = runtime.session_transition.failed();
  return token;
}
/*
 * Withdraws preload and image publication, queues disable intent and stops public telemetry. Actual native
 * ownership changes remain the responsibility of a verified observer update.
 */
void stop_camera(bool keep_telemetry) noexcept {
  set_world_preload_view(-1);
  auto& runtime = state();
  const std::lock_guard start_lock(runtime.start_mutex);
  {
    const std::lock_guard lock(runtime.mutex);
    runtime.requested_start = false;
    runtime.retained_restart_requested = false;
    runtime.published.accepting_requests = false;
    ++runtime.requested_start_revision;
    runtime.recovery.stop();
    runtime.stop_detail = "Explicit camera OFF/Stop requested.";
    scene_handoff().stop_scene();
    const auto ownership = runtime.ownership.snapshot();
    if (!runtime.session_transition.holding() || !ownership.owned_id) {
      runtime.session_transition.consume();
      runtime.ownership.request_disable();
    }
  }
  if (!keep_telemetry)
    stop_simconnect_link();
}

void note_capture_progress(std::uint64_t now_ms) noexcept {
  auto& runtime = state();
  const std::lock_guard lock(runtime.mutex);
  runtime.recovery.capture_progress(now_ms);
}

ObserverCost observer_cost() noexcept {
  const auto& runtime = state();
  return {runtime.observer_us.load(std::memory_order_relaxed), runtime.night_vision_us.load(std::memory_order_relaxed)};
}

void request_night_vision(bool enabled) noexcept {
  state().requested_night_vision.store(enabled, std::memory_order_release);
}

void set_night_vision_gain(float gain) noexcept {
  state().night_vision_gain.store(gain, std::memory_order_relaxed);
}

void set_render_rate(std::uint32_t frames_per_second) noexcept {
  state().render_rate.store(frames_per_second, std::memory_order_relaxed);
}

void suspend_camera(bool suspended) noexcept {
  state().suspended.store(suspended);
  if (suspended)
    set_world_preload_view(-1);
}

/*
 * Copies the published observer status and current request/recovery state under the mutex. Borrowed native
 * pointers are not exposed as permission for the control thread to call the engine.
 */
CameraStatus camera_status() {
  auto& runtime = state();
  const std::lock_guard lock(runtime.mutex);
  auto result = runtime.published;
  result.ownership = runtime.ownership.snapshot();
  result.session_transition_token = runtime.session_transition_token;
  result.session_transition_pending = runtime.session_transition.pending();
  result.session_transition_ready = runtime.session_transition.ready();
  result.session_transition_failed = runtime.session_transition.failed();
  result.recovery_pending = runtime.recovery.pending();
  result.restart_pending = runtime.requested_start;
  result.stop_sequence = runtime.recovery.sequence();
  result.stop_reason = runtime.recovery.reason();
  return result;
}

}  /* namespace azp::native_camera */

namespace azp {
const char* camera_transition_message(const native_camera::CameraStatus& scene) noexcept {
  if (scene.session_transition_failed)
    return "Aircraft change paused: camera lifecycle validation failed. Restart MSFS to retry.";
  if (!scene.ownership.owned_id)
    return "Waiting for pending camera startup to finish before the aircraft session change.";
  return "Closing and validating the retained camera view for the new aircraft session.";
}

/*
 * Translates lifecycle failure categories into an operational message, distinguishing a wait/retry from a
 * latched refusal.
 */
const char* camera_stop_message(const native_camera::CameraStatus& scene) noexcept {
  using native_camera::StopReason;
  if (scene.stop_reason == StopReason::none || scene.stop_reason == StopReason::explicit_stop ||
      scene.stop_reason == StopReason::resolution_changed)
    return nullptr;
  if (native_camera::retryable_stop(scene.stop_reason)) {
    if (scene.recovery_pending)
      return "Camera startup interrupted; waiting for safe cleanup before retrying.";
    if (scene.restart_pending)
      return "Retrying camera startup; waiting for a validated aircraft position and camera state.";
    if (scene.ownership.state == engine_camera::State::active)
      return nullptr;
    return "Camera startup stopped after validation failed. See the camera status and the bridge log for details.";
  }
  switch (scene.stop_reason) {
    case StopReason::identity_refused:
      return "Camera safety check failed: simulator camera identity could not be verified. See the camera status and the bridge log for "
             "details.";
    case StopReason::pose_invalid:
      return "Camera startup stopped: aircraft position could not be validated. See the camera status and the bridge log for details.";
    case StopReason::creation_failed:
      return "Camera startup failed: the simulator did not create the camera view. See the camera status and the bridge log for details.";
    case StopReason::capture_stalled:
      return "Camera capture stopped. See the camera status and the bridge log for details.";
    case StopReason::exception:
      return "Camera startup stopped after an internal error. See the camera status and the bridge log for details.";
    default:
      return nullptr;
  }
}

}  /* namespace azp */
