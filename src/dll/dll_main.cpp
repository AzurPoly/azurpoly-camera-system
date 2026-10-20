/*
 * Runs the camera service inside MSFS. A worker reads the EXE mailbox and WASM demand, coordinates native
 * camera ownership and GPU image delivery, and returns operational status. Native camera mutations are
 * delegated to verified engine callbacks; this worker does not move the aircraft or draw the menu.
 */

#include "../camera/simconnect_link.hpp"
#include "../camera/camera.hpp"
#include "../camera/scenery_preload.hpp"
#include "../camera/zoom_detail.hpp"
#include "../engine/engine_api.hpp"
#include "../video/capture_progress.hpp"
#include "../video/display_exposure.hpp"
#include "../shared/protocol.hpp"
#include "../shared/configuration.hpp"
#include "../video/d3d12_bridge.hpp"
#include "../video/native_hooks.hpp"
#include "../video/queue_hooks.hpp"
#include "diagnostic.hpp"
#include <atomic>
#include <cstdio>
#include <cstring>
#include <string>

namespace {
using namespace azp;
namespace win = standalone;
std::atomic<bool> started{};
/*
 * The EXE control channel. It outlives the control loop because the SimConnect worker also reads it for
 * presence (below).
 */
win::Mailbox mailbox;
/*
 * EXE presence for the aircraft: the EXE's counter while its link is alive and ON, otherwise 0. It follows
 * the EXE link and the ON/OFF button only, so the aircraft keeps its IMA screen while the camera loads,
 * streams, pauses or changes session. Only the SimConnect worker calls it, on its own thread with its own
 * mailbox cache, so a long camera start on the control thread cannot delay it.
 */
std::uint64_t presence() noexcept {
  static win::CompanionControl control;
  control.refresh(mailbox);
  return control.connected(GetTickCount64()) && control.settings().enabled ? control.settings().heartbeat_counter : 0;
}
/*
 * Appends a compact status event to this DLL load's bridge log (diagnostic.hpp). A missing log folder loses
 * the line but does not stop camera service.
 */
void log_status(const win::Status& s, const char* detail = "") {
  char line[2048];
  const auto length = std::snprintf(line, sizeof(line),
                                    "%llu graphics=%u camera=%u streaming=%u uploads=%llu captures=%llu images=%llu "
                                    "input_copies=%llu hook_failures=%llu cpu_ms=%.1f/%.1f | %s | %s",
                                    static_cast<unsigned long long>(GetTickCount64()), s.graphics_ready, s.camera_ready, s.streaming,
                                    static_cast<unsigned long long>(s.wasm_uploads), static_cast<unsigned long long>(s.captures),
                                    static_cast<unsigned long long>(s.images), static_cast<unsigned long long>(s.input_copies),
                                    static_cast<unsigned long long>(s.hook_failures), s.camera_cpu_ms, s.ir_cpu_ms, s.message, detail);
  if (length > 0 && static_cast<size_t>(length) < sizeof(line))
    diagnostic::append_bridge_log(line);
}
/*
 * Everything one control tick reads before deciding anything.
 */
struct Inputs {
  std::uint64_t now = 0;
  win::Settings settings;
  bool connected = false;
  std::uint64_t session_epoch = 0;
  /*
   * The EXE's settings were issued for the current aircraft session.
   */
  bool session_settings = false;
  bool rafale = false;
  native_camera::CameraAvailability availability;
};

/*
 * What the video pipeline should do this tick. `wanted` gathers every precondition: EXE present and enabled,
 * the Rafale loaded, native graphics ready, and the WASM asking for video. IR is honoured only while wanted.
 */
struct Demand {
  bool wanted = false;
  bool night_vision = false;
};

/*
 * The DLL control loop, one tick every 25 ms. Each tick runs the same steps in order:
 *
 *   1. read inputs      EXE mailbox, aircraft session, WASM availability and stream demand;
 *   2. session change   park the camera until the retained view is revalidated for the new aircraft session;
 *   3. decide           the Demand above;
 *   4. apply            preload, zoom detail, TV/IR mode, capture demand, camera start/suspend;
 *   5. service video    exposure, composition of completed captures, capture-progress tracking;
 *   6. publish          status for the EXE, state changes for the bridge log.
 *
 * The EXE controls exposure, the native resolution and the render limit; the WASM controls mount, aim, zoom,
 * TV/IR and whether video is needed. The TV filter is always on. Stream stop or heartbeat loss closes the
 * render gate and disables extra preload/capture work, while resource lifetime and ordered source-state
 * tracking stay active so that resuming cannot reuse stale evidence. TV/IR changes invalidate old-mode images
 * and wait for the observer to acknowledge the native mode before composition resumes. Video is copied only
 * into the explicitly marked WASM input image; an executing gauge callback or image upload alone is not proof
 * that its mesh is visible.
 */
class ControlLoop {
 public:
  explicit ControlLoop(win::Mailbox& mailbox) noexcept : mailbox_(mailbox) {}

  void tick() {
    auto in = read_inputs();
    if (in.connected && service_session_change(in))
      return;
    service_periodic(in.now);
    const auto demand = decide(in);
    apply(in, demand);
    /*
     * From here on, D3D12 work recorded by this thread is AZP's own and must not be observed as simulator work.
     */
    const win::OwnedWork owned;
    apply_capture(in, demand);
    const auto video = service_video(in, demand);
    publish(in, video);
  }

 private:
  struct Video {
    native_camera::CameraStatus camera;
    scene_runtime::Snapshot output;
    bool mode_acknowledged = false;
  };

  Inputs read_inputs() {
    control_.refresh(mailbox_);
    Inputs in;
    in.now = GetTickCount64();
    in.settings = control_.settings();
    in.connected = control_.connected(in.now);
    in.session_epoch = native_camera::get_aircraft_session_epoch();
    in.session_settings = in.settings.aircraft_session_epoch == in.session_epoch;
    return in;
  }

  /*
   * A new aircraft/flight session parks the camera and withdraws every image of the old session, then waits
   * until the observer has revalidated the retained view. Returns true while the transition is still pending
   * (the rest of the tick is skipped).
   */
  bool service_session_change(const Inputs& in) {
    if (!changing_session_ && session_applied_ && in.session_epoch == applied_session_epoch_)
      return false;
    native_camera::set_world_preload_enabled(false);
    native_camera::set_zoom_detail_enabled(false);
    if (!changing_session_ || in.session_epoch != pending_session_epoch_) {
      scene_runtime::set_wasm_input_active(false);
      win::set_graphics_observation_demand(false);
      native_camera::suspend_camera(true);
      scene_runtime::manager().stop_source_tracking();
      scene_handoff().stop_scene();
      scene_runtime::reset_feed();
      pending_session_epoch_ = in.session_epoch;
      transition_token_ = 0;
      changing_session_ = true;
      requested_ = failed_ = false;
    }
    native_camera::suspend_camera(true);
    if (!transition_token_)
      transition_token_ = native_camera::begin_session_transition();
    const auto transition = native_camera::camera_status();
    const bool ready = transition_token_ && transition.session_transition_token == transition_token_ &&
                       transition.session_transition_ready && !transition.session_transition_pending &&
                       !transition.session_transition_failed;
    if (!ready) {
      publish_transition(in, transition);
      scene_runtime::service();
      if (in.now >= next_telemetry_) {
        native_camera::start_simconnect_link();
        next_telemetry_ = in.now + 2000;
      }
      return true;
    }
    /*
     * Restart the SimConnect link for the new session; the periodic telemetry step reconnects it at once.
     */
    native_camera::stop_simconnect_link();
    session_applied_ = true;
    applied_session_epoch_ = pending_session_epoch_;
    char detail[256]{};
    std::snprintf(detail, sizeof(detail), "Aircraft transition: token=%llu session=%llu retained_entry=%llu",
                  static_cast<unsigned long long>(transition_token_), static_cast<unsigned long long>(applied_session_epoch_),
                  static_cast<unsigned long long>(transition.ownership.owned_id));
    log_status(status_, detail);
    progress_ = {};
    next_telemetry_ = next_cleanup_ = 0;
    changing_session_ = false;
    return false;
  }

  /*
   * Keeps the SimConnect worker running and bounds the D3D12 registry.
   */
  void service_periodic(std::uint64_t now) {
    if (now >= next_telemetry_) {
      native_camera::start_simconnect_link();
      next_telemetry_ = now + 2000;
    }
    if (now >= next_cleanup_) {
      win::collect_retired_objects();
      next_cleanup_ = now + 100;
    }
  }

  Demand decide(Inputs& in) {
    in.rafale = native_camera::aircraft_is_rafale();
    in.availability = native_camera::get_camera_availability();
    Demand demand;
    demand.wanted = in.connected && in.session_settings && in.settings.enabled && in.rafale && win::graphics_ready() &&
                    in.availability.valid && in.availability.stream_requested;
    demand.night_vision = demand.wanted && in.availability.night_vision;
    return demand;
  }

  /*
   * Scenery preload, zoom detail and the TV/IR mode follow demand immediately. A TV/IR change discards images
   * composed for the other mode.
   */
  void apply(const Inputs&, const Demand& demand) {
    native_camera::set_world_preload_enabled(demand.wanted);
    native_camera::set_zoom_detail_enabled(demand.wanted);
    native_camera::request_night_vision(demand.night_vision);
    scene_runtime::set_night_vision(demand.night_vision);
    if (!demand.wanted)
      failed_ = false;
    streaming_ = demand.wanted && !failed_;
    win::set_graphics_observation_demand(streaming_);
    scene_runtime::set_wasm_input_active(streaming_);
  }

  /*
   * Starts the camera once when video is first wanted, and suspends it whenever video is not wanted or the
   * start failed. The render limit follows the EXE at once. The native resolution is taken only while no
   * camera entry exists: an established camera cannot reallocate its output, and replacing a live view was
   * never proven (a refused replacement cannot be removed safely and stays until MSFS closes), so the camera
   * keeps its resolution for the whole MSFS session. Capture recognition reads the same value, so it must
   * not change under an existing camera either. The EXE window locks the setting while MSFS runs.
   */
  void apply_capture(const Inputs& in, const Demand& demand) {
    if (in.connected) {
      if (!native_camera::camera_status().ownership.owned_id)
        native_camera_side.store(in.settings.camera_resolution, std::memory_order_relaxed);
      native_camera::set_render_rate(in.settings.render_rate);
    }
    native_camera::suspend_camera(!streaming_);
    if (!demand.wanted || requested_ || failed_)
      return;
    if (scene_runtime::prepare()) {
      scene_runtime::reset_feed();
      scene_runtime::manager().begin_source_tracking();
      native_camera::start_camera(true);
      requested_ = native_camera::camera_status().accepting_requests;
    }
    failed_ = !requested_;
    if (failed_) {
      streaming_ = false;
      scene_runtime::set_wasm_input_active(false);
      native_camera::suspend_camera(true);
      win::set_graphics_observation_demand(false);
    }
  }

  /*
   * Applies exposure, composes completed captures once the camera acknowledges the TV/IR mode, and tracks
   * capture progress for recovery.
   */
  Video service_video(const Inputs& in, const Demand& demand) {
    if (!engine_logged_ && native_camera::engine_api_resolved()) {
      engine_logged_ = true;
      log_status(status_, native_camera::engine_api_summary(native_camera::engine_api()).substr(0, 1500).c_str());
      diagnostic::write_report(native_camera::engine_api());
    }
    /*
     * TV shows the engine-exposed final image with the TV exposure setting as a gain. In IR the night-vision
     * intensity ramps toward the aim of the last measurement and waits where it was in TV. Each composition
     * records the IR exposure it is made with, so its brightness measurement can be judged against it.
     */
    Video video;
    if (demand.night_vision)
      native_camera::set_night_vision_gain(std::exp2(ir_exposure_.advance(in.now)));
    scene_runtime::set_tv_exposure(in.settings.tv_exposure);
    scene_runtime::set_measurement_exposure(ir_exposure_.ev());
    video.camera = native_camera::camera_status();
    video.mode_acknowledged = video.camera.night_vision == demand.night_vision;
    if (video.mode_acknowledged)
      scene_runtime::service();
    video.output = scene_runtime::snapshot();
    /*
     * The measurement sets the IR aim; the next ticks ramp toward it. IR is displayed inverted, so its native
     * brightness is 1 - mean and a positive IR exposure (brighter on screen) lowers the native target.
     */
    if (const auto& measured = video.output.brightness; video.mode_acknowledged && measured.sequence && demand.night_vision)
      ir_exposure_.measure(1 - measured.mean, measured.sequence, measured.exposure_ev, -in.settings.ir_exposure);
    if (video.camera.stop_sequence != last_stop_sequence_)
      progress_.reset();
    if (in.now >= next_recovery_) {
      const auto& camera = video.camera;
      const bool eligible = demand.wanted && !failed_ && !video.output.failed && camera.ownership.state == engine_camera::State::active &&
                            !camera.pose_waiting && !camera.view_waiting && video.mode_acknowledged &&
                            native_camera::sample_body_pose(in.now).valid;
      if (eligible && video.output.frames != last_frames_)
        native_camera::note_capture_progress(in.now);
      last_frames_ = video.output.frames;
      progress_.observe(in.now, eligible, video.output.frames, video.output.capture.source_draws,
                        std::strcmp(video.output.capture.tail_status, "unknown_source_state") == 0);
      next_recovery_ = in.now + 250;
    }
    return video;
  }

  /*
   * The first condition that explains why video is not flowing, from the outermost dependency inwards.
   */
  std::string state_message(const Inputs& in, const Video& video) const {
    const auto& camera = video.camera;
    const auto& output = video.output;
    const auto stopped_camera = camera_stop_message(camera);
    return !in.connected                                  ? "Waiting for Windows companion heartbeat."
           : !in.settings.enabled                         ? "Camera service paused."
           : !in.rafale                                   ? "Waiting for the Rafale aircraft identity."
           : !in.availability.valid                       ? "Waiting for fresh WASM camera state."
           : !in.availability.stream_requested            ? "Camera idle: WASM is not requesting video (cockpit + IMA required)."
           : failed_                                      ? camera.message.c_str()
           : camera.view_waiting                          ? camera.message.c_str()
           : !output.wasm_uploads                         ? "Waiting for AZP_Camera WASM image uploads."
           : stopped_camera                               ? stopped_camera
           : !streaming_                                  ? "Camera rendering suspended."
           : !requested_                                  ? camera.message.c_str()
           : output.failed                                ? output.message
           : !video.mode_acknowledged                     ? "Waiting for the selected image configuration."
           : output.output && !output.input_latency_valid ? "Camera images ready; waiting for a completed WASM input copy."
           : output.output && output.input_age_ms > 2000  ? "The WASM video input is not receiving new images."
           : progress_.stalled()                          ? "Capture paused: waiting for verified GPU state; camera views retained."
           : output.output && !output.input_copies        ? "Camera images ready; waiting for a marked WASM upload to replace."
                                                          : output.message;
  }

  void publish(const Inputs& in, const Video& video) {
    const auto graphics = win::graphics_status();
    const auto& camera = video.camera;
    const auto& output = video.output;
    status_ = {};
    status_.heartbeat = in.now;
    std::snprintf(status_.night_vision_message, sizeof(status_.night_vision_message), "%s", camera.night_vision_message.c_str());
    if (const auto& measured = output.brightness; measured.sequence && camera.night_vision)
      std::snprintf(status_.night_vision_message, sizeof(status_.night_vision_message), "%s | gain %.0f | brightness %.2f (black %.0f%%, white %.0f%%)",
                    camera.night_vision_message.c_str(), std::exp2(ir_exposure_.ev()), 1 - measured.mean, 100 * measured.white,
                    100 * measured.black);
    else if (measured.sequence)
      std::snprintf(status_.night_vision_message, sizeof(status_.night_vision_message),
                    "TV | exposure %+.1f EV | brightness %.2f (black %.0f%%, white %.0f%%)", in.settings.tv_exposure, measured.mean,
                    100 * measured.black, 100 * measured.white);
    status_.aircraft_session_epoch = in.session_epoch;
    const auto ima = native_camera::get_ima_camera();
    status_.ima_valid = ima.valid;
    status_.ima_has_target = ima.has_target;
    status_.ima_angles = {ima.pitch, ima.bank, ima.heading, ima.fov};
    const auto preload = native_camera::world_preload_status();
    status_.world_preload_active = preload.active;
    status_.world_preload_requests = preload.requests;
    status_.world_preload_releases = preload.releases;
    std::snprintf(status_.world_preload_message, sizeof(status_.world_preload_message), "%s", preload.message);
    status_.graphics_ready = graphics.ready;
    status_.hook_failures = graphics.hook_failures;
    status_.availability_valid = in.availability.valid;
    status_.ima_available_left = in.availability.left_available;
    status_.ima_available_right = in.availability.right_available;
    status_.camera_ready = camera.ready;
    status_.streaming = streaming_;
    status_.wasm_uploads = output.wasm_uploads;
    std::snprintf(status_.wasm_input_status, sizeof(status_.wasm_input_status), "%s", output.wasm_input_status);
    measure_callbacks(in.now);
    status_.camera_cpu_ms = camera_cpu_ms_;
    status_.ir_cpu_ms = ir_cpu_ms_;
    status_.camera_resolution = camera.resolution;
    status_.render_rate = in.settings.render_rate;
    status_.captures = output.capture.captures;
    status_.images = output.frames;
    status_.input_copies = output.input_copies;
    status_.input_frames = output.input_frames;
    status_.input_fps = output.input_fps;
    status_.input_latency_ms = output.input_latency_ms;
    status_.input_age_ms = output.input_age_ms;
    status_.input_latency_valid = output.input_latency_valid;
    std::snprintf(status_.message, sizeof(status_.message), "%s", state_message(in, video).c_str());
    if (mailbox_.lock()) {
      mailbox_.data()->status = status_;
      mailbox_.unlock();
    }
    const bool changed = !logged_ || in.connected != last_connected_ || requested_ != last_requested_ ||
                         status_.streaming != last_logged_.streaming || status_.camera_ready != last_logged_.camera_ready ||
                         camera.stop_sequence != last_stop_sequence_ || std::strcmp(status_.message, last_logged_.message);
    if (changed) {
      log_status(status_);
      last_logged_ = status_;
      last_connected_ = in.connected;
      last_requested_ = requested_;
      last_stop_sequence_ = camera.stop_sequence;
      logged_ = true;
    }
    if (const auto detail = native_camera::zoom_detail_message(); detail != zoom_detail_) {
      log_status(status_, detail.c_str());
      zoom_detail_ = detail;
    }
    const auto queue = queue_hooks::statistics();
    const bool new_queue_data =
        queue.reentrant != last_queue_log_.reentrant || queue.reentrant_affecting != last_queue_log_.reentrant_affecting ||
        queue.contended != last_queue_log_.contended || queue.contended_affecting != last_queue_log_.contended_affecting ||
        output.capture.escaped_invalidations != last_escaped_invalidations_;
    if (new_queue_data && in.now >= next_gpu_log_) {
      char detail[384]{};
      std::snprintf(detail, sizeof(detail),
                    "GPU auxiliary submits: reentrant=%llu/%llu affecting, contended=%llu/%llu affecting; "
                    "source invalidations=%llu, tail=%s",
                    static_cast<unsigned long long>(queue.reentrant), static_cast<unsigned long long>(queue.reentrant_affecting),
                    static_cast<unsigned long long>(queue.contended), static_cast<unsigned long long>(queue.contended_affecting),
                    static_cast<unsigned long long>(output.capture.escaped_invalidations), output.capture.tail_status);
      log_status(status_, detail);
      last_queue_log_ = queue;
      last_escaped_invalidations_ = output.capture.escaped_invalidations;
      next_gpu_log_ = in.now + 10000;
    }
  }

  /*
   * While a session transition is pending, the EXE gets a reduced status and the bridge log a line every
   * five seconds.
   */
  void publish_transition(const Inputs& in, const native_camera::CameraStatus& transition) {
    const auto graphics = win::graphics_status();
    win::Status pending{};
    pending.heartbeat = in.now;
    const auto preload = native_camera::world_preload_status();
    pending.world_preload_active = preload.active;
    pending.world_preload_requests = preload.requests;
    pending.world_preload_releases = preload.releases;
    std::snprintf(pending.world_preload_message, sizeof(pending.world_preload_message), "%s", preload.message);
    pending.graphics_ready = graphics.ready;
    pending.hook_failures = graphics.hook_failures;
    pending.aircraft_session_epoch = in.session_epoch;
    std::snprintf(pending.message, sizeof(pending.message), "%s", camera_transition_message(transition));
    if (mailbox_.lock()) {
      mailbox_.data()->status = pending;
      mailbox_.unlock();
    }
    if (in.now >= next_log_) {
      char detail[384]{};
      std::snprintf(detail, sizeof(detail),
                    "Aircraft transition waiting: token=%llu session=%llu failed=%u request_pending=%u creation_pending=%u entry=%llu "
                    "created_total=%llu",
                    static_cast<unsigned long long>(transition_token_), static_cast<unsigned long long>(in.session_epoch),
                    transition.session_transition_failed, transition.ownership.request_pending, transition.ownership.creation_pending,
                    static_cast<unsigned long long>(transition.ownership.owned_id),
                    static_cast<unsigned long long>(transition.created_total));
      log_status(pending, detail);
      next_log_ = in.now + 5000;
    }
  }

  win::Mailbox& mailbox_;
  win::CompanionControl control_;
  CaptureProgress progress_;
  /*
   * Camera start: requested once per wanted period; a failed start stays failed until video is unwanted.
   */
  bool requested_ = false, failed_ = false, streaming_ = false;
  /*
   * IR automatic exposure, in log2 of the night-vision intensity.
   */
  ExposureController ir_exposure_{std::log2(kNightVisionDefaultIntensity), std::log2(kNightVisionMinimumIntensity),
                                  std::log2(kNightVisionMaximumIntensity), kIrTargetBrightness};
  /*
   * Aircraft session transition.
   */
  bool session_applied_ = false, changing_session_ = false;
  std::uint64_t applied_session_epoch_ = 0, pending_session_epoch_ = 0, transition_token_ = 0;
  std::uint64_t next_telemetry_ = 0, next_cleanup_ = 0, next_recovery_ = 0, next_log_ = 0, next_gpu_log_ = 0, last_frames_ = 0;
  /*
   * Status publication and change detection for the bridge log.
   */
  win::Status status_{}, last_logged_{};
  bool logged_ = false, last_connected_ = false, last_requested_ = false, engine_logged_ = false;
  std::uint64_t last_stop_sequence_ = 0;
  queue_hooks::Statistics last_queue_log_{};
  std::uint64_t last_escaped_invalidations_ = 0;
  std::string zoom_detail_;
  /*
   * Engine-callback time over the last full second, in ms per second.
   */
  native_camera::ObserverCost last_cost_{};
  std::uint64_t cost_since_ = 0;
  float camera_cpu_ms_ = 0, ir_cpu_ms_ = 0;

  /*
   * Turns the cumulative engine-callback time into milliseconds per second, over windows of at least one
   * second.
   */
  void measure_callbacks(std::uint64_t now) {
    if (cost_since_ && now - cost_since_ < 1000)
      return;
    const auto cost = native_camera::observer_cost();
    if (cost_since_) {
      const double seconds = double(now - cost_since_) / 1000.0;
      camera_cpu_ms_ = static_cast<float>(double(cost.camera_us - last_cost_.camera_us) / 1000.0 / seconds);
      ir_cpu_ms_ = static_cast<float>(double(cost.night_vision_us - last_cost_.night_vision_us) / 1000.0 / seconds);
    }
    last_cost_ = cost;
    cost_since_ = now;
  }
};

/*
 * Opens the EXE mailbox and native graphics, then runs the control loop until the process exits.
 */
DWORD run_impl() {
  if (!mailbox.open(GetCurrentProcessId(), false))
    return ERROR_INVALID_DATA;
  native_camera::set_presence_source(presence);
  if (!win::initialize_graphics()) {
    win::Status status{};
    const auto graphics = win::graphics_status();
    std::snprintf(status.message, sizeof(status.message), "Native graphics refused: %s", graphics.error);
    status.hook_failures = graphics.hook_failures;
    status.heartbeat = GetTickCount64();
    if (mailbox.lock(100)) {
      mailbox.data()->status = status;
      mailbox.unlock();
    }
    log_status(status);
    return ERROR_NOT_SUPPORTED;
  }
  win::set_graphics_observation_demand(false);
  ControlLoop loop(mailbox);
  for (;;) {
    loop.tick();
    Sleep(25);
  }
}
/*
 * Contains C++ failures at the worker boundary and withdraws rendering/preload demand on failure. No
 * exception may escape the Windows thread entry point.
 */
DWORD WINAPI run(void*) noexcept {
  try {
    return run_impl();
  } catch (...) {
    native_camera::set_world_preload_enabled(false);
    native_camera::set_zoom_detail_enabled(false);
    win::set_graphics_observation_demand(false);
    scene_runtime::manager().stop_source_tracking();
    native_camera::stop_camera(true);
    return ERROR_NOT_ENOUGH_MEMORY;
  }
}
}  /* namespace */
/*
 * Starts one bridge worker only inside the expected simulator executable. It pins this DLL for callbacks
 * that can outlive the EXE and closes only the thread handle, not the worker itself.
 */
extern "C" __declspec(dllexport) DWORD WINAPI AzpCameraStart(void*) {
  wchar_t path[32768]{};
  const DWORD length = GetModuleFileNameW(nullptr, path, 32768);
  const wchar_t* name = std::wcsrchr(path, L'\\');
  if (!length || length >= 32768 || _wcsicmp(name ? name + 1 : path, L"FlightSimulator2024.exe"))
    return ERROR_BAD_ENVIRONMENT;
  if (started.exchange(true))
    return ERROR_SUCCESS;
  HMODULE pinned{};
  if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN,
                          reinterpret_cast<LPCWSTR>(&AzpCameraStart), &pinned))
    return GetLastError();
  HANDLE thread = CreateThread(nullptr, 0, run, nullptr, 0, nullptr);
  if (!thread) {
    started = false;
    return GetLastError();
  }
  CloseHandle(thread);
  return ERROR_SUCCESS;
}
BOOL WINAPI DllMain(HMODULE value, DWORD reason, LPVOID) {
  if (reason == DLL_PROCESS_ATTACH)
    DisableThreadLibraryCalls(value);
  return TRUE;
}
