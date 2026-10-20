/*
 * Defines the camera control interface and the status snapshot returned to the DLL worker. Start, stop and
 * mode requests express intent; their native effects are applied by the guarded engine observer in
 * camera.cpp.
 */

#pragma once
#include <array>
#include <cstdint>
#include <string>
#include "../engine/camera_ownership.hpp"
#include "../shared/configuration.hpp"
#include "lifecycle.hpp"
#include "pose.hpp"

namespace azp::native_camera {

/*
 * What the camera observer last published. The observer also uses it as its working report during one
 * inspection (ready, flags, dimensions). The DLL control loop reads the rest.
 */
struct CameraStatus {
  /*
   * TV/IR mode of the last inspection, whose final image the camera publishes.
   */
  bool night_vision = false;
  std::string night_vision_message = "TV";
  bool accepting_requests = false;
  std::uint64_t session_transition_token = 0;
  bool session_transition_pending = false;
  bool session_transition_ready = false;
  bool session_transition_failed = false;
  /*
   * Cumulative nonzero IDs returned by creation, including later rollbacks.
   */
  std::uint64_t created_total = 0;
  std::uint32_t free_views = 0;
  bool ready = false;
  bool resource_present = false;
  ViewDimensions dimensions{};
  /*
   * Side of the established camera's square image, in pixels; 0 before its initial resize or without camera.
   */
  std::uint32_t resolution = 0;
  std::array<std::uint64_t, 2> flags{};
  engine_camera::Snapshot ownership{};
  bool pose_waiting = false;
  bool view_waiting = false;
  bool recovery_pending = false;
  /*
   * A bounded retry has been accepted and is waiting for pose/creation.
   */
  bool restart_pending = false;
  std::uint64_t stop_sequence = 0;
  StopReason stop_reason = StopReason::none;
  std::string message = "The camera has not started.";
};

/*
 * Called by the UI, outside DllMain. Installs the observer only after verifying the fixed main executable
 * and pins this add-on until process exit. Engine functions are never called here: requests are consumed by
 * the update observer.
 */
void start_camera(bool reuse_camera = false) noexcept;
/*
 * Parks the camera for an aircraft/flight session change. Each request gets a unique completion token; zero
 * refuses the request. The existing view is closed and revalidated by the observer and never replaced.
 */
std::uint64_t begin_session_transition() noexcept;
/*
 * A recoverable pause retains public telemetry so the same view can resume.
 */
void stop_camera(bool keep_telemetry = false) noexcept;
void note_capture_progress(std::uint64_t now_ms) noexcept;
/*
 * Atomic configuration only; consumed by the observer, never calls the engine. Suspending closes the render
 * gate while retaining the owned view; no ownership change.
 */
void suspend_camera(bool suspended) noexcept;
/*
 * IMA mode request; consumed by the owned-view observer at a safe engine phase.
 */
void request_night_vision(bool enabled) noexcept;
/*
 * IR gain (native night-vision intensity) for the next NV callbacks; atomic. Out-of-range values fall back
 * to the validated default.
 */
void set_night_vision_gain(float gain) noexcept;
/*
 * Native render limit in frames per second, 0 for every simulator frame. Atomic; applied by the observer to
 * the render gate of a streaming camera.
 */
void set_render_rate(std::uint32_t frames_per_second) noexcept;
CameraStatus camera_status();
/*
 * Cumulative time spent in the camera's two engine callbacks, the manager update and the per-view NV render
 * callback, in microseconds. Both run on simulator threads, so this is time taken from the simulator's
 * frames; the control loop turns it into milliseconds per second.
 */
struct ObserverCost {
  std::uint64_t camera_us = 0, night_vision_us = 0;
};
ObserverCost observer_cost() noexcept;

}  /* namespace azp::native_camera */

namespace azp {

const char* camera_transition_message(const native_camera::CameraStatus& scene) noexcept;

/*
 * A retained stop reason can outlive successful recovery. Only temporary failures on a stopped camera (or
 * pending cleanup) override normal capture status.
 */
const char* camera_stop_message(const native_camera::CameraStatus& scene) noexcept;

}  /* namespace azp */
