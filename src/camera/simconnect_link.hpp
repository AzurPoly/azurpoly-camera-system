/*
 * Declares the public-data worker and its camera-facing snapshots. Consumers obtain aircraft, lighting,
 * demand and guidance data without sharing the SimConnect connection or borrowing the worker's mutable
 * storage.
 */

#pragma once
#include <cstdint>
#include <limits>
#include "aircraft_identity.hpp"
#include "pose.hpp"

namespace azp::native_camera {
/*
 * WASM-owned availability and stream demand; native code does not decide page/view policy.
 */
struct CameraAvailability {
  bool valid = false, left_available = false, right_available = false, stream_requested = false;
  bool night_vision = false;  /* Shared TV/IR selection from the IMA menu. */
};
void set_video_input_ack(std::uint64_t packed) noexcept;
/*
 * Supplies the EXE presence counter to relay to the aircraft: the EXE's counter while its link is alive and
 * ON, otherwise 0. The SimConnect worker calls it on its own thread, about every 50 ms, so presence never
 * waits for the control loop, which can be busy for seconds while the camera starts.
 */
using PresenceSource = std::uint64_t (*)() noexcept;
void set_presence_source(PresenceSource source) noexcept;
CameraAvailability get_camera_availability() noexcept;
struct BodyPoseSnapshot {
  bool valid = false;
  std::uint64_t sample_ms = 0;
  BodyPose pose{};
  const char* error = "not_initialized";
};
struct GroundSpeedSample {
  bool valid = false;
  double knots = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t sample_ms = 0;
  const char* error = "not_initialized";
};

struct WorldPreloadTarget {
  bool valid = false;
  double latitude = 0, longitude = 0, altitude_m = 0;
  std::uint64_t sample_ms = 0;
};
struct LightingSample {
  static constexpr std::uint64_t MaximumAgeMs = 1500;
  bool valid = false;
  double ambient = std::numeric_limits<double>::quiet_NaN();
  double brightness = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t sample_ms = 0;
  const char* error = "not_initialized";
};
AircraftIdentitySample get_aircraft_identity() noexcept;
/*
 * Changes only when the flight/aircraft session changes.
 */
std::uint64_t get_aircraft_session_epoch() noexcept;
/*
 * True when a fresh identity sample shows the Rafale package path.
 */
bool aircraft_is_rafale() noexcept;
bool start_simconnect_link() noexcept;
void stop_simconnect_link() noexcept;
void reset_body_pose_calibration() noexcept;
/*
 * Validate the current model against fresh aircraft telemetry, including its local altitude datum. No
 * stationary camera or pilot-camera API is required.
 */
bool validate_aircraft_scene_pose(const BodyPose& scene, std::uint64_t now_ms) noexcept;
BodyPoseSnapshot sample_body_pose(std::uint64_t now_ms) noexcept;
/*
 * Cached public GROUND VELOCITY in knots, independent of body calibration. No simulator calls; a sample
 * older than500ms is unavailable, never zeroed.
 */
GroundSpeedSample get_ground_speed() noexcept;
/*
 * Final IMA world orientation; stale/missing publication pauses the camera.
 */
ImaCameraSample get_ima_camera() noexcept;
/*
 * Selected IMA target from public Lvars; only the current flight and age <1s.
 */
WorldPreloadTarget get_world_preload_target() noexcept;
/*
 * Optional public lighting values sampled at 2 Hz, independent of pose validity. AMBIENT LIGHT SENSOR is a
 * Number, not a claimed radiometric/lux measurement.
 */
LightingSample get_lighting() noexcept;
}  /* namespace azp::native_camera */
