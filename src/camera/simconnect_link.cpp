/*
 * Runs the SimConnect worker that receives aircraft telemetry, WASM guidance, stream demand, TV/IR selection
 * and preload multipliers. It publishes locked, freshness-checked snapshots and relays EXE presence.
 * Telemetry validates the flight context; the rendered aircraft hierarchy supplies the actual camera
 * attachment.
 */

#include "simconnect_link.hpp"
#include <windows.h>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <cwchar>
#include <utility>
#include "../engine/aircraft_scene_pose.hpp"
#include "../shared/configuration.hpp"
#include "aircraft_identity.hpp"
#include "pose.hpp"
#include "scenery_preload.hpp"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

namespace azp::native_camera {
namespace {
using Open = HRESULT(WINAPI*)(HANDLE*, const char*, HWND, DWORD, HANDLE, DWORD);
using Close = HRESULT(WINAPI*)(HANDLE);
using Define = HRESULT(WINAPI*)(HANDLE, DWORD, const char*, const char*, DWORD, float, DWORD);
using Request = HRESULT(WINAPI*)(HANDLE, DWORD, DWORD, DWORD, DWORD, DWORD, DWORD, DWORD, DWORD);
using Dispatch = HRESULT(WINAPI*)(HANDLE, void**, DWORD*);
using SystemState = HRESULT(WINAPI*)(HANDLE, DWORD, const char*);
using Subscribe = HRESULT(WINAPI*)(HANDLE, DWORD, const char*);
using LastPacket = HRESULT(WINAPI*)(HANDLE, DWORD*);
using SetData = HRESULT(WINAPI*)(HANDLE, DWORD, DWORD, DWORD, DWORD, DWORD, void*);
struct AircraftData {
  body_math::Telemetry pose;
  double ground_speed_knots;
};
static_assert(sizeof(AircraftData) == 56);
static_assert(offsetof(AircraftData, ground_speed_knots) == 48);
struct State {
  SRWLOCK lock = SRWLOCK_INIT;
  HANDLE worker = nullptr, stop = nullptr;
  body_math::Telemetry aircraft{};
  std::uint64_t aircraft_ms = 0;
  double ground_speed_knots = 0;
  std::uint64_t ground_speed_ms = 0;
  const char* ground_speed_error = "not_initialized";
  ImaCameraCache ima_camera;
  WorldPreloadTarget world_preload;
  CameraAvailability availability;
  std::uint64_t availability_ms = 0;
  AircraftIdentityCache identity;
  AircraftSessionLifecycle aircraft_session;
  double ambient = 0, brightness = 0;
  std::uint64_t lighting_ms = 0;
  const char* lighting_error = "not_initialized";
  double correction = 0;
  const char* error = "not_initialized";
};
State state;
std::atomic<std::uint64_t> video_input_ack{0};
std::atomic<PresenceSource> presence_source{nullptr};
SRWLOCK lifecycle = SRWLOCK_INIT;
/*
 * Clears cached aircraft/guidance state when the flight context changes. The caller holds the publication
 * lock so readers cannot combine old and new sessions. EXE presence is not cached here: it is sampled from
 * its source, so a flight change cannot withdraw it.
 */
void reset_session_locked() noexcept {
  state.identity = {};
  state.aircraft_ms = state.ground_speed_ms = state.lighting_ms = 0;
  state.ima_camera = {};
  state.world_preload = {};
  state.availability = {};
  state.availability_ms = 0;
  state.correction = 0;
  state.error = state.ground_speed_error = state.lighting_error = "aircraft_session_changed";
}
bool accept_session_packet(const void* raw, DWORD bytes) noexcept {
  AcquireSRWLockExclusive(&state.lock);
  const bool changed = state.aircraft_session.accept(raw, bytes);
  if (changed || !state.aircraft_session.running())
    reset_session_locked();
  ReleaseSRWLockExclusive(&state.lock);
  return changed;
}
/*
 * Validates the dedicated identity reply under the publication lock and resets dependent caches when
 * aircraft identity changes.
 */
bool accept_identity_packet(const void* raw, DWORD bytes, std::uint64_t now) noexcept {
  AcquireSRWLockExclusive(&state.lock);
  const auto before = state.identity.sample(now);
  const bool accepted = state.identity.accept(raw, bytes, now);
  const auto after = state.identity.sample(now);
  const bool changed = accepted && before.sample_ms && after.sample_ms != before.sample_ms &&
                       (std::strcmp(before.type.data(), after.type.data()) || std::strcmp(before.path.data(), after.path.data()));
  if (changed) {
    state.aircraft_session.changed();
    reset_session_locked();
  }
  ReleaseSRWLockExclusive(&state.lock);
  return changed;
}
void lighting_failure(const char* text) noexcept {
  AcquireSRWLockExclusive(&state.lock);
  state.lighting_ms = 0;
  state.lighting_error = text;
  ReleaseSRWLockExclusive(&state.lock);
}
/*
 * Publishes a worker failure under the shared-state lock, invalidating connection-dependent data rather than
 * leaving stale telemetry apparently live.
 */
void failure(const char* text, bool invalidate_ground_speed = true) noexcept {
  AcquireSRWLockExclusive(&state.lock);
  state.error = text;
  state.aircraft_ms = 0;
  state.world_preload = {};
  if (invalidate_ground_speed) {
    state.ground_speed_ms = 0;
    state.ground_speed_error = text;
    state.lighting_ms = 0;
    state.lighting_error = text;
  }
  ReleaseSRWLockExclusive(&state.lock);
}
/*
 * Validates the dedicated FLOAT64 telemetry packet before updating aircraft pose and speed. Pose validity
 * and speed validity are independent; a rejected pose cannot fabricate a zero groundspeed.
 *
 * SIMCONNECT_OBJECT_ID_USER(0) is the request alias. The response identifies the actual aircraft object
 * (observed as 2883584), so correlate our dedicated request/definition IDs rather than requiring a zero
 * returned object ID.
 *
 * Independent validity: a camera calibration or pose failure must not turn an otherwise current public
 * groundspeed into a fabricated zero.
 */
bool accept_aircraft_packet(const void* raw, DWORD bytes, std::uint64_t sample_ms) noexcept {
  if (!raw || bytes < 40 || bytes > 65536 || !sample_ms)
    return false;
  std::array<DWORD, 10> header{};
  std::memcpy(header.data(), raw, sizeof(header));
  constexpr DWORD PacketBytes = 40 + sizeof(AircraftData);
  if (header[2] != 8 || header[0] != PacketBytes || bytes < PacketBytes || header[3] != 1 || header[5] != 1 || header[6] != 0 ||
      header[9] != 7)
    return false;
  AircraftData value{};
  std::memcpy(&value, static_cast<const unsigned char*>(raw) + 40, sizeof(value));
  const bool pose_valid = body_math::valid(value.pose);
  const bool speed_valid = std::isfinite(value.ground_speed_knots) && value.ground_speed_knots >= 0 && value.ground_speed_knots <= 2000;
  AcquireSRWLockExclusive(&state.lock);
  if (pose_valid) {
    state.aircraft = value.pose;
    state.aircraft_ms = sample_ms;
    state.error = "";
  } else {
    state.aircraft_ms = 0;
    state.error = "aircraft_values";
  }
  if (speed_valid) {
    state.ground_speed_knots = value.ground_speed_knots;
    state.ground_speed_ms = sample_ms;
    state.ground_speed_error = "";
  } else {
    state.ground_speed_ms = 0;
    state.ground_speed_error = "ground_speed_values";
  }
  ReleaseSRWLockExclusive(&state.lock);
  return true;
}
/*
 * Accepts finite lighting values from the lighting definition only. Its own validity and age remain
 * independent of aircraft guidance.
 *
 * Lighting has its own packet definition; it cannot change aircraft telemetry layout.
 */
bool accept_lighting_packet(const void* raw, DWORD bytes, std::uint64_t sample_ms) noexcept {
  if (!raw || bytes != 56 || !sample_ms)
    return false;
  std::array<DWORD, 10> header{};
  std::memcpy(header.data(), raw, sizeof(header));
  if (header[0] != 56 || header[2] != 8 || header[3] != 4 || header[5] != 4 || header[6] != 0 || header[9] != 2)
    return false;
  std::array<double, 2> values{};
  std::memcpy(values.data(), static_cast<const unsigned char*>(raw) + 40, sizeof(values));
  const bool valid =
      std::isfinite(values[0]) && values[0] >= 0 && values[0] <= 1e7 && std::isfinite(values[1]) && values[1] >= 0 && values[1] <= 1;
  AcquireSRWLockExclusive(&state.lock);
  if (valid) {
    state.ambient = values[0];
    state.brightness = values[1];
    state.lighting_ms = sample_ms;
    state.lighting_error = "";
  } else {
    state.lighting_ms = 0;
    state.lighting_error = "lighting_values";
  }
  ReleaseSRWLockExclusive(&state.lock);
  return true;
}
/*
 * Accepts the dedicated preload multiplier reply, normalizes unusable values and publishes both scalars
 * under the lock. Optional tuning cannot invalidate aircraft telemetry.
 *
 * RequestDataOnSimObject returns entry 1 of 1, including the user aircraft.
 */
bool accept_world_preload_packet(const void* raw, DWORD bytes, std::uint64_t sample_ms) noexcept {
  if (!raw || bytes != 96 || !sample_ms)
    return false;
  std::array<DWORD, 10> header{};
  std::memcpy(header.data(), raw, sizeof(header));
  if (header[0] != 96 || header[2] != 8 || header[3] != 10 || header[5] != 10 || header[6] != 0 || header[7] != 1 || header[8] != 1 ||
      header[9] != 7)
    return false;
  std::array<double, 7> values{};
  std::memcpy(values.data(), static_cast<const unsigned char*>(raw) + 40, sizeof(values));
  WorldPreloadTarget target;
  const unsigned first = values[0] == 2 ? 4 : values[0] == 3 ? 1 : 0;
  if (first) {
    const double latitude = values[first], longitude = values[first + 1], altitude_m = values[first + 2] * 0.3048;
    if (std::isfinite(latitude) && latitude >= -90 && latitude <= 90 && std::isfinite(longitude) && longitude >= -180 && longitude <= 180 &&
        std::isfinite(altitude_m) && (latitude != 0 || longitude != 0))
      target = {true, latitude, longitude, altitude_m, sample_ms};
  }
  AcquireSRWLockExclusive(&state.lock);
  state.world_preload = state.aircraft_session.running() ? target : WorldPreloadTarget{};
  ReleaseSRWLockExclusive(&state.lock);
  return true;
}
WorldPreloadTarget world_preload_locked(std::uint64_t now) noexcept {
  const auto target = state.world_preload;
  if (!state.aircraft_session.running() || !target.valid || !target.sample_ms || now < target.sample_ms || now - target.sample_ms >= 1000)
    return {};
  return target;
}
/*
 * Owns the SimConnect connection and dispatch loop. It loads the installed DLL, registers separate
 * definitions for aircraft telemetry, WASM guidance, demand/TV-IR, target/preload and lighting, then waits
 * on data or shutdown instead of spinning.
 *
 * Aircraft telemetry uses seven FLOAT64 fields, with groundspeed in knots and updates at SIM_FRAME. Guidance
 * keeps its own FLOAT64 publication contract. Separate packet IDs and caches prevent optional target,
 * lighting or preload failures from invalidating an otherwise current aircraft sample.
 *
 * The worker relays the EXE presence counter without incrementing it, so a frozen companion remains
 * detectable. Presence follows the EXE link alone: it is not tied to the camera, the stream or the flight
 * session, and a reconnect does not write 0. The aircraft's own three-second timeout covers the gap.
 * WASM demand is independent of image uploads, allowing the producer to restart while no image
 * is available. Preload multipliers are sampled once per second; missing or invalid values receive neutral
 * defaults without overwriting valid positive custom values.
 *
 * Lifecycle events end this connection so queued old-flight replies cannot be accepted in the next session
 * and definitions are recreated for the loaded aircraft. Dedicated request/definition IDs identify aircraft
 * replies: response ObjectID is the actual aircraft ID, not necessarily the user-request alias 0.
 *
 * Lighting is optional display data: ambient uses the observed 1..4000 range and glass-cockpit brightness
 * uses 0..1. Lighting and identity refreshes run on the wait timeout without changing guidance packet
 * layout.
 *
 * Contract references:
 * https://docs.flightsimulator.com/html/Programming_Tools/SimConnect/API_Reference/Events_And_Data/SimConnect_SubscribeToSystemEvent.htm
 * and
 * https://docs.flightsimulator.com/msfs2024/retail/programming-apis/simconnect/api-reference/structures-and-enumerations/simconnect_period/
 */
DWORD WINAPI worker(void*) noexcept {
  wchar_t path[32768]{};
  const DWORD length = GetModuleFileNameW(nullptr, path, 32768);
  if (!length || length >= 32768) {
    failure("main_image_path");
    return 0;
  }
  wchar_t* slash = std::wcsrchr(path, L'\\');
  if (!slash || static_cast<std::size_t>(slash - path) + 25 >= 32768) {
    failure("main_image_path");
    return 0;
  }
  std::wcscpy(slash + 1, L"SimConnect_internal.dll");
  const auto dll = LoadLibraryExW(path, nullptr, LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR | LOAD_LIBRARY_SEARCH_SYSTEM32);
  if (!dll) {
    failure("simconnect_client_unavailable");
    return 0;
  }
  const auto open = reinterpret_cast<Open>(GetProcAddress(dll, "SimConnect_Open"));
  const auto close = reinterpret_cast<Close>(GetProcAddress(dll, "SimConnect_Close"));
  const auto define = reinterpret_cast<Define>(GetProcAddress(dll, "SimConnect_AddToDataDefinition"));
  const auto request = reinterpret_cast<Request>(GetProcAddress(dll, "SimConnect_RequestDataOnSimObject"));
  const auto dispatch = reinterpret_cast<Dispatch>(GetProcAddress(dll, "SimConnect_GetNextDispatch"));
  const auto system_state = reinterpret_cast<SystemState>(GetProcAddress(dll, "SimConnect_RequestSystemState"));
  const auto subscribe = reinterpret_cast<Subscribe>(GetProcAddress(dll, "SimConnect_SubscribeToSystemEvent"));
  const auto last_packet = reinterpret_cast<LastPacket>(GetProcAddress(dll, "SimConnect_GetLastSentPacketID"));
  const auto set_data = reinterpret_cast<SetData>(GetProcAddress(dll, "SimConnect_SetDataOnSimObject"));
  if (!open || !close || !define || !request || !dispatch || !system_state || !subscribe) {
    failure("simconnect_exports");
    FreeLibrary(dll);
    return 0;
  }
  HANDLE session = nullptr;
  const HANDLE notification = CreateEventW(nullptr, FALSE, FALSE, nullptr);
  if (!notification) {
    failure("simconnect_notification_event");
    FreeLibrary(dll);
    return 0;
  }
  HRESULT hr = open(&session, "AZP Camera aircraft and IMA telemetry", nullptr, 0, notification, 0);
  for (const auto event :
       {std::pair{AircraftSessionLifecycle::SimEvent, "Sim"}, std::pair{AircraftSessionLifecycle::AircraftEvent, "AircraftLoaded"},
        std::pair{AircraftSessionLifecycle::FlightEvent, "FlightLoaded"}}) {
    if (SUCCEEDED(hr))
      hr = subscribe(session, event.first, event.second);
  }
  const char* names[]{"PLANE LATITUDE",      "PLANE LONGITUDE",    "PLANE ALTITUDE",
                      "PLANE PITCH DEGREES", "PLANE BANK DEGREES", "PLANE HEADING DEGREES TRUE",
                      "GROUND VELOCITY"};
  for (unsigned i = 0; SUCCEEDED(hr) && i < std::size(names); ++i)
    hr = define(session, 1, names[i], i == 6 ? "knots" : i == 2 ? "meters" : "degrees", 4, 0, 0xffffffffu);
  if (FAILED(hr)) {
    failure("simconnect_open_or_definition");
    if (session)
      close(session);
    CloseHandle(notification);
    FreeLibrary(dll);
    return 0;
  }
  if (FAILED(request(session, 1, 1, 0, 3, 0, 0, 0, 0))) {
    failure("simconnect_body_stream");
    close(session);
    CloseHandle(notification);
    FreeLibrary(dll);
    return 0;
  }
  std::array<DWORD, 16> ima_packets{};
  unsigned ima_packet_cursor = 0;
  const auto remember_ima_packet = [&]() {
    DWORD id = 0;
    if (last_packet && SUCCEEDED(last_packet(session, &id)) && id)
      ima_packets[ima_packet_cursor++ % ima_packets.size()] = id;
  };
  bool ima_defined = true;
  for (const auto name : ImaCameraVariables) {
    if (!ima_defined)
      break;
    ima_defined = SUCCEEDED(define(session, 9, name, "number", 4, 0, 0xffffffffu));
    remember_ima_packet();
  }
  if (ima_defined) {
    ima_defined = SUCCEEDED(request(session, 9, 9, 0, 3, 0, 0, 0, 0));
    remember_ima_packet();
  }
  const char* target_names[]{"L:AZP_RAF_PDS_MAIN_TASK_ACTIVE",           "L:AZP_RAF_CTM_GROUND_TARGET_SELECTED_LAT",
                             "L:AZP_RAF_CTM_GROUND_TARGET_SELECTED_LON", "L:AZP_RAF_CTM_GROUND_TARGET_SELECTED_ALT",
                             "L:AZP_RAF_CTM_TRAFFIC_SELECTED_LAT",       "L:AZP_RAF_CTM_TRAFFIC_SELECTED_LON",
                             "L:AZP_RAF_CTM_TRAFFIC_SELECTED_ALT"};
  const char* target_units[]{"enum", "degree latitude", "degree longitude", "feet", "degree latitude", "degree longitude", "feet"};
  static_assert(std::size(target_names) == std::size(target_units));
  std::array<DWORD, 16> target_packets{};
  unsigned target_packet_cursor = 0;
  const auto remember_target_packet = [&]() {
    DWORD id = 0;
    if (last_packet && SUCCEEDED(last_packet(session, &id)) && id)
      target_packets[target_packet_cursor++ % target_packets.size()] = id;
  };
  bool target_defined = true;
  for (unsigned i = 0; i < std::size(target_names); ++i) {
    if (!target_defined)
      break;
    target_defined = SUCCEEDED(define(session, 10, target_names[i], target_units[i], 4, 0, 0xffffffffu));
    remember_target_packet();
  }
  if (target_defined) {
    target_defined = SUCCEEDED(request(session, 10, 10, 0, 3, 0, 0, 0, 0));
    remember_target_packet();
  }
  const bool type_defined = SUCCEEDED(define(session, 5, "ATC TYPE", nullptr, 9, 0, 0xffffffffu));
  const bool heartbeat_defined = set_data && SUCCEEDED(define(session, 12, "L:AZP_CAMERA_HEARTBEAT", "number", 4, 0, 0xffffffffu));
  bool availability_defined = true;
  for (const auto name :
       {"L:AZP_RAF_VTLG_CAMERA_AVAILABLE", "L:AZP_RAF_VTLD_CAMERA_AVAILABLE", "L:AZP_CAMERA_STREAM_REQUESTED", "L:AZP_CAMERA_NIGHT_VISION"})
    if (availability_defined)
      availability_defined = SUCCEEDED(define(session, 13, name, "number", 4, 0, 0xffffffffu));
  if (availability_defined)
    request(session, 13, 13, 0, 3, 0, 0, 0, 0);
  const bool video_ack_defined = set_data && SUCCEEDED(define(session, 14, "L:AZP_CAMERA_VIDEO_ACK", "number", 4, 0, 0xffffffffu));
  const char* preload_names[]{"L:AZP_CAMERA_PRELOAD_DENSITY_SCALAR", "L:AZP_CAMERA_PRELOAD_AREA_SCALAR"};
  std::array<double, 2> preload_scalars{1, 1};
  set_world_preload_scalars(1, 1);
  for (unsigned i = 0; i < preload_scalars.size(); ++i)
    if (SUCCEEDED(define(session, 15 + i, preload_names[i], "number", 4, 0, 0xffffffffu)))
      request(session, 15 + i, 15 + i, 0, 4, 0, 0, 0, 0);
  double last_video_ack = -1;
  double last_heartbeat = -1;
  std::array<DWORD, 16> lighting_packets{};
  unsigned lighting_packet_cursor = 0;
  const auto remember_lighting_packet = [&]() {
    DWORD id = 0;
    if (last_packet && SUCCEEDED(last_packet(session, &id)) && id)
      lighting_packets[lighting_packet_cursor++ % lighting_packets.size()] = id;
  };
  bool lighting_defined = last_packet != nullptr;
  for (const auto name : {"AMBIENT LIGHT SENSOR", "GLASSCOCKPIT AUTOMATIC BRIGHTNESS"}) {
    if (!lighting_defined)
      break;
    lighting_defined = SUCCEEDED(define(session, 4, name, "number", 4, 0, 0xffffffffu));
    remember_lighting_packet();
  }
  if (!lighting_defined)
    lighting_failure("lighting_definition_unavailable");
  std::uint64_t previous_lighting = 0, previous_type = 0;
  DWORD identity_request = 1000;
  bool reconnect = false;
  const HANDLE waits[]{state.stop, notification};
  while (true) {
    const auto wake = WaitForMultipleObjects(2, waits, FALSE, 50);
    if (wake == WAIT_OBJECT_0)
      break;
    if (wake != WAIT_OBJECT_0 + 1 && wake != WAIT_TIMEOUT) {
      failure("simconnect_notification_wait");
      break;
    }
    const auto now = GetTickCount64();
    if (heartbeat_defined) {
      const auto source = presence_source.load(std::memory_order_acquire);
      double heartbeat = source ? static_cast<double>(source()) : 0;
      if (heartbeat != last_heartbeat && SUCCEEDED(set_data(session, 12, 0, 0, 0, sizeof(heartbeat), &heartbeat)))
        last_heartbeat = heartbeat;
    }
    if (video_ack_defined) {
      double ack = static_cast<double>(video_input_ack.load(std::memory_order_acquire));
      if (ack != last_video_ack && SUCCEEDED(set_data(session, 14, 0, 0, 0, sizeof(ack), &ack)))
        last_video_ack = ack;
    }
    if (type_defined && now - previous_type >= 1000) {
      previous_type = now;
      if (identity_request > 0xfffffffcu)
        break;
      AcquireSRWLockExclusive(&state.lock);
      state.identity.begin_request(identity_request, identity_request + 1);
      ReleaseSRWLockExclusive(&state.lock);
      request(session, identity_request, 5, 0, 1, 0, 0, 0, 0);
      system_state(session, identity_request + 1, "AircraftLoaded");
      identity_request += 2;
    }
    if (lighting_defined && now - previous_lighting >= 500) {
      previous_lighting = now;
      const auto lighting_hr = request(session, 4, 4, 0, 1, 0, 0, 0, 0);
      remember_lighting_packet();
      if (FAILED(lighting_hr))
        lighting_failure("lighting_request_failed");
    }
    for (unsigned n = 0; n < 64; ++n) {
      void* raw = nullptr;
      DWORD bytes = 0;
      if (FAILED(dispatch(session, &raw, &bytes)) || !raw)
        break;
      if (bytes < 12 || bytes > 65536) {
        failure("simconnect_packet_bounds");
        continue;
      }
      std::array<DWORD, 10> header{};
      std::memcpy(header.data(), raw, bytes >= 40 ? 40 : 12);
      if (header[0] < 12 || header[0] > bytes) {
        if (bytes >= 40 && header[2] == 8 && (header[3] == 15 || header[3] == 16 || header[5] == 15 || header[5] == 16))
          continue;
        if (bytes >= 40 && header[2] == 8 && (header[3] == 10 || header[5] == 10)) {
          AcquireSRWLockExclusive(&state.lock);
          state.world_preload = {};
          ReleaseSRWLockExclusive(&state.lock);
          continue;
        }
        if (bytes >= 40 && header[2] == 8 && (header[3] == 4 || header[5] == 4)) {
          lighting_failure("lighting_packet_bounds");
          continue;
        }
        failure("simconnect_packet_bounds");
        continue;
      }
      if (header[2] == 4 || header[2] == 6) {
        reconnect = accept_session_packet(raw, bytes);
        if (reconnect)
          break;
      } else if (header[2] == 15 || (header[2] == 8 && bytes >= 40 && header[5] == 5)) {
        reconnect = accept_identity_packet(raw, bytes, GetTickCount64());
        if (reconnect)
          break;
      } else if (header[2] == 8) {
        if (bytes >= 40 && (header[3] == 15 || header[3] == 16 || header[5] == 15 || header[5] == 16)) {
          if (bytes != 48 || header[0] != 48 || header[3] != header[5] || header[6] != 0 || header[9] != 1)
            continue;
          double scalar = 1;
          std::memcpy(&scalar, static_cast<const unsigned char*>(raw) + 40, sizeof(scalar));
          if (!std::isfinite(scalar) || scalar <= 0) {
            scalar = 1;
            if (set_data)
              set_data(session, header[5], 0, 0, 0, sizeof(scalar), &scalar);
          }
          preload_scalars[header[5] - 15] = scalar;
          set_world_preload_scalars(preload_scalars[0], preload_scalars[1]);
        } else if (bytes >= 40 && (header[3] == 13 || header[5] == 13)) {
          std::array<double, 4> values{};
          bool valid = bytes == 72 && header[0] == 72 && header[3] == 13 && header[5] == 13 && header[6] == 0 && header[9] == 4;
          if (valid) {
            std::memcpy(values.data(), static_cast<const unsigned char*>(raw) + 40, sizeof(values));
            for (double value : values)
              valid = valid && (value == 0 || value == 1);
          }
          AcquireSRWLockExclusive(&state.lock);
          state.availability = {};
          state.availability_ms = 0;
          if (valid) {
            state.availability = {true, values[0] != 0, values[1] != 0, values[2] != 0, values[3] != 0};
            state.availability_ms = GetTickCount64();
          }
          ReleaseSRWLockExclusive(&state.lock);
        } else if (bytes >= 40 && (header[3] == 9 || header[5] == 9)) {
          AcquireSRWLockExclusive(&state.lock);
          state.ima_camera.accept(raw, bytes, GetTickCount64());
          ReleaseSRWLockExclusive(&state.lock);
        } else if (bytes >= 40 && (header[3] == 10 || header[5] == 10)) {
          if (!accept_world_preload_packet(raw, bytes, GetTickCount64())) {
            AcquireSRWLockExclusive(&state.lock);
            state.world_preload = {};
            ReleaseSRWLockExclusive(&state.lock);
          }
        } else if (bytes >= 40 && (header[3] == 4 || header[5] == 4)) {
          if (!accept_lighting_packet(raw, bytes, GetTickCount64()))
            lighting_failure("lighting_packet_layout");
        } else {
          accept_aircraft_packet(raw, bytes, GetTickCount64());
        }
      } else if (header[2] == 1) {
        std::array<DWORD, 6> exception{};
        if (bytes >= sizeof(exception))
          std::memcpy(exception.data(), raw, sizeof(exception));
        for (const auto sent : ima_packets)
          if (sent && sent == exception[4]) {
            AcquireSRWLockExclusive(&state.lock);
            state.ima_camera = {};
            ReleaseSRWLockExclusive(&state.lock);
          }
        for (const auto sent : target_packets)
          if (sent && sent == exception[4]) {
            AcquireSRWLockExclusive(&state.lock);
            state.world_preload = {};
            ReleaseSRWLockExclusive(&state.lock);
          }
        for (const auto sent : lighting_packets)
          if (sent && sent == exception[4])
            lighting_failure("lighting_simconnect_exception");
      } else if (header[2] == 3) {
        failure("simulator_quit");
        SetEvent(state.stop);
        break;
      }
    }
    if (reconnect)
      break;
  }
  AcquireSRWLockExclusive(&state.lock);
  state.ima_camera = {};
  state.world_preload = {};
  state.availability = {};
  state.availability_ms = 0;
  ReleaseSRWLockExclusive(&state.lock);
  lighting_failure("lighting_session_closed");
  close(session);
  CloseHandle(notification);
  FreeLibrary(dll);
  return 0;
}
bool fresh(std::uint64_t sample, std::uint64_t now) noexcept {
  return sample && now >= sample && now - sample <= 500;
}
/*
 * Returns speed only while the worker and its independently validated speed sample are current. The caller
 * already holds the shared-state lock.
 */
GroundSpeedSample ground_speed_locked(std::uint64_t now) noexcept {
  GroundSpeedSample out;
  out.sample_ms = state.ground_speed_ms;
  if (fresh(out.sample_ms, now)) {
    out.valid = true;
    out.knots = state.ground_speed_knots;
    out.error = "";
  } else {
    out.error = out.sample_ms ? "ground_speed_telemetry_stale" : state.ground_speed_error;
  }
  return out;
}
/*
 * Builds a lighting snapshot with its age/availability under the existing lock, keeping exposure adaptation
 * independent of pose reception.
 */
LightingSample lighting_locked(std::uint64_t now) noexcept {
  LightingSample out;
  out.sample_ms = state.lighting_ms;
  if (out.sample_ms && now >= out.sample_ms && now - out.sample_ms <= LightingSample::MaximumAgeMs) {
    out.valid = true;
    out.ambient = state.ambient;
    out.brightness = state.brightness;
    out.error = "";
  } else {
    out.error = out.sample_ms ? "lighting_telemetry_stale" : state.lighting_error;
  }
  return out;
}
}  /* namespace */
void set_video_input_ack(std::uint64_t packed) noexcept {
  video_input_ack.store(packed, std::memory_order_release);
}
void set_presence_source(PresenceSource source) noexcept {
  presence_source.store(source, std::memory_order_release);
}
/*
 * Returns the current WASM demand/TV-IR snapshot while holding a shared lock. Availability is based on a
 * live publication rather than texture uploads.
 */
CameraAvailability get_camera_availability() noexcept {
  AcquireSRWLockShared(&state.lock);
  const auto now = GetTickCount64();
  const auto result = state.aircraft_session.running() && state.availability_ms && now >= state.availability_ms &&
                              now - state.availability_ms <= 1000 && state.ima_camera.sample(now).valid
                          ? state.availability
                          : CameraAvailability{};
  ReleaseSRWLockShared(&state.lock);
  return result;
}
AircraftIdentitySample get_aircraft_identity() noexcept {
  AcquireSRWLockShared(&state.lock);
  const auto result = state.aircraft_session.running() ? state.identity.sample(GetTickCount64()) : AircraftIdentitySample{};
  ReleaseSRWLockShared(&state.lock);
  return result;
}
std::uint64_t get_aircraft_session_epoch() noexcept {
  AcquireSRWLockShared(&state.lock);
  const auto result = state.aircraft_session.epoch();
  ReleaseSRWLockShared(&state.lock);
  return result;
}
/*
 * True when a fresh identity sample shows the Rafale package path. ATC brand text alone cannot identify it.
 */
bool aircraft_is_rafale() noexcept {
  const auto identity = get_aircraft_identity();
  return identity.fresh && identity.rafale;
}
/*
 * Starts one telemetry worker and its stop event under the state lock. The module is pinned before a thread
 * can execute callbacks after the caller returns.
 *
 * Aircraft telemetry can start before any native hook pins the addon. FROM_ADDRESS also works for the
 * standalone test executable's main image.
 */
bool start_simconnect_link() noexcept {
  AcquireSRWLockExclusive(&lifecycle);
  if (state.worker && WaitForSingleObject(state.worker, 0) == WAIT_OBJECT_0) {
    CloseHandle(state.worker);
    CloseHandle(state.stop);
    state.worker = nullptr;
    state.stop = nullptr;
  }
  if (state.worker) {
    ReleaseSRWLockExclusive(&lifecycle);
    return true;
  }
  HMODULE provider_module = nullptr;
  if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN, reinterpret_cast<LPCWSTR>(&worker),
                          &provider_module)) {
    ReleaseSRWLockExclusive(&lifecycle);
    failure("provider_module_pin");
    return false;
  }
  state.stop = CreateEventW(nullptr, TRUE, FALSE, nullptr);
  if (state.stop)
    state.worker = CreateThread(nullptr, 0, worker, nullptr, 0, nullptr);
  const bool ready = state.worker != nullptr;
  if (!ready && state.stop) {
    CloseHandle(state.stop);
    state.stop = nullptr;
  }
  ReleaseSRWLockExclusive(&lifecycle);
  return ready;
}
/*
 * Requests worker shutdown, waits outside the state lock, then closes its handles. This avoids blocking the
 * worker while it tries to publish its final state.
 */
void stop_simconnect_link() noexcept {
  AcquireSRWLockExclusive(&lifecycle);
  if (state.worker) {
    SetEvent(state.stop);
    WaitForSingleObject(state.worker, INFINITE);
    CloseHandle(state.worker);
    CloseHandle(state.stop);
    state.worker = nullptr;
    state.stop = nullptr;
  }
  AcquireSRWLockExclusive(&state.lock);
  state.aircraft_ms = 0;
  state.ima_camera = {};
  state.world_preload = {};
  state.correction = 0;
  state.identity = {};
  state.ground_speed_ms = 0;
  state.ground_speed_error = "not_initialized";
  state.lighting_ms = 0;
  state.lighting_error = "not_initialized";
  state.error = "not_initialized";
  ReleaseSRWLockExclusive(&state.lock);
  ReleaseSRWLockExclusive(&lifecycle);
}
void reset_body_pose_calibration() noexcept {
  AcquireSRWLockExclusive(&state.lock);
  state.correction = 0;
  ReleaseSRWLockExclusive(&state.lock);
}
/*
 * Compares the native scene transform with fresh public telemetry using a speed/latency-aware position
 * tolerance. This is a plausibility check, not interpolation or a replacement attachment transform.
 *
 * Position tolerance includes the distance flown during telemetry latency. This does not move the camera:
 * its attachment uses the validated model.
 */
bool validate_aircraft_scene_pose(const BodyPose& scene, std::uint64_t now) noexcept {
  Vector3 lla{};
  if (!valid_body_pose(scene) || !body_math::geodetic(scene.origin, lla))
    return false;
  AcquireSRWLockExclusive(&state.lock);
  now = std::max(now, GetTickCount64());
  const double correction = lla[2] - state.aircraft.altitude;
  const bool current = fresh(state.aircraft_ms, now);
  const double latency = current ? (now - state.aircraft_ms) / 1000.0 + 0.05 : 0;
  const double speed = fresh(state.ground_speed_ms, now) ? state.ground_speed_knots * 0.5144444444444444 : 0;
  const bool accepted = current && std::isfinite(correction) && std::abs(correction) <= 150 &&
                        scene_body_matches_public(scene, body_math::body(state.aircraft, correction), 25 + speed * latency);
  if (accepted)
    state.correction = correction;
  ReleaseSRWLockExclusive(&state.lock);
  return accepted;
}
/*
 * Copies a fresh public aircraft sample and converts it to the common world/body basis. Invalid connection
 * state, age or geometry leaves the sample unavailable.
 */
BodyPoseSnapshot sample_body_pose(std::uint64_t now) noexcept {
  BodyPoseSnapshot out;
  AcquireSRWLockShared(&state.lock);
  const auto locked_now = GetTickCount64();
  if (now < locked_now)
    now = locked_now;
  if (!fresh(state.aircraft_ms, now))
    out.error = state.aircraft_ms ? "aircraft_telemetry_stale" : state.error;
  else {
    out.pose = body_math::body(state.aircraft, state.correction);
    out.sample_ms = state.aircraft_ms;
    out.valid = valid_body_pose(out.pose);
    out.error = out.valid ? "" : "invalid_body_basis";
  }
  ReleaseSRWLockShared(&state.lock);
  return out;
}
GroundSpeedSample get_ground_speed() noexcept {
  AcquireSRWLockShared(&state.lock);
  const auto out = ground_speed_locked(GetTickCount64());
  ReleaseSRWLockShared(&state.lock);
  return out;
}

ImaCameraSample get_ima_camera() noexcept {
  AcquireSRWLockShared(&state.lock);
  const auto result = state.ima_camera.sample(GetTickCount64());
  ReleaseSRWLockShared(&state.lock);
  return result;
}
WorldPreloadTarget get_world_preload_target() noexcept {
  AcquireSRWLockShared(&state.lock);
  const auto result = world_preload_locked(GetTickCount64());
  ReleaseSRWLockShared(&state.lock);
  return result;
}
LightingSample get_lighting() noexcept {
  AcquireSRWLockShared(&state.lock);
  const auto out = lighting_locked(GetTickCount64());
  ReleaseSRWLockShared(&state.lock);
  return out;
}
}  /* namespace azp::native_camera */

namespace azp::native_camera {
/*
 * Validates the lifecycle event payload and advances the session only for an accepted change. Packet layout
 * and string bounds are checked before reading event data.
 */
bool AircraftSessionLifecycle::accept(const void* raw, std::uint32_t bytes) noexcept {
  if (!raw || bytes < 24 || bytes > 65536)
    return false;
  std::array<std::uint32_t, 6> h{};
  std::memcpy(h.data(), raw, sizeof(h));
  if (h[0] != bytes)
    return false;
  if (h[2] == 4 && bytes == 24 && h[4] == SimEvent && h[5] <= 1) {
    const bool transition = known_ && running_ != (h[5] != 0);
    known_ = true;
    running_ = h[5] != 0;
    if (transition)
      changed();
    return transition;
  }
  if (h[2] == 6 && bytes == 288 && (h[4] == AircraftEvent || h[4] == FlightEvent) &&
      std::memchr(static_cast<const char*>(raw) + 24, 0, 260)) {
    changed();
    return true;
  }
  return false;
}

void AircraftSessionLifecycle::changed() noexcept {
  if (epoch_ != UINT64_MAX)
    ++epoch_;
}

void AircraftIdentityCache::begin_request(std::uint32_t type_request, std::uint32_t path_request) noexcept {
  type_request_ = type_request;
  path_request_ = path_request;
  pending_ = {};
  pending_type_ms_ = pending_path_ms_ = 0;
}

/*
 * Correlates the identity response with its pending request and validates bounded strings before publishing
 * the aircraft path/type.
 */
bool AircraftIdentityCache::accept(const void* raw, std::uint32_t bytes, std::uint64_t now) noexcept {
  if (!raw || bytes < 24 || bytes > 65536 || !now)
    return false;
  std::array<std::uint32_t, 10> h{};
  std::memcpy(h.data(), raw, bytes >= 40 ? 40 : 24);
  const bool type = bytes == 296 && h[0] == 296 && h[2] == 8 && h[3] == type_request_ && h[5] == 5 && h[6] == 0 && h[9] == 1;
  const bool path = bytes == 284 && h[0] == 284 && h[2] == 15 && h[3] == path_request_;
  if (!type && !path)
    return false;
  const auto* text = static_cast<const char*>(raw) + (type ? 40 : 24);
  if (!std::memchr(text, 0, type ? 256 : 260))
    return false;
  if (type) {
    std::memcpy(pending_.type.data(), text, 256);
    pending_type_ms_ = now;
  } else {
    std::memcpy(pending_.path.data(), text, 260);
    pending_path_ms_ = now;
  }
  if (pending_type_ms_ && pending_path_ms_) {
    value_ = pending_;
    type_ms_ = pending_type_ms_;
    path_ms_ = pending_path_ms_;
    pending_type_ms_ = pending_path_ms_ = 0;
  }
  return true;
}

AircraftIdentitySample AircraftIdentityCache::sample(std::uint64_t now) const noexcept {
  auto out = value_;
  out.sample_ms = std::min(type_ms_, path_ms_);
  out.fresh = out.sample_ms && now >= out.sample_ms && now - out.sample_ms <= 3000;
  if (out.sample_ms)
    out.rafale = is_rafale_package(out.path.data());
  return out;
}

}  /* namespace azp::native_camera */
