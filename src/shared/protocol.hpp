/*
 * Defines the versioned EXE/DLL messages and their synchronization wrapper. The EXE publishes settings and
 * presence; the DLL returns camera/video status. This is separate from the existing WASM LVAR and
 * image-marker contracts.
 */

#pragma once
#include <windows.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cwchar>
#include "configuration.hpp"
#include "version.hpp"

namespace azp::standalone {
constexpr std::uint32_t ProtocolMagic = 0x54415849, ProtocolVersion = 37;
constexpr const wchar_t* Version = AZP_CAMERA_VERSION_WIDE;
struct Settings {
  std::uint32_t enabled = 1;
  /*
   * TV and IR exposure chosen in the EXE window, in EV (bounds in configuration.hpp): TV applies it as a gain
   * to the camera's final image; IR shifts the brightness its automatic exposure aims at
   * (video/display_exposure.hpp).
   */
  float tv_exposure = 0, ir_exposure = 0;
  /*
   * Native camera resolution (square side, pixels, 240 to 1080) and render limit (frames per second, 0 = every
   * simulator frame), chosen in the EXE window.
   */
  std::uint32_t camera_resolution = kCameraWidth, render_rate = 0;
  std::uint64_t aircraft_session_epoch{};
  /*
   * Incremented once per second by the EXE; the DLL only relays it to WASM.
   */
  std::uint64_t heartbeat_counter{};
};
/*
 * What the EXE shows (window, periodic log) and what the bridge log records. `heartbeat` is the DLL tick
 * (GetTickCount64, ms) when the status was written; the EXE treats an older status as stale.
 */
struct Status {
  char message[384]{};
  char night_vision_message[256]{};
  char wasm_input_status[96]{};
  char world_preload_message[160]{};
  std::uint64_t heartbeat{}, aircraft_session_epoch{};
  /*
   * Pipeline stages, counted independently: marked WASM uploads recognized, camera captures, composed images
   * and GPU copies recorded into the WASM input. Their totals are not expected to match.
   */
  std::uint64_t wasm_uploads{}, captures{}, images{}, input_copies{};
  /*
   * Video input: distinct composed images whose copy into the WASM input completed on the GPU, their
   * one-second rate, and capture-to-copy latency (an upper bound, fence polling) and age, in milliseconds.
   * Not the final gauge draw rate; screen presentation is not included.
   */
  std::uint64_t input_frames{};
  float input_fps{}, input_latency_ms{}, input_age_ms{};
  std::uint32_t input_latency_valid{};
  std::uint32_t graphics_ready{}, camera_ready{}, streaming{};
  std::uint64_t hook_failures{};
  std::uint32_t availability_valid{}, ima_available_left{}, ima_available_right{};
  std::uint32_t world_preload_active{};
  std::uint64_t world_preload_requests{}, world_preload_releases{};
  /*
   * Latest WASM guidance: pitch, bank, heading and FOV in degrees.
   */
  std::uint32_t ima_valid{}, ima_has_target{};
  std::array<double, 4> ima_angles{};
  /*
   * Time the camera's engine callbacks took from simulator threads over the last second, in ms per second:
   * the manager-update observer and the per-view NV render callback (IR only).
   */
  float camera_cpu_ms{}, ir_cpu_ms{};
  /*
   * Resolution of the camera in use (0 while none is established) and the applied render limit (0 = none).
   */
  std::uint32_t camera_resolution{}, render_rate{};
};
struct Shared {
  std::uint32_t magic{}, version{}, bytes{}, owner_pid{};
  std::uint64_t owner_heartbeat{};
  Settings settings;
  Status status;
};
bool valid_settings(const Settings& s) noexcept;
class Mailbox {
 public:
  ~Mailbox() { close(); }
  Mailbox() = default;
  Mailbox(const Mailbox&) = delete;
  Mailbox& operator=(const Mailbox&) = delete;
  bool open(DWORD pid, bool create);
  enum class LockResult { acquired, busy, invalid };
  bool lock(DWORD timeout = 0) noexcept { return try_lock(timeout) == LockResult::acquired; }
  LockResult try_lock(DWORD timeout = 0) noexcept;
  void unlock() noexcept { ReleaseMutex(mutex_); }
  Shared* data() const noexcept { return memory_; }
  void close() noexcept;

 private:
  HANDLE mutex_{}, mapping_{};
  Shared* memory_{};
};
}  /* namespace azp::standalone */

namespace azp::standalone {
/*
 * Only a complete locked message replaces the cache. Contention is not OFF.
 */
class CompanionControl {
 public:
  CompanionControl() noexcept { settings_.enabled = 0; }
  /*
   * Copies a complete locked message into the local control cache after validating its header and settings.
   * Ordinary lock contention retains the last complete message; connection freshness is checked separately.
   */
  Mailbox::LockResult refresh(Mailbox& mailbox) noexcept {
    const auto result = mailbox.try_lock();
    if (result == Mailbox::LockResult::busy) {
      ++busy_reads_;
      return result;
    }
    if (result != Mailbox::LockResult::acquired) {
      invalidate();
      return result;
    }
    const auto& message = *mailbox.data();
    if (message.magic == ProtocolMagic && message.version == ProtocolVersion && message.bytes == sizeof(Shared) && message.owner_pid &&
        valid_settings(message.settings)) {
      settings_ = message.settings;
      heartbeat_ = message.owner_heartbeat;
    } else {
      invalidate();
    }
    mailbox.unlock();
    return result;
  }
  /*
   * Sample now AFTER refresh: a writer may advance the heartbeat after loop entry.
   */
  bool connected(std::uint64_t now) const noexcept { return heartbeat_ && now >= heartbeat_ && now - heartbeat_ <= MaximumHeartbeatAgeMs; }
  const Settings& settings() const noexcept { return settings_; }
  std::uint64_t busy_reads() const noexcept { return busy_reads_; }
  static constexpr std::uint64_t MaximumHeartbeatAgeMs = 5000;

 private:
  void invalidate() noexcept {
    settings_ = {};
    settings_.enabled = 0;
    heartbeat_ = 0;
  }
  Settings settings_;
  std::uint64_t heartbeat_{}, busy_reads_{};
};
}  /* namespace azp::standalone */
