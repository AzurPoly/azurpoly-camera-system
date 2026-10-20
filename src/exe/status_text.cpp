/*
 * Formats the EXE's view of the camera: the video rate the window shows and one compact key=value line for
 * the periodic log. A DLL status older than five seconds means the camera bridge is not running (or MSFS is
 * stalled); only that fact is shown then.
 */

#include "status_text.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>

namespace azp::app {
namespace {
/*
 * printf-style formatting into a std::string.
 */
template <class... Args>
std::string text(const char* format, Args... args) {
  char buffer[2048]{};
  std::snprintf(buffer, sizeof(buffer), format, args...);
  return buffer;
}

unsigned long long count(std::uint64_t value) {
  return static_cast<unsigned long long>(value);
}
}  /* namespace */

InputMetrics::InputMetrics(const standalone::Status& status) {
  if (std::isfinite(status.input_fps) && status.input_fps >= 0)
    rate = text("%.1f", status.input_fps);
  const bool sample = status.input_latency_valid && std::isfinite(status.input_age_ms) && status.input_age_ms >= 0;
  if (sample)
    age = text("%.0f", status.input_age_ms);
  if (sample && status.input_age_ms <= 1000 && std::isfinite(status.input_latency_ms) && status.input_latency_ms >= 0)
    latency = text("%.1f", status.input_latency_ms);
}

/*
 * The DLL's sensor message starts with "TV" in TV, alone or followed by the exposure measurement; every IR
 * message describes the night-vision state instead.
 */
bool infrared(const standalone::Status& status) {
  return *status.night_vision_message && std::strncmp(status.night_vision_message, "TV", 2) != 0;
}

/*
 * The periodic log line: mode, stream state, video rate/latency/age, native resolution and limit, callback
 * CPU, pipeline counters, then the DLL message and the sensor message.
 */
std::string log_line(const Snapshot& snapshot) {
  const auto& status = snapshot.status;
  if (!snapshot.simulator)
    return "waiting for MSFS 2024";
  if (!snapshot.fresh)
    return text("msfs=%lu no recent status from the camera bridge", static_cast<unsigned long>(snapshot.simulator));
  const InputMetrics input(status);
  const auto limit = snapshot.settings.render_rate ? text("%u", snapshot.settings.render_rate) : std::string("none");
  return text(
      "msfs=%lu mode=%s streaming=%u camera=%u video_fps=%s latency_ms=%s age_ms=%s resolution=%u requested=%u limit_fps=%s "
      "camera_cpu_ms=%.1f ir_cpu_ms=%.1f uploads=%llu captures=%llu images=%llu copies=%llu preload=%s | %s | sensor: %s",
      static_cast<unsigned long>(snapshot.simulator), infrared(status) ? "IR" : "TV", status.streaming, status.camera_ready,
      input.rate.c_str(), input.latency.c_str(), input.age.c_str(), status.camera_resolution, snapshot.settings.camera_resolution,
      limit.c_str(), status.camera_cpu_ms, status.ir_cpu_ms, count(status.wasm_uploads), count(status.captures),
      count(status.images), count(status.input_copies), status.world_preload_active ? "active" : "inactive", status.message,
      *status.night_vision_message ? status.night_vision_message : "TV");
}

std::string narrow(const std::wstring& text) {
  if (text.empty())
    return {};
  const int size = WideCharToMultiByte(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), nullptr, 0, nullptr, nullptr);
  std::string result(size > 0 ? size : 0, '\0');
  if (size > 0)
    WideCharToMultiByte(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), result.data(), size, nullptr, nullptr);
  return result;
}

std::wstring widen(const std::string& text) {
  if (text.empty())
    return {};
  const int size = MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), nullptr, 0);
  std::wstring result(size > 0 ? size : 0, L'\0');
  if (size > 0)
    MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), result.data(), size);
  return result;
}

}  /* namespace azp::app */
