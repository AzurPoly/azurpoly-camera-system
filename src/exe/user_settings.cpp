/*
 * Reads and writes settings.json. The file is small and rewritten whole; reading validates every value, so a
 * damaged or hand-edited file can only fall back to defaults.
 */

#include "user_settings.hpp"
#include <nlohmann/json.hpp>
#include <cmath>
#include <cstdio>
#include <vector>

namespace azp::app {
namespace {
using Json = nlohmann::ordered_json;

constexpr DWORD kMaximumFileBytes = 64 * 1024;

bool valid_exposure(double ev) noexcept {
  return std::isfinite(ev) && ev >= kMinimumExposureShiftEv && ev <= kMaximumExposureShiftEv;
}
}  /* namespace */

bool valid_user_settings(const UserSettings& settings) noexcept {
  return valid_camera_resolution(settings.resolution) && valid_render_rate(settings.fps_limit) && valid_exposure(settings.tv_exposure) &&
         valid_exposure(settings.ir_exposure);
}

UserSettings load_user_settings(const std::wstring& path, std::string& note) {
  UserSettings settings;
  note.clear();
  HANDLE file = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr, OPEN_EXISTING,
                            FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    if (GetLastError() != ERROR_FILE_NOT_FOUND && GetLastError() != ERROR_PATH_NOT_FOUND)
      note = "settings.json cannot be read (Windows error " + std::to_string(GetLastError()) + "); defaults are used.";
    return settings;
  }
  std::string text(kMaximumFileBytes, '\0');
  DWORD read = 0;
  const bool complete = ReadFile(file, text.data(), kMaximumFileBytes, &read, nullptr) && read < kMaximumFileBytes;
  CloseHandle(file);
  text.resize(read);
  const auto json = complete ? Json::parse(text, nullptr, false) : Json(nullptr);
  if (json.is_discarded() || !json.is_object()) {
    note = "settings.json is not a valid settings file; defaults are used.";
    return settings;
  }
  std::vector<std::string> invalid;
  const auto count = [&](const char* key, std::uint32_t& value, bool (*valid)(unsigned) noexcept) {
    const auto found = json.find(key);
    if (found == json.end())
      return;
    if (found->is_number_unsigned() && found->get<std::uint64_t>() <= UINT32_MAX && valid(found->get<std::uint32_t>()))
      value = found->get<std::uint32_t>();
    else
      invalid.push_back(key);
  };
  count("resolution", settings.resolution, valid_camera_resolution);
  count("fpsLimit", settings.fps_limit, valid_render_rate);
  /*
   * The former fixed TV exposure ("exposure") and "automaticExposure" are ignored.
   */
  const auto shift = [&](const char* key, float& value) {
    const auto found = json.find(key);
    if (found == json.end())
      return;
    if (found->is_number() && valid_exposure(found->get<double>()))
      value = found->get<float>();
    else
      invalid.push_back(key);
  };
  shift("tvExposure", settings.tv_exposure);
  shift("irExposure", settings.ir_exposure);
  if (!invalid.empty()) {
    note = "settings.json: invalid";
    for (const auto& key : invalid)
      note += " " + key;
    note += "; the default is used instead.";
  }
  return settings;
}

bool save_user_settings(const std::wstring& path, const UserSettings& settings) {
  const Json json{{"resolution", settings.resolution},
                  {"fpsLimit", settings.fps_limit},
                  {"tvExposure", settings.tv_exposure},
                  {"irExposure", settings.ir_exposure}};
  const auto text = json.dump(2) + "\n";
  HANDLE file = CreateFileW(path.c_str(), GENERIC_WRITE, FILE_SHARE_READ, nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE)
    return false;
  DWORD written = 0;
  const bool ok = SetEndOfFile(file) && WriteFile(file, text.data(), static_cast<DWORD>(text.size()), &written, nullptr) &&
                  written == text.size();
  CloseHandle(file);
  return ok;
}

standalone::Settings dll_settings(const UserSettings& user) {
  standalone::Settings settings;
  settings.tv_exposure = user.tv_exposure;
  settings.ir_exposure = user.ir_exposure;
  settings.camera_resolution = user.resolution;
  settings.render_rate = user.fps_limit;
  return settings;
}

std::string describe(const UserSettings& settings) {
  char text[160]{};
  std::snprintf(text, sizeof(text), "camera %u x %u, %s, TV exposure %+.1f EV, IR exposure %+.1f EV", settings.resolution,
                settings.resolution,
                settings.fps_limit ? ("render limit " + std::to_string(settings.fps_limit) + " fps").c_str() : "every simulator frame",
                settings.tv_exposure, settings.ir_exposure);
  return text;
}

}  /* namespace azp::app */
