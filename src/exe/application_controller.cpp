/*
 * Implements the interface's commands and state. The commands are few, to keep the JavaScript/C++ contract
 * readable (ui/src/domain/camera.ts): refresh, setEnabled, setSettings, setStartup, openLogFolder and quit.
 * Closing the window, not a command, only hides it.
 */

#include "application_controller.hpp"
#include <nlohmann/json.hpp>
#include <shellapi.h>
#include <stdexcept>

namespace azp::app {
namespace {
using Json = nlohmann::json;

constexpr std::uint64_t kStartupRefreshMs = 5000;
/*
 * The window's Low, Medium and High resolutions, in pixels. settings.json accepts any valid resolution, so a
 * value saved by an older version keeps working.
 */
constexpr std::uint32_t kResolutions[] = {480, 720, 1080};

Json settings_json(const UserSettings& settings) {
  return {{"resolution", settings.resolution},
          {"fpsLimit", settings.fps_limit},
          {"tvExposure", settings.tv_exposure},
          {"irExposure", settings.ir_exposure}};
}

const Json& field(const Json& object, const char* key) {
  const auto found = object.find(key);
  if (found == object.end())
    throw std::invalid_argument(std::string("missing field ") + key);
  return *found;
}

std::uint32_t count(const Json& object, const char* key) {
  const auto& value = field(object, key);
  if (!value.is_number_unsigned() || value.get<std::uint64_t>() > UINT32_MAX)
    throw std::invalid_argument(std::string(key) + " must be a whole number");
  return value.get<std::uint32_t>();
}

/*
 * The settings of a setSettings command: every value is required and validated like settings.json.
 */
UserSettings settings_from(const Json& command) {
  const auto& values = field(command, "settings");
  if (!values.is_object())
    throw std::invalid_argument("settings must be an object");
  UserSettings settings;
  settings.resolution = count(values, "resolution");
  settings.fps_limit = count(values, "fpsLimit");
  const auto& tv_exposure = field(values, "tvExposure");
  const auto& ir_exposure = field(values, "irExposure");
  if (!tv_exposure.is_number() || !ir_exposure.is_number())
    throw std::invalid_argument("invalid exposure settings");
  settings.tv_exposure = tv_exposure.get<float>();
  settings.ir_exposure = ir_exposure.get<float>();
  if (!valid_user_settings(settings))
    throw std::invalid_argument("settings out of range");
  return settings;
}

/*
 * The header's status word: waiting without MSFS, starting while the camera bridge gives no fresh status,
 * then streaming or standby.
 */
Json status_json(const Snapshot& snapshot) {
  const bool streaming = snapshot.fresh && snapshot.status.streaming;
  const char* text = !snapshot.simulator ? "Waiting for MSFS" : !snapshot.fresh ? "Starting" : streaming ? "Streaming" : "Standby";
  return {{"text", text}, {"streaming", streaming}};
}

}  /* namespace */

ApplicationController::ApplicationController(Companion& companion,
                                             RunLog& log,
                                             std::wstring settings_path,
                                             const UserSettings& settings,
                                             std::wstring executable,
                                             std::function<void()> quit)
    : companion_(companion),
      log_(log),
      settings_path_(std::move(settings_path)),
      executable_(std::move(executable)),
      settings_(settings),
      quit_(std::move(quit)) {
  update();
}

void ApplicationController::update() {
  const auto now = GetTickCount64();
  if (startup_read_ && now - startup_read_ < kStartupRefreshMs)
    return;
  startup_ = simulator_startup(executable_);
  startup_read_ = now;
}

void ApplicationController::handle_command(std::string_view json) {
  try {
    const auto command = Json::parse(json);
    const auto& type = field(command, "type");
    if (!type.is_string())
      throw std::invalid_argument("invalid command type");
    const auto name = type.get<std::string>();
    if (name == "setSettings") {
      change_settings(settings_from(command));
    } else if (name == "setEnabled") {
      const auto& enabled = field(command, "enabled");
      if (!enabled.is_boolean())
        throw std::invalid_argument("enabled must be true or false");
      error_.clear();
      if (enabled_ != enabled.get<bool>()) {
        enabled_ = enabled.get<bool>();
        companion_.set_enabled(enabled_);
      }
    } else if (name == "setStartup") {
      const auto& enabled = field(command, "enabled");
      if (!enabled.is_boolean())
        throw std::invalid_argument("enabled must be true or false");
      change_startup(enabled.get<bool>());
    } else if (name == "openLogFolder") {
      open_log_folder();
    } else if (name == "quit") {
      if (!quit_)
        throw std::runtime_error("quit is unavailable");
      quit_();
    } else if (name != "refresh") {
      throw std::invalid_argument("unknown command " + name);
    }
  } catch (const std::exception& error) {
    error_ = std::string("Interface command refused: ") + error.what();
  }
}

/*
 * An established camera keeps its resolution until MSFS closes (the DLL never recreates it for a new one),
 * so while MSFS runs the resolution stays as it is and only the other settings change.
 */
void ApplicationController::change_settings(UserSettings settings) {
  error_.clear();
  if (companion_.snapshot().simulator)
    settings.resolution = settings_.resolution;
  if (settings == settings_)
    return;
  settings_ = settings;
  companion_.apply(settings);
  if (settings_path_.empty() || !save_user_settings(settings_path_, settings)) {
    error_ = "The new settings apply now but could not be saved: the next start uses the previous ones.";
    log_.write("settings.json could not be written: " + narrow(settings_path_));
  }
}

void ApplicationController::change_startup(bool enabled) {
  bool ok = false;
  startup_ = set_simulator_startup(executable_, enabled, ok);
  startup_read_ = GetTickCount64();
  std::string detail = startup_.empty() ? " no MSFS 2024 installation found." : "";
  for (const auto& startup : startup_)
    if (!startup.error.empty())
      detail += " " + narrow(startup.edition) + ": " + startup.error;
  log_.write(std::string(enabled ? "MSFS startup enabled" : "MSFS startup disabled") + (ok ? "." : ", with errors:" + detail));
  error_ = ok ? std::string() : "MSFS startup could not be changed:" + detail;
}

void ApplicationController::open_log_folder() {
  const auto file = companion_.snapshot().log_path;
  const auto slash = file.find_last_of(L"\\/");
  const auto folder = slash == std::wstring::npos ? std::wstring() : file.substr(0, slash);
  const auto opened = !folder.empty() && reinterpret_cast<INT_PTR>(ShellExecuteW(nullptr, L"open", folder.c_str(), nullptr, nullptr,
                                                                                 SW_SHOWNORMAL)) > 32;
  error_ = opened ? std::string() : "The log folder could not be opened.";
}

std::string ApplicationController::state_json() const {
  const auto snapshot = companion_.snapshot();
  bool starts = !startup_.empty();
  for (const auto& startup : startup_)
    starts = starts && startup.current;
  const Json state{
      {"appVersion", narrow(standalone::Version)},
      {"enabled", enabled_},
      {"status", enabled_ ? status_json(snapshot) : Json{{"text", "Off"}, {"streaming", false}}},
      {"settings", settings_json(settings_)},
      {"defaults", settings_json(UserSettings{})},
      {"limits",
       {{"resolutions", kResolutions},
        {"fpsMin", kMinimumRenderRate},
        {"fpsMax", kMaximumRenderRate},
        {"exposureMin", kMinimumExposureShiftEv},
        {"exposureMax", kMaximumExposureShiftEv}}},
      {"resolutionLocked", snapshot.simulator != 0},
      {"startup", {{"enabled", starts}, {"available", !startup_.empty()}}},
      {"fps", enabled_ && snapshot.fresh && snapshot.status.streaming ? InputMetrics(snapshot.status).rate : std::string()},
      {"error", error_},
  };
  /*
   * DLL messages are ASCII; a non-UTF-8 byte in any text is replaced rather than failing the whole state.
   */
  return state.dump(-1, ' ', false, Json::error_handler_t::replace);
}

}  /* namespace azp::app */
