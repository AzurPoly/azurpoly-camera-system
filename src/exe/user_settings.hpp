/*
 * The camera settings chosen in the window. They are saved as JSON in
 * %LOCALAPPDATA%\AzurPoly\CameraSystem\settings.json and read by every run, including the background run MSFS
 * starts. Defaults: 720 x 720, every simulator frame, TV exposure 0 EV, IR exposure 0 EV. Camera guidance
 * stays with the WASM.
 */

#pragma once
#include "../shared/protocol.hpp"
#include <string>

namespace azp::app {
/*
 * `resolution`: side of the square image the simulator renders for the camera, in pixels. `fps_limit`:
 * rendered frames per second at most, 0 = every simulator frame. `tv_exposure` (EV) is a gain on the TV
 * image; `ir_exposure` (EV) shifts the brightness the IR automatic exposure aims at.
 */
struct UserSettings {
  std::uint32_t resolution = kCameraWidth, fps_limit = 0;
  float tv_exposure = 0, ir_exposure = 0;
  bool operator==(const UserSettings&) const = default;
};
bool valid_user_settings(const UserSettings& settings) noexcept;
/*
 * Reads `path`. A missing file gives the defaults; a missing value gives its default, and an invalid one its
 * default plus a `note` naming it.
 */
UserSettings load_user_settings(const std::wstring& path, std::string& note);
/*
 * Rewrites `path` with `settings`. Returns false when the file cannot be written.
 */
bool save_user_settings(const std::wstring& path, const UserSettings& settings);
/*
 * The DLL settings for these user settings.
 */
standalone::Settings dll_settings(const UserSettings& settings);
/*
 * One line for the log, for example "camera 720 x 720, every simulator frame, TV exposure +0.0 EV, IR
 * exposure +0.0 EV".
 */
std::string describe(const UserSettings& settings);

}  /* namespace azp::app */
