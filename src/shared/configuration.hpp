/*
 * Collects the constants shared by the EXE and the DLL: the WASM image size, the native camera resolution and
 * render-rate limits, exposure bounds, and how the Rafale and its WASM image are recognized. The
 * WASM image size connects GPU copies and the aircraft gauge; changing it requires the external WASM and panel
 * to agree. The native resolution only changes what the engine renders; the compositor scales it to the WASM
 * image.
 */

#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <string_view>

namespace azp {

/*
 * MSFS 2024 settings folders, below %APPDATA% for Steam and %LOCALAPPDATA% for the Microsoft Store. Each holds
 * exe.xml and UserCfg.opt.
 */
inline constexpr const wchar_t* kSteamSettingsFolder = L"\\Microsoft Flight Simulator 2024";
inline constexpr const wchar_t* kStoreSettingsFolder = L"\\Packages\\Microsoft.Limitless_8wekyb3d8bbwe\\LocalCache";

/*
 * Native night-vision intensity written into the AZP view in IR: validated starting value and the bounds of
 * its automatic gain (video/display_exposure.hpp). MSFS's tonemapping shader applies night vision only above
 * intensity 1 and multiplies the scene luminance by it, after the view's own exposure. That exposure can sit
 * far more than 4 EV too low, hence the wide upper bound (about 14 EV above 250).
 */
inline constexpr float kNightVisionDefaultIntensity = 250.f, kNightVisionMinimumIntensity = 2.f, kNightVisionMaximumIntensity = 4e6f;
/*
 * TV and IR exposure chosen in the EXE window, in EV: TV applies it as a gain to the camera's final image, IR
 * shifts the brightness its automatic exposure aims at.
 */
inline constexpr float kMinimumExposureShiftEv = -2.f;
inline constexpr float kMaximumExposureShiftEv = 2.f;
/*
 * TV gain at setting 0, in EV: the engine's final image is displayed 1.5 EV darker, so the
 * TV setting spans -3.5 to +0.5 EV of actual gain.
 */
inline constexpr float kTvNeutralExposureEv = -1.5f;
/*
 * The WASM input image: GPU composition output, WASM-format copies and the upload marker check.
 */
inline constexpr unsigned kCameraWidth = 720;
inline constexpr unsigned kCameraHeight = 720;
/*
 * Side of the square image the engine renders for the camera, in pixels (EXE window setting), a multiple of
 * 8. The compositor scales it to the WASM image: up below 720, down (supersampled) above.
 */
inline constexpr unsigned kMinimumCameraResolution = 240;
inline constexpr unsigned kMaximumCameraResolution = 1080;
inline constexpr bool valid_camera_resolution(unsigned side) noexcept {
  return side >= kMinimumCameraResolution && side <= kMaximumCameraResolution && side % 8 == 0;
}
/*
 * Native camera render limit, in frames per second (EXE window setting); 0 renders on every simulator frame.
 */
inline constexpr unsigned kMinimumRenderRate = 5;
inline constexpr unsigned kMaximumRenderRate = 120;
inline constexpr bool valid_render_rate(unsigned rate) noexcept {
  return !rate || (rate >= kMinimumRenderRate && rate <= kMaximumRenderRate);
}
/*
 * D3D12 texture-to-buffer rows are aligned to 256 bytes; pixels are RGBA8.
 */
inline constexpr unsigned kCameraRowPitch = (kCameraWidth * 4 + 255) & ~255u;

/*
 * Path component that identifies the Rafale package in the loaded aircraft path.
 */
inline constexpr std::string_view kRafalePackageMarker = "simobjects/airplanes/azurpoly_rafale";
/*
 * DXGI formats the WASM image may use: RGBA8 and BGRA8, UNORM, sRGB or typeless.
 */
inline constexpr std::array<unsigned, 6> kWasmImageFormats{28, 29, 87, 91, 27, 90};

/*
 * DLL only: the native resolution in use, set by the control loop before a camera is created. Camera-source
 * recognition compares render targets with it.
 */
inline std::atomic<unsigned> native_camera_side{kCameraWidth};
inline bool camera_sized(unsigned width, unsigned height) noexcept {
  const auto side = native_camera_side.load(std::memory_order_relaxed);
  return width == side && height == side;
}
inline constexpr bool wasm_image_format(unsigned format) noexcept {
  for (auto supported : kWasmImageFormats)
    if (supported == format)
      return true;
  return false;
}
/*
 * Match a complete path component (or component sequence), independent of drive, slash direction and case.
 */
bool path_contains(std::string_view path, std::string_view marker) noexcept;
bool is_rafale_package(std::string_view aircraft_path) noexcept;

}  /* namespace azp */
