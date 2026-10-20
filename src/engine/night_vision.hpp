/*
 * Defines the native night-vision controller and the proof it retains after initialization. The same proof
 * enables the owned view's post-processing, which gives the camera its final image in TV and IR. Callers
 * provide a current owned-view inspection and the correct callback context before any parameter or flag
 * change is allowed.
 */

#pragma once
#include <array>
#include <cstdint>
#include <string>
#include "camera_layout.hpp"
#include "observer_hook.hpp"
#include "view.hpp"

namespace azp::native_camera {

/*
 * The reviewed renderer suppresses stages 41, 52 and 61 when this bit is set.
 */
inline constexpr std::uint64_t kViewBypassPostProcessing = std::uint64_t{1} << 36;
/*
 * With this bit set, the reviewed post-processing setup computes the view's exposure with
 * SID_VS_LUMINANCE_INIT (key / clamped luminance of the current frame's histogram) instead of
 * SID_VS_LUMINANCE_ADAPTATION, which smooths from the previous frame's exposure. The AZP view's previous
 * exposure goes stale: without this bit its exposure stayed frozen for minutes in IR, then jumped by up to
 * 18 EV. MSFS sets the same bit for its manual exposure. The engine starts the AZP view with it clear; AZP
 * sets it whenever it enables the view's post-processing.
 */
inline constexpr std::uint64_t kViewPerFrameExposure = std::uint64_t{1} << 19;

/*
 * Verified locations of the reviewed pre-SU7 night-vision path, resolved once by engine_api.cpp. All values
 * are RVAs in the main image; `error` explains why the camera cannot use it.
 */
struct NightVisionContract {
  bool valid = false;
  std::string error = "NV unavailable: not resolved.";
  std::uint32_t override_rva = 0;
  std::uint32_t world_vtable_rva = 0;
  std::uint32_t callback_return_rva = 0;
  /*
   * The world render-callback slot, its current original implementation, and the read-only image section
   * that contains the slot (required by the hook installer's slot proof).
   */
  std::uint32_t callback_slot_rva = 0;
  std::uint32_t callback_original_rva = 0;
  std::uint32_t slot_section_rva = 0, slot_section_size = 0;
  /*
   * The optional proof that kViewPerFrameExposure selects SID_VS_LUMINANCE_INIT (shader 0x548) in the
   * post-processing setup. Without it the final image keeps the native adaptation.
   */
  bool per_frame_exposure = false;
};
NightVisionContract resolve_night_vision_contract(discovery::ImageReader& reader, const discovery::Inventory& image, std::uintptr_t base);

/*
 * Post-processing and native fullscreen night vision for the owned IMA camera. Engine contracts and
 * view/world generation checks isolate this from the pilot's camera. Parameters are applied only in the
 * verified per-view render callback, after the engine's unconditional reset. TV relies on that reset; no
 * values persist across view reuse and no heap pointer is retained here.
 */
class NightVision {
 public:
  /*
   * Installs the verified world render-callback observer. An invalid contract or a refused hook leaves it
   * unavailable, and the camera does not start.
   */
  void initialize(const NightVisionContract& contract,
                  std::uintptr_t base,
                  std::uint32_t image_size,
                  engine_hook::ViewParametersObserver observer);
  bool available() const noexcept { return available_; }
  /*
   * Only the camera observer may change these flags, with the owned gate closed, and it fully reinspects the
   * view before reopening it.
   */
  bool enable_post_processing(const engine_camera::OwnedViewSnapshot& view, const CameraImageLayout& layout);
  /*
   * The view flag word enable_post_processing writes: post-processing allowed (bit 36 clear), with per-frame
   * exposure when that is verified.
   */
  std::uint64_t post_processing_flags(std::uint64_t flags) const noexcept;
  /*
   * A fresh owned-view proof and the native callback arguments are required. `intensity` is the IR gain
   * (kNightVisionMinimumIntensity..kNightVisionMaximumIntensity); anything else falls back to the default.
   */
  void apply(const engine_camera::OwnedViewSnapshot& view, void* world, void* context, std::uintptr_t return_address, float intensity);
  /*
   * The inline render context the per-view callback receives for the view at `view_address`.
   */
  static std::uintptr_t render_context(std::uint64_t view_address) noexcept { return view_address + 0x5c90; }
  const std::string& message() const noexcept { return message_; }

 private:
  bool available_ = false, per_frame_exposure_ = false;
  std::uint32_t override_rva_ = 0;
  std::uint32_t world_vtable_rva_ = 0;
  std::uint32_t callback_return_rva_ = 0;
  std::uint32_t image_size_ = 0;
  std::string message_ = "Native post-processing and night vision not initialized.";
};

}  /* namespace azp::native_camera */
