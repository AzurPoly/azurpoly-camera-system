/*
 * Defines the IR automatic exposure. The control loop measures the displayed image (scene_frame_output.cpp)
 * on every 8th composition and gives each IR measurement to the controller; every IR tick it moves the
 * exposure toward the controller's aim. The exposure is the log2 of the native night-vision intensity, which
 * the NV render callback writes.
 */

#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include "../shared/configuration.hpp"

namespace azp {

/*
 * One automatic exposure, in EV. The AZP view's native IR exposure cannot be relied on, so IR is regulated
 * from the displayed image, by day and by night. Brightness is the display-ready value averaged over the
 * image (0 black, 1 white). The target is `target` shifted by the window's exposure setting (EV).
 *
 * Each measurement sets the aim: the exposure the measured image was composed with, plus GainPerEv times the
 * brightness error. The image is tone-mapped and sRGB-encoded, so one EV of exposure moves the measured
 * brightness by roughly half an EV and twice the error lands near the target in one measurement; this stays
 * stable while the response is under one EV per EV. Starting from the measured image's own exposure, not the
 * current one, keeps a measurement taken before the ramp moved from being applied twice. Errors inside the
 * dead band aim at the measured exposure, so the image does not hunt.
 *
 * Every control tick the exposure follows the aim at up to RateEvPerSecond (at least ten times the former
 * 0.5 EV/s): at most 0.125 EV per 25 ms tick, a ramp rather than visible steps. The exposure
 * stays inside [minimum_ev, maximum_ev].
 */
class ExposureController {
 public:
  static constexpr float DeadBandEv = .15f, GainPerEv = 2, RateEvPerSecond = 5;

  ExposureController(float start_ev, float minimum_ev, float maximum_ev, float target) noexcept
      : ev_(start_ev), aim_(start_ev), minimum_ev_(minimum_ev), maximum_ev_(maximum_ev), target_(target) {}
  float ev() const noexcept { return ev_; }
  /*
   * Takes a new measurement of an image composed at `measured_ev`. `sequence` counts measurements; 0 means
   * none, and a repeated one is ignored. `shift_ev` is the exposure setting.
   */
  void measure(float brightness, std::uint64_t sequence, float measured_ev, float shift_ev = 0) noexcept;
  /*
   * Moves the exposure toward the aim and returns it. Time is in ms; a gap longer than 100 ms counts as
   * 100 ms, so a mode that resumes does not jump.
   */
  float advance(std::uint64_t now_ms) noexcept;

 private:
  float ev_, aim_, minimum_ev_, maximum_ev_, target_;
  std::uint64_t sequence_ = 0, previous_ms_ = 0;
};

/*
 * ponytail: provisional target, to be calibrated from runs (the log shows each gain with its
 * brightness). IR aims at the native brightness, before the display inversion: .7 shows the inverted image
 * 1 EV darker than the former .35.
 */
inline constexpr float kIrTargetBrightness = .7f;

}  /* namespace azp */
