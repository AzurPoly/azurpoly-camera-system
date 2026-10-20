/*
 * Implements the automatic exposure (display_exposure.hpp): an aim per brightness measurement and a bounded
 * ramp toward it per control tick.
 */

#include "display_exposure.hpp"

namespace azp {

/*
 * Brightness is clamped away from 0 and 1 so a fully black or saturated image still gives a finite error in
 * the right direction. The target stays inside [0.02, 0.9] whatever the shift.
 */
void ExposureController::measure(float brightness, std::uint64_t sequence, float measured_ev, float shift_ev) noexcept {
  if (!sequence || sequence == sequence_ || !std::isfinite(brightness) || !std::isfinite(measured_ev))
    return;
  sequence_ = sequence;
  const auto target = std::clamp(target_ * std::exp2(std::isfinite(shift_ev) ? shift_ev : 0.f), .02f, .9f);
  const auto error_ev = std::log2(target / std::clamp(brightness, 1.f / 255, 254.f / 255));
  aim_ = std::clamp(measured_ev + (std::fabs(error_ev) > DeadBandEv ? GainPerEv * error_ev : 0.f), minimum_ev_, maximum_ev_);
}

float ExposureController::advance(std::uint64_t now_ms) noexcept {
  const auto elapsed = previous_ms_ && now_ms >= previous_ms_ ? std::min<std::uint64_t>(now_ms - previous_ms_, 100) : 0;
  previous_ms_ = now_ms;
  const auto step = static_cast<float>(elapsed) * (RateEvPerSecond / 1000);
  ev_ += std::clamp(aim_ - ev_, -step, step);
  return ev_;
}

}  /* namespace azp */
