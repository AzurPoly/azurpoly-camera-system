/*
 * Declares the video service, its settings, upload replacement and status. The interface keeps camera
 * control separate from GPU composition and exposes input-copy metrics without claiming final gauge
 * presentation timing.
 */

#pragma once
#include "../shared/configuration.hpp"
#include "scene_capture_manager.hpp"
#include "scene_frame_output.hpp"

namespace azp::scene_runtime {
/*
 * Consumes only our versioned, marked WASM RGBA upload. A valid original copy supplies the destination
 * layout; we replace its pixels without an RT bounce.
 */
bool replace_wasm_upload(ID3D12GraphicsCommandList*,
                         const D3D12_TEXTURE_COPY_LOCATION*,
                         UINT x,
                         UINT y,
                         UINT z,
                         const D3D12_TEXTURE_COPY_LOCATION*,
                         const D3D12_BOX*,
                         const char* recording_refusal);
struct Snapshot {
  /*
   * Recognized marked uploads, and the state of the last one: a refusal reason once the marker was seen.
   */
  std::uint64_t wasm_uploads = 0;
  const char* wasm_input_status = "waiting_for_marked_wasm_upload";
  bool initialized = false;
  bool output = false;
  bool failed = false;
  std::uint64_t frames = 0;
  std::uint64_t input_copies = 0;
  /*
   * Distinct images whose GPU copy into the WASM input image completed. Latency is capture submission to
   * observed fence completion, not monitor Present or the final gauge draw.
   */
  std::uint64_t input_frames = 0;
  float input_fps = 0, input_latency_ms = 0, input_age_ms = 0;
  bool input_latency_valid = false;
  float tv_exposure_ev = 0;
  /*
   * Displayed image brightness, measured on every 8th composition; sequence 0 until the first one of the
   * current TV/IR mode. IR is inverted for display, so its native brightness is 1 - mean.
   */
  SceneFrameOutput::Brightness brightness;
  const char* message = "Camera video pipeline not started.";
  SceneCaptureManager::Statistics capture;
};
SceneCaptureManager& manager();
bool init_device(ID3D12Device* device);
bool init_queue(ID3D12CommandQueue* queue);
bool prepare();
/*
 * TV exposure setting in EV, clamped to the window's range, applied as a gain from the next TV composition
 * without restarting the scene.
 */
bool set_tv_exposure(float ev);
/*
 * The exposure (EV) the next compositions are made with, recorded with their brightness measurements: the
 * log2 night-vision intensity in IR.
 */
void set_measurement_exposure(float ev);
/*
 * Mode changes discard frames from the previous mode, which the compositor displays differently.
 */
void set_night_vision(bool enabled);
/*
 * True while the WASM input image is being fed; input-copy metrics and consumer fences follow it.
 */
void set_wasm_input_active(bool active);
void reset_feed();
void service();
Snapshot snapshot();
}  /* namespace azp::scene_runtime */
