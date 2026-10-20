/*
 * Runs the GPU video pipeline: select completed captures, compose TV/IR, retain a fresh output and feed
 * marked WASM uploads. Image identity, command-list state and capture age are checked before replacement;
 * the WASM draws the menu after sampling this input image.
 */

#include "scene_runtime.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include "../camera/simconnect_link.hpp"
#include "native_hooks.hpp"
#include "resource_guards.hpp"
#include "scene_frame_output.hpp"

namespace azp::scene_runtime {
namespace {
/*
 * Video state of the simulator's device. `pending` holds the newest usable capture; `output` owns the
 * composed image and the WASM-format copy buffers.
 */
struct Device {
  ID3D12Device* native = nullptr;
  SceneFrameOutput output;
  SceneCaptureManager::Frame pending;
  SceneCopyMatch committed;
  SceneCaptureManager::FrameOrder newest;
  Snapshot status;
  std::uint64_t output_capture_time_us = 0;
  bool night_vision = false;
  unsigned compositions = 0;
  float measurement_exposure_ev = 0;
};
struct Runtime {
  std::mutex mutex;
  Device device;
};
Runtime& runtime() {
  static auto* const instance = new Runtime;
  return *instance;
}
/*
 * The device once init_device registered it, else null.
 */
Device* device() {
  auto& item = runtime().device;
  return item.native ? &item : nullptr;
}
/*
 * Capture preserves the bytes. A typeless 8-bit color output is sampled as UNORM for the camera image; this
 * is not a calibrated color-space claim.
 */
DXGI_FORMAT image_format(ID3D12Resource* resource) {
  return scene_texture_view(resource->GetDesc().Format);
}
void discard(Device& item) {
  if (item.pending.token)
    manager().discard_frame(item.pending.token);
  item.pending = {};
}
/*
 * Age is measured from capture, not repeated gauge writes. Source loss must never keep an old image alive
 * for longer than two seconds.
 */
bool displayable_output(const Device& item) {
  if (!item.status.output)
    return false;
  const auto now = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
  return item.output_capture_time_us && now >= item.output_capture_time_us && now - item.output_capture_time_us < 2000000;
}
}  /* namespace */

SceneCaptureManager& manager() {
  static auto* const instance = new SceneCaptureManager(scene_handoff());
  return *instance;
}

/*
 * Registers the simulator's device once with the runtime and capture management; the manager retains it
 * until process exit.
 */
bool init_device(ID3D12Device* native) {
  const std::lock_guard lock(runtime().mutex);
  auto& item = runtime().device;
  if (!native || item.native || !manager().register_device(native))
    return false;
  item.native = native;
  return true;
}
/*
 * Hooks a same-device direct queue's submissions for ordered capture. A failure disables capture for good.
 */
bool init_queue(ID3D12CommandQueue* queue) {
  if (!queue || queue->GetDesc().Type != D3D12_COMMAND_LIST_TYPE_DIRECT)
    return true;
  const bool ready = queue_hooks::register_queue(queue);
  if (!ready) {
    manager().submission_refused(queue, queue_hooks::Refusal::invalid_batch);
    const std::lock_guard lock(runtime().mutex);
    if (auto* item = device()) {
      item->status.failed = true;
      item->status.message = "Native queue observation failed; scene capture disabled.";
    }
  }
  return ready;
}
/*
 * Initializes the private composition queue, compositor and WASM-format patch pipelines once.
 */
bool prepare() {
  const standalone::OwnedWork owned_work_guard;
  const std::lock_guard lock(runtime().mutex);
  auto* item = device();
  if (!item || item->status.failed)
    return false;
  if (item->status.initialized)
    return true;
  if (!item->output.initialize(item->native)) {
    item->status.failed = true;
    item->status.message = item->output.error();
    return false;
  }
  item->status.initialized = true;
  item->status.message = "Waiting for a completed image from the fixed camera.";
  return true;
}

/*
 * Recognizes the explicit WASM marker in a complete supported CPU-visible placeholder upload, validates
 * image/nonce/draw identity and registers consumer ordering. It substitutes the fresh GPU video region and
 * acknowledges a rounded-down remaining capture lifetime; unrelated uploads retain their original behavior.
 *
 * Shape narrows upload inspection, never authorizes pixel substitution. Include padded/partial uploads; the
 * marker and full-region checks below still have to establish ownership before any GPU command is changed.
 *
 * Once our actual marker is seen, unrelated same-sized images must not overwrite its status reason. This
 * string is polled, never logged per copy.
 *
 * Custom CPU-visible heaps are legal upload sources too. Map is never used to read the camera output or a
 * default GPU-only buffer.
 *
 * The engine may allocate a larger backing texture. The original call still proves a complete logical image
 * region; copy exactly that region, without scaling or touching padding. Formats must belong to the same
 * D3D12 family.
 *
 * Suppress our placeholder upload even while waiting, preserving the previous image. WASM independently
 * hides that image no later than its capture expiry.
 *
 * MSFS can upload RGBA/BGRA bytes through a typeless footprint. Our compositor needs a typed render target;
 * UNORM keeps the byte encoding, and D3D12 permits copying it into a typeless destination in the same format
 * family.
 *
 * Input-copy accounting uses one logical WASM video input, not a cockpit render target.
 *
 * One exact 52-bit LVAR value: nonce20 / frame24 / remaining8. Eight-ms units round down. WASM anchors this
 * remaining budget to its earlier draw request, so SimConnect transit time can never extend the two-second
 * capture lifetime.
 */
bool replace_wasm_upload(ID3D12GraphicsCommandList* list,
                         const D3D12_TEXTURE_COPY_LOCATION* dest,
                         UINT x,
                         UINT y,
                         UINT z,
                         const D3D12_TEXTURE_COPY_LOCATION* src,
                         const D3D12_BOX* box,
                         const char* recording_refusal) {
  if (!list)
    return false;
  const auto type = list->GetType();
  if (!dest || !src || !dest->pResource || !src->pResource || src->Type != D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT ||
      dest->Type != D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX)
    return false;
  const auto& placed = src->PlacedFootprint;
  const auto& footprint = placed.Footprint;
  if (footprint.Width != kCameraWidth && footprint.Height != kCameraHeight)
    return false;
  const auto source_desc = src->pResource->GetDesc();
  const auto target_desc = dest->pResource->GetDesc();
  const std::lock_guard lock(runtime().mutex);
  auto* item = device();
  if (!item)
    return false;
  D3D12_HEAP_PROPERTIES heap{};
  D3D12_HEAP_FLAGS heap_flags{};
  std::array<unsigned char, 128> header{}, again{};
  bool marked = false;
  /*
   * Refuses this upload. Once the exact marker was recognized, the reason becomes the visible input status;
   * refusals of unrelated same-sized images never overwrite it.
   */
  const auto report = [&](const char* reason) {
    if (marked)
      item->status.wasm_input_status = reason;
    return false;
  };
  if (source_desc.Dimension != D3D12_RESOURCE_DIMENSION_BUFFER || placed.Offset > source_desc.Width ||
      header.size() > source_desc.Width - placed.Offset)
    return report("upload_buffer_bounds");
  if (FAILED(src->pResource->GetHeapProperties(&heap, &heap_flags)))
    return report("upload_heap_unavailable");
  if (heap.Type != D3D12_HEAP_TYPE_UPLOAD && heap.CPUPageProperty != D3D12_CPU_PAGE_PROPERTY_WRITE_COMBINE &&
      heap.CPUPageProperty != D3D12_CPU_PAGE_PROPERTY_WRITE_BACK)
    return report("upload_heap_not_cpu_visible");
  void* mapped = nullptr;
  const D3D12_RANGE read_range{static_cast<SIZE_T>(placed.Offset), static_cast<SIZE_T>(placed.Offset + header.size())};
  if (FAILED(src->pResource->Map(0, &read_range, &mapped)) || !mapped)
    return report("upload_map_failed");
  std::memcpy(header.data(), static_cast<const unsigned char*>(mapped) + placed.Offset, header.size());
  std::memcpy(again.data(), static_cast<const unsigned char*>(mapped) + placed.Offset, again.size());
  const D3D12_RANGE no_writes{0, 0};
  src->pResource->Unmap(0, &no_writes);
  if (header != again)
    return report("upload_header_changed");
  std::array<unsigned char, 32> bytes{};
  for (unsigned i = 0; i < bytes.size(); ++i) {
    if (header[i * 4] != header[i * 4 + 1] || header[i * 4] != header[i * 4 + 2] || header[i * 4 + 3] != 255)
      return report("marker_pixel_encoding_missing");
    bytes[i] = header[i * 4];
  }
  if (std::memcmp(bytes.data(), "AZPVIDEOBRIDGE01!", 16))
    return report("marker_signature_missing");
  marked = true;
  const auto word = [&](unsigned offset) {
    return std::uint32_t(bytes[offset]) | (std::uint32_t(bytes[offset + 1]) << 8) | (std::uint32_t(bytes[offset + 2]) << 16) |
           (std::uint32_t(bytes[offset + 3]) << 24);
  };
  const auto nonce = word(16), frame = word(20);
  if (!nonce || nonce >= (1u << 20) || !frame || frame >= (1u << 24) || word(24) != kCameraWidth || word(28) != kCameraHeight)
    return report("marker_metadata_invalid");
  if (type != D3D12_COMMAND_LIST_TYPE_DIRECT)
    return report("upload_queue_not_direct");
  if (recording_refusal)
    return report(recording_refusal);
  const bool rgba = footprint.Format == DXGI_FORMAT_R8G8B8A8_TYPELESS || footprint.Format == DXGI_FORMAT_R8G8B8A8_UNORM ||
                    footprint.Format == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
  const bool bgra = footprint.Format == DXGI_FORMAT_B8G8R8A8_TYPELESS || footprint.Format == DXGI_FORMAT_B8G8R8A8_UNORM ||
                    footprint.Format == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB;
  if ((!rgba && !bgra) || footprint.Width != kCameraWidth || footprint.Height != kCameraHeight || footprint.Depth != 1 ||
      footprint.RowPitch < kCameraWidth * 4 || footprint.RowPitch % D3D12_TEXTURE_DATA_PITCH_ALIGNMENT ||
      placed.Offset % D3D12_TEXTURE_DATA_PLACEMENT_ALIGNMENT)
    return report("unsupported_upload_footprint");
  const auto upload_bytes = UINT64(footprint.RowPitch) * (kCameraHeight - 1) + kCameraWidth * 4;
  if (x || y || z || dest->SubresourceIndex || upload_bytes > source_desc.Width - placed.Offset ||
      (box && (box->left || box->top || box->front || box->right != kCameraWidth || box->bottom != kCameraHeight || box->back != 1)))
    return report("partial_upload_not_supported");
  const bool target_rgba = target_desc.Format == DXGI_FORMAT_R8G8B8A8_TYPELESS || target_desc.Format == DXGI_FORMAT_R8G8B8A8_UNORM ||
                           target_desc.Format == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
  const bool target_bgra = target_desc.Format == DXGI_FORMAT_B8G8R8A8_TYPELESS || target_desc.Format == DXGI_FORMAT_B8G8R8A8_UNORM ||
                           target_desc.Format == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB;
  if (target_desc.Dimension != D3D12_RESOURCE_DIMENSION_TEXTURE2D || target_desc.Width < kCameraWidth ||
      target_desc.Height < kCameraHeight || target_desc.MipLevels != 1 || target_desc.DepthOrArraySize != 1 ||
      target_desc.SampleDesc.Count != 1 || !((rgba && target_rgba) || (bgra && target_bgra)))
    return report("unsupported_target_layout");
  if (!same_native_device(src->pResource, item->native) || !same_native_device(dest->pResource, item->native))
    return report("upload_device_mismatch");
  ++item->status.wasm_uploads;
  item->status.wasm_input_status = "waiting_for_completed_camera_image";
  if (!displayable_output(*item) || item->status.failed)
    return true;
  auto video_format = footprint.Format;
  if (video_format == DXGI_FORMAT_R8G8B8A8_TYPELESS)
    video_format = DXGI_FORMAT_R8G8B8A8_UNORM;
  else if (video_format == DXGI_FORMAT_B8G8R8A8_TYPELESS)
    video_format = DXGI_FORMAT_B8G8R8A8_UNORM;
  const D3D12_RECT content{0, 0, kCameraWidth, kCameraHeight};
  if (!item->output.request_patch(video_format, kCameraWidth, kCameraHeight, content)) {
    item->status.wasm_input_status = "preparing_wasm_pixel_format";
    return true;
  }
  const auto patch = item->output.patch(video_format, kCameraWidth, kCameraHeight, content);
  if (!patch.buffer || patch.buffer == dest->pResource || !manager().register_consumer_recording(list)) {
    item->status.wasm_input_status = "waiting_for_ordered_video_buffer";
    return true;
  }
  D3D12_TEXTURE_COPY_LOCATION replacement{};
  replacement.pResource = patch.buffer;
  replacement.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
  replacement.PlacedFootprint = patch.footprint;
  list->CopyTextureRegion(dest, 0, 0, 0, &replacement, nullptr);
  manager().mark_input_copy(list);
  ++item->status.input_copies;
  item->status.wasm_input_status = "gpu_video_copy_recorded";
  const auto now = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
  const auto age = now >= item->output_capture_time_us ? now - item->output_capture_time_us : 2000000;
  const auto remaining = age < 2000000 ? (2000000 - age) / 8000 : 0;
  native_camera::set_video_input_ack(((std::uint64_t(nonce) << 24) | frame) * 256 + remaining);
  return true;
}

/*
 * Discards pending and composed output while retaining the device/capture timeline identity.
 */
void reset_feed() {
  const std::lock_guard lock(runtime().mutex);
  if (auto* item = device(); item && !item->status.failed) {
    discard(*item);
    item->status.output = false;
    item->output_capture_time_us = 0;
    item->committed = {};
    item->status.message = "Waiting for a completed camera image.";
  }
}
/*
 * Applies a TV/IR mode change and drops old-mode output: an image rendered for one mode is never displayed
 * as the other, even though the engine reuses the same output resource.
 */
void set_night_vision(bool enabled) {
  const std::lock_guard lock(runtime().mutex);
  if (auto* item = device(); item && item->night_vision != enabled) {
    discard(*item);
    item->output.discard_brightness();
    item->status.brightness = {};
    item->night_vision = enabled;
    item->status.output = false;
    item->output_capture_time_us = 0;
    item->committed = {};
    item->status.message = "Waiting for the selected TV/IR image.";
  }
}
void set_measurement_exposure(float ev) {
  const std::lock_guard lock(runtime().mutex);
  if (auto* item = device())
    item->measurement_exposure_ev = ev;
}
/*
 * Accepts only a finite EV, clamped to the window's range, for the next compositions.
 */
bool set_tv_exposure(float ev) {
  if (!std::isfinite(ev))
    return false;
  const std::lock_guard lock(runtime().mutex);
  auto* item = device();
  if (!item || item->status.failed)
    return false;
  item->status.tv_exposure_ev = std::clamp(ev, kMinimumExposureShiftEv, kMaximumExposureShiftEv);
  return true;
}

void set_wasm_input_active(bool active) {
  manager().set_input_active(active);
}
namespace {
/*
 * Composes the newest usable capture into the output once the previous composition has finished on the GPU,
 * then hands the capture lease over to that composition's fence.
 */
void compose(Device& item) {
  if (item.status.output && !displayable_output(item)) {
    item.status.output = false;
    item.status.message = "Camera image expired or source identity changed; waiting for fresh camera images.";
  }
  if (!item.native || !item.status.initialized || item.status.failed || !item.output.idle())
    return;
  auto& frame = item.pending;
  if (frame.token && !scene_handoff().is_current(frame.match)) {
    manager().discard_frame(frame.token);
    frame = {};
  }
  if (!frame.token)
    return;
  if (++item.compositions % 8 == 0)
    item.output.request_brightness(item.measurement_exposure_ev);
  if (!item.output.set_display(item.status.tv_exposure_ev, item.night_vision) ||
      !item.output.prepare(frame.resource, image_format(frame.resource))) {
    item.status.failed = true;
    item.status.message = item.output.error();
    return;
  }
  const auto submission = manager().begin_private_submission(item.output.queue());
  if (!submission.receipt) {
    if (item.output.discard_prepared())
      discard(item);
    item.status.failed = true;
    item.status.message = "Camera output queue ordering failed.";
    return;
  }
  const bool submitted = item.output.submit();
  const bool ordered = manager().end_private_submission(submission.receipt, submitted ? item.output.submissions() : 0, frame.capture_time_us);
  if (!submitted || !ordered) {
    item.status.failed = true;
    item.status.message = "Camera composition submission failed; snapshot leases retained.";
    return;
  }
  item.committed = frame.match;
  item.output_capture_time_us = frame.capture_time_us;
  manager().finish_consumption(frame.token, submission.fence, submission.value);
  frame = {};
  item.status.output = true;
  ++item.status.frames;
  if (const auto brightness = item.output.brightness(); brightness.sequence)
    item.status.brightness = brightness;
  item.status.message = item.night_vision ? "IR image available on AZP_CAMERA." : "TV image available on AZP_CAMERA.";
}
}  /* namespace */

/*
 * Collects completed current-generation captures of the current TV/IR mode and keeps the newest. When the
 * output slot is idle, it records and submits composition on the shared timeline, then retires capture
 * leases only after their consumer fence is established.
 *
 * Keep this high-water mark after composition/reset. Device timeline values never restart, and an older
 * recording may retire in a later service call.
 *
 * A failed recording may already reference these resources. Retain leases.
 */
void service() {
  const standalone::OwnedWork owned_work_guard;
  const std::lock_guard lock(runtime().mutex);
  std::array<SceneCaptureManager::Frame, SceneCaptureManager::MaximumPackets> incoming{};
  const auto count = manager().poll_completed_frames(incoming.data(), incoming.size());
  for (std::size_t i = 0; i < count; ++i) {
    auto& frame = incoming[i];
    auto* item = device();
    if (!item || !item->status.initialized || item->status.failed || frame.match.night_vision != item->night_vision ||
        !scene_handoff().is_current(frame.match) || !frame.order.newer_than(item->newest)) {
      manager().discard_frame(frame.token);
      continue;
    }
    item->newest = frame.order;
    if (item->pending.token)
      manager().discard_frame(item->pending.token);
    item->pending = frame;
  }
  compose(runtime().device);
}
/*
 * Builds a locked operational snapshot from current output freshness and completed input-copy counters.
 * Recording/submission alone never counts as a displayed gauge frame.
 */
Snapshot snapshot() {
  const std::lock_guard lock(runtime().mutex);
  Snapshot result;
  if (auto* item = device()) {
    result = item->status;
    const auto input = manager().input_statistics();
    result.input_frames = input.completed_frames;
    result.input_fps = input.frames_per_second;
    result.input_latency_ms = input.latency_ms;
    result.input_age_ms = input.age_ms;
    result.input_latency_valid = input.latency_valid;
    if (result.output && !displayable_output(*item)) {
      result.output = false;
      result.message = "Camera image expired or source identity changed; waiting for fresh camera images.";
    }
  }
  result.capture = manager().statistics();
  return result;
}
}  /* namespace azp::scene_runtime */
