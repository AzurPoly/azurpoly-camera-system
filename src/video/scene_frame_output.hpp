/*
 * Defines composed output slots and their preparation/submission lifecycle. The runtime borrows a ready
 * output for WASM delivery; it must respect the producer/consumer fences and the capture timestamp.
 */

#pragma once
#include <d3d12.h>
#include <array>
#include <cstdint>
#include "../shared/configuration.hpp"

namespace azp {
class CameraCompositorD3D12;
class PatchShaderD3D12;

/*
 * Stable GPU buffers for the composed image and its typed "patches": copies of that image converted to the
 * exact RGBA/BGRA format of the WASM upload they replace. All application recordings copying a patch must
 * join the capture manager's per-device submission timeline. Buffers/descriptors remain allocated until
 * process exit because closed application recordings may replay them. The caller brackets submit with
 * begin/end_private_submission on that SAME timeline; this orders prior reads before writes and future reads
 * afterward. No engine resources are passed here: inputs are completed owned snapshots.
 */
class SceneFrameOutput {
 public:
  struct Patch {
    ID3D12Resource* buffer = nullptr;
    D3D12_PLACED_SUBRESOURCE_FOOTPRINT footprint{};
  };
  /*
   * Brightness of one composed image, 0 to 1: the mean of its red channel (the TV filter is grayscale) over
   * every 8th pixel of every 8th row, and the fractions of those samples that are nearly black or nearly
   * white. `exposure_ev` is the exposure the image was composed with, as given to request_brightness().
   * `sequence` counts measurements; 0 means none yet.
   */
  struct Brightness {
    float mean = 0, black = 0, white = 0, exposure_ev = 0;
    std::uint64_t sequence = 0;
  };
  static constexpr UINT Width = kCameraWidth;
  static constexpr UINT Height = kCameraHeight;
  static constexpr UINT RowPitch = kCameraRowPitch;
  static constexpr UINT64 BufferBytes = UINT64(RowPitch) * Height;
  SceneFrameOutput() = default;
  SceneFrameOutput(const SceneFrameOutput&) = delete;
  SceneFrameOutput& operator=(const SceneFrameOutput&) = delete;
  /*
   * Intentionally retain GPU objects after publication: application recordings may replay the stable buffer
   * until process exit, beyond our own fence.
   */
  ~SceneFrameOutput() = default;
  bool initialize(ID3D12Device* device) noexcept;
  /*
   * Serialized with prepare/submit. Changes only the next recording; a closed prepared recording must first
   * be submitted or discarded. TV exposure in EV; IR is displayed inverted, without exposure.
   */
  bool set_display(float tv_exposure_ev, bool infrared) noexcept;
  /*
   * Metadata-only demand, admitted against the camera's exact shape. Requests never allocate or record GPU
   * work. Every admitted slot remains refreshed on later compositions, because closed application recordings
   * can replay its stable buffer indefinitely.
   */
  bool request_patch(DXGI_FORMAT format, UINT width, UINT height, const D3D12_RECT& content) noexcept;
  /*
   * Stable process-retained buffer; consumer registration/timeline is required before every recorded copy.
   * No allocation or CPU image access here.
   */
  Patch patch(DXGI_FORMAT format, UINT width, UINT height, const D3D12_RECT& content) const noexcept;
  bool prepare(ID3D12Resource* image, DXGI_FORMAT format) noexcept;
  /*
   * Discard a closed prepared list which was NEVER submitted (e.g. the manager refused a private
   * transaction). Reset discards its command/descriptor uses before later prepare may replace compositor
   * input references.
   */
  bool discard_prepared() noexcept;
  bool submit() noexcept;
  /*
   * Asks the next prepare() to also copy its composed image into a CPU readback buffer. That copy is read
   * by the prepare() after its submission, which runs only once the private fence proves it complete.
   * `exposure_ev` (EV) travels with the measurement. discard_brightness() forgets a copy not read yet (e.g.
   * after a TV/IR change).
   */
  void request_brightness(float exposure_ev) noexcept {
    brightness_requested_ = true;
    requested_exposure_ev_ = exposure_ev;
  }
  void discard_brightness() noexcept {
    brightness_requested_ = brightness_recorded_ = brightness_pending_ = false;
    brightness_ = {};
  }
  Brightness brightness() const noexcept { return brightness_; }
  bool idle() const noexcept;
  ID3D12CommandQueue* queue() const noexcept { return queue_; }
  ID3D12Resource* buffer() const noexcept { return buffer_; }
  D3D12_GPU_VIRTUAL_ADDRESS address() const noexcept { return address_; }
  std::uint64_t submissions() const noexcept { return submitted_; }
  std::uint64_t completed_submissions() const noexcept;
  ID3D12Fence* completion_fence() const noexcept { return fence_; }
  const char* error() const noexcept { return error_; }

 private:
  bool fail(const char* error) noexcept;
  bool prepare_patches() noexcept;
  void read_brightness() noexcept;
  bool valid_patch_request(DXGI_FORMAT, UINT width, UINT height, const D3D12_RECT& content) const noexcept;
  struct PatchStorage {
    Patch view;
    ID3D12Resource* texture = nullptr;
    D3D12_CPU_DESCRIPTOR_HANDLE rtv{};
    D3D12_RECT content{};
    DXGI_FORMAT format = DXGI_FORMAT_UNKNOWN;
    UINT width = 0, height = 0;
    unsigned drawer = 0;
    bool written = false, recorded = false;
  };
  ID3D12Device* device_ = nullptr;
  ID3D12CommandQueue* queue_ = nullptr;
  ID3D12CommandAllocator* allocator_ = nullptr;
  ID3D12GraphicsCommandList* list_ = nullptr;
  ID3D12Fence* fence_ = nullptr;
  ID3D12Resource* buffer_ = nullptr;
  ID3D12Resource* meter_ = nullptr;  /* READBACK copy of one composed image, RowPitch layout. */
  Brightness brightness_;
  bool brightness_requested_ = false, brightness_recorded_ = false, brightness_pending_ = false;
  float requested_exposure_ev_ = 0, measured_exposure_ev_ = 0;
  CameraCompositorD3D12* compositor_ = nullptr;
  std::array<PatchShaderD3D12*, 4> patch_drawers_{};
  std::array<PatchStorage, 8> patches_{};
  ID3D12DescriptorHeap* patch_heap_ = nullptr;
  D3D12_GPU_VIRTUAL_ADDRESS address_ = 0;
  std::uint64_t submitted_ = 0;
  bool prepared_ = false;
  bool failed_ = false;
  const char* error_ = "";
};
}  /* namespace azp */
