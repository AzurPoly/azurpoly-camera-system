/*
 * Defines compositor input, output resources and recording operations. Resource states and producer
 * completion remain explicit: composing an image records GPU work, while the caller owns submission and
 * fence-based lifetime management.
 */

#pragma once
#include <d3d12.h>
#include <d3dcompiler.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include "../shared/configuration.hpp"
#include "resource_guards.hpp"

namespace azp {

/*
 * GPU composition only: this class does not obtain or render simulator cameras. Caller contract: - Use an
 * OPEN, caller-owned PRIVATE DIRECT list outside any render pass. This class overwrites graphics bindings;
 * it must never receive an MSFS list. - Serialize all methods. Producer work must finish before the input
 * read, using queue ordering/fences as appropriate. The input before state describes mip zero exactly;
 * pending split barriers and heap aliasing are unsupported. - Submit recorded lists and output copies in
 * order. Keep all objects alive until GPU completion. AddRef does not synchronize access to texture
 * contents. - Changing the input, release, and destruction require completion of EVERY list using the old
 * descriptor/resource. Unsubmitted lists must be discarded. - output() is borrowed, remains COPY_SOURCE after
 * record, and may only be read until the next serialized record. The caller owns output-copy
 * synchronization. initialize compiles/allocates once. set_input writes the descriptor only when the
 * resource/format changes. record allocates, compiles, uploads and reads back nothing; it restores the
 * input's mip-zero state and leaves the output COPY_SOURCE. The input is the camera's display-ready final
 * image, sampled as declared by the typed SRV and stretched over the whole output.
 */
class CameraCompositorD3D12 {
 public:
  static constexpr UINT Width = kCameraWidth;
  static constexpr UINT Height = kCameraHeight;

  CameraCompositorD3D12() = default;
  CameraCompositorD3D12(const CameraCompositorD3D12&) = delete;
  CameraCompositorD3D12& operator=(const CameraCompositorD3D12&) = delete;
  ~CameraCompositorD3D12() { release(); }

  ID3D12Resource* output() const noexcept { return output_.get(); }
  const char* last_error() const noexcept { return error_.data(); }
  /*
   * Recorded root constants capture these values; no resource or descriptor changes. TV applies the TV
   * exposure setting (EV, clamped to its window range) as a gain; IR takes no exposure and is displayed
   * inverted.
   */
  bool set_display(float tv_exposure_ev, bool infrared) noexcept;
  HRESULT initialize(ID3D12Device* device) noexcept;

  /*
   * Only single-layer, non-MSAA default-heap 2D textures are supported. Mip zero is sampled; other mips are
   * not transitioned. Typed RGBA/BGRA UNORM, sRGB or RGBA16_FLOAT views must match a typed resource or its
   * typeless family. R11G11B10_FLOAT requires an exact typed resource and view.
   */
  HRESULT set_input(ID3D12Resource* image, DXGI_FORMAT format) noexcept;

  /*
   * Supported exact before states: COMMON, RENDER_TARGET, UNORDERED_ACCESS, COPY_DEST, or any nonempty
   * combination of PIXEL_SHADER_RESOURCE, NON_PIXEL_SHADER_RESOURCE and COPY_SOURCE. Other state sets are
   * refused.
   */
  HRESULT record(ID3D12GraphicsCommandList* private_list, D3D12_RESOURCE_STATES image_before) noexcept;

  /*
   * Call only after checked GPU completion, including downstream output copies.
   */
  void release() noexcept;

 private:
  template <typename T>
  class Reference {
   public:
    Reference() = default;
    Reference(const Reference&) = delete;
    Reference& operator=(const Reference&) = delete;
    ~Reference() { reset(); }
    T* get() const noexcept { return value_; }
    T* operator->() const noexcept { return value_; }
    T** put() noexcept { return &value_; }
    void reset() noexcept {
      if (value_)
        value_->Release();
      value_ = nullptr;
    }
    void retain(T* value) noexcept {
      if (value)
        value->AddRef();
      reset();
      value_ = value;
    }

   private:
    T* value_ = nullptr;
  };

  static bool same_object(IUnknown* first, IUnknown* second) noexcept;

  bool same_device(ID3D12DeviceChild* child) const noexcept { return same_native_device(child, device_.get()); }

  static DXGI_FORMAT typeless_family(DXGI_FORMAT format) noexcept;

  bool validate_input(ID3D12Resource* resource, DXGI_FORMAT view_format, D3D12_RESOURCE_DESC& description) const noexcept;

  static bool valid_state(D3D12_RESOURCE_STATES state, const D3D12_RESOURCE_DESC& input) noexcept;

  static void transition(ID3D12GraphicsCommandList* list,
                         ID3D12Resource* texture,
                         D3D12_RESOURCE_STATES before,
                         D3D12_RESOURCE_STATES after) noexcept;

  HRESULT fail(HRESULT status, const char* message) noexcept;

  HRESULT initialization_failed(HRESULT status, const char* message) noexcept;

  HRESULT compile(const char* entry, const char* profile, ID3DBlob** bytecode) noexcept;

  HRESULT initialize_pipeline() noexcept;

  static const char Shader[];

  Reference<ID3D12Device> device_;
  Reference<ID3D12RootSignature> root_signature_;
  Reference<ID3D12PipelineState> pipeline_;
  Reference<ID3D12DescriptorHeap> srv_heap_;
  Reference<ID3D12DescriptorHeap> rtv_heap_;
  Reference<ID3D12Resource> output_;
  Reference<ID3D12Resource> input_;
  DXGI_FORMAT format_ = DXGI_FORMAT_UNKNOWN;
  D3D12_RESOURCE_DESC input_description_{};
  bool infrared_ = false;
  float exposure_ev_ = 0;
  D3D12_CPU_DESCRIPTOR_HANDLE rtv_{};
  std::array<char, 1024> error_{};
};

}  /* namespace azp */
