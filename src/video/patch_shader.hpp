/*
 * Declares the pixel shader that turns the composed camera image into a WASM-format "patch". The composed
 * image lives in a packed RGBA8 GPU buffer; the WASM upload may be RGBA or BGRA, sRGB or not. One pipeline
 * per target format reads that buffer and draws it into a private typed texture, which scene_frame_output.cpp
 * then copies into the buffer used to replace the upload.
 */

#pragma once
#include <d3d12.h>

namespace azp {

class PatchShaderD3D12 {
 public:
  PatchShaderD3D12() = default;
  PatchShaderD3D12(const PatchShaderD3D12&) = delete;
  PatchShaderD3D12& operator=(const PatchShaderD3D12&) = delete;
  ~PatchShaderD3D12() { release(); }
  /*
   * Exact typed RTV format, single sample, no depth: RGBA/BGRA8 UNORM or sRGB. Initialize outside draw
   * callbacks; creates the root signature and PSO and compiles the embedded shader once.
   */
  HRESULT initialize(ID3D12Device*, DXGI_FORMAT target_format) noexcept;
  /*
   * PRIVATE command list only. The caller binds our owned patch RTV. The buffer at `address` contains
   * kCameraWidth x kCameraHeight RGBA8 pixels with kCameraRowPitch-byte rows (shared/configuration.hpp).
   * Pixels of `destination` outside `content` are painted opaque black.
   */
  bool record_private_patch(ID3D12GraphicsCommandList*,
                            ID3D12Device* buffer_device,
                            D3D12_GPU_VIRTUAL_ADDRESS address,
                            UINT width,
                            UINT height,
                            const D3D12_RECT* destination = nullptr,
                            const D3D12_RECT* content = nullptr) noexcept;
  void release() noexcept;

 private:
  ID3D12Device* device_ = nullptr;
  ID3D12RootSignature* root_ = nullptr;
  ID3D12PipelineState* pipeline_ = nullptr;
};
}  /* namespace azp */
