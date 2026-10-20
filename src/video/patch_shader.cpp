/*
 * Compiles and records the WASM-format patch shader: a full-screen triangle whose pixel shader samples the
 * packed RGBA8 composed image and writes it to a private typed render target. Only scene_frame_output.cpp
 * records it, on its own command list; no simulator command list is touched here.
 */

#include "patch_shader.hpp"
#include <d3dcompiler.h>
#include "../shared/configuration.hpp"
#include "resource_guards.hpp"

namespace azp {
namespace {
template <class T>
void release_pointer(T*& p) noexcept {
  if (p)
    p->Release();
  p = nullptr;
}
bool format_supported(DXGI_FORMAT f) noexcept {
  return f == DXGI_FORMAT_R8G8B8A8_UNORM || f == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB || f == DXGI_FORMAT_B8G8R8A8_UNORM ||
         f == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB;
}
}  /* namespace */
/*
 * Builds the patch shader, root signature and pipeline for one supported target format. Partial failures
 * release created resources; successful initialization retains its device lifetime.
 */
HRESULT PatchShaderD3D12::initialize(ID3D12Device* device, DXGI_FORMAT format) noexcept {
  if (!device || device_ || !format_supported(format))
    return E_INVALIDARG;
  constexpr char shader[] = R"(
ByteAddressBuffer Pixels : register(t0);
cbuffer Parameters : register(b0) {
  uint TargetWidth; uint TargetHeight; uint OriginX; uint OriginY;
  uint InsetLeft; uint InsetTop; uint InsetRight; uint InsetBottom;
  uint SourceWidth; uint SourceHeight; uint SourceRowPitch;
};
/*
 * Generates a full-screen triangle from the vertex ID without a vertex buffer.
 */
float4 vs_main(uint id : SV_VertexID) : SV_Position {
  float2 uv = float2((id << 1) & 2, id & 2);
  return float4(uv.x * 2 - 1, 1 - uv.y * 2, 0, 1);
}
/*
 * Reads the addressed RGBA pixel from the packed GPU buffer, clamping the coordinate to its content
 * rectangle; pixels outside the content rectangle are opaque black.
 */
float4 ps_main(float4 position : SV_Position) : SV_Target {
  float2 local = position.xy - float2(OriginX, OriginY);
  uint2 contentEnd = uint2(TargetWidth - InsetRight, TargetHeight - InsetBottom);
  if (local.x < (float)InsetLeft || local.y < (float)InsetTop || local.x >= (float)contentEnd.x || local.y >= (float)contentEnd.y)
    return float4(0, 0, 0, 1);
  uint2 contentSize = contentEnd - uint2(InsetLeft, InsetTop);
  uint x = min((uint)((local.x - InsetLeft) * SourceWidth / contentSize.x), SourceWidth - 1);
  uint y = min((uint)((local.y - InsetTop) * SourceHeight / contentSize.y), SourceHeight - 1);
  uint rgba = Pixels.Load(y * SourceRowPitch + x * 4);
  return float4(rgba & 255, (rgba >> 8) & 255, (rgba >> 16) & 255, 255) / 255.0;
}
)";
  ID3DBlob *serialized = nullptr, *vs = nullptr, *ps = nullptr;
  D3D12_ROOT_PARAMETER params[2]{};
  params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
  params[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
  params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
  params[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
  params[1].Constants.Num32BitValues = 11;
  D3D12_ROOT_SIGNATURE_DESC desc{};
  desc.NumParameters = 2;
  desc.pParameters = params;
  HRESULT hr = D3D12SerializeRootSignature(&desc, D3D_ROOT_SIGNATURE_VERSION_1, &serialized, nullptr);
  if (SUCCEEDED(hr))
    hr = device->CreateRootSignature(0, serialized->GetBufferPointer(), serialized->GetBufferSize(), IID_PPV_ARGS(&root_));
  if (SUCCEEDED(hr))
    hr = D3DCompile(shader, sizeof(shader) - 1, "patch_shader", nullptr, nullptr, "vs_main", "vs_5_0",
                    D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, &vs, nullptr);
  if (SUCCEEDED(hr))
    hr = D3DCompile(shader, sizeof(shader) - 1, "patch_shader", nullptr, nullptr, "ps_main", "ps_5_0",
                    D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, &ps, nullptr);
  if (SUCCEEDED(hr)) {
    D3D12_GRAPHICS_PIPELINE_STATE_DESC p{};
    p.pRootSignature = root_;
    p.VS = {vs->GetBufferPointer(), vs->GetBufferSize()};
    p.PS = {ps->GetBufferPointer(), ps->GetBufferSize()};
    auto& blend = p.BlendState.RenderTarget[0];
    blend.SrcBlend = blend.SrcBlendAlpha = D3D12_BLEND_ONE;
    blend.DestBlend = blend.DestBlendAlpha = D3D12_BLEND_ZERO;
    blend.BlendOp = blend.BlendOpAlpha = D3D12_BLEND_OP_ADD;
    blend.LogicOp = D3D12_LOGIC_OP_NOOP;
    blend.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
    p.SampleMask = UINT_MAX;
    p.SampleDesc.Count = 1;
    p.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
    p.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
    p.RasterizerState.DepthClipEnable = TRUE;
    p.DepthStencilState.DepthEnable = FALSE;
    p.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ZERO;
    p.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_ALWAYS;
    p.DepthStencilState.StencilEnable = FALSE;
    p.DepthStencilState.FrontFace =
        p.DepthStencilState.BackFace = {D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP, D3D12_COMPARISON_FUNC_ALWAYS};
    p.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    p.NumRenderTargets = 1;
    p.RTVFormats[0] = format;
    p.DSVFormat = DXGI_FORMAT_UNKNOWN;
    hr = device->CreateGraphicsPipelineState(&p, IID_PPV_ARGS(&pipeline_));
  }
  release_pointer(serialized);
  release_pointer(vs);
  release_pointer(ps);
  if (FAILED(hr)) {
    release();
    return hr;
  }
  device_ = device;
  device_->AddRef();
  return S_OK;
}
/*
 * Validates target/content rectangles before recording the patch draw. Invalid input records nothing.
 */
bool PatchShaderD3D12::record_private_patch(ID3D12GraphicsCommandList* list,
                                            ID3D12Device* buffer_device,
                                            D3D12_GPU_VIRTUAL_ADDRESS address,
                                            UINT width,
                                            UINT height,
                                            const D3D12_RECT* destination,
                                            const D3D12_RECT* content) noexcept {
  if (!list || !pipeline_ || !address || address % 4 || buffer_device != device_ || width < 1 || height < 2 || width > 16384 ||
      height > 16384 || list->GetType() != D3D12_COMMAND_LIST_TYPE_DIRECT)
    return false;
  const D3D12_RECT rect = destination ? *destination : D3D12_RECT{0, 0, static_cast<LONG>(width), static_cast<LONG>(height)};
  if (rect.left < 0 || rect.top < 0 || rect.right <= rect.left || rect.bottom <= rect.top || rect.right > static_cast<LONG>(width) ||
      rect.bottom > static_cast<LONG>(height))
    return false;
  const D3D12_RECT inner = content ? *content : rect;
  if (inner.left < rect.left || inner.top < rect.top || inner.right > rect.right || inner.bottom > rect.bottom ||
      inner.right <= inner.left || inner.bottom <= inner.top)
    return false;
  const D3D12_VIEWPORT viewport{static_cast<float>(rect.left),
                                static_cast<float>(rect.top),
                                static_cast<float>(rect.right - rect.left),
                                static_cast<float>(rect.bottom - rect.top),
                                0,
                                1};
  const D3D12_RECT scissor = rect;
  const UINT constants[11]{static_cast<UINT>(rect.right - rect.left),
                           static_cast<UINT>(rect.bottom - rect.top),
                           static_cast<UINT>(rect.left),
                           static_cast<UINT>(rect.top),
                           static_cast<UINT>(inner.left - rect.left),
                           static_cast<UINT>(inner.top - rect.top),
                           static_cast<UINT>(rect.right - inner.right),
                           static_cast<UINT>(rect.bottom - inner.bottom),
                           kCameraWidth,
                           kCameraHeight,
                           kCameraRowPitch};
  list->SetPipelineState(pipeline_);
  list->SetGraphicsRootSignature(root_);
  list->SetGraphicsRootShaderResourceView(0, address);
  list->SetGraphicsRoot32BitConstants(1, 11, constants, 0);
  list->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
  list->RSSetViewports(1, &viewport);
  list->RSSetScissorRects(1, &scissor);
  list->DrawInstanced(3, 1, 0, 0);
  return true;
}
void PatchShaderD3D12::release() noexcept {
  release_pointer(pipeline_);
  release_pointer(root_);
  release_pointer(device_);
}
}  /* namespace azp */
