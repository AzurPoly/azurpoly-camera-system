/*
 * Implements the private GPU compositor and its embedded shaders. The camera's final native image is already
 * display-ready: TV applies its exposure setting as a gain, IR is inverted. Both pass through the grayscale
 * TV lens/contrast filter, which is always on, before delivery to WASM.
 */

#include "compositor.hpp"

namespace azp {
bool CameraCompositorD3D12::set_display(float tv_exposure_ev, bool infrared) noexcept {
  if (!std::isfinite(tv_exposure_ev))
    return false;
  exposure_ev_ = std::clamp(tv_exposure_ev, kMinimumExposureShiftEv, kMaximumExposureShiftEv);
  infrared_ = infrared;
  return true;
}

/*
 * Creates the private render target, descriptor heaps and shader pipeline for one device and output size.
 * Partial initialization releases owned resources and reports the failing stage.
 */
HRESULT CameraCompositorD3D12::initialize(ID3D12Device* device) noexcept {
  if (!device)
    return fail(E_INVALIDARG, "A native D3D12 device is required.");
  if (device_.get())
    return same_object(device_.get(), device) ? S_FALSE : fail(E_INVALIDARG, "The compositor already belongs to another device.");
  device_.retain(device);
  HRESULT status = initialize_pipeline();
  if (FAILED(status)) {
    release();
    return status;
  }

  D3D12_DESCRIPTOR_HEAP_DESC heap{};
  heap.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
  heap.NumDescriptors = 1;
  heap.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
  status = device->CreateDescriptorHeap(&heap, IID_PPV_ARGS(srv_heap_.put()));
  if (FAILED(status))
    return initialization_failed(status, "Creating the input SRV heap failed.");
  heap.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
  heap.NumDescriptors = 1;
  heap.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
  status = device->CreateDescriptorHeap(&heap, IID_PPV_ARGS(rtv_heap_.put()));
  if (FAILED(status))
    return initialization_failed(status, "Creating the output RTV heap failed.");

  D3D12_HEAP_PROPERTIES properties{};
  properties.Type = D3D12_HEAP_TYPE_DEFAULT;
  properties.CreationNodeMask = properties.VisibleNodeMask = 1;
  D3D12_RESOURCE_DESC texture{};
  texture.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
  texture.Width = Width;
  texture.Height = Height;
  texture.DepthOrArraySize = texture.MipLevels = 1;
  texture.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
  texture.SampleDesc.Count = 1;
  texture.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
  texture.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;
  status = device->CreateCommittedResource(&properties, D3D12_HEAP_FLAG_NONE, &texture, D3D12_RESOURCE_STATE_COPY_SOURCE, nullptr,
                                           IID_PPV_ARGS(output_.put()));
  if (FAILED(status))
    return initialization_failed(status, "Creating the compositor output texture failed.");
  rtv_ = rtv_heap_->GetCPUDescriptorHandleForHeapStart();
  device->CreateRenderTargetView(output_.get(), nullptr, rtv_);
  error_[0] = '\0';
  return S_OK;
}

/*
 * Validates the input's identity, format and device before creating its shader view.
 */
HRESULT CameraCompositorD3D12::set_input(ID3D12Resource* image, DXGI_FORMAT format) noexcept {
  if (!output_.get())
    return fail(E_UNEXPECTED, "Initialize the compositor before binding its input.");
  if (!image || same_object(image, output_.get()))
    return fail(E_INVALIDARG, "The camera input is required and must not alias the output.");
  if (same_object(image, input_.get()) && format == format_) {
    error_[0] = '\0';
    return S_FALSE;
  }
  D3D12_RESOURCE_DESC description{};
  if (!validate_input(image, format, description))
    return fail(E_INVALIDARG, "The input has an unsupported device, texture description, heap or typed SRV format.");
  D3D12_SHADER_RESOURCE_VIEW_DESC view{};
  view.Format = format;
  view.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
  view.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
  view.Texture2D.MipLevels = 1;
  device_->CreateShaderResourceView(image, &view, srv_heap_->GetCPUDescriptorHandleForHeapStart());
  input_.retain(image);
  format_ = format;
  input_description_ = description;
  error_[0] = '\0';
  return S_OK;
}

/*
 * Records resource transitions, shader constants and one full-image draw on the supplied private list.
 * Input/output states are explicit, and the constant layout must agree with the embedded shader.
 *
 * The constant block must match the shader's four 32-bit fields exactly.
 */
HRESULT CameraCompositorD3D12::record(ID3D12GraphicsCommandList* private_list, D3D12_RESOURCE_STATES image_before) noexcept {
  if (!output_.get() || !input_.get() || !private_list || private_list->GetType() != D3D12_COMMAND_LIST_TYPE_DIRECT ||
      !same_device(private_list) || !valid_state(image_before, input_description_))
    return fail(E_INVALIDARG, "A private direct list, a bound input and a supported exact mip-zero state are required.");
  transition(private_list, input_.get(), image_before, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
  transition(private_list, output_.get(), D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET);

  private_list->SetGraphicsRootSignature(root_signature_.get());
  private_list->SetPipelineState(pipeline_.get());
  ID3D12DescriptorHeap* heaps[]{srv_heap_.get()};
  private_list->SetDescriptorHeaps(1, heaps);
  private_list->SetGraphicsRootDescriptorTable(0, srv_heap_->GetGPUDescriptorHandleForHeapStart());
  const struct {
    UINT infrared;
    float exposure;
    float width, height;
  } display{infrared_ ? 1u : 0u, std::exp2(exposure_ev_ + kTvNeutralExposureEv), static_cast<float>(Width), static_cast<float>(Height)};
  static_assert(sizeof(display) == 4 * sizeof(UINT));
  private_list->SetGraphicsRoot32BitConstants(1, 4, &display, 0);
  private_list->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
  const D3D12_VIEWPORT viewport{0, 0, static_cast<float>(Width), static_cast<float>(Height), 0, 1};
  const D3D12_RECT scissor{0, 0, static_cast<LONG>(Width), static_cast<LONG>(Height)};
  private_list->RSSetViewports(1, &viewport);
  private_list->RSSetScissorRects(1, &scissor);
  private_list->OMSetRenderTargets(1, &rtv_, FALSE, nullptr);
  private_list->DrawInstanced(3, 1, 0, 0);

  transition(private_list, output_.get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_SOURCE);
  transition(private_list, input_.get(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, image_before);
  error_[0] = '\0';
  return S_OK;
}

/*
 * Releases owned GPU references and resets initialization state once the caller has satisfied their lifetime
 * obligations.
 */
void CameraCompositorD3D12::release() noexcept {
  input_.reset();
  output_.reset();
  srv_heap_.reset();
  rtv_heap_.reset();
  pipeline_.reset();
  root_signature_.reset();
  device_.reset();
  format_ = DXGI_FORMAT_UNKNOWN;
  input_description_ = {};
  rtv_ = {};
}

bool CameraCompositorD3D12::same_object(IUnknown* first, IUnknown* second) noexcept {
  if (!first || !second)
    return false;
  Reference<IUnknown> first_identity;
  Reference<IUnknown> second_identity;
  return SUCCEEDED(first->QueryInterface(IID_PPV_ARGS(first_identity.put()))) &&
         SUCCEEDED(second->QueryInterface(IID_PPV_ARGS(second_identity.put()))) && first_identity.get() == second_identity.get();
}

/*
 * Maps supported texture formats to their compatible family for resource/view checks. Formats with no
 * typeless alias, such as the packed R11G11B10_FLOAT, require an exact typed match.
 */
DXGI_FORMAT CameraCompositorD3D12::typeless_family(DXGI_FORMAT format) noexcept {
  switch (format) {
    case DXGI_FORMAT_R11G11B10_FLOAT:
      return DXGI_FORMAT_R11G11B10_FLOAT;
    case DXGI_FORMAT_R8G8B8A8_UNORM:
    case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
      return DXGI_FORMAT_R8G8B8A8_TYPELESS;
    case DXGI_FORMAT_B8G8R8A8_UNORM:
    case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
      return DXGI_FORMAT_B8G8R8A8_TYPELESS;
    case DXGI_FORMAT_R16G16B16A16_FLOAT:
      return DXGI_FORMAT_R16G16B16A16_TYPELESS;
    case DXGI_FORMAT_R8_UNORM:
    case DXGI_FORMAT_R8G8_UNORM:
    case DXGI_FORMAT_R16_UNORM:
    case DXGI_FORMAT_R16_FLOAT:
    case DXGI_FORMAT_R16G16_UNORM:
    case DXGI_FORMAT_R16G16_FLOAT:
    case DXGI_FORMAT_R32_FLOAT:
    case DXGI_FORMAT_R32G32_FLOAT:
    case DXGI_FORMAT_R32G32B32A32_FLOAT:
      return format;
    default:
      return DXGI_FORMAT_UNKNOWN;
  }
}

/*
 * Requires a compatible same-device 2D resource, supported sampling format and suitable heap/shape before
 * binding it as a compositor input.
 *
 * MinGW's COM headers expose the Microsoft aggregate-return ABI explicitly.
 */
bool CameraCompositorD3D12::validate_input(ID3D12Resource* resource,
                                           DXGI_FORMAT view_format,
                                           D3D12_RESOURCE_DESC& description) const noexcept {
  if (!same_device(resource) || typeless_family(view_format) == DXGI_FORMAT_UNKNOWN)
    return false;
#if defined(__MINGW32__)
#ifndef WIDL_EXPLICIT_AGGREGATE_RETURNS
#error This MinGW build requires explicit Microsoft COM aggregate-return wrappers.
#endif
  resource->GetDesc(&description);
#else
  description = resource->GetDesc();
#endif
  if (description.Dimension != D3D12_RESOURCE_DIMENSION_TEXTURE2D || description.Width == 0 || description.Width > 16384 ||
      description.Height == 0 || description.Height > 16384 || description.DepthOrArraySize != 1 || description.MipLevels == 0 ||
      description.SampleDesc.Count != 1 || description.SampleDesc.Quality != 0 ||
      (description.Flags & (D3D12_RESOURCE_FLAG_DENY_SHADER_RESOURCE | D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL)) != 0 ||
      (description.Format != view_format && description.Format != typeless_family(view_format)))
    return false;
  D3D12_HEAP_PROPERTIES heap{};
  D3D12_HEAP_FLAGS flags{};
  if (FAILED(resource->GetHeapProperties(&heap, &flags)) || heap.Type != D3D12_HEAP_TYPE_DEFAULT)
    return false;
  D3D12_FEATURE_DATA_FORMAT_SUPPORT support{view_format, D3D12_FORMAT_SUPPORT1_NONE, D3D12_FORMAT_SUPPORT2_NONE};
  const auto required = D3D12_FORMAT_SUPPORT1_TEXTURE2D | D3D12_FORMAT_SUPPORT1_SHADER_SAMPLE;
  return SUCCEEDED(device_->CheckFeatureSupport(D3D12_FEATURE_FORMAT_SUPPORT, &support, sizeof(support))) &&
         (support.Support1 & required) == required;
}

/*
 * Checks that the known input layout can be safely transitioned for sampling; unknown or incompatible state
 * refuses the draw.
 */
bool CameraCompositorD3D12::valid_state(D3D12_RESOURCE_STATES state, const D3D12_RESOURCE_DESC& input) noexcept {
  if (state == D3D12_RESOURCE_STATE_COMMON || state == D3D12_RESOURCE_STATE_COPY_DEST)
    return true;
  if (state == D3D12_RESOURCE_STATE_RENDER_TARGET)
    return (input.Flags & D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET) != 0;
  if (state == D3D12_RESOURCE_STATE_UNORDERED_ACCESS)
    return (input.Flags & D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS) != 0;
  constexpr UINT ReadStates =
      D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_COPY_SOURCE;
  return (static_cast<UINT>(state) & ~ReadStates) == 0;
}

/*
 * Records a whole-resource transition only when the known before and required after states differ.
 */
void CameraCompositorD3D12::transition(ID3D12GraphicsCommandList* list,
                                       ID3D12Resource* texture,
                                       D3D12_RESOURCE_STATES before,
                                       D3D12_RESOURCE_STATES after) noexcept {
  if (before == after)
    return;
  D3D12_RESOURCE_BARRIER barrier{};
  barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
  barrier.Transition.pResource = texture;
  barrier.Transition.Subresource = 0;
  barrier.Transition.StateBefore = before;
  barrier.Transition.StateAfter = after;
  list->ResourceBarrier(1, &barrier);
}

HRESULT CameraCompositorD3D12::fail(HRESULT status, const char* message) noexcept {
  const auto size = (std::min)(std::strlen(message), error_.size() - 1);
  std::memcpy(error_.data(), message, size);
  error_[size] = '\0';
  return status;
}

HRESULT CameraCompositorD3D12::initialization_failed(HRESULT status, const char* message) noexcept {
  fail(status, message);
  release();
  return status;
}

/*
 * Compiles an embedded shader entry point and preserves a bounded compiler error on failure. No pipeline is
 * created from missing shader bytecode.
 */
HRESULT CameraCompositorD3D12::compile(const char* entry, const char* profile, ID3DBlob** bytecode) noexcept {
  Reference<ID3DBlob> diagnostics;
  const HRESULT status = D3DCompile(Shader, std::strlen(Shader), "camera_compositor", nullptr, nullptr, entry, profile,
                                    D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_OPTIMIZATION_LEVEL3 | D3DCOMPILE_WARNINGS_ARE_ERRORS, 0,
                                    bytecode, diagnostics.put());
  if (FAILED(status)) {
    if (diagnostics.get()) {
      const auto size = (std::min)(diagnostics->GetBufferSize(), error_.size() - 1);
      std::memcpy(error_.data(), diagnostics->GetBufferPointer(), size);
      error_[size] = '\0';
    } else {
      fail(status, "Compiling the compositor shader failed.");
    }
  }
  return status;
}

/*
 * Builds the root signature, compiles the vertex/pixel shaders and creates the graphics pipeline whose
 * constants and texture slots match the compositor draw.
 */
HRESULT CameraCompositorD3D12::initialize_pipeline() noexcept {
  D3D12_DESCRIPTOR_RANGE range{};
  range.RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
  range.NumDescriptors = 1;
  range.OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;
  std::array<D3D12_ROOT_PARAMETER, 2> parameters{};
  parameters[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
  parameters[0].DescriptorTable.NumDescriptorRanges = 1;
  parameters[0].DescriptorTable.pDescriptorRanges = &range;
  parameters[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
  parameters[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
  parameters[1].Constants.Num32BitValues = 4;
  parameters[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
  D3D12_STATIC_SAMPLER_DESC sampler{};
  sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
  sampler.AddressU = sampler.AddressV = sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
  sampler.MaxAnisotropy = 1;
  sampler.ComparisonFunc = D3D12_COMPARISON_FUNC_ALWAYS;
  sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
  D3D12_ROOT_SIGNATURE_DESC signature{};
  signature.NumParameters = static_cast<UINT>(parameters.size());
  signature.pParameters = parameters.data();
  signature.NumStaticSamplers = 1;
  signature.pStaticSamplers = &sampler;
  Reference<ID3DBlob> serialized;
  Reference<ID3DBlob> diagnostics;
  HRESULT status = D3D12SerializeRootSignature(&signature, D3D_ROOT_SIGNATURE_VERSION_1, serialized.put(), diagnostics.put());
  if (FAILED(status))
    return fail(status, "Serializing the compositor root signature failed.");
  status =
      device_->CreateRootSignature(0, serialized->GetBufferPointer(), serialized->GetBufferSize(), IID_PPV_ARGS(root_signature_.put()));
  if (FAILED(status))
    return fail(status, "Creating the compositor root signature failed.");
  Reference<ID3DBlob> vertex;
  Reference<ID3DBlob> pixel;
  status = compile("vs_main", "vs_5_0", vertex.put());
  if (FAILED(status))
    return status;
  status = compile("ps_main", "ps_5_0", pixel.put());
  if (FAILED(status))
    return status;
  D3D12_GRAPHICS_PIPELINE_STATE_DESC pipeline{};
  pipeline.pRootSignature = root_signature_.get();
  pipeline.VS = {vertex->GetBufferPointer(), vertex->GetBufferSize()};
  pipeline.PS = {pixel->GetBufferPointer(), pixel->GetBufferSize()};
  auto& blend = pipeline.BlendState.RenderTarget[0];
  blend.SrcBlend = blend.SrcBlendAlpha = D3D12_BLEND_ONE;
  blend.DestBlend = blend.DestBlendAlpha = D3D12_BLEND_ZERO;
  blend.BlendOp = blend.BlendOpAlpha = D3D12_BLEND_OP_ADD;
  blend.LogicOp = D3D12_LOGIC_OP_NOOP;
  blend.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
  pipeline.SampleMask = UINT_MAX;
  pipeline.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
  pipeline.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
  pipeline.RasterizerState.DepthClipEnable = TRUE;
  pipeline.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ZERO;
  pipeline.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_ALWAYS;
  pipeline.DepthStencilState.StencilReadMask = pipeline.DepthStencilState.StencilWriteMask = 0xff;
  pipeline.DepthStencilState.FrontFace = {D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP,
                                          D3D12_COMPARISON_FUNC_ALWAYS};
  pipeline.DepthStencilState.BackFace = pipeline.DepthStencilState.FrontFace;
  pipeline.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
  pipeline.NumRenderTargets = 1;
  pipeline.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
  pipeline.SampleDesc.Count = 1;
  status = device_->CreateGraphicsPipelineState(&pipeline, IID_PPV_ARGS(pipeline_.put()));
  return FAILED(status) ? fail(status, "Creating the compositor pipeline failed.") : S_OK;
}

}  /* namespace azp */

namespace azp {
const char CameraCompositorD3D12::Shader[] = R"(
Texture2D<float4> Image : register(t0);
SamplerState LinearClamp : register(s0);
cbuffer Display : register(b0) {
  uint Infrared; float Exposure;
  float2 DisplaySize;
};
/*
 * Applies the TV exposure setting to one channel of the final image, whose pixels are sRGB-encoded: decode,
 * scale, re-encode, clipping above white.
 */
float shifted_channel(float value) {
  float light = value <= 0.04045 ? value / 12.92 : pow((max(value, 0) + 0.055) / 1.055, 2.4);
  float exposed = saturate(light * Exposure);
  return exposed <= 0.0031308 ? 12.92 * exposed : 1.055 * pow(exposed, 1.0 / 2.4) - 0.055;
}
/*
 * Generates a full-screen triangle from the vertex ID without a vertex buffer.
 */
float4 vs_main(uint id : SV_VertexID) : SV_Position {
  float2 uv = float2((id << 1) & 2, id & 2);
  return float4(uv.x * 2 - 1, 1 - uv.y * 2, 0, 1);
}
/*
 * Samples the camera's final image. TV takes its exposure setting; IR passes unchanged and is inverted
 * after the TV filter.
 */
float3 camera_pixel(float2 uv) {
  float3 rgb = Image.SampleLevel(LinearClamp, uv, 0).rgb;
  float3 result = rgb;
  if (Infrared == 0)
    result = float3(shifted_channel(rgb.r), shifted_channel(rgb.g), shifted_channel(rgb.b));
  return result;
}
/*
 * Blurs the camera image horizontally with a three-tap filter measured in output pixels.
 */
float3 tv_blurred_pixel(float2 uv) {
  float2 step = float2(1.0 / DisplaySize.x, 0.0);
  return camera_pixel(uv) * 0.5 + (camera_pixel(uv - step) + camera_pixel(uv + step)) * 0.25;
}
/*
 * Applies the TV appearance to display-ready pixels: radial lens mapping, chromatic separation, blur, a
 * contrast curve, grayscale and highlight clipping. The same processing applies to TV and IR.
 *
 * The radial coefficient is 0.075. Normalization by the corner radius and slight overscan keep all
 * channel/blur samples inside the source, avoiding a curved black border. Red/blue offsets are symmetric
 * around green and span the whole image, with stronger separation away from the centre; their distance is
 * measured in output pixels.
 *
 * After the display-space S-curve, luminance is computed with Rec.709 weights and contrast is multiplied by
 * 1.5 around middle grey. Clamping deliberately clips extreme tones; the final highlight response whitens
 * bright areas without lifting shadows.
 */
float3 tv_pixel(float2 uv) {
  float2 centered = uv * 2.0 - 1.0;
  float radiusSquared = dot(centered, centered);
  float2 screen = centered * (1.0 + 0.075 * radiusSquared) * (0.98 / 1.15);
  float2 warped = screen * 0.5 + 0.5;
  float2 chroma = float2(1.0 + 1.2 * radiusSquared, 0.35 * centered.y) / DisplaySize;
  float3 color = float3(tv_blurred_pixel(warped + chroma).r,
                        tv_blurred_pixel(warped).g,
                        tv_blurred_pixel(warped - chroma).b);
  color = saturate(color);
  color = lerp(color, color * color * (3.0 - 2.0 * color), 0.85);
  float luma = dot(color, float3(0.2126, 0.7152, 0.0722));
  const float contrast = 1.5;
  luma = saturate(0.5 + (luma - 0.5) * contrast);
  color = luma.xxx;
  color = saturate(color * (1.0 + 0.15 * smoothstep(0.6, 0.9, luma)));
  return color;
}
/*
 * Applies the TV filter, then inverts IR. The output is opaque video; WASM adds its menu later.
 */
float4 ps_main(float4 position : SV_Position) : SV_Target {
  float3 color = tv_pixel(position.xy / DisplaySize);
  if (Infrared != 0) color = 1.0 - saturate(color);
  return float4(color, 1);
}
)";
}
