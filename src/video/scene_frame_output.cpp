/*
 * Builds the composed output image from the captured camera image on a private command list, then draws
 * each requested WASM-format patch from it (patch_shader.cpp). Each submission is associated with a fence
 * before its buffers can be reused.
 */

#include "scene_frame_output.hpp"
#include <limits>
#include <new>
#include "compositor.hpp"
#include "patch_shader.hpp"

namespace azp {
namespace {
constexpr std::array<DXGI_FORMAT, 4> PatchFormats{DXGI_FORMAT_R8G8B8A8_UNORM, DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, DXGI_FORMAT_B8G8R8A8_UNORM,
                                                  DXGI_FORMAT_B8G8R8A8_UNORM_SRGB};
bool same_rect(const D3D12_RECT& a, const D3D12_RECT& b) noexcept {
  return a.left == b.left && a.top == b.top && a.right == b.right && a.bottom == b.bottom;
}
}  /* namespace */
/*
 * A patch covers the whole camera image in one of the WASM image formats.
 */
bool SceneFrameOutput::valid_patch_request(DXGI_FORMAT format, UINT width, UINT height, const D3D12_RECT& content) const noexcept {
  const D3D12_RECT whole{0, 0, static_cast<LONG>(kCameraWidth), static_cast<LONG>(kCameraHeight)};
  return !failed_ && wasm_image_format(static_cast<unsigned>(format)) && width == kCameraWidth && height == kCameraHeight &&
         same_rect(content, whole);
}
/*
 * Deduplicates valid typed patch requests within fixed capacity. A request records demand only; no native
 * commands are inserted until a matching output is prepared.
 */
bool SceneFrameOutput::request_patch(DXGI_FORMAT format, UINT width, UINT height, const D3D12_RECT& content) noexcept {
  if (!valid_patch_request(format, width, height, content))
    return false;
  unsigned drawer = 0;
  while (drawer < PatchFormats.size() && PatchFormats[drawer] != format)
    ++drawer;
  if (drawer == PatchFormats.size())
    return false;
  for (const auto& p : patches_)
    if (p.width == width && p.height == height && p.format == format && same_rect(p.content, content))
      return true;
  for (auto& p : patches_)
    if (!p.width) {
      p.format = format;
      p.width = width;
      p.height = height;
      p.content = content;
      p.drawer = drawer;
      return true;
    }
  return false;
}
SceneFrameOutput::Patch SceneFrameOutput::patch(DXGI_FORMAT format, UINT width, UINT height, const D3D12_RECT& content) const noexcept {
  if (valid_patch_request(format, width, height, content) && submitted_)
    for (const auto& p : patches_)
      if (p.written && p.view.footprint.Footprint.Format == format && p.view.footprint.Footprint.Width == width &&
          p.view.footprint.Footprint.Height == height && same_rect(p.content, content))
        return p.view;
  return {};
}
bool SceneFrameOutput::set_display(float tv_exposure_ev, bool infrared) noexcept {
  return compositor_ && !prepared_ && !failed_ && compositor_->set_display(tv_exposure_ev, infrared);
}
bool SceneFrameOutput::fail(const char* error) noexcept {
  error_ = error;
  failed_ = true;
  return false;
}

/*
 * Creates the private queue, list, fence, composed buffer and patch descriptors for one device. The object
 * retains these resources across successive completed frames.
 */
bool SceneFrameOutput::initialize(ID3D12Device* device) noexcept {
  if (failed_)
    return false;
  if (device_)
    return device_ == device && !failed_;
  if (!device)
    return fail("A live native D3D12 device is required for the camera output.");
  device_ = device;
  device_->AddRef();
  D3D12_COMMAND_QUEUE_DESC queue_desc{};
  queue_desc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
  if (FAILED(device_->CreateCommandQueue(&queue_desc, IID_PPV_ARGS(&queue_))) ||
      FAILED(device_->CreateCommandAllocator(queue_desc.Type, IID_PPV_ARGS(&allocator_))) ||
      FAILED(device_->CreateCommandList(0, queue_desc.Type, allocator_, nullptr, IID_PPV_ARGS(&list_))) || FAILED(list_->Close()) ||
      FAILED(device_->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence_))))
    return fail("Creating the private camera composition queue failed.");
  D3D12_HEAP_PROPERTIES heap{};
  heap.Type = D3D12_HEAP_TYPE_DEFAULT;
  heap.CreationNodeMask = heap.VisibleNodeMask = 1;
  D3D12_RESOURCE_DESC desc{};
  desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
  desc.Width = BufferBytes;
  desc.Height = desc.DepthOrArraySize = desc.MipLevels = 1;
  desc.SampleDesc.Count = 1;
  desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
  if (FAILED(device_->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COMMON, nullptr,
                                              IID_PPV_ARGS(&buffer_))))
    return fail("Creating the stable camera image buffer failed.");
  heap.Type = D3D12_HEAP_TYPE_READBACK;
  if (FAILED(device_->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                              IID_PPV_ARGS(&meter_))))
    return fail("Creating the camera brightness readback buffer failed.");
  address_ = buffer_->GetGPUVirtualAddress();
  compositor_ = new (std::nothrow) CameraCompositorD3D12;
  if (!address_ || !compositor_ || FAILED(compositor_->initialize(device_)))
    return fail("Initializing the camera compositor failed.");
  D3D12_DESCRIPTOR_HEAP_DESC patch_heap_desc{D3D12_DESCRIPTOR_HEAP_TYPE_RTV, static_cast<UINT>(patches_.size()),
                                             D3D12_DESCRIPTOR_HEAP_FLAG_NONE, 0};
  if (FAILED(device_->CreateDescriptorHeap(&patch_heap_desc, IID_PPV_ARGS(&patch_heap_))))
    return fail("Creating the private patch descriptor heap failed.");
  for (unsigned i = 0; i < patch_drawers_.size(); ++i) {
    patch_drawers_[i] = new (std::nothrow) PatchShaderD3D12;
    if (!patch_drawers_[i] || FAILED(patch_drawers_[i]->initialize(device_, PatchFormats[i])))
      return fail("Creating a private WASM-format patch pipeline failed.");
  }
  return true;
}

/*
 * Allocates supported typed patch surfaces as needed, draws each requested region from the composed buffer,
 * then copies it into its GPU delivery buffer with explicit barriers.
 */
bool SceneFrameOutput::prepare_patches() noexcept {
  const auto base = patch_heap_->GetCPUDescriptorHandleForHeapStart();
  const auto stride = device_->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
  for (auto& storage : patches_) {
    auto* patch = &storage;
    if (!patch->width)
      continue;
    const auto width = patch->width, height = patch->height;
    const D3D12_RECT destination{0, 0, static_cast<LONG>(width), static_cast<LONG>(height)};
    if (!patch->texture) {
      D3D12_HEAP_PROPERTIES heap{};
      heap.Type = D3D12_HEAP_TYPE_DEFAULT;
      heap.CreationNodeMask = heap.VisibleNodeMask = 1;
      D3D12_RESOURCE_DESC desc{};
      desc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
      desc.Width = width;
      desc.Height = height;
      desc.DepthOrArraySize = desc.MipLevels = desc.SampleDesc.Count = 1;
      desc.Format = patch->format;
      desc.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;
      if (FAILED(device_->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COPY_SOURCE, nullptr,
                                                  IID_PPV_ARGS(&patch->texture))))
        return fail("Creating a private WASM-format patch texture failed.");
      UINT64 bytes{};
      device_->GetCopyableFootprints(&desc, 0, 1, 0, &patch->view.footprint, nullptr, nullptr, &bytes);
      desc = {};
      desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
      desc.Width = bytes;
      desc.Height = desc.DepthOrArraySize = desc.MipLevels = desc.SampleDesc.Count = 1;
      desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
      if (FAILED(device_->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COMMON, nullptr,
                                                  IID_PPV_ARGS(&patch->view.buffer))))
        return fail("Creating a stable WASM-format patch buffer failed.");
      patch->rtv = {base.ptr + static_cast<SIZE_T>(patch - patches_.data()) * stride};
      device_->CreateRenderTargetView(patch->texture, nullptr, patch->rtv);
    }
    D3D12_RESOURCE_BARRIER barrier{};
    barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    barrier.Transition = {patch->texture, 0, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET};
    list_->ResourceBarrier(1, &barrier);
    list_->OMSetRenderTargets(1, &patch->rtv, FALSE, nullptr);
    if (!patch_drawers_[patch->drawer]->record_private_patch(list_, device_, address_, width, height, &destination, &patch->content))
      return fail("Recording the WASM-format patch failed.");
    barrier.Transition = {patch->texture, 0, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_SOURCE};
    list_->ResourceBarrier(1, &barrier);
    barrier.Transition = {patch->view.buffer, 0, D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_COPY_DEST};
    list_->ResourceBarrier(1, &barrier);
    D3D12_TEXTURE_COPY_LOCATION source{}, target{};
    source.pResource = patch->texture;
    source.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    target.pResource = patch->view.buffer;
    target.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
    target.PlacedFootprint = patch->view.footprint;
    list_->CopyTextureRegion(&target, 0, 0, 0, &source, nullptr);
    barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
    barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_COMMON;
    list_->ResourceBarrier(1, &barrier);
    patch->recorded = true;
  }
  return true;
}

bool SceneFrameOutput::idle() const noexcept {
  if (failed_ || !fence_)
    return false;
  const auto completed = fence_->GetCompletedValue();
  return completed != UINT64_MAX && completed >= submitted_;
}
/*
 * Returns only fence values within the submitted sequence. UINT64_MAX indicates device removal and must not
 * count as successful completion.
 */
std::uint64_t SceneFrameOutput::completed_submissions() const noexcept {
  if (failed_ || !fence_)
    return 0;
  const auto completed = fence_->GetCompletedValue();
  return completed <= submitted_ ? completed : 0;
}

/*
 * Waits for an idle output slot, binds the captured image and records composition plus requested patches.
 * Failed recording preserves resource lifetime until its commands are safely discarded.
 *
 * Buffers begin each ExecuteCommandLists scope in COMMON. Our private patch draw promotes this buffer for
 * its root SRV read and it decays afterward.
 * https://learn.microsoft.com/en-us/windows/win32/direct3d12/using-resource-barriers-to-synchronize-resource-states-in-direct3d-12
 */
bool SceneFrameOutput::prepare(ID3D12Resource* image, DXGI_FORMAT format) noexcept {
  if (!idle() || prepared_)
    return false;
  if (brightness_pending_)
    read_brightness();
  brightness_recorded_ = false;
  if (FAILED(device_->GetDeviceRemovedReason()))
    return fail("The camera composition device was removed.");
  if (FAILED(compositor_->set_input(image, format))) {
    error_ = compositor_->last_error();
    return false;
  }
  if (FAILED(allocator_->Reset()) || FAILED(list_->Reset(allocator_, nullptr)))
    return fail("Resetting the completed private composition list failed.");
  if (FAILED(compositor_->record(list_, D3D12_RESOURCE_STATE_COPY_DEST)))
    return fail("Recording the camera composition failed.");

  D3D12_RESOURCE_BARRIER barrier{};
  barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
  barrier.Transition = {buffer_, 0, D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_COPY_DEST};
  list_->ResourceBarrier(1, &barrier);
  D3D12_TEXTURE_COPY_LOCATION destination{}, source{};
  destination.pResource = buffer_;
  destination.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
  destination.PlacedFootprint.Footprint = {DXGI_FORMAT_R8G8B8A8_UNORM, Width, Height, 1, RowPitch};
  source.pResource = compositor_->output();
  source.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
  list_->CopyTextureRegion(&destination, 0, 0, 0, &source, nullptr);
  barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
  barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_COMMON;
  list_->ResourceBarrier(1, &barrier);
  if (brightness_requested_) {
    destination.pResource = meter_;
    list_->CopyTextureRegion(&destination, 0, 0, 0, &source, nullptr);
    measured_exposure_ev_ = requested_exposure_ev_;
    brightness_requested_ = false;
    brightness_recorded_ = true;
  }

  if (!prepare_patches())
    return false;

  if (FAILED(list_->Close()))
    return fail("Closing the private composition list failed.");
  prepared_ = true;
  return true;
}

/*
 * Executes the prepared private list once and signals its completion fence. Submitted outputs and patches
 * retain that value until the GPU completes.
 */
bool SceneFrameOutput::submit() noexcept {
  if (!prepared_ || failed_ || submitted_ >= std::numeric_limits<std::uint64_t>::max() - 1)
    return false;
  ID3D12CommandList* lists[]{list_};
  queue_->ExecuteCommandLists(1, lists);
  prepared_ = false;
  ++submitted_;
  if (FAILED(queue_->Signal(fence_, submitted_)))
    return fail("Signaling the private composition completion fence failed.");
  for (auto& patch : patches_)
    if (patch.recorded) {
      patch.written = true;
      patch.recorded = false;
    }
  brightness_pending_ = brightness_pending_ || brightness_recorded_;
  brightness_recorded_ = false;
  return true;
}

/*
 * Reads the completed readback copy: the private fence was reached before this prepare() (idle()), so the
 * GPU no longer writes it. Thresholds 8 and 247 of 255 count nearly black and nearly white samples.
 */
void SceneFrameOutput::read_brightness() noexcept {
  brightness_pending_ = false;
  void* mapped = nullptr;
  const D3D12_RANGE read{0, static_cast<SIZE_T>(BufferBytes)};
  if (!meter_ || FAILED(meter_->Map(0, &read, &mapped)) || !mapped)
    return;
  const auto* bytes = static_cast<const std::uint8_t*>(mapped);
  std::uint64_t sum = 0, black = 0, white = 0, samples = 0;
  for (UINT y = 4; y < Height; y += 8)
    for (UINT x = 4; x < Width; x += 8) {
      const auto red = bytes[std::size_t{y} * RowPitch + std::size_t{x} * 4];
      sum += red;
      black += red < 8;
      white += red > 247;
      ++samples;
    }
  const D3D12_RANGE none{0, 0};
  meter_->Unmap(0, &none);
  brightness_.mean = static_cast<float>(sum) / (255.f * static_cast<float>(samples));
  brightness_.black = static_cast<float>(black) / static_cast<float>(samples);
  brightness_.white = static_cast<float>(white) / static_cast<float>(samples);
  brightness_.exposure_ev = measured_exposure_ev_;
  ++brightness_.sequence;
}

/*
 * Resets a never-submitted prepared recording only when the output is idle. It cannot discard resources
 * still referenced by submitted GPU work.
 */
bool SceneFrameOutput::discard_prepared() noexcept {
  if (!prepared_ || !idle())
    return false;
  if (FAILED(allocator_->Reset()) || FAILED(list_->Reset(allocator_, nullptr)) || FAILED(list_->Close()))
    return fail("Discarding the never-submitted private composition list failed.");
  for (auto& patch : patches_)
    patch.recorded = false;
  brightness_recorded_ = false;
  prepared_ = false;
  return true;
}
}  /* namespace azp */
