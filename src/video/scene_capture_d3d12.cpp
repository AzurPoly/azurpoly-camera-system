/*
 * Owns the D3D12 resources for a single camera-image capture and its reuse cycle. It records copies only in
 * proven source states, waits for producer and consumer fences, and keeps submitted resources alive until
 * the GPU has finished with them.
 */

#include "scene_capture_d3d12.hpp"
#include <limits>
#include "resource_guards.hpp"

namespace azp {
namespace {
bool valid_description(const D3D12_RESOURCE_DESC& desc) noexcept {
  const auto bytes = scene_texture_bytes(desc.Format);
  return bytes != 0 && desc.Dimension == D3D12_RESOURCE_DIMENSION_TEXTURE2D && desc.Width >= 1 && desc.Width <= 8192 && desc.Height >= 1 &&
         desc.Height <= 8192 && desc.DepthOrArraySize == 1 && desc.MipLevels == 1 && desc.SampleDesc.Count == 1 &&
         desc.SampleDesc.Quality == 0 && desc.Layout == D3D12_TEXTURE_LAYOUT_UNKNOWN &&
         (desc.Flags & D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL) == 0 &&
         desc.Width * desc.Height * bytes + D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT <= SceneCaptureD3D12::MaximumBytes;
}

bool compatible(const D3D12_RESOURCE_DESC& first, const D3D12_RESOURCE_DESC& second) noexcept {
  return valid_description(first) && first.Width == second.Width && first.Height == second.Height && first.Format == second.Format;
}
}  /* namespace */

/*
 * Dropping these raw owning references without Release is deliberate if the recording or consumer might
 * still use them. Never silently free in-flight GPU storage because its CPU owner went out of scope.
 */
SceneCaptureD3D12::~SceneCaptureD3D12() {
  release_idle();
}

bool SceneCaptureD3D12::fail(const char* message) noexcept {
  error_ = message;
  return false;
}

bool SceneCaptureD3D12::device_ok() const noexcept {
  return device_ != nullptr && SUCCEEDED(device_->GetDeviceRemovedReason());
}

bool SceneCaptureD3D12::same_device(ID3D12DeviceChild* object) const noexcept {
  return same_native_device(object, device_);
}

/*
 * Creates a capture texture and synchronization resources for a supported source description. Device, shape
 * and allocation limits must agree before the packet becomes reusable.
 */
HRESULT SceneCaptureD3D12::initialize(ID3D12Device* device, const D3D12_RESOURCE_DESC& description) noexcept {
  if (state_ != State::empty || device == nullptr || !valid_description(description)) {
    fail("An empty packet, live device and supported bounded single-subresource texture are required.");
    return E_INVALIDARG;
  }
  if (FAILED(device->GetDeviceRemovedReason()))
    return DXGI_ERROR_DEVICE_REMOVED;
  device_ = device;
  device_->AddRef();
  description_ = description;
  description_.Alignment = 0;
  description_.Flags = D3D12_RESOURCE_FLAG_NONE;
  const auto allocation = device_->GetResourceAllocationInfo(0, 1, &description_);
  if (allocation.SizeInBytes == 0 || allocation.SizeInBytes > MaximumBytes) {
    release_objects();
    fail("The device-reported texture allocation exceeds the packet's byte limit.");
    return E_INVALIDARG;
  }
  allocation_bytes_ = allocation.SizeInBytes;
  D3D12_HEAP_PROPERTIES heap{};
  heap.Type = D3D12_HEAP_TYPE_DEFAULT;
  heap.CreationNodeMask = heap.VisibleNodeMask = 1;
  HRESULT result = device_->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &description_, D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                                    IID_PPV_ARGS(&snapshot_));
  if (SUCCEEDED(result))
    result = device_->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&producer_fence_));
  if (FAILED(result)) {
    release_objects();
    fail("Preallocating the capture texture or completion fence failed.");
    return result;
  }
  state_ = State::idle;
  error_ = "";
  return S_OK;
}

/*
 * Checks packet availability and source identity before beginning a render-target capture. A busy packet
 * cannot be overwritten while producer or consumer work still refers to it.
 */
bool SceneCaptureD3D12::begin_render_capture(ID3D12GraphicsCommandList* list, ID3D12Resource* source) noexcept {
  if (!list || !source || source == snapshot_ || state_ != State::idle || !device_ok() || !same_device(list) || !same_device(source) ||
      list->GetType() != D3D12_COMMAND_LIST_TYPE_DIRECT)
    return fail("Render capture requires an idle packet and a live DIRECT recording.");
  const auto desc = source->GetDesc();
  if (!compatible(desc, description_) || (desc.Flags & D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET) == 0 ||
      (desc.Flags & D3D12_RESOURCE_FLAG_ALLOW_SIMULTANEOUS_ACCESS) != 0)
    return fail("Render capture requires a matching ordinary single-subresource color render target.");
  source->AddRef();
  source_ = source;
  recording_list_ = list;
  return true;
}

/*
 * Records the source transition, copy and restoration from a proven legacy render-target state.
 */
bool SceneCaptureD3D12::record_render_target_source(ID3D12GraphicsCommandList* list, ID3D12Resource* source) noexcept {
  if (!begin_render_capture(list, source))
    return false;
  D3D12_RESOURCE_BARRIER barrier{};
  barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
  barrier.Transition = {source, 0, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_SOURCE};
  list->ResourceBarrier(1, &barrier);
  list->CopyResource(snapshot_, source);
  barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_SOURCE;
  barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_RENDER_TARGET;
  list->ResourceBarrier(1, &barrier);
  state_ = State::recorded;
  error_ = "";
  return true;
}

/*
 * Records the corresponding enhanced-barrier transition/copy/restore sequence using the observed
 * synchronization, access and layout evidence.
 */
bool SceneCaptureD3D12::record_render_target_source_enhanced(ID3D12GraphicsCommandList7* list, ID3D12Resource* source) noexcept {
  if (!begin_render_capture(list, source))
    return false;
  D3D12_TEXTURE_BARRIER barrier{};
  barrier.SyncBefore = D3D12_BARRIER_SYNC_ALL;
  barrier.SyncAfter = D3D12_BARRIER_SYNC_COPY;
  barrier.AccessBefore = D3D12_BARRIER_ACCESS_RENDER_TARGET;
  barrier.AccessAfter = D3D12_BARRIER_ACCESS_COPY_SOURCE;
  barrier.LayoutBefore = D3D12_BARRIER_LAYOUT_RENDER_TARGET;
  barrier.LayoutAfter = D3D12_BARRIER_LAYOUT_COPY_SOURCE;
  barrier.pResource = source;
  barrier.Subresources.NumMipLevels = barrier.Subresources.NumArraySlices = barrier.Subresources.NumPlanes = 1;
  D3D12_BARRIER_GROUP group{};
  group.Type = D3D12_BARRIER_TYPE_TEXTURE;
  group.NumBarriers = 1;
  group.pTextureBarriers = &barrier;
  list->Barrier(1, &group);
  list->CopyResource(snapshot_, source);
  barrier.SyncBefore = D3D12_BARRIER_SYNC_COPY;
  barrier.SyncAfter = D3D12_BARRIER_SYNC_ALL;
  barrier.AccessBefore = D3D12_BARRIER_ACCESS_COPY_SOURCE;
  barrier.AccessAfter = D3D12_BARRIER_ACCESS_RENDER_TARGET;
  barrier.LayoutBefore = D3D12_BARRIER_LAYOUT_COPY_SOURCE;
  barrier.LayoutAfter = D3D12_BARRIER_LAYOUT_RENDER_TARGET;
  list->Barrier(1, &group);
  state_ = State::recorded;
  error_ = "";
  return true;
}

/*
 * Releases a retired recording only when its capture work was never submitted, so no GPU execution can still
 * reference that recording.
 */
bool SceneCaptureD3D12::discard_unsubmitted_retired() noexcept {
  if (state_ != State::recorded || !source_)
    return fail("Only a never-submitted, definitively retired recording may be discarded.");
  source_->Release();
  source_ = nullptr;
  recording_list_ = nullptr;
  state_ = State::idle;
  error_ = "";
  return true;
}

/*
 * Retires recording ownership while retaining submitted capture resources until their serialized GPU
 * obligations complete.
 */
bool SceneCaptureD3D12::retire_serialized_recording(ID3D12CommandQueue* queue) noexcept {
  if (state_ != State::recorded || queue == nullptr || !device_ok() || !same_device(queue) ||
      queue->GetDesc().Type != D3D12_COMMAND_LIST_TYPE_DIRECT || producer_value_ == std::numeric_limits<std::uint64_t>::max() - 1)
    return fail("Retirement requires the actual completed-submission queue and a non-exhausted fence sequence.");
  ++producer_value_;
  if (FAILED(queue->Signal(producer_fence_, producer_value_))) {
    state_ = State::failed;
    return fail("Producer queue Signal failed; the capture remains quarantined.");
  }
  state_ = State::retiring;
  return true;
}

/*
 * Checks the capture fence and keeps a completed frame available until the consumer accepts it. A submitted
 * command is not considered ready before the GPU reaches its recorded fence value.
 */
bool SceneCaptureD3D12::poll_ready() noexcept {
  if (state_ == State::ready)
    return device_ok();
  if (state_ != State::retiring || !device_ok())
    return false;
  const auto completed = producer_fence_->GetCompletedValue();
  if (completed == std::numeric_limits<std::uint64_t>::max()) {
    state_ = State::failed;
    return fail("Producer fence reported device removal.");
  }
  if (completed < producer_value_)
    return false;
  source_->Release();
  source_ = nullptr;
  recording_list_ = nullptr;
  state_ = State::ready;
  return true;
}

ID3D12Resource* SceneCaptureD3D12::ready_resource() const noexcept {
  return state_ == State::ready && device_ok() ? snapshot_ : nullptr;
}

/*
 * Attaches the consumer completion fence/value to the captured frame so recycling cannot precede its last
 * GPU reader.
 */
bool SceneCaptureD3D12::finish_consumption(ID3D12Fence* fence, std::uint64_t value) noexcept {
  if (state_ != State::ready || fence == nullptr || value == 0 || value == std::numeric_limits<std::uint64_t>::max() ||
      !same_device(fence) || !device_ok())
    return fail("Consumption requires a same-device fence value covering every submitted reader.");
  fence->AddRef();
  consumer_fence_ = fence;
  consumer_value_ = value;
  state_ = State::consuming;
  return true;
}

/*
 * Returns a packet to the reusable state only after both producer and consumer obligations have completed;
 * device removal is treated as failure, not success.
 */
bool SceneCaptureD3D12::recycle() noexcept {
  if (state_ != State::consuming || !device_ok())
    return false;
  const auto completed = consumer_fence_->GetCompletedValue();
  if (completed == std::numeric_limits<std::uint64_t>::max()) {
    state_ = State::failed;
    return fail("Consumer fence reported device removal.");
  }
  if (completed < consumer_value_)
    return false;
  consumer_fence_->Release();
  consumer_fence_ = nullptr;
  consumer_value_ = 0;
  state_ = State::idle;
  error_ = "";
  return true;
}

bool SceneCaptureD3D12::release_idle() noexcept {
  if (state_ != State::idle && state_ != State::empty)
    return false;
  release_objects();
  state_ = State::empty;
  return true;
}

/*
 * Releases the packet's owned resources and resets state after the caller has established that ordinary
 * release is permitted.
 */
void SceneCaptureD3D12::release_objects() noexcept {
  if (snapshot_)
    snapshot_->Release();
  if (producer_fence_)
    producer_fence_->Release();
  if (device_)
    device_->Release();
  snapshot_ = nullptr;
  producer_fence_ = nullptr;
  device_ = nullptr;
  allocation_bytes_ = 0;
}
}  /* namespace azp */
