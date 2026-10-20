/*
 * Declares the ExecuteCommandLists hook. Around each observed submission it lets the capture manager place
 * its own GPU work and fence signals on the actual queue timeline, instead of assuming that recording a
 * command means it has completed.
 */

#pragma once
#include <d3d12.h>
#include <cstdint>

#ifndef NOMINMAX
#define NOMINMAX
#endif

namespace azp::queue_hooks {

inline constexpr unsigned kMaximumQueues = 8;
inline constexpr unsigned kMaximumCommandLists = 256;

/*
 * Why a forwarded batch could not be observed; the capture manager decides whether that is fatal.
 */
enum class Refusal { oversized_batch, invalid_batch, reentrant_submission };

/*
 * Process-lifetime counts of auxiliary submissions and the subset whose command lists could affect camera
 * capture. These diagnostics do not change queue behavior.
 */
struct Statistics {
  std::uint64_t reentrant = 0, reentrant_affecting = 0;
  std::uint64_t contended = 0, contended_affecting = 0;
};

/*
 * Registers one live native DIRECT queue. All queues share one hooked vtable cell (ExecuteCommandLists,
 * slot 10); at most eight queues are observed. A submitting queue that is not registered yet is offered to
 * the D3D12 tracker first, which registers it when it is a DIRECT queue of the simulator's device; other
 * queues on that table forward unchanged. The queue is AddRef'd for the life of the process.
 *
 * Observed before/original/after calls stay serialized per queue, so a Wait queued before the batch always
 * has a later Signal. A thread that cannot take the submit lock immediately forwards the batch without
 * waiting (Present and frame-generation helpers must never block on AZP). Before forwarding, a non-blocking
 * classifier distinguishes tracked lists with no capture effects from uncertain or camera-related work.
 * Only the latter invalidates capture observation. Batches with no capture work
 * release the lock before the original call. The application's queue, count and list array are forwarded
 * exactly once, even when capture is refused. Submissions nested inside the capture manager's own work (its
 * private queue-tail copies) forward without observation.
 */
bool register_queue(ID3D12CommandQueue* queue) noexcept;
Statistics statistics() noexcept;

}  /* namespace azp::queue_hooks */
