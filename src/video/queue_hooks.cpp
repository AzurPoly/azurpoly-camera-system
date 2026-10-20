/*
 * Wraps native ExecuteCommandLists calls to observe ordered GPU submissions. The capture manager may add its
 * separate work around an accepted submission, but the simulator's original list batch is forwarded
 * unchanged exactly once; reentry and contention refuse observation rather than blocking the engine.
 */

#include "queue_hooks.hpp"
#include <array>
#include <atomic>
#include "d3d12_objects.hpp"
#include "native_hooks.hpp"
#include "scene_runtime.hpp"

namespace azp::queue_hooks {
namespace {

using Execute = void(STDMETHODCALLTYPE*)(ID3D12CommandQueue*, UINT, ID3D12CommandList* const*);

struct QueueState {
  SRWLOCK submit_lock = SRWLOCK_INIT;
  std::atomic<ID3D12CommandQueue*> queue{nullptr};
  std::atomic<bool> enabled{false};
  std::atomic<std::uint64_t> reentrant{0}, reentrant_affecting{0};
  std::atomic<std::uint64_t> contended{0}, contended_affecting{0};
};

SRWLOCK control_lock = SRWLOCK_INIT;
std::array<QueueState, kMaximumQueues> queues;
standalone::HookSlot execute_slot;
thread_local bool discovering_queue = false;
thread_local bool inside_wrapper = false;
thread_local bool nested_capture_risk = false;

struct ExclusiveLock {
  SRWLOCK& lock;
  bool owned = false;
  explicit ExclusiveLock(SRWLOCK& value, bool try_only = false) noexcept : lock(value) {
    if (try_only)
      owned = TryAcquireSRWLockExclusive(&lock) != FALSE;
    else {
      AcquireSRWLockExclusive(&lock);
      owned = true;
    }
  }
  ~ExclusiveLock() { release(); }
  void release() noexcept {
    if (!owned)
      return;
    ReleaseSRWLockExclusive(&lock);
    owned = false;
  }
  ExclusiveLock(const ExclusiveLock&) = delete;
  ExclusiveLock& operator=(const ExclusiveLock&) = delete;
};

QueueState* find_queue(ID3D12CommandQueue* queue) noexcept {
  if (queue)
    for (auto& entry : queues)
      if (entry.queue.load(std::memory_order_acquire) == queue)
        return &entry;
  return nullptr;
}

/*
 * Reports a forwarded batch that could not be observed. D3D12 calls made meanwhile are AZP's own work.
 */
void refuse(ID3D12CommandQueue* queue, Refusal reason) noexcept {
  const standalone::OwnedWork owned;
  scene_runtime::manager().submission_refused(queue, reason);
}

/*
 * Uses only try-lock metadata paths to decide whether an unordered batch is provably unrelated to capture.
 * Any unavailable or incomplete evidence remains capture-affecting.
 */
bool affects_capture(ID3D12CommandQueue* queue, UINT count, ID3D12CommandList* const* lists) noexcept {
  const standalone::OwnedWork owned;
  return scene_runtime::manager().unordered_submission_affects_capture(queue, count, lists);
}

/*
 * Forwards the application's exact command-list batch once, with capture before/after steps only when queue
 * identity and serialization allow observation. Reentry or contention refuses capture observation instead
 * of waiting inside a potentially dependent DXGI submission.
 *
 * Frame-generation and Present helpers share this queue. Waiting here deadlocks DXGI when the owner is
 * already inside original Execute.
 *
 * Never replay, split, omit, replace or add lists to the application's batch.
 */
void STDMETHODCALLTYPE submit(ID3D12CommandQueue* queue, UINT count, ID3D12CommandList* const* lists) noexcept {
  const auto forward = execute_slot.forward<Execute>();
  auto* entry = find_queue(queue);
  if (!entry && !inside_wrapper && !discovering_queue) {
    discovering_queue = true;
    standalone::discover_queue(queue);
    discovering_queue = false;
    entry = find_queue(queue);
  }
  if (inside_wrapper) {
    if (standalone::owned_depth) {
      forward(queue, count, lists);
      return;
    }
    const bool affects = !entry || !entry->enabled.load(std::memory_order_acquire) || affects_capture(queue, count, lists);
    if (entry) {
      entry->reentrant.fetch_add(1, std::memory_order_relaxed);
      if (affects)
        entry->reentrant_affecting.fetch_add(1, std::memory_order_relaxed);
    }
    nested_capture_risk |= affects;
    forward(queue, count, lists);
    return;
  }
  if (!entry || !entry->enabled.load(std::memory_order_acquire)) {
    forward(queue, count, lists);
    return;
  }
  ExclusiveLock lock(entry->submit_lock, true);
  if (!lock.owned) {
    const bool affects = affects_capture(queue, count, lists);
    entry->contended.fetch_add(1, std::memory_order_relaxed);
    if (affects)
      entry->contended_affecting.fetch_add(1, std::memory_order_relaxed);
    if (affects) {
      const standalone::OwnedWork owned;
      scene_runtime::manager().defer_unordered_submission();
    }
    forward(queue, count, lists);
    if (affects) {
      const standalone::OwnedWork owned;
      scene_runtime::manager().defer_unordered_submission();
    }
    return;
  }
  if (!entry->enabled.load(std::memory_order_acquire)) {
    forward(queue, count, lists);
    return;
  }
  struct ReentryGuard {
    ReentryGuard() noexcept {
      inside_wrapper = true;
      nested_capture_risk = false;
    }
    ~ReentryGuard() { inside_wrapper = false; }
  } guard;
  const bool oversized = count > kMaximumCommandLists;
  const bool invalid = count == 0 || lists == nullptr;
  std::uint64_t receipt = 0;
  if (!oversized && !invalid) {
    const standalone::OwnedWork owned;
    receipt = scene_runtime::manager().before_submission(queue, count, lists);
  }
  if (!receipt && !oversized && !invalid) {
    lock.release();
    forward(queue, count, lists);
    if (nested_capture_risk)
      refuse(queue, Refusal::reentrant_submission);
    return;
  }
  forward(queue, count, lists);
  if (oversized || invalid)
    refuse(queue, oversized ? Refusal::oversized_batch : Refusal::invalid_batch);
  else if (nested_capture_risk)
    refuse(queue, Refusal::reentrant_submission);
  else {
    const standalone::OwnedWork owned;
    scene_runtime::manager().after_submission(queue, receipt);
  }
}

}  /* namespace */

/*
 * Installs the shared hook on the first registration, then retains the queue and enables it. A queue whose
 * table differs from the hooked one, or whose hooked cell was replaced, is refused. Registering the same
 * queue again re-verifies the hook.
 */
bool register_queue(ID3D12CommandQueue* queue) noexcept {
  if (inside_wrapper || !queue)
    return false;
  const ExclusiveLock lock(control_lock);
  auto* entry = find_queue(queue);
  const bool existed = entry != nullptr;
  if (!entry)
    for (auto& candidate : queues)
      if (candidate.queue.load(std::memory_order_acquire) == nullptr) {
        entry = &candidate;
        break;
      }
  if (!entry || !execute_slot.install(queue, 10, reinterpret_cast<void*>(&submit)))
    return false;
  if (!existed) {
    queue->AddRef();
    entry->queue.store(queue, std::memory_order_release);
  }
  entry->enabled.store(true, std::memory_order_release);
  return true;
}

Statistics statistics() noexcept {
  Statistics result;
  for (const auto& entry : queues) {
    result.reentrant += entry.reentrant.load(std::memory_order_relaxed);
    result.reentrant_affecting += entry.reentrant_affecting.load(std::memory_order_relaxed);
    result.contended += entry.contended.load(std::memory_order_relaxed);
    result.contended_affecting += entry.contended_affecting.load(std::memory_order_relaxed);
  }
  return result;
}

}  /* namespace azp::queue_hooks */
