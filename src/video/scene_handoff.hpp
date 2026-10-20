/*
 * Defines the thread-safe handoff between camera inspection and GPU resource tracking. Publications carry
 * identity and generation evidence; returned tickets must be checked again before capture or delivery.
 */

#pragma once
#include <atomic>
#include <cstdint>
#include <mutex>
#include <unordered_map>

namespace azp {

struct SceneManagerIdentity {
  /*
   * Opaque internal identity and generation. Never dereference or serialize.
   */
  std::uint64_t identity = 0;
  std::uint64_t generation = 0;
  bool operator==(const SceneManagerIdentity&) const = default;
};

struct SceneResourceIdentity {
  std::uint64_t resource_id = 0;  /* Existing graphics registry target ID. */
  std::uint64_t generation = 0;   /* Unique registration within this handoff. */
  bool operator==(const SceneResourceIdentity&) const = default;
};

struct SceneCaptureTicket {
  std::uint64_t scene_epoch = 0;
  std::uint64_t lifecycle_epoch = 0;
  std::uint64_t sequence = 0;
};

struct SceneCopyMatch {
  bool matched = false;
  std::uint64_t scene_epoch = 0;
  std::uint64_t capture_sequence = 0;
  SceneManagerIdentity manager;
  std::uint64_t entry_id = 0;
  SceneResourceIdentity resource;
  bool night_vision = false;  /* TV or IR. */
};

/*
 * CPU identity matching only: no graphics/engine calls, pointer dereference, AddRef, resource-state
 * inference or GPU completion/lifetime guarantee. Lifecycle callbacks must remain registered even when
 * activity discovery is paused. Register the device and resource creation before any capture may consume it;
 * unregister at the corresponding destruction callback. Every such event invalidates open inspection
 * tickets, including untracked resources. A completed publication survives unrelated resource churn;
 * touching the published handle or registering the device invalidates publication. A repeated native address
 * receives a fresh registration generation. The authorized scene-start path calls begin_scene. Stop and any
 * failed or retired engine-owner phase call stop_scene immediately, before native erase. For each engine
 * snapshot: begin_capture BEFORE inspecting the complete live owned view, then publish with its same manager
 * generation, nonzero entry ID and freshly read opaque resource member. A lifecycle event in between, an
 * unregistered resource address or an owner change refuses. No unregistered pointer can become a resource
 * merely by being published. Opening a refresh or rejecting its ticket for unrelated resource churn keeps the
 * previous complete publication. A latest valid ticket with malformed or unresolvable data revokes it; actual
 * owner/resource/scene invalidation remains immediate. A failed engine inspection must still call
 * stop_scene. observe accepts ONLY handles from actual native GPU callbacks; the capture manager separately
 * proves the resource state and recording/submission/retirement contract. A returned match is a value
 * snapshot, not a lease. is_current checks it again before consuming a completed owned GPU capture. A
 * completed publication stays current while a new observer capture is in progress. Old captures may remain
 * current across later successful observations of the SAME view and resource generation. No method calls
 * external code under the mutex. Never hold this class's lock while invoking the engine or graphics API. The
 * 16384-resource cap bounds metadata; cap exhaustion simply refuses matching.
 */
class SceneHandoff {
 public:
  static constexpr std::size_t maximum_resources = 16384;

  /*
   * Called once with the simulator's device; resources are registered only after it.
   */
  bool register_device();
  bool register_resource(std::uint64_t resource_handle, std::uint64_t registry_resource_id);
  void unregister_resource(std::uint64_t resource_handle);

  std::uint64_t begin_scene();
  void stop_scene();
  SceneCaptureTicket begin_capture();
  /*
   * Discard the displayed source and open tickets without replacing owned views.
   */
  void invalidate_publication();
  /*
   * Publishes the owned view's final image, or revokes it with a zero handle.
   */
  bool publish(SceneCaptureTicket ticket, SceneManagerIdentity manager, std::uint64_t entry_id, std::uint64_t resource_handle,
               bool night_vision);
  SceneCopyMatch observe(std::uint64_t resource_handle) const;
  /*
   * Metadata-only hint: the last completely inspected image, including while the next inspection is open.
   * NEVER capture/admission proof; resource/device/scene changes clear it, and authoritative matching is
   * still required before recording GPU work.
   */
  bool observed(std::uint64_t resource_handle) const noexcept;

  bool is_current(const SceneCopyMatch& match) const;

 private:
  struct Resource {
    std::uint64_t id = 0;
    std::uint64_t generation = 0;
  };
  struct Publication {
    bool valid = false;
    bool night_vision = false;
    std::uint64_t scene_epoch = 0;
    std::uint64_t sequence = 0;
    SceneManagerIdentity manager;
    std::uint64_t entry_id = 0;
    std::uint64_t handle = 0;
    SceneResourceIdentity resource;
  };
  bool advance(std::uint64_t& counter);
  bool lifecycle_event(std::uint64_t changed_handle = 0);
  void clear_handles() noexcept;
  SceneCopyMatch match(std::uint64_t handle) const;

  mutable std::mutex mutex_;
  bool device_registered_ = false;
  std::unordered_map<std::uint64_t, Resource> resources_;
  std::uint64_t resource_counter_ = 0;
  std::uint64_t lifecycle_epoch_ = 1;
  std::uint64_t scene_epoch_ = 0;
  std::uint64_t capture_sequence_ = 0;
  std::uint64_t minimum_capture_sequence_ = 0;
  bool exhausted_ = false;
  bool active_ = false;
  bool owner_bound_ = false;
  SceneManagerIdentity owner_;
  std::uint64_t owned_entry_ = 0;
  Publication publication_;
  /*
   * Rejection-only hint for unrelated high-frequency callbacks. Every possible match still takes the mutex
   * and verifies the full publication.
   */
  std::atomic<std::uint64_t> candidate_handle_{0};
  std::atomic<std::uint64_t> observed_handle_{0};
};

/*
 * Process-lifetime storage, shared by the native observer and add-on callbacks. Creation of this object
 * performs no engine/graphics operation.
 */
SceneHandoff& scene_handoff();

}  /* namespace azp */
