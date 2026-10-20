/*
 * Defines ownership of the one native camera entry AZP creates, the manager identity token and the lifecycle
 * callbacks. The controller exchanges requests across threads but delegates engine calls to the caller's
 * verified update phase.
 */

#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>

namespace azp::engine_camera {

using EntryId = std::uint64_t;

/*
 * Opaque adapter identity, not a dereferenceable pointer. Generation must change if an address/identity is
 * reused for a different manager lifetime.
 */
struct ManagerToken {
  std::uint64_t identity = 0;
  std::uint64_t generation = 0;
  bool valid() const noexcept { return identity != 0 && generation != 0; }
  bool operator==(const ManagerToken&) const = default;
};

/*
 * The observed initializer touches at least 196 bytes. This deliberately padded backing store and its
 * alignment are local choices, NOT a recovered sizeof or engine ABI guarantee. A native adapter must
 * establish compatibility separately.
 */
struct alignas(16) DescriptorStorage {
  static constexpr std::size_t minimum_observed_extent = 196;
  std::array<std::uint8_t, 256> bytes{};
};

/*
 * Pack the verified independent-view descriptor after native initialization.
 */
bool pack_independent_pose(DescriptorStorage& descriptor) noexcept;

struct EngineCallbacks {
  void* context = nullptr;
  bool (*initialize)(void*, DescriptorStorage&) noexcept = nullptr;
  EntryId (*create)(void*, ManagerToken, const DescriptorStorage&) noexcept = nullptr;
  /*
   * True ONLY when the adapter has confirmed this ID is absent after erase. The recovered native erase takes
   * (manager, uint64 ID by value), with no established status return. This bool is the wrapper's
   * verification result.
   */
  bool (*erase)(void*, ManagerToken, EntryId) noexcept = nullptr;
};

enum class State { disabled, active, cleanup_pending, failed, blocked };
enum class Failure { none, initializer_failed, initializer_contract, first_create_failed, manager_destroyed };
enum class Blocked { none, invalid_manager, missing_callbacks, manager_mismatch };
enum class EmptyCameraCancel { cancelled, busy, owned };

struct Snapshot {
  State state = State::disabled;
  Failure failure = Failure::none;
  Blocked blocked = Blocked::none;
  ManagerToken owner{};
  EntryId owned_id = 0;
  /*
   * Last-wins mailbox: querying/publishing requests never calls the engine.
   */
  bool request_pending = false;
  bool creation_pending = false;
};

class CameraOwnership {
 public:
  void request_enable() noexcept;
  void request_disable() noexcept;
  Snapshot snapshot() const noexcept;

  /*
   * Cancel only uncreated work, without a manager or engine callback. This may run on a request thread: the
   * processing guard excludes in-flight creation even when its last published snapshot still looks empty.
   * Busy/owned leave every request and ownership field unchanged.
   */
  EmptyCameraCancel cancel_uncreated_request() noexcept;

  /*
   * Call ONLY from the approved update observer with a live manager. All engine callbacks happen here,
   * synchronously and without the mailbox mutex held. False means a concurrent/reentrant update was refused,
   * with no engine call. Failures consume the creation request; only a new enable retries creation.
   */
  bool process_update(ManagerToken manager, const EngineCallbacks& engine) noexcept;

  /*
   * Explicit observer-side lifetime event, not a cleanup shortcut. Call only AFTER this exact manager
   * lifetime has been destroyed by its owner. Forgets its invalidated IDs and clears queued requests without
   * calling any engine.
   */
  bool acknowledge_manager_destroyed(ManagerToken manager) noexcept;

 private:
  enum class Command { none, enable, disable };
  struct Request {
    Command command = Command::none;
  };
  bool owns_entry() const noexcept { return id_ != 0; }
  void publish(State state, Blocked blocked = Blocked::none) noexcept;
  void cleanup(const EngineCallbacks& engine) noexcept;
  bool prepare(const EngineCallbacks& engine, DescriptorStorage& descriptor) noexcept;
  void creation_failed(const EngineCallbacks& engine, Failure failure) noexcept;

  mutable std::mutex mailbox_mutex_;
  Request mailbox_{};
  Snapshot published_{};
  std::atomic_flag processing_ = ATOMIC_FLAG_INIT;
  /*
   * Below: accessed only while processing_ is held by update/lifetime/cancel.
   */
  ManagerToken owner_{};
  EntryId id_ = 0;
  bool desired_enabled_ = false;
  bool creation_pending_ = false;
  bool active_ = false;
  Failure failure_ = Failure::none;
};

}  /* namespace azp::engine_camera */
