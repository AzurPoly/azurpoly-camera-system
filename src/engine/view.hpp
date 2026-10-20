/*
 * Declares the inspections and guarded mutations of the engine's camera view objects (see view.cpp). Results
 * carry borrowed native addresses that are valid only inside the engine phase that produced them; a pending
 * object is reported distinctly from an invalid one.
 */

#pragma once
#include <array>
#include <cstdint>
#include <string>
#include "../shared/configuration.hpp"
#include "camera_layout.hpp"
#include "image_inventory.hpp"
#include "memory.hpp"

namespace azp::engine_camera {

enum class ViewPoolStatus {
  not_inspected,
  complete,
  invalid_renderer,
  invalid_array,
  invalid_view,
  duplicate_view,
  invalid_control,
  read_failed,
  changed,
  read_budget_exhausted
};

enum class ViewAssociation { unobserved, null_control, stale_generation, null_payload, occupied };

struct ViewSlot {
  std::uint32_t index = 0;
  /*
   * Internal addresses only. Do not serialize these into UI/log/JSON output.
   */
  std::uint64_t view_address = 0;
  ViewAssociation association = ViewAssociation::unobserved;
  bool free = false;
  bool association_valid = false;
};

struct ViewPoolSnapshot {
  bool valid = false;
  ViewPoolStatus status = ViewPoolStatus::not_inspected;
  /*
   * Internal pointer; never a lifetime token or a reservation.
   */
  std::uint64_t array_address = 0;
  std::uint32_t slots_examined = 0;
  std::int32_t failure_slot = -1;
  /*
   * Attempted bytes, including failed reads, capped at 8192. The current eight-slot walk plus complete trace
   * reread uses at most 592 bytes.
   */
  std::uint32_t read_bytes = 0;
  std::uint32_t read_failures = 0;
  std::array<ViewSlot, 8> slots{};
  /*
   * Published only after all slots and their consistency rechecks succeed.
   */
  std::uint32_t free_count = 0;
  std::array<std::int32_t, 2> first_free_indices{-1, -1};
};

/*
 * Reproduce the captured 66864720 availability predicate over eight EXISTING views. renderer+2752 contains a
 * pointer to an array, not an inline array. Read only its eight view pointers, each P+72 sixteen-byte
 * association, and nonnull control+28 generation / matching control+0 payload. Null/stale/null- payload
 * handles are free; a null or duplicate P is an error. Reread every observed field at its exact address
 * before publishing complete capacity. Control records may be byte-aligned; renderer, array and view
 * pointers still require eight-byte alignment. No pointer bits are masked. Never call an engine function,
 * reserve a slot, mutate memory, query the cached global or follow an association payload. The adapter owns
 * readable-region checks and cached-renderer/build validation. Call in the same engine phase as prospective
 * creation: this detects observed changes but is not atomic, a lifetime guarantee or a capacity reservation.
 * The reader must not throw; report unavailable memory by returning false. This helper is not an SEH fault
 * barrier.
 */
ViewPoolSnapshot inspect_view_pool(MemoryReader& reader, std::uint64_t renderer) noexcept;

}  /* namespace azp::engine_camera */

namespace azp::engine_camera {

inline constexpr std::uint32_t kOwnedEntryBucketLimit = 4096;
inline constexpr std::uint32_t kOwnedEntryNodeLimit = 1024;
inline constexpr std::uint32_t kOwnedEntryReadLimit = 128 * 1024;

struct OwnedEntry {
  bool found = false;
  /*
   * Internal adapter data only. Never log these addresses or retain them past the proven manager
   * lifetime/update phase. No output is usable if incomplete.
   */
  std::uint64_t address = 0;
  std::uint64_t key = 0;
  std::uint64_t payload_id = 0;
};

struct OwnedEntryInventory {
  /*
   * Only complete=true permits treating found=false as confirmed absence in this snapshot. It does not
   * establish an atomic snapshot or GPU retirement.
   */
  bool complete = false;
  const char* error = "";
  std::uint32_t read_bytes = 0; /* Attempted bytes, including failed reads. */
  std::uint32_t read_failures = 0;
  std::uint32_t entry_count = 0;
  std::uint32_t bucket_count = 0;
  std::uint32_t nodes_visited = 0;
  /*
   * Published only after full traversal and every consistency reread succeeds.
   */
  OwnedEntry entry;
};

/*
 * Build-specific fixed layout, no engine calls: T=manager+88 contains DWORD entry count+0, DWORD bucket
 * count+4 and pointer+8 to untagged 8-byte heads. Every live E contributes key+0, payload ID+16 and tagged
 * next+296 (mask bit0). No reuse list, ready byte, handles, strings or other payload fields are read. A zero
 * owned ID is never matched. Full distinct-node traversal,
 * count agreement and a reread of every observed header/head/node field are required. Malformed pointers,
 * cycles, duplicate keys, owned-key/payload-ID mismatch, caps and changed/unreadable fields leave
 * complete=false. Reader caps are not recovered engine allocation limits. The caller owns fresh build/code
 * proof, manager generation and update-thread lifetime/synchronization. This function cannot freeze or
 * acquire that state. Bucket heads are read as one exact contiguous field then fully reread. A MemoryReader
 * must accept up to32768 bytes for that field. Fixed local storage is about96 KiB; it performs no dynamic
 * allocation.
 */
OwnedEntryInventory inspect_owned_entries(MemoryReader& reader, std::uint64_t manager_address, std::uint64_t owned_id);

}  /* namespace azp::engine_camera */

namespace azp::engine_camera {

enum class OwnedViewStatus {
  not_inspected,
  pending,
  ready,
  invalid_request,
  invalid_pool,
  invalid_pointer,
  id_mismatch,
  invalid_ready_byte,
  invalid_view_index,
  pool_changed,
  node_unavailable,
  node_mismatch,
  wrong_camera_type,
  invalid_fov,
  material_mismatch,
  read_failed,
  changed,
  read_budget_exhausted
};

struct OwnedViewSnapshot {
  /*
   * complete includes a stable pending entry; ready requires the camera chain.
   */
  bool complete = false;
  bool ready = false;
  OwnedViewStatus status = OwnedViewStatus::not_inspected;
  const char* error = "";
  /*
   * Attempted bytes including failed reads; cap 8192, current full trace 518.
   */
  std::uint32_t read_bytes = 0;
  std::uint32_t read_failures = 0;
  std::int32_t view_index = -1;
  /*
   * Exact ready-entry E24 mode, reread with the full trace. Raw observation; callers that require
   * independent-pose semantics must require mode == 2.
   */
  std::uint32_t mode = 0;
  /*
   * Internal, borrowed addresses, published only after the entire trace reread. Never log/serialize or
   * retain them beyond the caller's proven engine phase.
   */
  std::uint64_t view_address = 0;
  std::uint64_t node_address = 0;
  std::uint64_t camera_address = 0;
  /*
   * Opaque member for lifecycle-bracketed resource identity matching only. Never dereference/AddRef or send
   * to UI/logs; no resource lifetime is acquired.
   */
  std::uint64_t resource_address = 0;
  float fov = 0;
  /*
   * Numeric diagnostics only, published with the complete ready snapshot. P+16/+20, P+24/+28, P+32/+36,
   * respectively. The captured setup copies primary-view dimensions into these pairs; the third is used for
   * output allocation. These observations are not a resource description or setters.
   */
  std::array<std::array<std::int32_t, 2>, 3> dimensions{};
  /*
   * Slot0 Bitmap+40/+44 metadata, read only when a nonnull resource member is present, and published only
   * after the complete trace reread. Zero when output is unavailable. Signed values are observations, not
   * size validation or a native GPU resource description; callers must check desired bounds.
   */
  std::array<std::int32_t, 2> output_dimensions{};
  /*
   * Exact P+48/P+56 flag words. Preserve as integers/hexadecimal strings in consumers; no guessed meanings,
   * masking, writes or floating conversion.
   */
  std::array<std::uint64_t, 2> flags{};
  /*
   * Only a nonnull pointer-shaped resource member was observed. No COM method, interface check, AddRef,
   * description, resource state or GPU data is inspected.
   */
  bool resource_present = false;
};

/*
 * Separate authority for an owned mode2 gate closure only. Never convertible to OwnedViewSnapshot: no
 * Camera, pose or output-resource proof is supplied. Every retained field, including both Node handles, is
 * exactly reread before complete becomes true. Pending/unavailable/mismatching results publish no borrowed
 * address. Current full close trace:16 fields,32 reads,258 bytes.
 */
struct OwnedViewCloseSnapshot {
  bool complete = false;
  OwnedViewStatus status = OwnedViewStatus::not_inspected;
  const char* error = "";
  std::uint32_t read_bytes = 0, read_failures = 0;
  std::int32_t view_index = -1;
  /*
   * Borrowed only in this observer phase, for verifying the close's flag write.
   */
  std::uint64_t view_address = 0;
  std::array<std::array<std::int32_t, 2>, 3> dimensions{};
  std::array<std::uint64_t, 2> flags{};
};

/*
 * Caller retains the full current manager/table/pool/lifetime guards. This only admits E8==1/E24==2,
 * matching E0/E16, a current pool index and equal nonnull generation-resolved E96/P104 Node identities. It
 * does not dereference Node, Camera, material or Bitmap. No native call, publication or ownership acquired.
 */
OwnedViewCloseSnapshot inspect_owned_view_for_close(MemoryReader& reader,
                                                    std::uint64_t entry_address,
                                                    std::uint64_t expected_id,
                                                    const ViewPoolSnapshot& pool) noexcept;
/*
 * Fixed captured layout, not a general engine ABI. The caller must supply a freshly verified owned entry and
 * complete pool from the same renderer/update phase. Check E+0/+16 IDs, E+8 ready, E+76 pool index and its
 * current array slot. Resolve generation-checked E+96/P+104 to one Node; Node+256 must point to a Camera
 * with WORD+160=7 and finite positive float+1616. Optional output observation resolves E+80/P+144 to
 * matching material payloads, then Material+520 bitmap handle -> Bitmap+88 record -> record+16 wrapper ->
 * wrapper+168 resource member. Null/stale output references report no resource;
 * malformed/unreadable/mismatched or changing fields refuse the entire result. No output is published on
 * failure, including failures in this optional chain. Every observed byte is reread, but this does not make
 * a snapshot atomic or acquire references. No engine calls, global reads or payload virtual calls. Reader
 * owns accessible-region/lifetime/build checks and must not throw. For Node, material and bitmap handles,
 * control-record +28/+0 reads alone permit byte alignment. No low bits are masked or rounded. Entry, view,
 * payload, record, wrapper and resource pointers retain eight-byte alignment;
 * null/generation/range/region/budget/full-trace checks remain required. The three dimension pairs and two
 * flag words above are included in that trace. Zero, negative or unusual dimensions remain diagnostic
 * observations; this reader does not infer validity limits or silently resize a view.
 */
OwnedViewSnapshot inspect_owned_view(MemoryReader& reader,
                                     std::uint64_t entry_address,
                                     std::uint64_t expected_id,
                                     const ViewPoolSnapshot& pool) noexcept;

}  /* namespace azp::engine_camera */

namespace azp::native_camera {

enum class SourceViewStatus {
  not_inspected,
  ready,
  no_source,
  invalid_pool,
  invalid_pointer,
  pool_changed,
  read_failed,
  changed,
  read_limit
};

struct SourceViewSnapshot {
  bool complete = false; /* True only after a usable source and complete trace reread. */
  SourceViewStatus status = SourceViewStatus::not_inspected;
  const char* error = "";
  /*
   * Semantic field name only; no address or pointed-to data is logged.
   */
  const char* field = "";
  std::uint8_t rejected_alignment = 0; /* Low three bits only, never an address. */
  std::uint32_t read_bytes = 0;
  std::uint32_t read_failures = 0;
  std::uint32_t candidates_examined = 0;
  std::int32_t view_index = -1;
  /*
   * Borrowed internal addresses, never log/serialize or retain across the caller's proven update phase. No
   * address is published on any failure.
   */
  std::uint64_t source_address = 0;
  std::uint64_t node_address = 0;
  std::uint64_t camera_address = 0;
  float fov = 0;
};

/*
 * PRECREATION ONLY: caller guarantees its owned view does not yet exist, and supplies a fresh pool from the
 * same validated renderer/update phase. There is no owned-ID list here, so this helper cannot independently
 * exclude an existing owned view. Choose the first occupied pool slot with a valid camera pose. Recheck
 * current array slot; resolve P+104 handle (control+28 generation/+0 payload) to Node. Require Node+256
 * Camera WORD+160=7, finite positive float Camera+1616, Node+296 matrix pointer with finite translation
 * doubles+96..119, and finite 3x3 Camera doubles at +1648/+1680/+1712. Norm-squared and pairwise dot
 * products must be within 1e-3 of an orthonormal basis. No pose conversion. Stable absent/unusable
 * candidates are skipped; malformed/unreadable/changing fields refuse all output. Reread every observation,
 * including skipped views. Maximum eight candidates, 8192 attempted bytes (current worst case 2464). No
 * global reads, engine/COM calls, acquired references or scheduling claims. Reader owns
 * readable-region/build/lifetime checks and must not throw. Control-record +28/+0 reads alone permit byte
 * alignment, reproducing the captured MOV accesses. The exact pointer is used without clearing low bits;
 * null/overflow, region, byte-budget and trace checks still apply. Other object pointers keep their
 * eight-byte alignment guard. This admission by itself does not establish a valid live handle or a
 * tagged-pointer interpretation.
 */
SourceViewSnapshot inspect_source_view(engine_camera::MemoryReader& reader, const engine_camera::ViewPoolSnapshot& pool) noexcept;

/*
 * Same validation for one independently verified source object with its Node handle at +104. Does not read a
 * pool or infer the source object's identity. Caller must establish the exact source lifetime/layout before
 * access. A successful result describes the captured current view, not an aircraft body pose or taxi-camera
 * geometry. view_index remains -1. Maximum 292 read bytes. Optional numeric output uses the already observed
 * translation and adds no reads. It is cleared first and published only after the full trace recheck.
 */
SourceViewSnapshot inspect_source_pose(engine_camera::MemoryReader& reader,
                                       std::uint64_t source_with_node_handle_at104,
                                       std::array<double, 3>* validated_position = nullptr) noexcept;

}  /* namespace azp::native_camera */

namespace azp::native_camera {

struct ActivationMaskInventory {
  bool valid = false;
  std::string error;
  std::uint32_t read_bytes = 0;
  std::uint32_t read_failures = 0;
  /*
   * Semantic flag words, never pointers or generic raw-byte output. The first observation remains diagnostic
   * only unless valid is true.
   */
  std::array<std::uint64_t, 2> words{};
};

/*
 * Read the 16-byte flag pair at the resolved layout's activation-disable RVA from declared readable,
 * nonwritable, nonexecutable, nondiscardable image data. Require the complete pair {1,0} and an identical
 * full reread (32B maximum). Caller separately verifies the current code contract before using the native
 * activation(false) call; this helper performs no calls or memory writes and cannot guarantee future
 * immutability or an engine scheduling contract.
 */
ActivationMaskInventory inspect_activation_disable_mask(discovery::ImageReader& reader,
                                                        const discovery::Inventory& image,
                                                        const CameraImageLayout& layout);

}  /* namespace azp::native_camera */

namespace azp::native_camera {

inline constexpr std::uint64_t kViewAaFlag = std::uint64_t{1} << 31;

struct ViewAaResult {
  bool complete = false;
  bool write_attempted = false;
  const char* error = "not_attempted";
};

/*
 * Only a freshly verified owned mode2 view, in the current validated engine observer. The caller validates
 * the current code contract proving ToggleVpEffectAA's P+48 bit31 and the two global override locations.
 * This clears that one per-view bit with the gate closed, preserving every other bit and P+56. An
 * already-clear active view is validated without any write. No engine command, global write, allocation,
 * protection change or AA SDK call. Rereads both flag words and overrides; the caller closes the gate on
 * failure. A write_attempted result requires a fresh full owned-view inspection before activation, including
 * on success. This helper cannot establish ownership or freeze engine lifetime independently of its caller's
 * observer contract.
 */
ViewAaResult disable_owned_view_aa(const engine_camera::OwnedViewSnapshot& view,
                                   discovery::ImageReader& image,
                                   const CameraImageLayout& layout) noexcept;

}  /* namespace azp::native_camera */

namespace azp::native_camera {

using ViewDimensions = std::array<std::array<std::int32_t, 2>, 3>;

/*
 * Render, display and output dimensions are validated independently. The owned camera requests the square
 * native resolution `side` (shared/configuration.hpp) for all three.
 */
bool plan_view_resize(const ViewDimensions& inherited, unsigned side, ViewDimensions& desired) noexcept;

/*
 * The original manager returns before initializing its primary-size cache when it has no entries. Newly
 * created views must survive one ORIGINAL update with closed gates before resizing, or that update can
 * overwrite their dimensions. This gate uses the observer iteration counter, not a timer or guessed cache
 * write. The caller must keep the gate closed while pending() is true.
 */
class ViewResizeWarmup {
 public:
  bool begin(std::uint64_t id, std::uint64_t iteration) noexcept {
    if (!id || !iteration)
      return false;
    id_ = id;
    iteration_ = iteration;
    pending_ = true;
    return true;
  }
  bool may_resize(std::uint64_t id, std::uint64_t iteration) const noexcept { return pending_ && id == id_ && iteration > iteration_; }
  bool finish(std::uint64_t id, std::uint64_t iteration) noexcept {
    if (!may_resize(id, iteration))
      return false;
    pending_ = false;
    return true;
  }
  bool pending() const noexcept { return pending_; }
  void clear() noexcept { *this = {}; }

 private:
  std::uint64_t id_ = 0;
  std::uint64_t iteration_ = 0;
  bool pending_ = false;
};

enum class ViewResizeStatus {
  not_attempted,
  resized,
  dimensions_restored,
  output_mismatch,
  unchanged,
  invalid_snapshot,
  invalid_dimensions,
  gate_open,
  invalid_mapping,
  changed,
  read_failed,
  write_failed,
  refresh_failed,
  allocation_failed,
};

struct ViewResizeResult {
  ViewResizeStatus status = ViewResizeStatus::not_attempted;
  /*
   * A failed/partial write is also a mutation attempt. Caller must retain its owned ID and keep the gate
   * closed for normal engine cleanup on ANY failure.
   */
  bool write_attempted = false;
  bool complete = false;
};

struct ViewResizeCallbacks {
  void* context = nullptr;
  bool (*refresh_projection)(void*, std::uint64_t view_address) noexcept = nullptr;
  std::uint64_t (*ensure_output)(void*, std::uint64_t view_address) noexcept = nullptr;
};

/*
 * Caller supplies a freshly validated owned view in the same engine update phase, with its render gate
 * closed. Only the exact 24 bytes P+16..39 can be written; current dimensions and P+48/+56 must still match
 * the snapshot. Requires committed PAGE_READWRITE MEM_PRIVATE memory and makes no protection change. The
 * write is immediately reread before calling projection refresh, then the captured output routine. Its
 * return must equal P+144. This does not acquire object ownership or prove completed GPU allocation. Callers
 * must revalidate the complete owned-view chain afterwards. Native callbacks must have their build
 * fingerprints/ABI and engine phase verified. No calls occur for refused inputs; failed callbacks never
 * reopen the gate.
 */
ViewResizeResult resize_owned_view(const engine_camera::OwnedViewSnapshot& view,
                                   const ViewDimensions& desired,
                                   const ViewResizeCallbacks& callbacks) noexcept;

/*
 * Established mode2 views can have their three size fields overwritten when the primary view changes.
 * Restore only when the fully reread existing Bitmap already has the desired dimensions. No output
 * allocation/replacement occurs; ensure_output is never called. Caller keeps the gate closed, retains the
 * entry ID, and revalidates the complete chain/resource after this operation.
 */
ViewResizeResult restore_owned_view_dimensions(const engine_camera::OwnedViewSnapshot& view,
                                               const ViewDimensions& desired,
                                               const ViewResizeCallbacks& callbacks) noexcept;

const char* view_resize_status_name(ViewResizeStatus status) noexcept;

/*
 * Copy both far limits from a verified main view, preserving our near plane. Requires a closed, freshly
 * inspected mode-2 view; revalidate before reopening. Camera+1520/+1524/+1528 are covered by the update_view
 * contract. Returns null after readback and projection refresh succeed.
 */
const char* set_owned_view_far_planes(const engine_camera::OwnedViewSnapshot& view,
                                      const std::array<float, 2>& far_planes,
                                      const ViewResizeCallbacks& callbacks) noexcept;

}  /* namespace azp::native_camera */
