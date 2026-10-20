/*
 * Everything the DLL knows about the engine's camera view objects: the view pool, the manager's entry
 * table, one owned view and its output image, the pilot's source view, and the few guarded mutations it
 * performs on its own view (resize, far planes, anti-aliasing flag, render-gate mask check).
 *
 * Every inspection follows a fixed, build-specific object layout with a bounded "traced reader": each field
 * is read once, recorded, and the whole trace is reread before any result is published. A failed or changing
 * read publishes nothing. Mutations require a freshly inspected view with its render gate closed and are
 * reread afterwards. Nothing here calls the engine except through callbacks supplied by camera.cpp.
 */

#include "view.hpp"
#include <windows.h>
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>

namespace azp {
namespace {
/*
 * Shared bounds and little-endian decoding for every traced reader below. A pointer must be nonzero and the
 * addressed range must not overflow; object pointers must also be eight-byte aligned, while generation
 * control records may be byte aligned (require_aligned = false).
 */
bool pointer_range(std::uint64_t address, std::uint64_t offset, std::uint32_t size, bool require_aligned = true) noexcept {
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  return address != 0 && (!require_aligned || address % 8 == 0) && offset <= maximum - address && size <= maximum - (address + offset);
}

std::uint32_t u32(const std::uint8_t* bytes) noexcept {
  std::uint32_t value = 0;
  for (unsigned index = 0; index < 4; ++index)
    value |= std::uint32_t(bytes[index]) << (index * 8);
  return value;
}

std::uint64_t u64(const std::uint8_t* bytes) noexcept {
  return std::uint64_t(u32(bytes)) | (std::uint64_t(u32(bytes + 4)) << 32);
}
}  /* namespace */
}  /* namespace azp */

/* ---- View pool -------------------------------------------------------------------------------------- */

namespace azp::engine_camera {
namespace {

constexpr std::uint32_t kPoolReadBudget = 8192;
constexpr std::uint64_t kArrayOffset = 2752;
constexpr std::uint64_t kAssociationOffset = 72;
constexpr std::size_t kPoolObservations = 1 + 8 * 4;

struct PoolObservation {
  std::uint64_t address = 0;
  std::uint32_t size = 0;
  std::int32_t slot = -1;
  std::array<std::uint8_t, 16> bytes{};
};

class PoolReader {
 public:
  PoolReader(MemoryReader& reader, ViewPoolSnapshot& result) noexcept : reader_(reader), result_(result) {}

  /*
   * Reads one pool metadata field and stores its bytes as endpoint evidence.
   */
  bool capture(std::uint64_t address, std::uint32_t size, std::uint8_t* output) noexcept {
    if (count_ == observations_.size()) {
      result_.status = ViewPoolStatus::read_budget_exhausted;
      return false;
    }
    if (!read(address, size, output))
      return false;
    auto& observation = observations_[count_++];
    observation.address = address;
    observation.size = size;
    observation.slot = result_.failure_slot;
    std::copy_n(output, size, observation.bytes.begin());
    return true;
  }

  /*
   * Rereads every captured pool field before the pool snapshot can be accepted.
   */
  bool recheck() noexcept {
    for (std::size_t index = 0; index < count_; ++index) {
      const auto& observation = observations_[index];
      result_.failure_slot = observation.slot;
      std::array<std::uint8_t, 16> current{};
      if (!read(observation.address, observation.size, current.data()))
        return false;
      if (!std::equal(current.begin(), current.begin() + observation.size, observation.bytes.begin())) {
        result_.status = ViewPoolStatus::changed;
        return false;
      }
    }
    return true;
  }

 private:
  /*
   * Reads a supported 4-, 8- or 16-byte metadata field through the supplied reader and charges the pool
   * inspection budget.
   */
  bool read(std::uint64_t address, std::uint32_t size, std::uint8_t* output) noexcept {
    if ((size != 4 && size != 8 && size != 16) || size > kPoolReadBudget - result_.read_bytes) {
      result_.status = ViewPoolStatus::read_budget_exhausted;
      return false;
    }
    result_.read_bytes += size;
    if (reader_.read(address, output, size))
      return true;
    result_.status = ViewPoolStatus::read_failed;
    ++result_.read_failures;
    return false;
  }

  MemoryReader& reader_;
  ViewPoolSnapshot& result_;
  std::array<PoolObservation, kPoolObservations> observations_{};
  std::size_t count_ = 0;
};

}  /* namespace */

/*
 * Validates pool containers, sizes and generation records, then rereads their metadata. Packed
 * control-record addresses remain exact rather than being aligned or masked heuristically.
 *
 * Generation records may be packed at byte alignment. Captured native MOVs use this exact pointer; never
 * mask or round its low bits.
 */
ViewPoolSnapshot inspect_view_pool(MemoryReader& reader, std::uint64_t renderer) noexcept {
  ViewPoolSnapshot result;
  for (std::uint32_t index = 0; index < result.slots.size(); ++index)
    result.slots[index].index = index;
  if (!pointer_range(renderer, kArrayOffset, 8)) {
    result.status = ViewPoolStatus::invalid_renderer;
    return result;
  }
  PoolReader source(reader, result);
  std::array<std::uint8_t, 16> bytes{};
  if (!source.capture(renderer + kArrayOffset, 8, bytes.data()))
    return result;
  result.array_address = u64(bytes.data());
  if (!pointer_range(result.array_address, 0, result.slots.size() * 8)) {
    result.status = ViewPoolStatus::invalid_array;
    return result;
  }
  for (std::uint32_t index = 0; index < result.slots.size(); ++index) {
    result.failure_slot = static_cast<std::int32_t>(index);
    auto& slot = result.slots[index];
    if (!source.capture(result.array_address + index * 8, 8, bytes.data()))
      return result;
    slot.view_address = u64(bytes.data());
    if (!pointer_range(slot.view_address, kAssociationOffset, 16)) {
      result.status = ViewPoolStatus::invalid_view;
      return result;
    }
    for (std::uint32_t previous = 0; previous < index; ++previous) {
      if (slot.view_address == result.slots[previous].view_address) {
        result.status = ViewPoolStatus::duplicate_view;
        return result;
      }
    }
    if (!source.capture(slot.view_address + kAssociationOffset, 16, bytes.data()))
      return result;
    const auto control = u64(bytes.data());
    const auto generation = u32(bytes.data() + 8);
    if (control == 0) {
      slot.association = ViewAssociation::null_control;
    } else {
      if (!pointer_range(control, 28, 4, false)) {
        result.status = ViewPoolStatus::invalid_control;
        return result;
      }
      if (!source.capture(control + 28, 4, bytes.data()))
        return result;
      if (u32(bytes.data()) != generation) {
        slot.association = ViewAssociation::stale_generation;
      } else {
        if (!source.capture(control, 8, bytes.data()))
          return result;
        slot.association = u64(bytes.data()) == 0 ? ViewAssociation::null_payload : ViewAssociation::occupied;
      }
    }
    slot.association_valid = slot.association == ViewAssociation::occupied;
    slot.free = !slot.association_valid;
    ++result.slots_examined;
  }
  if (!source.recheck())
    return result;
  for (const auto& slot : result.slots) {
    if (!slot.free)
      continue;
    if (result.free_count < result.first_free_indices.size())
      result.first_free_indices[result.free_count] = static_cast<std::int32_t>(slot.index);
    ++result.free_count;
  }
  result.failure_slot = -1;
  result.valid = true;
  result.status = ViewPoolStatus::complete;
  return result;
}

}  /* namespace azp::engine_camera */

/* ---- Owned entry table ------------------------------------------------------------------------------ */

namespace azp::engine_camera {
namespace {

bool aligned(std::uint64_t address) {
  return address != 0 && (address & 7) == 0;
}

struct EntryNode {
  std::uint64_t address = 0;
  std::uint64_t key = 0;
  std::uint64_t payload_id = 0;
  std::uint64_t next = 0; /* Retain the tag for exact consistency comparisons. */
};

class EntryReader {
 public:
  EntryReader(MemoryReader& reader, OwnedEntryInventory& result) : reader_(reader), result_(result) {}

  /*
   * Reads one aligned, bounded manager-table field and preserves it for the final consistency check.
   */
  bool field(std::uint64_t address, std::uint64_t offset, void* destination, std::uint32_t size) {
    constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
    if (!aligned(address) || offset > maximum - address || size > maximum - (address + offset)) {
      result_.error = "An entry-table field has a null, misaligned or overflowing address.";
      return false;
    }
    if (size > kOwnedEntryReadLimit - result_.read_bytes) {
      result_.error = "The bounded entry-table read allowance was exhausted.";
      return false;
    }
    result_.read_bytes += size;
    if (!reader_.read(address + offset, destination, size)) {
      ++result_.read_failures;
      result_.error = "A required entry-table field could not be read exactly.";
      return false;
    }
    return true;
  }

  bool word(std::uint64_t address, std::uint64_t offset, std::uint64_t& value) {
    std::array<std::uint8_t, 8> bytes{};
    if (!field(address, offset, bytes.data(), bytes.size()))
      return false;
    value = u64(bytes.data());
    return true;
  }

 private:
  MemoryReader& reader_;
  OwnedEntryInventory& result_;
};

}  /* namespace */

/*
 * Traverses the manager's bounded ID-table chains to find the requested owned entries. It rereads all
 * observed heads and nodes, including unrelated ones, before publishing ownership evidence.
 *
 * This is one contiguous metadata field. Read all declared heads at once, rather than issuing
 * VirtualQuery/ReadProcessMemory for every eight bytes. The same bytes and final full reread remain
 * required.
 *
 * Reread every observed word, including unowned nodes and all empty heads. This detects observed changes but
 * cannot make concurrent memory atomic.
 */
OwnedEntryInventory inspect_owned_entries(MemoryReader& reader, std::uint64_t manager_address, std::uint64_t owned_id) {
  OwnedEntryInventory result;

  EntryReader source(reader, result);
  std::array<std::uint8_t, 16> header{};
  if (!source.field(manager_address, 88, header.data(), header.size()))
    return result;
  result.entry_count = u32(header.data());
  result.bucket_count = u32(header.data() + 4);
  const auto buckets = u64(header.data() + 8);
  if (result.entry_count > kOwnedEntryNodeLimit || result.bucket_count > kOwnedEntryBucketLimit) {
    result.error = "The entry or bucket count exceeds the bounded verification limits.";
    return result;
  }
  if (result.bucket_count == 0) {
    if (result.entry_count != 0 || buckets != 0) {
      result.error = "The empty entry-table header is inconsistent.";
      return result;
    }
  } else if (!aligned(buckets)) {
    result.error = "A nonempty bucket array has a null or misaligned address.";
    return result;
  }

  std::array<std::uint64_t, kOwnedEntryBucketLimit> heads{};
  std::array<EntryNode, kOwnedEntryNodeLimit> nodes{};
  OwnedEntry found;
  if (result.bucket_count != 0 && !source.field(buckets, 0, heads.data(), result.bucket_count * 8))
    return result;
  for (std::uint32_t bucket = 0; bucket < result.bucket_count; ++bucket) {
    auto address = heads[bucket];
    while (address != 0) {
      if (!aligned(address)) {
        result.error = "A live bucket head or masked entry link is misaligned.";
        return result;
      }
      if (result.nodes_visited == kOwnedEntryNodeLimit) {
        result.error = "The bounded distinct-entry limit was exhausted.";
        return result;
      }
      for (std::uint32_t previous = 0; previous < result.nodes_visited; ++previous) {
        if (nodes[previous].address == address) {
          result.error = "A cycle or repeated live entry crosses the bucket traversal.";
          return result;
        }
      }
      EntryNode node;
      node.address = address;
      if (!source.word(address, 0, node.key) || !source.word(address, 16, node.payload_id) || !source.word(address, 296, node.next))
        return result;
      for (std::uint32_t previous = 0; previous < result.nodes_visited; ++previous) {
        if (nodes[previous].key == node.key) {
          result.error = "Duplicate live keys make the entry table inconsistent.";
          return result;
        }
      }
      if (owned_id != 0 && node.key == owned_id) {
        if (node.payload_id != owned_id) {
          result.error = "An owned lookup key disagrees with its entry payload ID.";
          return result;
        }
        found = {true, address, node.key, node.payload_id};
      }
      nodes[result.nodes_visited++] = node;
      if (result.nodes_visited > result.entry_count) {
        result.error = "More live entries were observed than the table declares.";
        return result;
      }
      address = node.next & ~std::uint64_t{1};
    }
  }
  if (result.nodes_visited != result.entry_count) {
    result.error = "The complete bucket traversal disagrees with the declared entry count.";
    return result;
  }

  for (std::uint32_t index = 0; index < result.nodes_visited; ++index) {
    const auto& node = nodes[index];
    EntryNode current;
    if (!source.word(node.address, 0, current.key) || !source.word(node.address, 16, current.payload_id) ||
        !source.word(node.address, 296, current.next))
      return result;
    if (current.key != node.key || current.payload_id != node.payload_id || current.next != node.next) {
      result.error = "An entry key, payload ID or tagged link changed during verification.";
      return result;
    }
  }
  std::array<std::uint64_t, kOwnedEntryBucketLimit> current_heads{};
  if (result.bucket_count != 0 && !source.field(buckets, 0, current_heads.data(), result.bucket_count * 8))
    return result;
  for (std::uint32_t bucket = 0; bucket < result.bucket_count; ++bucket) {
    if (current_heads[bucket] != heads[bucket]) {
      result.error = "A bucket head changed during verification.";
      return result;
    }
  }
  std::array<std::uint8_t, 16> current_header{};
  if (!source.field(manager_address, 88, current_header.data(), current_header.size()))
    return result;
  if (current_header != header) {
    result.error = "The manager's entry-table header changed during verification.";
    return result;
  }
  result.entry = found;
  result.complete = true;
  return result;
}

}  /* namespace azp::engine_camera */

/* ---- Owned view and its output image ---------------------------------------------------------------- */

namespace azp::engine_camera {
namespace {

constexpr std::uint32_t kOwnedViewReadBudget = 8192;

template <typename Result>
bool fail(Result& result, OwnedViewStatus status, const char* error) noexcept {
  result.status = status;
  result.error = error;
  return false;
}

struct OwnedViewObservation {
  std::uint64_t address = 0;
  std::uint32_t size = 0;
  std::array<std::uint8_t, 16> bytes{};
};

template <typename Result>
class OwnedViewReader {
 public:
  OwnedViewReader(MemoryReader& reader, Result& result) noexcept : reader_(reader), result_(result) {}

  /*
   * Reads a bounded view field and records its exact bytes for endpoint revalidation.
   */
  bool field(std::uint64_t address, std::uint64_t offset, std::uint32_t size, std::uint8_t* output, bool require_aligned = true) noexcept {
    if (!pointer_range(address, offset, size, require_aligned))
      return fail(result_, OwnedViewStatus::invalid_pointer, "A required field has a null, misaligned or overflowing pointer.");
    if (count_ == observations_.size())
      return fail(result_, OwnedViewStatus::read_budget_exhausted, "The bounded observation count was exhausted.");
    if (!read(address + offset, size, output))
      return false;
    auto& observation = observations_[count_++];
    observation.address = address + offset;
    observation.size = size;
    std::copy_n(output, size, observation.bytes.begin());
    return true;
  }

  bool word(std::uint64_t address, std::uint64_t offset, std::uint64_t& output, bool require_aligned = true) noexcept {
    std::array<std::uint8_t, 8> bytes{};
    if (!field(address, offset, bytes.size(), bytes.data(), require_aligned))
      return false;
    output = u64(bytes.data());
    return true;
  }

  /*
   * Resolves the reviewed generation handle without modifying its control-record address. Generation and
   * payload checks distinguish a current object from a reused pointer.
   *
   * The verified generation resolver uses ordinary MOV loads. Controls alone may be byte aligned: preserve
   * their exact addresses, including low bits.
   *
   * Null/stale/null-payload references are available observations with output=0. A nonzero payload is
   * range-checked without dereferencing its contents here.
   */
  bool handle(std::uint64_t address, std::uint64_t offset, std::uint64_t& output) noexcept {
    output = 0;
    std::array<std::uint8_t, 16> bytes{};
    if (!field(address, offset, bytes.size(), bytes.data()))
      return false;
    const auto control = u64(bytes.data());
    const auto generation = u32(bytes.data() + 8);
    if (control == 0)
      return true;
    if (!field(control, 28, 4, bytes.data(), false))
      return false;
    if (u32(bytes.data()) != generation)
      return true;
    if (!word(control, 0, output, false))
      return false;
    if (output != 0 && !pointer_range(output, 0, 8))
      return fail(result_, OwnedViewStatus::invalid_pointer, "A generation-valid handle contains a malformed payload pointer.");
    return true;
  }

  /*
   * Rereads every recorded view-chain field and rejects the result if any identity or value changed during
   * inspection.
   */
  bool recheck() noexcept {
    for (std::size_t index = 0; index < count_; ++index) {
      const auto& observation = observations_[index];
      std::array<std::uint8_t, 16> bytes{};
      if (!read(observation.address, observation.size, bytes.data()))
        return false;
      if (!std::equal(bytes.begin(), bytes.begin() + observation.size, observation.bytes.begin()))
        return fail(result_, OwnedViewStatus::changed, "An observed field changed during the complete trace recheck.");
    }
    return true;
  }

 private:
  bool read(std::uint64_t address, std::uint32_t size, std::uint8_t* output) noexcept {
    if (size == 0 || size > 16 || size > kOwnedViewReadBudget - result_.read_bytes)
      return fail(result_, OwnedViewStatus::read_budget_exhausted, "The bounded attempted-read allowance was exhausted.");
    result_.read_bytes += size;
    if (reader_.read(address, output, size))
      return true;
    ++result_.read_failures;
    return fail(result_, OwnedViewStatus::read_failed, "A required field could not be read exactly.");
  }

  MemoryReader& reader_;
  Result& result_;
  std::array<OwnedViewObservation, 32> observations_{};
  std::size_t count_ = 0;
};

/*
 * Checks the supplied pool snapshot and its ranges before resolving an owned entry through it.
 */
bool valid_owned_view_pool(const ViewPoolSnapshot& pool) noexcept {
  if (!pool.valid || pool.status != ViewPoolStatus::complete || pool.slots_examined != 8 || pool.read_failures != 0 ||
      !pointer_range(pool.array_address, 0, 64))
    return false;
  for (std::uint32_t index = 0; index < pool.slots.size(); ++index) {
    const auto& slot = pool.slots[index];
    if (slot.index != index || !pointer_range(slot.view_address, 0, 8))
      return false;
    for (std::uint32_t previous = 0; previous < index; ++previous) {
      if (slot.view_address == pool.slots[previous].view_address)
        return false;
    }
  }
  return true;
}

/*
 * Follows the validated material/bitmap chain to an opaque output-resource identity and its dimensions.
 * Dimensions alone do not prove completed GPU allocation.
 *
 * The reviewed native output update compares Bitmap40/44 with view32/36 before its output replacement
 * branch. Observe these exact fields, never dereference the opaque resource member or infer completed
 * allocation from these values.
 */
template <typename Result>
bool material_resource(OwnedViewReader<Result>& source,
                       Result& result,
                       std::uint64_t material,
                       std::uint32_t slot,
                       std::uint64_t& address,
                       std::array<std::int32_t, 2>& output_dimensions) noexcept {
  if (material == 0)
    return true;
  std::uint64_t bitmap = 0;
  if (!source.handle(material, 520 + std::uint64_t(slot) * 16, bitmap))
    return false;
  if (bitmap == 0)
    return true;
  std::uint64_t record = 0;
  if (!source.word(bitmap, 88, record))
    return false;
  if (record == 0)
    return true;
  std::uint64_t wrapper = 0;
  if (!source.word(record, 16, wrapper))
    return false;
  if (wrapper == 0)
    return true;
  std::uint64_t resource = 0;
  if (!source.word(wrapper, 168, resource))
    return false;
  if (resource != 0 && !pointer_range(resource, 0, 8))
    return fail(result, OwnedViewStatus::invalid_pointer, "The optional resource member is malformed.");
  if (resource != 0) {
    std::array<std::uint8_t, 8> bytes{};
    if (!source.field(bitmap, 40, bytes.size(), bytes.data()))
      return false;
    output_dimensions = {std::bit_cast<std::int32_t>(u32(bytes.data())), std::bit_cast<std::int32_t>(u32(bytes.data() + 4))};
  }
  address = resource;
  return true;
}

bool output_resource(OwnedViewReader<OwnedViewSnapshot>& source,
                     OwnedViewSnapshot& result,
                     std::uint64_t entry,
                     std::uint64_t view,
                     std::uint64_t& address,
                     std::array<std::int32_t, 2>& output_dimensions) noexcept {
  std::uint64_t entry_material = 0;
  std::uint64_t view_material = 0;
  if (!source.handle(entry, 80, entry_material) || !source.handle(view, 144, view_material))
    return false;
  if (entry_material != view_material)
    return fail(result, OwnedViewStatus::material_mismatch, "The entry and view do not resolve to the same output material.");
  return material_resource(source, result, entry_material, 0, address, output_dimensions);
}

}  /* namespace */

/*
 * Builds the minimal freshly validated chain needed to close the owned render gate. Missing image outputs do
 * not prevent closure, but no output ticket or erase permission is inferred.
 */
OwnedViewCloseSnapshot inspect_owned_view_for_close(MemoryReader& reader,
                                                    std::uint64_t entry_address,
                                                    std::uint64_t expected_id,
                                                    const ViewPoolSnapshot& pool) noexcept {
  OwnedViewCloseSnapshot result;
  if (!expected_id || !pointer_range(entry_address, 0, 8)) {
    fail(result, OwnedViewStatus::invalid_request, "Gate closure requires a nonzero owned ID and aligned entry.");
    return result;
  }
  if (!valid_owned_view_pool(pool)) {
    fail(result, OwnedViewStatus::invalid_pool, "Gate closure requires the complete current pool proof.");
    return result;
  }
  OwnedViewReader source(reader, result);
  std::uint64_t key = 0, payload_id = 0;
  std::array<std::uint8_t, 16> bytes{};
  if (!source.word(entry_address, 0, key) || !source.word(entry_address, 16, payload_id))
    return result;
  if (key != expected_id || payload_id != expected_id) {
    fail(result, OwnedViewStatus::id_mismatch, "The entry key or payload ID changed before closure.");
    return result;
  }
  if (!source.field(entry_address, 8, 1, bytes.data()))
    return result;
  if (bytes[0] != 1) {
    fail(result, bytes[0] == 0 ? OwnedViewStatus::pending : OwnedViewStatus::invalid_ready_byte,
         "Gate closure requires a ready owned entry.");
    return result;
  }
  if (!source.field(entry_address, 24, 4, bytes.data()))
    return result;
  if (u32(bytes.data()) != 2) {
    fail(result, OwnedViewStatus::invalid_request, "Gate closure requires an independent-pose mode2 entry.");
    return result;
  }
  if (!source.field(entry_address, 76, 4, bytes.data()))
    return result;
  const auto index = std::bit_cast<std::int32_t>(u32(bytes.data()));
  if (index < 0 || index >= 8) {
    fail(result, OwnedViewStatus::invalid_view_index, "The entry index is outside the captured eight-view pool.");
    return result;
  }
  std::uint64_t view = 0;
  if (!source.word(pool.array_address, std::uint64_t(index) * 8, view))
    return result;
  if (view != pool.slots[index].view_address) {
    fail(result, OwnedViewStatus::pool_changed, "The selected view changed since the complete pool inspection.");
    return result;
  }
  std::uint64_t node = 0, view_node = 0;
  if (!source.handle(entry_address, 96, node) || !source.handle(view, 104, view_node))
    return result;
  if (!node || !view_node) {
    fail(result, OwnedViewStatus::node_unavailable, "The owned entry/view association is unavailable.");
    return result;
  }
  if (node != view_node) {
    fail(result, OwnedViewStatus::node_mismatch, "The entry and view Node identities differ.");
    return result;
  }
  std::array<std::array<std::int32_t, 2>, 3> dimensions{};
  for (std::uint32_t pair = 0; pair < dimensions.size(); ++pair) {
    if (!source.field(view, 16 + pair * 8, 8, bytes.data()))
      return result;
    dimensions[pair] = {std::bit_cast<std::int32_t>(u32(bytes.data())), std::bit_cast<std::int32_t>(u32(bytes.data() + 4))};
  }
  if (!source.field(view, 48, 16, bytes.data()))
    return result;
  const std::array<std::uint64_t, 2> flags{u64(bytes.data()), u64(bytes.data() + 8)};
  if (!source.recheck())
    return result;
  result.complete = true;
  result.status = OwnedViewStatus::ready;
  result.view_index = index;
  result.view_address = view;
  result.dimensions = dimensions;
  result.flags = flags;
  return result;
}
/*
 * Validates entry, world, node, view flags, camera fields and output identities against the pool/layout
 * evidence. It returns distinct pending/readiness/dimension failures and publishes results only after a
 * consistent final reread.
 */
OwnedViewSnapshot inspect_owned_view(MemoryReader& reader,
                                     std::uint64_t entry_address,
                                     std::uint64_t expected_id,
                                     const ViewPoolSnapshot& pool) noexcept {
  OwnedViewSnapshot result;
  if (expected_id == 0 || !pointer_range(entry_address, 0, 8)) {
    fail(result, OwnedViewStatus::invalid_request, "Owned-entry inspection requires a nonzero ID and aligned entry address.");
    return result;
  }
  if (!valid_owned_view_pool(pool)) {
    fail(result, OwnedViewStatus::invalid_pool, "The supplied pool is not a complete, bounded snapshot of eight distinct views.");
    return result;
  }
  OwnedViewReader source(reader, result);
  std::uint64_t key = 0;
  std::uint64_t payload_id = 0;
  std::array<std::uint8_t, 16> bytes{};
  if (!source.word(entry_address, 0, key) || !source.word(entry_address, 16, payload_id))
    return result;
  if (key != expected_id || payload_id != expected_id) {
    fail(result, OwnedViewStatus::id_mismatch, "The entry key or payload ID differs from the expected owned ID.");
    return result;
  }
  if (!source.field(entry_address, 8, 1, bytes.data()))
    return result;
  if (bytes[0] == 0) {
    if (source.recheck()) {
      result.complete = true;
      result.status = OwnedViewStatus::pending;
    }
    return result;
  }
  if (bytes[0] != 1) {
    fail(result, OwnedViewStatus::invalid_ready_byte, "The entry ready byte is neither zero nor one.");
    return result;
  }
  if (!source.field(entry_address, 24, 4, bytes.data()))
    return result;
  const auto mode = u32(bytes.data());
  if (!source.field(entry_address, 76, 4, bytes.data()))
    return result;
  const auto index = std::bit_cast<std::int32_t>(u32(bytes.data()));
  if (index < 0 || index >= 8) {
    fail(result, OwnedViewStatus::invalid_view_index, "The ready entry does not identify one of the eight pool views.");
    return result;
  }
  std::uint64_t view = 0;
  if (!source.word(pool.array_address, std::uint64_t(index) * 8, view))
    return result;
  if (view != pool.slots[index].view_address) {
    fail(result, OwnedViewStatus::pool_changed, "The selected pool pointer no longer matches the supplied snapshot.");
    return result;
  }
  std::uint64_t node = 0;
  std::uint64_t view_node = 0;
  if (!source.handle(entry_address, 96, node) || !source.handle(view, 104, view_node))
    return result;
  if (node == 0 || view_node == 0) {
    fail(result, OwnedViewStatus::node_unavailable, "A ready entry has an unavailable entry or view Node reference.");
    return result;
  }
  if (node != view_node) {
    fail(result, OwnedViewStatus::node_mismatch, "The entry and view Node references resolve to different objects.");
    return result;
  }
  std::uint64_t camera = 0;
  if (!source.word(node, 256, camera) || !source.field(camera, 160, 2, bytes.data()))
    return result;
  if ((std::uint32_t(bytes[0]) | (std::uint32_t(bytes[1]) << 8)) != 7) {
    fail(result, OwnedViewStatus::wrong_camera_type, "The Node payload does not have the captured Camera type value seven.");
    return result;
  }
  if (!source.field(camera, 1616, 4, bytes.data()))
    return result;
  const float fov = std::bit_cast<float>(u32(bytes.data()));
  if (!std::isfinite(fov) || fov <= 0) {
    fail(result, OwnedViewStatus::invalid_fov, "The Camera FOV is not finite and positive.");
    return result;
  }
  std::array<std::array<std::int32_t, 2>, 3> dimensions{};
  for (std::uint32_t pair = 0; pair < dimensions.size(); ++pair) {
    if (!source.field(view, 16 + pair * 8, 8, bytes.data()))
      return result;
    dimensions[pair] = {std::bit_cast<std::int32_t>(u32(bytes.data())), std::bit_cast<std::int32_t>(u32(bytes.data() + 4))};
  }
  if (!source.field(view, 48, 16, bytes.data()))
    return result;
  const std::array<std::uint64_t, 2> flags{u64(bytes.data()), u64(bytes.data() + 8)};
  std::uint64_t resource_address = 0;
  std::array<std::int32_t, 2> output_dimensions{};
  if (!output_resource(source, result, entry_address, view, resource_address, output_dimensions))
    return result;
  if (!source.recheck())
    return result;
  result.complete = true;
  result.ready = true;
  result.status = OwnedViewStatus::ready;
  result.view_index = index;
  result.mode = mode;
  result.view_address = view;
  result.node_address = node;
  result.camera_address = camera;
  result.fov = fov;
  result.dimensions = dimensions;
  result.output_dimensions = output_dimensions;
  result.flags = flags;
  result.resource_present = resource_address != 0;
  result.resource_address = resource_address;
  return result;
}

}  /* namespace azp::engine_camera */

/* ---- Source (pilot) view ---------------------------------------------------------------------------- */

namespace azp::native_camera {
namespace {
constexpr std::uint32_t kSourceViewReadLimit = 8192;
constexpr double kBasisTolerance = 1e-3;

bool fail(SourceViewSnapshot& result, SourceViewStatus status, const char* error) noexcept {
  result.status = status;
  result.error = error;
  return false;
}

struct SourceViewObservation {
  std::uint64_t address = 0;
  std::uint32_t size = 0;
  std::array<std::uint8_t, 24> bytes{};
  const char* field = "";
};

class SourceViewReader {
 public:
  SourceViewReader(engine_camera::MemoryReader& reader, SourceViewSnapshot& result) noexcept : reader_(reader), result_(result) {}

  /*
   * Reads and records a bounded native source-view field for the final consistency check.
   */
  bool field(std::uint64_t address,
             std::uint64_t offset,
             std::uint32_t size,
             std::uint8_t* output,
             const char* name,
             bool require_aligned = true) noexcept {
    result_.field = name;
    if (!pointer_range(address, offset, size, require_aligned)) {
      result_.rejected_alignment = static_cast<std::uint8_t>(address & 7);
      const char* reason = address == 0                          ? "The source-view field has a null base pointer."
                           : require_aligned && address % 8 != 0 ? "The source-view base is not aligned to eight bytes."
                                                                 : "The source-view field exceeds the pointer address range.";
      return fail(result_, SourceViewStatus::invalid_pointer, reason);
    }
    if (count_ == observations_.size())
      return fail(result_, SourceViewStatus::read_limit, "The source-view observation count was exhausted.");
    if (!read(address + offset, size, output))
      return false;
    auto& item = observations_[count_++];
    item.address = address + offset;
    item.size = size;
    item.field = name;
    std::copy_n(output, size, item.bytes.begin());
    return true;
  }

  bool word(std::uint64_t address, std::uint64_t offset, std::uint64_t& output, const char* name, bool require_aligned = true) noexcept {
    std::array<std::uint8_t, 8> bytes{};
    if (!field(address, offset, bytes.size(), bytes.data(), name, require_aligned))
      return false;
    output = u64(bytes.data());
    return true;
  }

  /*
   * Resolves a generation handle using its exact control-record address. Only reviewed control fields may be
   * byte aligned; payload alignment is validated separately.
   *
   * The captured MOV-based handle resolver imposes no control-record alignment. Read this exact address;
   * never mask, round or remove low bits. Only these control fields permit byte alignment. Payload objects
   * retain their separate alignment/range checks at the point they are followed.
   */
  bool node(std::uint64_t view, std::uint64_t& output) noexcept {
    output = 0;
    std::array<std::uint8_t, 16> bytes{};
    if (!field(view, 104, bytes.size(), bytes.data(), "source.node_handle"))
      return false;
    const auto control = u64(bytes.data());
    const auto generation = u32(bytes.data() + 8);
    if (control == 0)
      return true;
    if (!field(control, 28, 4, bytes.data(), "node_handle.generation", false))
      return false;
    if (u32(bytes.data()) != generation)
      return true;
    return word(control, 0, output, "node_handle.payload", false);
  }

  /*
   * Rereads the complete observed source-view chain before publishing any of its pose or projection data.
   */
  bool recheck() noexcept {
    for (std::size_t index = 0; index < count_; ++index) {
      const auto& item = observations_[index];
      result_.field = item.field;
      std::array<std::uint8_t, 24> bytes{};
      if (!read(item.address, item.size, bytes.data()))
        return false;
      if (!std::equal(bytes.begin(), bytes.begin() + item.size, item.bytes.begin()))
        return fail(result_, SourceViewStatus::changed, "A source-view field changed during the full consistency recheck.");
    }
    return true;
  }

 private:
  bool read(std::uint64_t address, std::uint32_t size, std::uint8_t* output) noexcept {
    if (size == 0 || size > 24 || size > kSourceViewReadLimit - result_.read_bytes)
      return fail(result_, SourceViewStatus::read_limit, "The source-view attempted-read allowance was exhausted.");
    result_.read_bytes += size;
    if (reader_.read(address, output, size))
      return true;
    ++result_.read_failures;
    return fail(result_, SourceViewStatus::read_failed, "A required source-view field could not be read exactly.");
  }

  engine_camera::MemoryReader& reader_;
  SourceViewSnapshot& result_;
  std::array<SourceViewObservation, 96> observations_{};
  std::size_t count_ = 0;
};

/*
 * Checks pool addresses and bounds before candidate view indices are followed.
 */
bool valid_source_pool(const engine_camera::ViewPoolSnapshot& pool) noexcept {
  using engine_camera::ViewAssociation;
  if (!pool.valid || pool.status != engine_camera::ViewPoolStatus::complete || pool.slots_examined != 8 || pool.read_failures != 0 ||
      !pointer_range(pool.array_address, 0, 64))
    return false;
  for (std::uint32_t index = 0; index < pool.slots.size(); ++index) {
    const auto& slot = pool.slots[index];
    if (slot.index != index || !pointer_range(slot.view_address, 0, 8) || slot.association == ViewAssociation::unobserved ||
        slot.association_valid != (slot.association == ViewAssociation::occupied) || slot.free == slot.association_valid)
      return false;
    for (std::uint32_t previous = 0; previous < index; ++previous) {
      if (slot.view_address == pool.slots[previous].view_address)
        return false;
    }
  }
  return true;
}

bool finite_vector(const std::array<std::uint8_t, 24>& bytes, std::array<double, 3>& values) noexcept {
  bool finite = true;
  for (unsigned index = 0; index < 3; ++index) {
    values[index] = std::bit_cast<double>(u64(bytes.data() + index * 8));
    finite &= std::isfinite(values[index]);
  }
  return finite;
}

/*
 * Checks finite axis lengths and pairwise perpendicularity within the accepted tolerance before a matrix
 * becomes a camera basis.
 */
bool orthonormal(const std::array<std::array<double, 3>, 3>& basis) noexcept {
  for (unsigned row = 0; row < 3; ++row) {
    for (unsigned other = row; other < 3; ++other) {
      double dot = 0;
      for (unsigned column = 0; column < 3; ++column)
        dot += basis[row][column] * basis[other][column];
      if (!std::isfinite(dot) || std::abs(dot - (row == other ? 1.0 : 0.0)) > kBasisTolerance)
        return false;
    }
  }
  return true;
}

struct Candidate {
  bool usable = false;
  std::uint64_t node = 0;
  std::uint64_t camera = 0;
  float fov = 0;
  std::array<double, 3> translation{};
};

/*
 * Resolves one source-view candidate and validates its flags, camera fields, finite projection and
 * orthonormal transform.
 */
bool inspect_candidate(SourceViewReader& source, std::uint64_t address, Candidate& candidate) noexcept {
  if (!source.node(address, candidate.node))
    return false;
  if (candidate.node == 0)
    return true;
  if (!source.word(candidate.node, 256, candidate.camera, "node.camera_pointer"))
    return false;
  if (candidate.camera == 0)
    return true;
  std::array<std::uint8_t, 24> bytes{};
  if (!source.field(candidate.camera, 160, 2, bytes.data(), "camera.type"))
    return false;
  if ((std::uint32_t(bytes[0]) | (std::uint32_t(bytes[1]) << 8)) != 7)
    return true;
  std::uint64_t matrix = 0;
  if (!source.word(candidate.node, 296, matrix, "node.matrix_pointer"))
    return false;
  if (matrix == 0)
    return true;
  if (!source.field(matrix, 96, bytes.size(), bytes.data(), "matrix.translation"))
    return false;
  if (!finite_vector(bytes, candidate.translation))
    return true;
  if (!source.field(candidate.camera, 1616, 4, bytes.data(), "camera.fov"))
    return false;
  candidate.fov = std::bit_cast<float>(u32(bytes.data()));
  if (!std::isfinite(candidate.fov) || candidate.fov <= 0)
    return true;
  std::array<std::array<double, 3>, 3> basis{};
  bool finite = true;
  for (unsigned row = 0; row < 3; ++row) {
    constexpr const char* fields[]{"camera.basis_x", "camera.basis_y", "camera.basis_z"};
    if (!source.field(candidate.camera, 1648 + row * 32, bytes.size(), bytes.data(), fields[row]))
      return false;
    finite &= finite_vector(bytes, basis[row]);
  }
  candidate.usable = finite && orthonormal(basis);
  return true;
}

void publish(SourceViewSnapshot& result, std::uint64_t address, const Candidate& candidate) noexcept {
  result.complete = true;
  result.status = SourceViewStatus::ready;
  result.field = "";
  result.source_address = address;
  result.node_address = candidate.node;
  result.camera_address = candidate.camera;
  result.fov = candidate.fov;
}

}  /* namespace */

/*
 * Searches the validated pool for an admissible source view, accepting it only after rereading the full
 * observation trace.
 */
SourceViewSnapshot inspect_source_view(engine_camera::MemoryReader& reader, const engine_camera::ViewPoolSnapshot& pool) noexcept {
  SourceViewSnapshot result;
  if (!valid_source_pool(pool)) {
    fail(result, SourceViewStatus::invalid_pool, "Source selection requires a complete pool of eight distinct classified views.");
    return result;
  }
  SourceViewReader source(reader, result);
  for (const auto& slot : pool.slots) {
    if (!slot.association_valid)
      continue;
    ++result.candidates_examined;
    std::uint64_t view = 0;
    if (!source.word(pool.array_address, slot.index * 8, view, "pool.view_pointer"))
      return result;
    if (view != slot.view_address) {
      fail(result, SourceViewStatus::pool_changed, "An occupied pool pointer no longer matches the supplied snapshot.");
      return result;
    }
    Candidate candidate;
    if (!inspect_candidate(source, view, candidate))
      return result;
    if (!candidate.usable)
      continue;
    if (!source.recheck())
      return result;
    publish(result, view, candidate);
    result.view_index = static_cast<std::int32_t>(slot.index);
    return result;
  }
  if (source.recheck())
    fail(result, SourceViewStatus::no_source, "No occupied view has the required generation-valid Camera and finite orthonormal pose.");
  return result;
}

/*
 * Revalidates the selected source identity and pose before returning it; a previous candidate selection is
 * not a lasting pointer guarantee.
 */
SourceViewSnapshot inspect_source_pose(engine_camera::MemoryReader& reader,
                                       std::uint64_t source_with_node_handle_at104,
                                       std::array<double, 3>* validated_position) noexcept {
  if (validated_position)
    *validated_position = {};
  SourceViewSnapshot result;
  SourceViewReader source(reader, result);
  Candidate candidate;
  result.candidates_examined = 1;
  if (!inspect_candidate(source, source_with_node_handle_at104, candidate) || !source.recheck())
    return result;
  if (candidate.usable) {
    publish(result, source_with_node_handle_at104, candidate);
    if (validated_position)
      *validated_position = candidate.translation;
  } else
    fail(result, SourceViewStatus::no_source,
         "The explicit source lacks the required generation-valid Camera and finite orthonormal pose.");
  return result;
}

}  /* namespace azp::native_camera */

/* ---- Resize and far planes -------------------------------------------------------------------------- */

namespace azp::native_camera {
namespace {
constexpr std::int32_t MaximumDimension = 16384;
constexpr std::int32_t MinimumDimension = 32;
static_assert(sizeof(ViewDimensions) == 24);

bool dimensions_bounded(const ViewDimensions& dimensions) noexcept {
  for (const auto& pair : dimensions)
    for (const auto value : pair)
      if (value < MinimumDimension || value > MaximumDimension)
        return false;
  return true;
}

/*
 * Walks the entire destination span and requires current writable private pages without overflow or gaps.
 */
bool writable_private(std::uint64_t address, std::size_t bytes) noexcept {
  if (!address || address > std::numeric_limits<std::uintptr_t>::max() - bytes)
    return false;
  const auto end = address + bytes;
  void* allocation = nullptr;
  while (address < end) {
    MEMORY_BASIC_INFORMATION region{};
    if (VirtualQuery(reinterpret_cast<void*>(address), &region, sizeof(region)) != sizeof(region) || region.State != MEM_COMMIT ||
        region.Type != MEM_PRIVATE || region.Protect != PAGE_READWRITE || !region.AllocationBase)
      return false;
    if (allocation && allocation != region.AllocationBase)
      return false;
    allocation = region.AllocationBase;
    const auto begin = reinterpret_cast<std::uintptr_t>(region.BaseAddress);
    if (begin > address || region.RegionSize > std::numeric_limits<std::uintptr_t>::max() - begin || begin + region.RegionSize <= address)
      return false;
    address = std::min<std::uint64_t>(end, begin + region.RegionSize);
  }
  return true;
}

template <typename T>
bool read(std::uint64_t address, T& output) noexcept {
  SIZE_T count = 0;
  T local{};
  if (!ReadProcessMemory(GetCurrentProcess(), reinterpret_cast<void*>(address), &local, sizeof(local), &count) || count != sizeof(local))
    return false;
  output = local;
  return true;
}
}  /* namespace */

/*
 * Validates the closed owned view, requested finite distances and current field contents before writing the
 * reviewed far-plane pairs and refreshing projection.
 */
const char* set_owned_view_far_planes(const engine_camera::OwnedViewSnapshot& view,
                                      const std::array<float, 2>& far_planes,
                                      const ViewResizeCallbacks& callbacks) noexcept {
  if (!view.complete || !view.ready || view.status != engine_camera::OwnedViewStatus::ready || view.mode != 2 || view.read_failures ||
      (view.error && *view.error) || !view.view_address || (view.view_address & 7) || view.view_address > UINTPTR_MAX - 64 ||
      !view.node_address || (view.node_address & 7) || view.node_address > UINTPTR_MAX - 264 || !view.camera_address ||
      (view.camera_address & 7) || view.camera_address > UINTPTR_MAX - 1532 || !callbacks.refresh_projection)
    return "range_invalid_owned_view";
  if (!(view.flags[0] & 1))
    return "range_gate_open";
  for (const auto value : far_planes)
    if (!std::isfinite(value) || value < 1000 || value > 10000000)
      return "range_invalid_far_plane";
  const auto field = view.camera_address + 1520;
  if (!writable_private(field, 12))
    return "range_camera_not_writable";
  std::array<float, 3> planes{};
  std::array<std::uint64_t, 2> flags{};
  std::uint64_t camera{};
  std::uint16_t type{};
  if (!read(field, planes) || !read(view.view_address + 48, flags) || !read(view.node_address + 256, camera) ||
      !read(view.camera_address + 160, type))
    return "range_read_failed";
  if (flags != view.flags || camera != view.camera_address || type != 7)
    return "range_identity_changed";
  if (!std::isfinite(planes[0]) || planes[0] <= 0 || planes[0] >= far_planes[0] || planes[0] >= far_planes[1] ||
      !std::isfinite(planes[1]) || !std::isfinite(planes[2]) || planes[1] <= planes[0] || planes[2] <= planes[0])
    return "range_invalid_inherited_planes";
  const std::array<float, 3> desired{planes[0], far_planes[0], far_planes[1]};
  if (planes == desired)
    return nullptr;
  SIZE_T written{};
  if (!WriteProcessMemory(GetCurrentProcess(), reinterpret_cast<void*>(field + 4), far_planes.data(), sizeof(far_planes), &written) ||
      written != sizeof(far_planes))
    return "range_write_failed";
  if (!read(field, planes) || planes != desired || !read(view.view_address + 48, flags) || flags != view.flags ||
      !read(view.node_address + 256, camera) || camera != view.camera_address)
    return "range_changed_after_write";
  if (!callbacks.refresh_projection(callbacks.context, view.view_address))
    return "range_projection_refresh_failed";
  if (!read(field, planes) || planes != desired || !read(view.view_address + 48, flags) || flags != view.flags ||
      !read(view.node_address + 256, camera) || camera != view.camera_address)
    return "range_changed_after_projection";
  return nullptr;
}

/*
 * All three size pairs (render, display, output) request the square native camera resolution `side`, in
 * pixels. The output bitmap therefore has that size; the compositor scales it to the WASM image.
 */
bool plan_view_resize(const ViewDimensions& inherited, unsigned side, ViewDimensions& desired) noexcept {
  static_assert(kMinimumCameraResolution >= 32 && kMaximumCameraResolution <= 2048);
  desired = {};
  if (!dimensions_bounded(inherited) || !valid_camera_resolution(side))
    return false;
  desired.fill({static_cast<std::int32_t>(side), static_cast<std::int32_t>(side)});
  return true;
}

/*
 * Checks each current size pair and the exact requested pane before changing a closed view. Initial resize
 * may allocate; restoration proves the existing bitmap already matches and updates fields without
 * reallocation.
 *
 * The first original manager update, as well as a later AA/upscaling switch, can copy different primary
 * render/display sizes into these pairs. Bound every inherited field independently; the requested three
 * pairs must still equal one valid square camera resolution. Closed gates and full field rereads govern both paths;
 * restoration additionally proves the existing Bitmap below and never calls the output allocator.
 */
static ViewResizeResult change_view_dimensions(const engine_camera::OwnedViewSnapshot& view,
                                               const ViewDimensions& desired,
                                               const ViewResizeCallbacks& callbacks,
                                               bool initialize_output) noexcept {
  ViewResizeResult result;
  const auto fail = [&](ViewResizeStatus status) {
    result.status = status;
    return result;
  };
  if (!view.complete || !view.ready || view.status != engine_camera::OwnedViewStatus::ready || view.read_failures ||
      (view.error && *view.error) || !view.view_address || (view.view_address & 7u) ||
      view.view_address > std::numeric_limits<std::uintptr_t>::max() - 160 || !callbacks.refresh_projection ||
      (initialize_output && !callbacks.ensure_output))
    return fail(ViewResizeStatus::invalid_snapshot);
  ViewDimensions planned{};
  if (!dimensions_bounded(view.dimensions) || desired[0][0] <= 0 ||
      !plan_view_resize(desired, static_cast<unsigned>(desired[0][0]), planned) || desired != planned)
    return fail(ViewResizeStatus::invalid_dimensions);
  if (!(view.flags[0] & 1u))
    return fail(ViewResizeStatus::gate_open);
  if (!initialize_output && (view.mode != 2 || !view.resource_present || view.output_dimensions != desired[0]))
    return fail(ViewResizeStatus::output_mismatch);
  const auto field = view.view_address + 16;
  if (!writable_private(field, sizeof(desired)) || !writable_private(view.view_address + 48, sizeof(view.flags)))
    return fail(ViewResizeStatus::invalid_mapping);
  ViewDimensions current{};
  std::array<std::uint64_t, 2> flags{};
  if (!read(field, current) || !read(view.view_address + 48, flags))
    return fail(ViewResizeStatus::read_failed);
  if (current != view.dimensions || flags != view.flags)
    return fail(ViewResizeStatus::changed);
  if (current == desired) {
    result.status = ViewResizeStatus::unchanged;
    result.complete = true;
    return result;
  }
  SIZE_T written = 0;
  result.write_attempted = true;
  if (!WriteProcessMemory(GetCurrentProcess(), reinterpret_cast<void*>(field), desired.data(), sizeof(desired), &written) ||
      written != sizeof(desired))
    return fail(ViewResizeStatus::write_failed);
  if (!read(field, current) || !read(view.view_address + 48, flags))
    return fail(ViewResizeStatus::read_failed);
  if (current != desired || flags != view.flags)
    return fail(ViewResizeStatus::changed);
  if (!callbacks.refresh_projection(callbacks.context, view.view_address))
    return fail(ViewResizeStatus::refresh_failed);
  if (!read(field, current) || !read(view.view_address + 48, flags))
    return fail(ViewResizeStatus::read_failed);
  if (current != desired || flags != view.flags)
    return fail(ViewResizeStatus::changed);
  if (initialize_output && callbacks.ensure_output(callbacks.context, view.view_address) != view.view_address + 144)
    return fail(ViewResizeStatus::allocation_failed);
  if (!read(field, current) || !read(view.view_address + 48, flags))
    return fail(ViewResizeStatus::read_failed);
  if (current != desired || flags != view.flags)
    return fail(ViewResizeStatus::changed);
  result.status = initialize_output ? ViewResizeStatus::resized : ViewResizeStatus::dimensions_restored;
  result.complete = true;
  return result;
}

ViewResizeResult resize_owned_view(const engine_camera::OwnedViewSnapshot& view,
                                   const ViewDimensions& desired,
                                   const ViewResizeCallbacks& callbacks) noexcept {
  return change_view_dimensions(view, desired, callbacks, true);
}

ViewResizeResult restore_owned_view_dimensions(const engine_camera::OwnedViewSnapshot& view,
                                               const ViewDimensions& desired,
                                               const ViewResizeCallbacks& callbacks) noexcept {
  return change_view_dimensions(view, desired, callbacks, false);
}

/*
 * Formats resize refusal categories so identity, gate, dimensions and native-operation failures remain
 * distinguishable.
 */
const char* view_resize_status_name(ViewResizeStatus status) noexcept {
  switch (status) {
    case ViewResizeStatus::not_attempted:
      return "not attempted";
    case ViewResizeStatus::resized:
      return "resized";
    case ViewResizeStatus::dimensions_restored:
      return "dimension fields restored; existing output retained";
    case ViewResizeStatus::output_mismatch:
      return "existing output does not match the requested pane; no reallocation attempted";
    case ViewResizeStatus::unchanged:
      return "already at requested resolution";
    case ViewResizeStatus::invalid_snapshot:
      return "owned-view proof is incomplete";
    case ViewResizeStatus::invalid_dimensions:
      return "unsupported inherited dimensions";
    case ViewResizeStatus::gate_open:
      return "render gate is not closed";
    case ViewResizeStatus::invalid_mapping:
      return "view fields are not committed writable private memory";
    case ViewResizeStatus::changed:
      return "view fields changed during resize";
    case ViewResizeStatus::read_failed:
      return "view field reread failed";
    case ViewResizeStatus::write_failed:
      return "dimension write failed";
    case ViewResizeStatus::refresh_failed:
      return "projection refresh refused";
    case ViewResizeStatus::allocation_failed:
      return "output routine returned an unexpected handle address";
  }
  return "unknown";
}
}  /* namespace azp::native_camera */

/* ---- Anti-aliasing flag ----------------------------------------------------------------------------- */

namespace azp::native_camera {
namespace {
bool writable_flags(std::uint64_t address) noexcept {
  return writable_private_span(address, 16);
}
bool read_flags(std::uint64_t address, std::array<std::uint64_t, 2>& flags) noexcept {
  return read_local_flag_words(address, flags);
}
bool read_overrides(discovery::ImageReader& image, std::array<std::uint64_t, 2>& value, const CameraImageLayout& layout) noexcept {
  return image.read(layout.view_flag_clear_override, &value[0], 8) && image.read(layout.view_flag_set_override, &value[1], 8);
}
}  /* namespace */

/*
 * Validates the reviewed AA override fields and closes no gates itself. Only an already closed, owned view
 * may receive the bounded override write; an open view is inspected without mutation.
 *
 * An open view permits validation only; changing AA requires a closed gate.
 */
ViewAaResult disable_owned_view_aa(const engine_camera::OwnedViewSnapshot& view,
                                   discovery::ImageReader& image,
                                   const CameraImageLayout& layout) noexcept {
  ViewAaResult result;
  const auto fail = [&](const char* error) {
    result.error = error;
    return result;
  };
  if (!camera_layout_detail::image_rva(layout.view_flag_clear_override, 8, 8) ||
      !camera_layout_detail::image_rva(layout.view_flag_set_override, 8, 8) ||
      layout.view_flag_clear_override == layout.view_flag_set_override)
    return fail("aa_invalid_image_layout");
  if (!view.complete || !view.ready || view.mode != 2 || view.status != engine_camera::OwnedViewStatus::ready || view.read_failures ||
      (view.error && *view.error) || !view.view_address || (view.view_address & 7) || view.view_address > UINTPTR_MAX - 64)
    return fail("aa_invalid_owned_view");
  const auto field = view.view_address + 48;
  if (!writable_flags(field))
    return fail("aa_flags_not_writable");
  std::array<std::uint64_t, 2> current{}, overrides{}, again{};
  if (!read_flags(field, current) || !read_overrides(image, overrides, layout))
    return fail("aa_read_failed");
  if (current != view.flags)
    return fail("aa_snapshot_changed");
  if ((overrides[1] & ~overrides[0] & kViewAaFlag) != 0)
    return fail("aa_forced_by_global_override");
  auto desired = current;
  desired[0] &= ~kViewAaFlag;
  if (desired != current) {
    if (!(current[0] & 1u))
      return fail("aa_gate_open");
    SIZE_T written = 0;
    result.write_attempted = true;
    if (!WriteProcessMemory(GetCurrentProcess(), reinterpret_cast<void*>(field), &desired[0], 8, &written) || written != 8)
      return fail("aa_write_failed");
  }
  if (!read_flags(field, current) || !read_overrides(image, again, layout))
    return fail("aa_recheck_failed");
  if (current != desired || again != overrides)
    return fail("aa_changed_during_update");
  result.complete = true;
  result.error = "";
  return result;
}

}  /* namespace azp::native_camera */

/* ---- Render-gate disable mask ----------------------------------------------------------------------- */

namespace azp::native_camera {

/*
 * Finds the disable-mask operand in the verified activation code and checks that its image range and
 * contents match the reviewed contract. Failure prevents gate mutation rather than supplying a guessed mask.
 */
ActivationMaskInventory inspect_activation_disable_mask(discovery::ImageReader& reader,
                                                        const discovery::Inventory& image,
                                                        const CameraImageLayout& layout) {
  ActivationMaskInventory result;
  const auto rva = layout.activation_disable_mask;
  constexpr std::uint32_t size = 16;
  constexpr std::uint32_t required = 0x40000000;
  constexpr std::uint32_t excluded = 0xa2000000;
  if (!camera_layout_detail::image_rva(rva, size, 8) || !image.valid_image || image.machine != 0x8664 || image.image_size == 0 ||
      image.image_size > 0x80000000u || image.section_count == 0 || image.section_count > 96 || image.sections.empty() ||
      image.sections.size() > image.section_count || rva > image.image_size || size > image.image_size - rva) {
    result.error = "The activation-mask image metadata or bounds are invalid.";
    return result;
  }
  const auto section = std::find_if(image.sections.begin(), image.sections.end(), [&](const auto& item) {
    return item.rva <= image.image_size && item.size <= image.image_size - item.rva && (item.flags & required) == required &&
           (item.flags & excluded) == 0 && rva >= item.rva && rva - item.rva <= item.size && size <= item.size - (rva - item.rva);
  });
  if (section == image.sections.end()) {
    result.error = "The fixed activation-mask pair is outside readable static image data.";
    return result;
  }
  const auto exact = [&](std::array<std::uint8_t, size>& output) {
    result.read_bytes += size;
    std::uint32_t offset = 0;
    while (offset < size) {
      const auto window = reader.query(rva + offset, size - offset);
      if (!window.readable || window.size == 0 || window.size > size - offset ||
          !reader.read(rva + offset, output.data() + offset, window.size)) {
        ++result.read_failures;
        result.error = "The fixed activation-mask pair could not be read exactly.";
        return false;
      }
      offset += window.size;
    }
    return true;
  };
  std::array<std::uint8_t, size> first{}, second{};
  if (!exact(first))
    return result;
  for (unsigned word = 0; word < 2; ++word)
    for (unsigned byte = 0; byte < 8; ++byte)
      result.words[word] |= std::uint64_t(first[word * 8 + byte]) << (byte * 8);
  if (!exact(second))
    return result;
  if (first != second) {
    result.error = "The fixed activation-mask pair changed during its full reread.";
    return result;
  }
  if (result.words != std::array<std::uint64_t, 2>{1, 0}) {
    result.error = "The captured activation mask is not the required bit-zero-only pair {1,0}.";
    return result;
  }
  result.valid = true;
  return result;
}

}  /* namespace azp::native_camera */
