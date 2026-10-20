/*
 * Locates engine virtual tables from the executable's MSVC runtime type metadata. It checks type names,
 * hierarchy structure, section membership and function pointers, then rereads the evidence before returning
 * a candidate.
 */

#include "rtti_vtables.hpp"
#include <algorithm>
#include <array>
#include <cstring>

namespace azp::native_camera {
namespace {
constexpr std::uint32_t Chunk = 32768;
constexpr std::uint64_t ScanLimit = 64ull * 1024 * 1024, MetadataLimit = 16ull * 1024 * 1024;
constexpr std::uint32_t ReadLimit = 131072, ColLimit = 128, PointerLimit = 4096;

std::uint32_t u32(const std::uint8_t* p) {
  return std::uint32_t(p[0]) | (std::uint32_t(p[1]) << 8) | (std::uint32_t(p[2]) << 16) | (std::uint32_t(p[3]) << 24);
}
std::uint64_t u64(const std::uint8_t* p) {
  return std::uint64_t(u32(p)) | (std::uint64_t(u32(p + 4)) << 32);
}
bool static_data(const discovery::ImageSection& s) {
  return (s.flags & 0x40000000u) && !(s.flags & 0xa2000000u);
}
bool contains(const discovery::ImageSection& s, std::uint32_t rva, std::uint32_t bytes) {
  return bytes && rva >= s.rva && std::uint64_t(rva) + bytes <= std::uint64_t(s.rva) + s.size;
}
bool named(const VtableRequest& request) {
  return !request.resolved_vtable_rva && request.methods.empty();
}

class Resolver {
 public:
  Resolver(discovery::ImageReader& reader,
           const discovery::Inventory& image,
           std::uint64_t base,
           const std::vector<VtableRequest>& requests)
      : reader_(reader), image_(image), base_(base), requests_(requests) {}

  /*
   * Validates the image/type requests, scans candidate runtime type records and resolves unambiguous
   * vtables. All retained structural evidence is reread before results are returned.
   */
  RttiVtables run() {
    if (!validate())
      return std::move(result_);
    selected_.resize(requests_.size());
    seen_.resize(requests_.size());
    if (type_read_size_ && !scan(24, 4, [&](std::uint32_t at, const std::uint8_t* bytes) { return locator(at, bytes); }))
      return std::move(result_);
    std::sort(locators_.begin(), locators_.end(), [](const auto& a, const auto& b) { return a.rva < b.rva; });
    for (std::size_t request = 0; request < requests_.size(); ++request)
      if (requests_[request].resolved_vtable_rva && !candidate(request, requests_[request].resolved_vtable_rva))
        return std::move(result_);
    if (!scan(8, 8, [&](std::uint32_t at, const std::uint8_t* bytes) { return pointer(at, bytes); }))
      return std::move(result_);
    for (std::size_t i = 0; i < selected_.size(); ++i) {
      if (!selected_[i].vtable) {
        fail("requested_primary_vtable_missing");
        return std::move(result_);
      }
      if (!recheck(i, selected_[i]))
        return std::move(result_);
    }
    for (const auto& selected : selected_)
      result_.vtables.push_back(selected.vtable);
    result_.valid = true;
    return std::move(result_);
  }

 private:
  struct BaseDescriptor {
    std::uint32_t rva = 0;
    std::array<std::uint8_t, 28> bytes{};
  };
  struct NamedLocator {
    std::uint32_t rva = 0, type_size = 0;
    std::size_t request = 0;
    std::array<std::uint8_t, 24> col{};
    std::array<std::uint8_t, 272> type{};
  };
  struct Evidence {
    std::uint32_t vtable = 0, col_rva = 0, type_rva = 0, hierarchy_rva = 0, type_size = 0;
    std::uint64_t first = 0, last = 0;
    std::array<std::uint8_t, 24> col{};
    std::array<std::uint8_t, 272> type{};
    std::array<std::uint8_t, 16> hierarchy{};
    std::uint32_t base_array_rva = 0, base_array_size = 0;
    std::array<std::uint8_t, 256> base_array{};
    std::vector<BaseDescriptor> bases;
  };
  bool fail(const char* error) {
    if (result_.error.empty())
      result_.error = error;
    return false;
  }
  /*
   * Checks section ranges, requested names and executable/data constraints before scanning any RTTI
   * candidate.
   */
  bool validate() {
    if (!image_.valid_image || image_.machine != 0x8664 || !image_.image_size || image_.image_size > 0x80000000u || !base_ || (base_ & 7) ||
        base_ > UINT64_MAX - image_.image_size || !image_.section_count || image_.section_count > 96 || image_.sections.empty() ||
        image_.sections.size() != image_.section_count)
      return fail("invalid_image_metadata");
    std::uint64_t scan_bytes = 0;
    for (std::size_t i = 0; i < image_.sections.size(); ++i) {
      const auto& s = image_.sections[i];
      if (s.rva >= image_.image_size || s.size > image_.image_size - s.rva)
        return fail("invalid_section_bounds");
      for (std::size_t j = 0; j < i; ++j) {
        const auto& other = image_.sections[j];
        if (s.size && other.size && s.rva < std::uint64_t(other.rva) + other.size && other.rva < std::uint64_t(s.rva) + s.size)
          return fail("overlapping_sections");
      }
      if (static_data(s))
        scan_bytes += s.size;
    }
    if (scan_bytes > ScanLimit)
      return fail("static_data_scan_limit");
    if (requests_.empty() || requests_.size() > 16)
      return fail("invalid_request_count");
    for (std::size_t i = 0; i < requests_.size(); ++i) {
      const auto& request = requests_[i];
      if (request.extent < 8 || request.extent > 4096 || (request.extent & 7) || request.methods.size() > 16)
        return fail("invalid_vtable_request");
      if (named(request)) {
        if (request.type_name.empty() || request.type_name.size() > 255 || request.type_name.find('\0') != std::string::npos)
          return fail("missing_or_invalid_type_identity");
        for (std::size_t j = 0; j < i; ++j)
          if (named(requests_[j]) && requests_[j].type_name == request.type_name)
            return fail("duplicate_type_request");
        type_read_size_ = std::max(type_read_size_, static_cast<std::uint32_t>(16 + request.type_name.size() + 1));
      }
      if (request.resolved_vtable_rva && ((request.resolved_vtable_rva & 7) || request.resolved_vtable_rva < 8 ||
                                          !data(request.resolved_vtable_rva - 8, request.extent + 8)))
        return fail("invalid_vtable_root");
      for (std::size_t j = 0; j < request.methods.size(); ++j) {
        const auto& method = request.methods[j];
        if ((method.offset & 7) || method.offset > request.extent - 8 || !method.method_rva || method.method_rva >= image_.image_size ||
            !executable(base_ + method.method_rva))
          return fail("invalid_method_constraint");
        for (std::size_t k = 0; k < j; ++k)
          if (request.methods[k].offset == method.offset)
            return fail("duplicate_method_offset");
      }
    }
    return true;
  }
  const discovery::ImageSection* data(std::uint32_t rva, std::uint32_t bytes, bool type = false) const {
    for (const auto& s : image_.sections)
      if ((type ? (s.flags & 0x40000000u) && !(s.flags & 0x20000000u) : static_data(s)) && contains(s, rva, bytes))
        return &s;
    return nullptr;
  }
  bool executable(std::uint64_t pointer) const {
    if (pointer < base_ || pointer - base_ >= image_.image_size)
      return false;
    const auto rva = static_cast<std::uint32_t>(pointer - base_);
    return std::any_of(image_.sections.begin(), image_.sections.end(), [&](const auto& s) {
      return (s.flags & 0x60000000u) == 0x60000000u && !(s.flags & 0x82000000u) && contains(s, rva, 1);
    });
  }
  /*
   * Reads a bounded image field across verified readable regions while preserving the resolver's byte
   * budget.
   */
  bool read(std::uint32_t rva, void* destination, std::size_t bytes, bool scan = false) {
    if (rva >= image_.image_size || !bytes || bytes > image_.image_size - rva)
      return fail("read_bounds");
    if (!scan) {
      if (bytes > MetadataLimit - metadata_bytes_)
        return fail("targeted_metadata_limit");
      metadata_bytes_ += bytes;
    }
    std::size_t offset = 0;
    while (offset < bytes) {
      if (result_.read_calls >= ReadLimit)
        return fail("read_call_limit");
      const auto maximum = static_cast<std::uint32_t>(std::min<std::size_t>(Chunk, bytes - offset));
      const auto at = rva + static_cast<std::uint32_t>(offset);
      const auto window = reader_.query(at, maximum);
      if (!window.readable || !window.size || window.size > maximum)
        return fail("unreadable_or_invalid_window");
      ++result_.read_calls;
      result_.read_bytes += window.size;
      if (!reader_.read(at, static_cast<std::uint8_t*>(destination) + offset, window.size))
        return fail("image_read_failed");
      offset += window.size;
    }
    return true;
  }
  bool word(std::uint32_t at, std::uint64_t& value) {
    std::array<std::uint8_t, 8> bytes{};
    if (!read(at, bytes.data(), bytes.size()))
      return false;
    value = u64(bytes.data());
    return true;
  }
  /*
   * Scans eligible static-data sections in bounded chunks, retaining overlap so records crossing chunk edges
   * remain discoverable.
   */
  template <typename Visit>
  bool scan(std::size_t width, std::uint32_t alignment, Visit visit) {
    std::array<std::uint8_t, Chunk + 24> buffer{};
    for (const auto& s : image_.sections) {
      if (!static_data(s))
        continue;
      std::size_t carry = 0;
      for (std::uint32_t offset = 0; offset < s.size;) {
        const auto count = std::min(Chunk, s.size - offset);
        result_.scanned_bytes += count;
        if (!read(s.rva + offset, buffer.data() + carry, count, true))
          return false;
        const auto size = carry + count;
        const auto start = s.rva + offset - static_cast<std::uint32_t>(carry);
        for (std::size_t i = (alignment - start % alignment) % alignment; i + width <= size; i += alignment)
          if (i + width > carry && !visit(start + static_cast<std::uint32_t>(i), buffer.data() + i))
            return false;
        carry = std::min(width - 1, size);
        std::memmove(buffer.data(), buffer.data() + size - carry, carry);
        offset += count;
      }
    }
    return true;
  }
  bool primary_col(std::uint32_t at, const std::uint8_t* bytes) const {
    const auto type = u32(bytes + 12), hierarchy = u32(bytes + 16);
    return at && !(at & 3) && u32(bytes) == 1 && !u32(bytes + 4) && !u32(bytes + 8) && u32(bytes + 20) == at && type && !(type & 7) &&
           data(type, 16, true) && hierarchy && !(hierarchy & 3) && data(hierarchy, 16);
  }
  /*
   * Recognizes a primary MSVC complete-object locator and matches its type descriptor to a requested bounded
   * name.
   */
  bool locator(std::uint32_t at, const std::uint8_t* bytes) {
    if (!primary_col(at, bytes))
      return true;
    const auto type = u32(bytes + 12);
    const auto section = data(type, 17, true);
    if (!section)
      return true;
    const auto length = std::min(type_read_size_, section->size - (type - section->rva));
    std::array<std::uint8_t, 272> identity{};
    if (!read(type, identity.data(), length))
      return false;
    for (std::size_t i = 0; i < requests_.size(); ++i) {
      const auto& request = requests_[i];
      if (!named(request) || 16 + request.type_name.size() >= length || identity[16 + request.type_name.size()] ||
          std::memcmp(identity.data() + 16, request.type_name.data(), request.type_name.size()))
        continue;
      if (locators_.size() >= ColLimit)
        return fail("rtti_candidate_limit");
      NamedLocator observed;
      observed.rva = at;
      observed.request = i;
      observed.type_size = static_cast<std::uint32_t>(16 + request.type_name.size() + 1);
      std::copy_n(bytes, observed.col.size(), observed.col.begin());
      observed.type = identity;
      locators_.push_back(observed);
    }
    return true;
  }
  /*
   * Validates a candidate vtable's locator, type hierarchy, function slots and image-section membership
   * before accepting it as type evidence.
   */
  bool candidate(std::size_t request_index, std::uint32_t vtable, std::uint32_t expected_locator = 0) {
    auto& seen = seen_[request_index];
    if (std::find(seen.begin(), seen.end(), vtable) != seen.end())
      return true;
    if (++result_.vtable_candidates > PointerLimit)
      return fail("vtable_candidate_limit");
    seen.push_back(vtable);
    const auto& request = requests_[request_index];
    if ((vtable & 7) || vtable < 8 || !data(vtable - 8, request.extent + 8))
      return true;
    std::uint64_t col_pointer = 0;
    if (!word(vtable - 8, col_pointer))
      return false;
    if (col_pointer < base_ || col_pointer - base_ >= image_.image_size)
      return true;
    Evidence proof;
    proof.vtable = vtable;
    proof.col_rva = static_cast<std::uint32_t>(col_pointer - base_);
    if (expected_locator && proof.col_rva != expected_locator)
      return fail("rtti_identity_changed");
    if (!data(proof.col_rva, 24))
      return true;
    if (!read(proof.col_rva, proof.col.data(), proof.col.size()))
      return false;
    if (!primary_col(proof.col_rva, proof.col.data()))
      return true;
    if (++result_.candidates > ColLimit)
      return fail("rtti_candidate_limit");
    proof.type_rva = u32(proof.col.data() + 12);
    proof.hierarchy_rva = u32(proof.col.data() + 16);
    proof.type_size = named(request) ? static_cast<std::uint32_t>(16 + request.type_name.size() + 1) : 16;
    if (!data(proof.type_rva, proof.type_size, true))
      return true;
    if (!read(proof.type_rva, proof.type.data(), proof.type_size) ||
        !read(proof.hierarchy_rva, proof.hierarchy.data(), proof.hierarchy.size()) || !word(vtable, proof.first) ||
        !word(vtable + request.extent - 8, proof.last))
      return false;
    if (named(request)) {
      const auto observed = std::lower_bound(locators_.begin(), locators_.end(), proof.col_rva,
                                             [](const auto& item, auto target) { return item.rva < target; });
      if (observed == locators_.end() || observed->rva != proof.col_rva || observed->request != request_index ||
          observed->col != proof.col || !std::equal(proof.type.begin(), proof.type.begin() + proof.type_size, observed->type.begin()))
        return fail("rtti_identity_changed");
    }
    if ((named(request) && (proof.type[16 + request.type_name.size()] ||
                            std::memcmp(proof.type.data() + 16, request.type_name.data(), request.type_name.size()))) ||
        !executable(proof.first) || !executable(proof.last))
      return true;
    for (const auto& method : request.methods) {
      std::uint64_t value = 0;
      if (!word(vtable + method.offset, value))
        return false;
      if (value != base_ + method.method_rva)
        return true;
    }
    if (!validate_hierarchy(proof))
      return false;
    if (selected_[request_index].vtable)
      return fail("ambiguous_primary_vtable");
    selected_[request_index] = proof;
    return true;
  }
  /*
   * Checks a possible vtable reference against candidate locations and the requested type layout rather than
   * accepting a pointer-shaped value alone.
   */
  bool pointer(std::uint32_t at, const std::uint8_t* bytes) {
    const auto value = u64(bytes);
    if (value < base_ || value - base_ >= image_.image_size)
      return true;
    const auto rva = static_cast<std::uint32_t>(value - base_);
    const auto found =
        std::lower_bound(locators_.begin(), locators_.end(), rva, [](const auto& item, auto target) { return item.rva < target; });
    if (found != locators_.end() && found->rva == rva && !candidate(found->request, at + 8, found->rva))
      return false;
    for (std::size_t i = 0; i < requests_.size(); ++i) {
      const auto& request = requests_[i];
      if (request.resolved_vtable_rva || named(request))
        continue;
      for (const auto& method : request.methods)
        if (rva == method.method_rva && at >= method.offset && !candidate(i, at - method.offset))
          return false;
    }
    return true;
  }
  /*
   * Checks the bounded one-level MSVC x64 hierarchy and its referenced identities. Nested hierarchy records
   * are range-checked, not recursively traversed.
   *
   * Same bounded one-level MSVC AMD64 layout as the offline RTTI inspector. Nested hierarchy references are
   * range-checked identities, never traversed.
   */
  bool validate_hierarchy(Evidence& proof) {
    const auto count = u32(proof.hierarchy.data() + 8);
    proof.base_array_rva = u32(proof.hierarchy.data() + 12);
    if (u32(proof.hierarchy.data()) || !count || count > 64 || !proof.base_array_rva || (proof.base_array_rva & 3) ||
        !data(proof.base_array_rva, count * 4))
      return fail("invalid_rtti_hierarchy");
    proof.base_array_size = count * 4;
    if (!read(proof.base_array_rva, proof.base_array.data(), proof.base_array_size))
      return false;
    for (std::uint32_t i = 0; i < count; ++i) {
      BaseDescriptor descriptor;
      descriptor.rva = u32(proof.base_array.data() + i * 4);
      if (!descriptor.rva || (descriptor.rva & 3) || !data(descriptor.rva, descriptor.bytes.size()))
        return fail("invalid_rtti_base_descriptor");
      if (!read(descriptor.rva, descriptor.bytes.data(), descriptor.bytes.size()))
        return false;
      const auto type = u32(descriptor.bytes.data()), contained = u32(descriptor.bytes.data() + 4);
      const auto attributes = u32(descriptor.bytes.data() + 20), hierarchy = u32(descriptor.bytes.data() + 24);
      if (!type || (type & 7) || !data(type, 16, true) || contained > count - i - 1 ||
          (hierarchy && ((hierarchy & 3) || !data(hierarchy, 16))) || ((attributes & 64) && !hierarchy) ||
          (i == 0 && (type != proof.type_rva || contained != count - 1)))
        return fail("invalid_rtti_base_identity");
      proof.bases.push_back(descriptor);
    }
    return true;
  }
  /*
   * Rereads type, locator, hierarchy and slot evidence so a changed table cannot seed a persistent resolved
   * identity.
   */
  bool recheck(std::size_t index, const Evidence& proof) {
    const auto& request = requests_[index];
    std::array<std::uint8_t, 24> col{};
    std::array<std::uint8_t, 272> type{};
    std::array<std::uint8_t, 16> hierarchy{};
    std::uint64_t preceding = 0, first = 0, last = 0;
    if (!read(proof.col_rva, col.data(), col.size()) || !read(proof.type_rva, type.data(), proof.type_size) ||
        !read(proof.hierarchy_rva, hierarchy.data(), hierarchy.size()) || !word(proof.vtable - 8, preceding) ||
        !word(proof.vtable, first) || !word(proof.vtable + request.extent - 8, last))
      return false;
    if (col != proof.col || hierarchy != proof.hierarchy || !std::equal(type.begin(), type.begin() + proof.type_size, proof.type.begin()) ||
        preceding != base_ + proof.col_rva || first != proof.first || last != proof.last)
      return fail("rtti_identity_changed");
    std::array<std::uint8_t, 256> base_array{};
    if (!read(proof.base_array_rva, base_array.data(), proof.base_array_size))
      return false;
    if (!std::equal(base_array.begin(), base_array.begin() + proof.base_array_size, proof.base_array.begin()))
      return fail("rtti_hierarchy_changed");
    for (const auto& descriptor : proof.bases) {
      std::array<std::uint8_t, 28> again{};
      if (!read(descriptor.rva, again.data(), again.size()))
        return false;
      if (again != descriptor.bytes)
        return fail("rtti_hierarchy_changed");
    }
    for (const auto& method : request.methods) {
      std::uint64_t value = 0;
      if (!word(proof.vtable + method.offset, value))
        return false;
      if (value != base_ + method.method_rva)
        return fail("vtable_method_changed");
    }
    return true;
  }

  discovery::ImageReader& reader_;
  const discovery::Inventory& image_;
  std::uint64_t base_;
  const std::vector<VtableRequest>& requests_;
  RttiVtables result_;
  std::uint64_t metadata_bytes_ = 0;
  std::uint32_t type_read_size_ = 0;
  std::vector<NamedLocator> locators_;
  std::vector<std::vector<std::uint32_t>> seen_;
  std::vector<Evidence> selected_;
};
}  /* namespace */

RttiVtables resolve_rtti_vtables(discovery::ImageReader& reader,
                                 const discovery::Inventory& image,
                                 std::uint64_t image_base,
                                 const std::vector<VtableRequest>& requests) {
  return Resolver(reader, image, image_base, requests).run();
}
}  /* namespace azp::native_camera */
