/*
 * Verifies complete native code fingerprints against a bounded executable image. PE sections and relocation
 * locations are checked so only explicitly modelled relocation bytes can vary; an address or short pattern
 * match alone is not accepted.
 */

#include "code_contract.hpp"
#include <algorithm>
#include <array>
#include <cstdio>
#include <utility>

namespace azp::native_camera {
namespace {

constexpr std::uint32_t kReadable = 0x40000000;
constexpr std::uint32_t kWritable = 0x80000000;
constexpr std::uint32_t kExecutable = 0x20000000;
constexpr std::uint32_t kDiscardable = 0x02000000;
constexpr std::uint32_t kCodeLimit = 65536;
constexpr std::uint32_t kMetadataLimit = 1024 * 1024;

std::uint16_t u16(const std::uint8_t* bytes) {
  return static_cast<std::uint16_t>(bytes[0] | (std::uint16_t(bytes[1]) << 8));
}

std::uint32_t u32(const std::uint8_t* bytes) {
  return bytes[0] | (std::uint32_t(bytes[1]) << 8) | (std::uint32_t(bytes[2]) << 16) | (std::uint32_t(bytes[3]) << 24);
}

bool within(std::uint64_t rva, std::uint64_t size, std::uint64_t begin, std::uint64_t extent) {
  return rva >= begin && rva - begin <= extent && size <= extent - (rva - begin);
}

bool overlap(std::uint64_t a, std::uint64_t a_size, std::uint64_t b, std::uint64_t b_size) {
  return a_size != 0 && b_size != 0 && a < b + b_size && b < a + a_size;
}

class Parser {
 public:
  Parser(discovery::ImageReader& reader, const discovery::Inventory& image, const std::vector<CodeRange>& ranges)
      : reader_(reader), image_(image), ranges_(ranges) {}

  /*
   * Validates the requested code ranges and PE metadata, checks permitted relocation bytes, and verifies
   * every required fingerprint before reporting a successful contract.
   */
  CodeContractInventory run() {
    if (!validate() || !headers() || !relocations()) {
      return std::move(result_);
    }
    result_.relocations_checked = true;
    result_.records.reserve(ranges_.size());
    std::array<std::uint8_t, 8192> bytes{};
    for (const auto& range : ranges_) {
      if (!exact(range.rva, bytes.data(), range.size, false)) {
        return std::move(result_);
      }
      std::uint64_t hash = 14695981039346656037ull;
      for (std::uint32_t index = 0; index < range.size; ++index) {
        hash ^= bytes[index];
        hash *= 1099511628211ull;
      }
      result_.records.push_back({range.rva, range.size, hash});
    }
    result_.valid = true;
    return std::move(result_);
  }

 private:
  bool fail(const char* message) {
    result_.error = message;
    return false;
  }

  /*
   * Requires an entire code/data span to remain inside a section with the requested characteristics.
   */
  bool section_range(std::uint64_t rva, std::uint64_t size, std::uint32_t required, std::uint32_t excluded) const {
    if (size == 0 || !within(rva, size, 0, image_.image_size)) {
      return false;
    }
    for (const auto& section : image_.sections) {
      if ((section.flags & required) == required && (section.flags & excluded) == 0 && within(rva, size, section.rva, section.size)) {
        return true;
      }
    }
    return false;
  }

  /*
   * Checks image and request bounds before fingerprint processing, refusing invalid section layouts,
   * overlapping ranges or unsupported proof inputs.
   */
  bool validate() {
    if (!image_.valid_image || image_.machine != 0x8664 || image_.image_size == 0 || image_.image_size > 0x80000000u ||
        image_.sections.empty() || image_.sections.size() > image_.section_count || image_.section_count == 0 ||
        image_.section_count > 96) {
      return fail("Invalid AMD64 loaded-image metadata.");
    }
    for (std::size_t index = 0; index < image_.sections.size(); ++index) {
      const auto& section = image_.sections[index];
      if (!within(section.rva, section.size, 0, image_.image_size)) {
        return fail("A declared image section extends beyond the image.");
      }
      if (section.size != 0) {
        header_limit_ = std::min(header_limit_, section.rva);
      }
      for (std::size_t previous = 0; previous < index; ++previous) {
        const auto& other = image_.sections[previous];
        if (overlap(section.rva, section.size, other.rva, other.size)) {
          return fail("Overlapping image sections are unsupported.");
        }
      }
    }
    if (ranges_.empty() || ranges_.size() > 32) {
      return fail("Request one to 32 explicit code ranges.");
    }
    std::uint32_t total = 0;
    for (std::size_t index = 0; index < ranges_.size(); ++index) {
      const auto& range = ranges_[index];
      if (range.size == 0 || range.size > 8192 || range.size > kCodeLimit - total ||
          !section_range(range.rva, range.size, kReadable | kExecutable, kWritable | kDiscardable)) {
        return fail("A code range exceeds its byte limits or readable static executable section.");
      }
      total += range.size;
      for (std::size_t previous = 0; previous < index; ++previous) {
        const auto& other = ranges_[previous];
        if (overlap(range.rva, range.size, other.rva, other.size)) {
          return fail("Duplicate or overlapping code ranges are unsupported.");
        }
      }
    }
    return true;
  }

  /*
   * Reads a complete expected byte range through the image reader; a short or inaccessible read fails
   * verification.
   */
  bool exact(std::uint64_t rva, void* output, std::uint32_t size, bool metadata) {
    if (!within(rva, size, 0, image_.image_size)) {
      return fail("A required read extends beyond the loaded image.");
    }
    auto& used = metadata ? result_.metadata_bytes : result_.read_bytes;
    const auto limit = metadata ? kMetadataLimit : kCodeLimit;
    if (size > limit - used) {
      return fail("The code or relocation metadata read allowance was exhausted.");
    }
    used += size;
    std::uint32_t offset = 0;
    while (offset < size) {
      const auto next = static_cast<std::uint32_t>(rva + offset);
      const auto window = reader_.query(next, size - offset);
      if (!window.readable || window.size == 0 || window.size > size - offset) {
        ++result_.read_failures;
        return fail("A code-contract query window is unreadable or invalid.");
      }
      if (!reader_.read(next, static_cast<std::uint8_t*>(output) + offset, window.size)) {
        ++result_.read_failures;
        return fail("Code-contract bytes could not be read exactly.");
      }
      offset += window.size;
    }
    return true;
  }

  bool header(std::uint64_t rva, void* output, std::uint32_t size) {
    if (!within(rva, size, 0, header_limit_)) {
      return fail("A PE header read reaches a declared image section.");
    }
    return exact(rva, output, size, true);
  }

  /*
   * Validates live PE headers and required directories before section/relocation information can be trusted.
   *
   * PE32+ NumberOfRvaAndSizes is optional+108; base-relocation data directory index5 is optional+152. Blocks
   * and relocation field widths below follow:
   * https://learn.microsoft.com/en-us/windows/win32/debug/pe-format
   */
  bool headers() {
    std::array<std::uint8_t, 64> dos{};
    if (!header(0, dos.data(), dos.size())) {
      return false;
    }
    const auto pe = u32(dos.data() + 60);
    if (u16(dos.data()) != 0x5a4d || pe < 64 || pe > 1024 * 1024) {
      return fail("Invalid DOS signature or PE header offset.");
    }
    std::array<std::uint8_t, 24> coff{};
    if (!header(pe, coff.data(), coff.size())) {
      return false;
    }
    const auto optional_size = u16(coff.data() + 20);
    if (u32(coff.data()) != 0x4550 || u16(coff.data() + 4) != image_.machine || u16(coff.data() + 6) != image_.section_count ||
        u32(coff.data() + 8) != image_.timestamp || optional_size < 112 || optional_size > 4096) {
      return fail("Fresh PE identity or optional-header bounds differ from the validated image.");
    }
    const auto optional_rva = std::uint64_t(pe) + 24;
    std::array<std::uint8_t, 112> optional{};
    if (!header(optional_rva, optional.data(), optional.size())) {
      return false;
    }
    const auto header_size = u32(optional.data() + 60);
    const auto directories = u32(optional.data() + 108);
    if (u16(optional.data()) != 0x20b || u32(optional.data() + 56) != image_.image_size || u32(optional.data() + 64) != image_.checksum ||
        header_size == 0 || header_size > image_.image_size || header_size > header_limit_ ||
        !within(optional_rva, std::uint64_t(optional_size) + image_.section_count * 40, 0, header_size) ||
        directories > (optional_size - 112u) / 8u) {
      return fail("The PE32+ header has invalid identity, directory or section-table bounds.");
    }
    if (directories > 5) {
      std::array<std::uint8_t, 8> directory{};
      if (!header(optional_rva + 152, directory.data(), directory.size())) {
        return false;
      }
      result_.relocation_rva = u32(directory.data());
      result_.relocation_size = u32(directory.data() + 4);
    }
    return true;
  }

  bool relocation_data(std::uint64_t rva, void* output, std::uint32_t size) {
    if (!within(rva, size, result_.relocation_rva, result_.relocation_size) || !section_range(rva, size, kReadable, kWritable)) {
      return fail("A relocation metadata read escapes its declared readable data extent.");
    }
    return exact(rva, output, size, true);
  }

  /*
   * Walks bounded PE relocation blocks and records only supported relocation entries. Malformed block sizes
   * or out-of-image targets invalidate the fingerprint model.
   *
   * On AMD64 the largest supported relocated field is DIR64 (eight bytes). Its start offset can be 4095,
   * extending seven bytes into the next page.
   *
   * ABSOLUTE padding performs no relocation.
   */
  bool relocations() {
    if (result_.relocation_rva == 0 && result_.relocation_size == 0) {
      return true;
    }
    if (result_.relocation_rva == 0 || result_.relocation_size < 8 || (result_.relocation_rva & 3) != 0 ||
        !section_range(result_.relocation_rva, result_.relocation_size, kReadable, kWritable)) {
      return fail("The base-relocation directory is invalid or outside readable image data.");
    }
    if (std::any_of(ranges_.begin(), ranges_.end(),
                    [&](const auto& range) { return overlap(range.rva, range.size, result_.relocation_rva, result_.relocation_size); })) {
      return fail("A requested code range overlaps relocation metadata storage.");
    }
    std::uint32_t offset = 0;
    while (offset < result_.relocation_size) {
      const auto block_rva = std::uint64_t(result_.relocation_rva) + offset;
      std::array<std::uint8_t, 8> block{};
      if ((block_rva & 3) != 0 || !relocation_data(block_rva, block.data(), block.size())) {
        if (result_.error.empty()) {
          fail("A relocation block is not aligned on a 32-bit boundary.");
        }
        return false;
      }
      const auto page = u32(block.data());
      const auto block_size = u32(block.data() + 4);
      if ((page & 4095) != 0 || page >= image_.image_size || block_size < 8 || (block_size & 1) != 0 ||
          block_size > result_.relocation_size - offset) {
        return fail("A base-relocation block has invalid page, size or directory bounds.");
      }
      ++result_.relocation_blocks;
      const bool relevant = std::any_of(ranges_.begin(), ranges_.end(),
                                        [page](const CodeRange& range) { return overlap(page, 4096 + 7, range.rva, range.size); });
      if (!relevant) {
        ++result_.relocation_blocks_skipped;
        offset += block_size;
        continue;
      }
      std::uint32_t consumed = 8;
      std::array<std::uint8_t, 512> entries{};
      while (consumed < block_size) {
        const auto size = std::min<std::uint32_t>(entries.size(), block_size - consumed);
        if (!relocation_data(block_rva + consumed, entries.data(), size)) {
          return false;
        }
        for (std::uint32_t index = 0; index < size; index += 2) {
          ++result_.relocation_entries;
          const auto entry = u16(entries.data() + index);
          const auto type = entry >> 12;
          if (type == 0) {
            continue;
          }
          std::uint32_t width = 0;
          if (type == 3) {
            width = 4;
          } else if (type == 10) {
            width = 8;
          } else {
            return fail("An unsupported base-relocation kind prevents proving code-byte stability.");
          }
          const auto site = std::uint64_t(page) + (entry & 4095);
          if (!within(site, width, 0, image_.image_size)) {
            return fail("A relocated field extends beyond the loaded image.");
          }
          for (const auto& range : ranges_) {
            if (overlap(site, width, range.rva, range.size)) {
              return fail("A requested code range contains bytes affected by a base relocation.");
            }
          }
        }
        consumed += size;
      }
      offset += block_size;
    }
    return true;
  }

  discovery::ImageReader& reader_;
  const discovery::Inventory& image_;
  const std::vector<CodeRange>& ranges_;
  CodeContractInventory result_;
  std::uint32_t header_limit_ = 0xffffffff;
};

}  /* namespace */

CodeContractInventory inspect_code_contract(discovery::ImageReader& reader,
                                            const discovery::Inventory& image,
                                            const std::vector<CodeRange>& ranges) {
  return Parser(reader, image, ranges).run();
}

/*
 * Rereads and verifies the supplied code proof against the current image. A successful earlier discovery
 * cannot authorize a call after the native code has changed.
 */
CodeContractInventory verify_code_contract(discovery::ImageReader& reader,
                                           const discovery::Inventory& image,
                                           const std::vector<CodeFingerprint>& expected) {
  CodeContractInventory invalid;
  if (expected.empty() || expected.size() > 32) {
    invalid.error = "Verify one to 32 required code fingerprints.";
    return invalid;
  }
  std::vector<CodeRange> ranges;
  ranges.reserve(expected.size());
  for (const auto& fingerprint : expected) {
    if (fingerprint.hash == 0) {
      invalid.error = "A zero placeholder hash cannot establish a code contract.";
      return invalid;
    }
    ranges.push_back({fingerprint.rva, fingerprint.size});
  }
  auto observed = inspect_code_contract(reader, image, ranges);
  if (!observed.valid) {
    return observed;
  }
  for (std::size_t index = 0; index < expected.size(); ++index) {
    const auto& actual = observed.records[index];
    const auto& required = expected[index];
    if (actual.rva != required.rva || actual.size != required.size || actual.hash != required.hash) {
      observed.valid = false;
      char detail[192]{};
      std::snprintf(detail, sizeof(detail), "Private code signature changed or moved at RVA 0x%08X (%u bytes). Update AZP Camera.",
                    static_cast<unsigned>(required.rva), static_cast<unsigned>(required.size));
      observed.error = detail;
      return observed;
    }
  }
  return observed;
}

}  /* namespace azp::native_camera */
