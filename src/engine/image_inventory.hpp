/*
 * Defines the executable inventory and its bounded reader interface. Resolvers use section ranges and
 * metadata to constrain searches before proving an engine function's full contract.
 */

#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace azp::discovery {

struct ReadWindow {
  /*
   * A nonzero bounded extent beginning at the requested RVA, even for a hole.
   */
  std::uint32_t size = 0;
  bool readable = false;
};

class ImageReader {
 public:
  virtual ~ImageReader() = default;
  virtual ReadWindow query(std::uint32_t rva, std::uint32_t maximum) = 0;
  virtual bool read(std::uint32_t rva, void* destination, std::size_t size) = 0;
};

struct Limits {
  /*
   * Caller values are clamped to hard limits; no unbounded mode is available.
   */
  std::uint32_t scan_bytes = 16 * 1024 * 1024;
  std::uint32_t metadata_bytes = 2 * 1024 * 1024;
  std::uint32_t records = 128;
  std::uint32_t export_names = 4096;
  std::uint32_t string_bytes = 240;
};

enum class RecordKind { ascii_literal, export_name, forwarded_export };

struct Record {
  RecordKind kind = RecordKind::ascii_literal;
  std::string text;
  std::string keyword;
  std::string section;
  std::uint32_t rva = 0;
  /*
   * Export target metadata only; the target can be code or data. A forwarded export's RVA points to a
   * string, not executable code. No record describes a calling convention or signature.
   */
  std::uint32_t function_rva = 0;
  std::uint32_t ordinal = 0;
};

struct ImageSection {
  std::string name;
  std::uint32_t rva = 0;
  std::uint32_t size = 0;
  std::uint32_t flags = 0;
};

struct Inventory {
  bool valid_image = false;
  std::string error;
  std::uint16_t machine = 0;
  std::uint16_t section_count = 0;
  std::uint32_t timestamp = 0;
  std::uint32_t image_size = 0;
  std::uint32_t checksum = 0;
  std::uint32_t exception_rva = 0;
  std::uint32_t exception_size = 0;
  std::uint32_t load_config_rva = 0;
  std::uint32_t load_config_size = 0;
  std::uint16_t dll_characteristics = 0;
  std::vector<ImageSection> sections;
  std::uint64_t scanned_bytes = 0;
  std::uint64_t metadata_bytes = 0;
  std::uint64_t skipped_bytes = 0;
  std::uint32_t read_failures = 0;
  std::uint32_t malformed_exports = 0;
  bool scan_truncated = false;
  bool exports_truncated = false;
  bool record_limit_reached = false;
  std::vector<Record> records;
};

/*
 * Parses an AMD64 PE32+ image in LOADED layout. All addresses are RVAs, never file offsets. Reads only
 * headers and declared non-writable, readable sections. Does not enumerate modules, write files, resolve
 * functions or call found code.
 */
Inventory inspect_image(ImageReader& reader, Limits limits = {});

}  /* namespace azp::discovery */
