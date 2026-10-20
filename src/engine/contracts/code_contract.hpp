/*
 * Defines code-fingerprint requests and verification results. The implementation reads through the supplied
 * image reader and reports why a proof fails before any associated engine operation is used.
 */

#pragma once
#include "../image_inventory.hpp"

namespace azp::native_camera {

struct CodeRange {
  std::uint32_t rva = 0;
  std::uint32_t size = 0;
};

struct CodeFingerprint {
  std::uint32_t rva = 0;
  std::uint32_t size = 0;
  std::uint64_t hash = 0;
};

struct CodeContractInventory {
  bool valid = false;
  std::string error;
  std::uint32_t read_bytes = 0;      /* Attempted code bytes, maximum 65536. */
  std::uint32_t metadata_bytes = 0;  /* Attempted PE/relocation bytes, maximum 1MiB. */
  std::uint32_t read_failures = 0;
  bool relocations_checked = false;
  std::uint32_t relocation_rva = 0;
  std::uint32_t relocation_size = 0;
  std::uint32_t relocation_entries = 0;
  std::uint32_t relocation_blocks = 0;
  std::uint32_t relocation_blocks_skipped = 0;
  /*
   * Successful earlier ranges survive a later failure; none are a usable contract unless valid is true. No
   * raw bytes or absolute addresses escape.
   */
  std::vector<CodeFingerprint> records;
};

/*
 * Fingerprints one to 32 non-overlapping explicit ranges, each 1..8192 bytes, with at most 65536 aggregate
 * code bytes. Every complete range must fit within one declared readable, non-writable, executable,
 * non-discardable image section. Hash is FNV-1a64 over the observed loaded bytes, not a cryptographic
 * signature. Fresh AMD64 PE32+ header identity and base-relocation metadata are checked first. Any
 * non-ABSOLUTE relocation affecting a requested byte is refused. Every relocation block header is checked.
 * Payloads are read only for pages that could affect a selected range, including a DIR64 field crossing up
 * to seven bytes into the next page. In those blocks, supported types are ABSOLUTE, HIGHLOW and DIR64; other
 * types fail conservatively. Skipped payloads are not validated. The byte allowance limits actual attempted
 * reads, not directory size; no block-count or read limit is bypassed. Storage must be readable,
 * non-writable, image data. The exact PE directory can reside in an executable section; parsing that
 * metadata never executes it. Its discardable flag is permitted only while query/read still succeeds.
 * Missing/inaccessible/malformed metadata cannot be bypassed, and no target pointer is followed or function
 * invoked. This detects changed bytes at inspection time, not future modification, authenticity, a calling
 * convention, thread safety or safe engine lifetime.
 */
CodeContractInventory inspect_code_contract(discovery::ImageReader& reader,
                                            const discovery::Inventory& image,
                                            const std::vector<CodeRange>& ranges);

/*
 * Requires every expected range/hash in the supplied order. Empty manifests, duplicate/overlapping ranges
 * and zero placeholder hashes refuse before reads.
 */
CodeContractInventory verify_code_contract(discovery::ImageReader& reader,
                                           const discovery::Inventory& image,
                                           const std::vector<CodeFingerprint>& expected);

}  /* namespace azp::native_camera */
