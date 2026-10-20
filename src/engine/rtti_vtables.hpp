/*
 * Declares the bounded RTTI/vtable resolver and its requested type descriptions. A resolved table
 * establishes type/layout evidence; separate instruction contracts still prove the functions used for engine
 * calls.
 */

#pragma once
#include "image_inventory.hpp"

namespace azp::native_camera {

struct VtableMethod {
  std::uint32_t offset = 0, method_rva = 0;
};

struct VtableRequest {
  std::string type_name;
  std::uint32_t extent = 8;
  std::vector<VtableMethod> methods;
  std::uint32_t resolved_vtable_rva = 0;
};

struct RttiVtables {
  bool valid = false;
  std::string error;
  /*
   * Request order. Empty on every failure, including a changing reread.
   */
  std::vector<std::uint32_t> vtables;
  std::uint64_t scanned_bytes = 0, read_bytes = 0;
  std::uint32_t read_calls = 0, candidates = 0, vtable_candidates = 0;
};

/*
 * Caller supplies fresh validated AMD64 PE metadata and reviewed method targets, constructor-derived vtable
 * roots, or exact type names. Methods/root take precedence over names; obfuscated names need not be parsed.
 * Only primary (zero-offset) RTTI identities are considered. No native calls, OS access or object/heap
 * reads. This proves neither ABI nor ownership. At most 16 requests, 64 MiB static data per pass (two
 * passes), 16 MiB targeted metadata reads, 131072 read calls, 128 matched COLs and 4096 pointer candidates.
 */
RttiVtables resolve_rtti_vtables(discovery::ImageReader& reader,
                                 const discovery::Inventory& image,
                                 std::uint64_t image_base,
                                 const std::vector<VtableRequest>& requests);

}  /* namespace azp::native_camera */
