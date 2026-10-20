/*
 * Provides checked reads of the current process and validates key engine identities. Short-lived query
 * caches reduce repeated page inspections, but endpoint revalidation is required before publishing results
 * or making a native call.
 */

#include "memory.hpp"
#include <psapi.h>
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string_view>
#include "aircraft_scene_pose.hpp"

/*
 * Declarations only: the optional Windows 10 RS1 API is resolved at runtime.
 */
#ifndef NTDDI_VERSION
#define NTDDI_VERSION 0x0A000002
#endif

namespace azp::native_camera {
namespace {

constexpr std::uint32_t kMaximumImageSize = 0x80000000;
constexpr std::uint32_t kHeaderExtent = 65536;
constexpr std::uint32_t kHeaderReadBudget = 8192;

thread_local ScopedLocalMemoryQueryCache* active_query_cache = nullptr;

/*
 * Obtains fresh access metadata for an address in the current process and records the Windows failure code
 * if querying fails.
 *
 * Use the documented explicit-process entry point with only the current process pseudo-handle. The MBI
 * contract and all caller validation remain unchanged; no handle/PID or lower-level syscall path is
 * configurable here.
 */
SIZE_T query_memory_uncached(const void* address, MEMORY_BASIC_INFORMATION& region, DWORD* error = nullptr) noexcept {
  const auto result = VirtualQueryEx(GetCurrentProcess(), address, &region, sizeof(region));
  if (error)
    *error = result == 0 ? GetLastError() : ERROR_SUCCESS;

  return result;
}

/*
 * Uses the already loaded Windows allocation-query API when available. It does not load an alternative DLL
 * or inspect a different process.
 *
 * KernelBase is a loaded Windows component; never search/load a DLL from disk.
 */
bool query_allocation(const void* address, WIN32_MEMORY_REGION_INFORMATION& region, DWORD& error) noexcept {
  static const auto query = reinterpret_cast<decltype(&QueryVirtualMemoryInformation)>(
      GetProcAddress(GetModuleHandleW(L"kernelbase.dll"), "QueryVirtualMemoryInformation"));
  if (!query) {
    error = ERROR_CALL_NOT_IMPLEMENTED;
    return false;
  }
  SIZE_T returned = 0;
  const auto result = query(GetCurrentProcess(), address, MemoryRegionInfo, &region, sizeof(region), &returned);
  error = !result ? GetLastError() : returned != sizeof(region) ? ERROR_BAD_LENGTH : ERROR_SUCCESS;

  return result && returned == sizeof(region);
}

bool query_pages(PSAPI_WORKING_SET_EX_INFORMATION* pages, std::size_t count, DWORD& error) noexcept {
  const auto result = K32QueryWorkingSetEx(GetCurrentProcess(), pages, static_cast<DWORD>(count * sizeof(*pages)));
  error = !result ? GetLastError() : ERROR_SUCCESS;

  return result != FALSE;
}

std::size_t page_size() noexcept {
  static const auto size = [] {
    SYSTEM_INFO info{};
    GetSystemInfo(&info);
    return std::size_t(info.dwPageSize);
  }();
  return size;
}

bool contains(std::uintptr_t base, std::size_t extent, std::uintptr_t address, std::size_t size) noexcept {
  return extent && extent <= std::numeric_limits<std::uintptr_t>::max() - base && base <= address && address - base < extent &&
         size <= extent - (address - base);
}

bool ordinary_private(const WIN32_MEMORY_REGION_INFORMATION& region) noexcept {
  return region.Private && !region.MappedDataFile && !region.MappedImage && !region.MappedPageFile && !region.MappedPhysical &&
         !region.DirectMapped;
}

bool allocation_type(const WIN32_MEMORY_REGION_INFORMATION& region, DWORD type) noexcept {
  if (type == MEM_PRIVATE)
    return ordinary_private(region);
  return type == MEM_IMAGE && region.MappedImage && !region.Private && !region.MappedDataFile && !region.MappedPageFile &&
         !region.MappedPhysical && !region.DirectMapped;
}

SIZE_T query_memory(const void* address, MEMORY_BASIC_INFORMATION& region) noexcept {
  return active_query_cache ? active_query_cache->query(address, region) : query_memory_uncached(address, region);
}

bool read_memory(const void* address, void* destination, SIZE_T size, SIZE_T& copied) noexcept {
  const bool result = ReadProcessMemory(GetCurrentProcess(), address, destination, size, &copied) != FALSE;
  const auto error = !result && active_query_cache ? GetLastError() : ERROR_SUCCESS;

  if ((!result || copied != size) && active_query_cache)
    active_query_cache->fail(address, size, copied, error);
  return result;
}

/*
 * https://learn.microsoft.com/en-us/windows/win32/memory/memory-protection-constants
 */
bool readable(DWORD protection) noexcept {
  if ((protection & (PAGE_GUARD | PAGE_NOACCESS)) != 0)
    return false;
  const auto basic = protection & 0xff;
  return basic == PAGE_READONLY || basic == PAGE_READWRITE || basic == PAGE_WRITECOPY || basic == PAGE_EXECUTE_READ ||
         basic == PAGE_EXECUTE_READWRITE || basic == PAGE_EXECUTE_WRITECOPY;
}

bool region_contains(const MEMORY_BASIC_INFORMATION& region, std::uintptr_t address, std::size_t size) noexcept {
  const auto start = reinterpret_cast<std::uintptr_t>(region.BaseAddress);
  return region.RegionSize != 0 && start <= address && region.RegionSize <= std::numeric_limits<std::uintptr_t>::max() - start &&
         address - start < region.RegionSize && size <= region.RegionSize - (address - start);
}

/*
 * Checks one page of an isolated private read with the same evidence as a private-pages transaction: the
 * allocation is ordinary private memory containing the page, and the page is resident, not bad and readable.
 * VirtualQuery is avoided because its cost grows with the size of the region it describes (about 1 ms per
 * 512 MB, 20 ms for a 2 GB committed region), and the simulator's heaps are that large. A non-resident page
 * has no working-set attributes; the caller then falls back to the exact region query.
 */
bool resident_private_page(std::uintptr_t page, std::size_t page_bytes) noexcept {
  WIN32_MEMORY_REGION_INFORMATION allocation{};
  DWORD error = ERROR_SUCCESS;
  if (!query_allocation(reinterpret_cast<const void*>(page), allocation, error) || !ordinary_private(allocation) ||
      !contains(reinterpret_cast<std::uintptr_t>(allocation.AllocationBase), allocation.RegionSize, page, page_bytes))
    return false;
  PSAPI_WORKING_SET_EX_INFORMATION info{};
  info.VirtualAddress = reinterpret_cast<void*>(page);
  return query_pages(&info, 1, error) && info.VirtualAttributes.Valid && !info.VirtualAttributes.Bad &&
         readable(static_cast<DWORD>(info.VirtualAttributes.Win32Protection));
}

/*
 * Validates every part of a private-memory range before copying it. A scoped page proof may reduce repeated
 * queries but cannot bypass the bounded read. Outside any transaction, resident pages are checked page by
 * page (resident_private_page); other pages use the exact region query.
 */
bool private_bytes(std::uintptr_t address, std::uint8_t* temporary, std::size_t size) noexcept {
  if (active_query_cache && active_query_cache->uses_private_pages()) {
    if (!active_query_cache->validate_private_range(address, size))
      return false;
    SIZE_T copied = 0;
    return read_memory(reinterpret_cast<const void*>(address), temporary, size, copied) && copied == size;
  }
  const auto page_bytes = page_size();
  std::size_t offset = 0;
  while (offset < size) {
    const auto current = address + offset;
    const auto page = page_bytes ? current - current % page_bytes : current;
    std::size_t chunk = 0;
    if (!active_query_cache && page_bytes && resident_private_page(page, page_bytes)) {
      chunk = std::min(size - offset, page_bytes - (current - page));
    } else {
      MEMORY_BASIC_INFORMATION region{};
      if (query_memory(reinterpret_cast<const void*>(current), region) != sizeof(region) || region.State != MEM_COMMIT ||
          region.Type != MEM_PRIVATE || !readable(region.Protect) || !region_contains(region, current, 1))
        return false;
      const auto available = region.RegionSize - (current - reinterpret_cast<std::uintptr_t>(region.BaseAddress));
      chunk = std::min(size - offset, available);
    }
    SIZE_T copied = 0;
    if (!read_memory(reinterpret_cast<const void*>(current), temporary + offset, chunk, copied) || copied != chunk)
      return false;
    offset += chunk;
  }
  return true;
}

/*
 * Keep the large temporary off the frequent scalar-read stack frame. No output is published if any part
 * fails, and no byte outside the field is requested.
 */
__declspec(noinline) bool large_private_field(std::uintptr_t address, void* destination, std::size_t size) noexcept {
  std::array<std::uint8_t, kLocalObjectFieldLimit> bytes;
  if (!private_bytes(address, bytes.data(), size))
    return false;
  std::memcpy(destination, bytes.data(), size);
  return true;
}

std::uint16_t u16(const std::uint8_t* bytes) {
  return static_cast<std::uint16_t>(bytes[0] | (std::uint16_t(bytes[1]) << 8));
}

std::uint32_t u32(const std::uint8_t* bytes) {
  std::uint32_t value = 0;
  for (unsigned index = 0; index < 4; ++index)
    value |= std::uint32_t(bytes[index]) << (index * 8);
  return value;
}

bool within(std::uint32_t start, std::uint32_t size, std::uint32_t bound) {
  return start <= bound && size <= bound - start;
}

class HeaderReader {
 public:
  HeaderReader(discovery::ImageReader& reader, discovery::Inventory& result) : reader_(reader), result_(result) {}
  /*
   * Copies an image range in checked pieces across region boundaries. Every piece must remain inside the
   * allowed image and readable region.
   */
  bool read(std::uint32_t rva, void* destination, std::uint32_t size) {
    if (size == 0 || !within(rva, size, kHeaderExtent) || size > kHeaderReadBudget - result_.metadata_bytes) {
      result_.error = "A header request exceeds the fixed extent or metadata allowance.";
      return false;
    }
    result_.metadata_bytes += size;
    auto* output = static_cast<std::uint8_t*>(destination);
    std::uint32_t offset = 0;
    while (offset < size) {
      const auto window = reader_.query(rva + offset, size - offset);
      if (!window.readable || window.size == 0 || window.size > size - offset ||
          !reader_.read(rva + offset, output + offset, window.size)) {
        ++result_.read_failures;
        result_.error = "A required main-image header field could not be read exactly.";
        return false;
      }
      offset += window.size;
    }
    return true;
  }

 private:
  discovery::ImageReader& reader_;
  discovery::Inventory& result_;
};

bool directory_valid(const discovery::Inventory& image, std::uint32_t rva, std::uint32_t size) {
  if (rva == 0 || size == 0)
    return rva == 0 && size == 0;
  return within(rva, size, image.image_size) && std::any_of(image.sections.begin(), image.sections.end(), [&](const auto& section) {
           return rva >= section.rva && within(rva - section.rva, size, section.size);
         });
}

}  /* namespace */

/*
 * Proves current writable private memory for a prospective native mutation, using allocation/page metadata
 * or a complete region walk. It deliberately avoids cached write permission.
 *
 * AllocationProtect is deliberately not used as current page permission.
 *
 * No cache: a flag mutation/private call must never inherit an old writable proof. Cold pages and
 * unavailable APIs retain the complete original walk.
 */
bool writable_private_span(std::uint64_t address, std::size_t size) noexcept {
  if (!address || !size || size > 16 || address > std::numeric_limits<std::uintptr_t>::max() - size)
    return false;
  const auto value = static_cast<std::uintptr_t>(address);
  const auto page_bytes = page_size();
  if (!page_bytes)
    return false;
  WIN32_MEMORY_REGION_INFORMATION allocation{};
  DWORD error = ERROR_SUCCESS;
  if (query_allocation(reinterpret_cast<const void*>(value), allocation, error)) {
    if (!allocation.AllocationBase || !ordinary_private(allocation) ||
        !contains(reinterpret_cast<std::uintptr_t>(allocation.AllocationBase), allocation.RegionSize, value, size))
      return false;
    std::array<PSAPI_WORKING_SET_EX_INFORMATION, 2> pages{};
    const auto first = value - value % page_bytes;
    const auto last = value + size - 1 - (value + size - 1) % page_bytes;
    const std::size_t count = first == last ? 1 : 2;
    pages[0].VirtualAddress = reinterpret_cast<void*>(first);
    pages[1].VirtualAddress = reinterpret_cast<void*>(last);
    if (query_pages(pages.data(), count, error)) {
      bool all_resident = true;
      for (std::size_t index = 0; index < count; ++index) {
        const auto& attributes = pages[index].VirtualAttributes;
        if (!attributes.Valid) {
          all_resident = false;
          continue;
        }
        if (attributes.Bad || attributes.Win32Protection != PAGE_READWRITE)
          return false;
      }
      if (all_resident)
        return true;
    }
  }
  const auto end = value + size;
  std::uintptr_t current = value;
  void* owner = nullptr;
  while (current < end) {
    MEMORY_BASIC_INFORMATION region{};
    if (query_memory_uncached(reinterpret_cast<const void*>(current), region) != sizeof(region) || region.State != MEM_COMMIT ||
        region.Type != MEM_PRIVATE || region.Protect != PAGE_READWRITE || !region.AllocationBase ||
        (owner && owner != region.AllocationBase) || !region_contains(region, current, 1))
      return false;
    owner = region.AllocationBase;
    current = std::min(end, reinterpret_cast<std::uintptr_t>(region.BaseAddress) + region.RegionSize);
  }
  return true;
}

/*
 * Reads the bounded flag-word range needed for a compare/exchange update. Overflow or incomplete access
 * refuses the mutation.
 */
bool read_local_flag_words(std::uint64_t address, std::array<std::uint64_t, 2>& flags) noexcept {
  std::array<std::uint64_t, 2> temporary{};
  if (!address || address > std::numeric_limits<std::uintptr_t>::max() - sizeof(temporary))
    return false;
  SIZE_T copied = 0;
  if (!read_memory(reinterpret_cast<const void*>(static_cast<std::uintptr_t>(address)), temporary.data(), sizeof(temporary), copied) ||
      copied != sizeof(temporary))
    return false;
  flags = temporary;
  return true;
}

ScopedLocalMemoryQueryCache::ScopedLocalMemoryQueryCache(LocalMemoryQueryMode mode) noexcept : previous_(active_query_cache), mode_(mode) {
  active_query_cache = this;
}

ScopedLocalMemoryQueryCache::~ScopedLocalMemoryQueryCache() {
  if (active_)
    active_query_cache = previous_;
}

/*
 * Formats a bounded failure description using reviewed numeric metadata. Unreviewed address-valued fields
 * are not exposed by this status formatter.
 *
 * User reports need the differing metadata, not heap/allocation addresses. Explicitly allow numeric metadata
 * fields so new address-valued fields remain private until their formatting is reviewed.
 */
std::string describe_local_memory_query_failure(const LocalMemoryQueryFailure& failure) {
  std::array<char, 256> detail{};
  constexpr std::array<std::string_view, 9> numeric_fields{
      "exact_read", "region_count", "query_size", "contains_address", "allocation_protect", "region_size", "state", "protect", "type"};
  if (std::find(numeric_fields.begin(), numeric_fields.end(), failure.field) != numeric_fields.end()) {
    std::snprintf(detail.data(), detail.size(), "stage=%s region=%zu field=%s expected=0x%llx observed=0x%llx error=%lu", failure.stage,
                  failure.region_index, failure.field, static_cast<unsigned long long>(failure.expected),
                  static_cast<unsigned long long>(failure.observed), static_cast<unsigned long>(failure.system_error));
  } else {
    std::snprintf(detail.data(), detail.size(), "stage=%s region=%zu field=%s expected=<redacted> observed=<redacted> error=%lu",
                  failure.stage, failure.region_index, failure.field, static_cast<unsigned long>(failure.system_error));
  }
  return detail.data();
}

/*
 * Latches the first query inconsistency and associates it with the saved region evidence. A later read
 * cannot clear the failed transaction.
 */
void ScopedLocalMemoryQueryCache::fail(const void* address, SIZE_T requested, SIZE_T copied, DWORD error) noexcept {
  if (!failed_) {
    const auto value = reinterpret_cast<std::uintptr_t>(address);
    std::size_t index = 0;
    while (index < count_ && !region_contains(regions_[index], value, 1))
      ++index;
    failure_ = {"read", "exact_read", value, index, requested, copied, error};
  }
  failed_ = true;
}

/*
 * Reuses region metadata only inside this inspection and only when it fully covers the requested field. A
 * final fresh comparison is mandatory before the inspection can be accepted.
 *
 * VirtualQuery reports only the suffix beginning at its queried page. Probe the containing 64 KiB window
 * first so descending graph fields can share one observation. This queries metadata only: no extra bytes are
 * read. The result is usable only when it contains the actual field; a preceding guard, reservation or
 * protection split falls back to the exact requested address. There is no merging, eviction or reuse beyond
 * this transaction. finish() freshly compares every saved region, including any newly observed prefix pages.
 */
SIZE_T ScopedLocalMemoryQueryCache::query(const void* address, MEMORY_BASIC_INFORMATION& region) noexcept {
  if (!active_ || failed_)
    return 0;
  const auto value = reinterpret_cast<std::uintptr_t>(address);
  for (std::size_t i = 0; i < count_; ++i) {
    if (region_contains(regions_[i], value, 1)) {
      region = regions_[i];

      return sizeof(region);
    }
  }
  if (count_ == regions_.size()) {
    failure_ = {"capacity", "region_count", value, count_, regions_.size(), count_ + 1};
    failed_ = true;
    return 0;
  }
  constexpr std::uintptr_t window_size = 65536;
  const auto window_start = value - value % window_size;
  DWORD error = ERROR_SUCCESS;
  auto queried = query_memory_uncached(reinterpret_cast<const void*>(window_start), region, &error);
  if (queried != sizeof(region) || !region_contains(region, value, 1)) {
    if (window_start != value)
      queried = query_memory_uncached(address, region, &error);
    if (queried != sizeof(region) || !region_contains(region, value, 1)) {
      failure_ = {"query",
                  queried != sizeof(region) ? "query_size" : "contains_address",
                  value,
                  count_,
                  queried != sizeof(region) ? sizeof(region) : 1,
                  queried != sizeof(region) ? queried : 0,
                  error};
      failed_ = true;
      return 0;
    }
  }
  regions_[count_++] = region;
  return sizeof(region);
}

bool ScopedLocalMemoryQueryCache::is_current() const noexcept {
  return active_ && !failed_ && active_query_cache == this;
}

bool ScopedLocalMemoryQueryCache::refuse(const char* stage,
                                         const char* field,
                                         std::uintptr_t address,
                                         std::size_t index,
                                         std::uint64_t expected,
                                         std::uint64_t observed,
                                         DWORD error) noexcept {
  if (!failed_)
    failure_ = {stage, field, address, index, expected, observed, error};
  failed_ = true;
  return false;
}

bool ScopedLocalMemoryQueryCache::validate_private_range(std::uintptr_t address, std::size_t size) noexcept {
  return validate_page_range(address, size, MEM_PRIVATE, 0);
}

bool ScopedLocalMemoryQueryCache::validate_image_range(std::uintptr_t address, std::size_t size, std::uintptr_t module) noexcept {
  if (!module || module != reinterpret_cast<std::uintptr_t>(GetModuleHandleW(nullptr)))
    return false;
  return validate_page_range(address, size, MEM_IMAGE, module);
}

/*
 * Checks each page/allocation in the requested range and stores bounded proof for endpoint validation.
 * Capacity limits switch new reads to the uncached path without evicting earlier evidence.
 *
 * Preserve previous proofs at capacity. Only new pages use the old path; no eviction, early acceptance, or
 * cross-inspection metadata is allowed.
 */
bool ScopedLocalMemoryQueryCache::validate_page_range(std::uintptr_t address,
                                                      std::size_t size,
                                                      DWORD type,
                                                      std::uintptr_t allocation) noexcept {
  if (!is_current() || !size || size > std::numeric_limits<std::uintptr_t>::max() - address)
    return false;
  const auto page_bytes = page_size();
  if (!page_bytes)
    return refuse("page", "region_size", address, 0, 1, 0);
  std::size_t offset = 0;
  while (offset < size) {
    const auto current = address + offset;
    const auto base = current - current % page_bytes;
    const auto chunk = std::min(size - offset, page_bytes - (current - base));
    std::size_t p = 0;
    while (p < page_count_ && pages_[p].base != base)
      ++p;
    if (p < page_count_) {
      const auto& owner = allocations_[pages_[p].allocation];
      if (owner.type != type || (allocation && owner.base != allocation))
        return refuse("page", "allocation_type", base, p, type, owner.type);

      offset += chunk;
      continue;
    }
    bool recorded = false;
    const auto legacy_covers = std::any_of(regions_.begin(), regions_.begin() + count_,
                                           [&](const auto& region) { return region_contains(region, current, chunk); });
    if (!legacy_covers && uses_private_pages() && page_count_ < pages_.size()) {
      std::size_t a = 0;
      while (a < allocation_count_ && !contains(allocations_[a].base, allocations_[a].size, base, page_bytes))
        ++a;
      if (a < allocation_count_ && (allocations_[a].type != type || (allocation && allocations_[a].base != allocation)))
        return refuse("page", "allocation_type", base, a, type, allocations_[a].type);
      DWORD error = ERROR_SUCCESS;
      if (a == allocation_count_ && a < allocations_.size()) {
        WIN32_MEMORY_REGION_INFORMATION region{};
        if (query_allocation(reinterpret_cast<const void*>(base), region, error) && allocation_type(region, type) &&
            (!allocation || reinterpret_cast<std::uintptr_t>(region.AllocationBase) == allocation) &&
            contains(reinterpret_cast<std::uintptr_t>(region.AllocationBase), region.RegionSize, base, page_bytes)) {
          allocations_[allocation_count_++] = {reinterpret_cast<std::uintptr_t>(region.AllocationBase), region.RegionSize,
                                               region.AllocationProtect, type};
        }
      }
      if (a < allocation_count_) {
        PSAPI_WORKING_SET_EX_INFORMATION page{};
        page.VirtualAddress = reinterpret_cast<void*>(base);
        if (query_pages(&page, 1, error) && page.VirtualAttributes.Valid) {
          const auto protection = static_cast<DWORD>(page.VirtualAttributes.Win32Protection);
          if (page.VirtualAttributes.Bad || !readable(protection))
            return refuse("page", "protect", base, page_count_, PAGE_READONLY, protection);
          pages_[page_count_++] = {base, protection, a};
          recorded = true;
        }
      }
    }
    if (!recorded) {
      MEMORY_BASIC_INFORMATION region{};
      if (query(reinterpret_cast<const void*>(current), region) != sizeof(region) || region.State != MEM_COMMIT || region.Type != type ||
          (allocation && reinterpret_cast<std::uintptr_t>(region.AllocationBase) != allocation) || !readable(region.Protect) ||
          !region_contains(region, current, chunk))
        return refuse("page", "allocation_readable", current, page_count_, 1, 0);
    }
    offset += chunk;
  }
  return true;
}

/*
 * Rechecks saved allocation/page identities and access at the transaction endpoint. Changes in residency
 * alone use fresh access metadata rather than being mistaken for object replacement.
 *
 * Residency is not identity. A formerly resident page can be paged out; freshly query its actual access
 * metadata instead of interpreting the invalid union or rejecting a harmless working-set transition.
 */
bool ScopedLocalMemoryQueryCache::finish_pages() noexcept {
  for (std::size_t a = 0; a < allocation_count_; ++a) {
    const auto& initial = allocations_[a];
    WIN32_MEMORY_REGION_INFORMATION current{};
    DWORD error = ERROR_SUCCESS;
    if (!query_allocation(reinterpret_cast<const void*>(initial.base), current, error))
      return refuse("allocation_endpoint", "query_size", initial.base, a, sizeof(current), 0, error);
    if (!allocation_type(current, initial.type))
      return refuse("allocation_endpoint", "type", initial.base, a, initial.type, 0);
    if (reinterpret_cast<std::uintptr_t>(current.AllocationBase) != initial.base)
      return refuse("allocation_endpoint", "allocation_base", initial.base, a, initial.base,
                    reinterpret_cast<std::uintptr_t>(current.AllocationBase));
    if (current.RegionSize != initial.size)
      return refuse("allocation_endpoint", "region_size", initial.base, a, initial.size, current.RegionSize);
    if (current.AllocationProtect != initial.protection)
      return refuse("allocation_endpoint", "allocation_protect", initial.base, a, initial.protection, current.AllocationProtect);
  }
  if (!page_count_)
    return true;
  std::array<PSAPI_WORKING_SET_EX_INFORMATION, kPageLimit> current{};
  for (std::size_t p = 0; p < page_count_; ++p)
    current[p].VirtualAddress = reinterpret_cast<void*>(pages_[p].base);
  DWORD error = ERROR_SUCCESS;
  const auto queried = query_pages(current.data(), page_count_, error);
  for (std::size_t p = 0; p < page_count_; ++p) {
    const auto& initial = pages_[p];
    DWORD protection = 0;
    if (queried && current[p].VirtualAttributes.Valid) {
      if (current[p].VirtualAttributes.Bad)
        return refuse("page_endpoint", "page_bad", initial.base, p, 0, 1);
      protection = static_cast<DWORD>(current[p].VirtualAttributes.Win32Protection);
    } else {
      MEMORY_BASIC_INFORMATION region{};
      const auto& allocation = allocations_[initial.allocation];
      if (query_memory_uncached(reinterpret_cast<const void*>(initial.base), region, &error) != sizeof(region) ||
          !region_contains(region, initial.base, page_size()) || region.State != MEM_COMMIT || region.Type != allocation.type)
        return refuse("page_endpoint", "allocation_committed", initial.base, p, 1, 0, error);
      if (reinterpret_cast<std::uintptr_t>(region.AllocationBase) != allocation.base || region.AllocationProtect != allocation.protection)
        return refuse("page_endpoint", "allocation_base", initial.base, p, allocation.base,
                      reinterpret_cast<std::uintptr_t>(region.AllocationBase));
      protection = region.Protect;
    }
    if (protection != initial.protection || !readable(protection))
      return refuse("page_endpoint", "protect", initial.base, p, initial.protection, protection);
  }
  return true;
}

/*
 * Validates all saved regions and page evidence at the end of inspection. Overlapping suffix observations
 * are compared consistently; conflicting evidence fails the entire result.
 *
 * Descending graph reads can save overlapping suffixes of one allocation. Query the earliest base first: its
 * fresh MBI also describes the exact suffix that VirtualQueryEx would return at a later saved page. Retain
 * every original observation and comparison, including its full extent; never merge records or carry a proof
 * past this endpoint. Fixed storage only.
 *
 * A conflicting suffix fails the transaction above; another query must never rescue it. Keep the
 * encompassing fresh proof, not a shorter suffix.
 */
bool ScopedLocalMemoryQueryCache::finish() noexcept {
  if (!active_ || active_query_cache != this)
    return false;
  if (!failed_)
    finish_pages();
  std::array<std::size_t, kRegionLimit> order{};
  for (std::size_t i = 0; i < count_; ++i)
    order[i] = i;
  std::sort(order.begin(), order.begin() + count_, [&](std::size_t left, std::size_t right) {
    const auto left_base = reinterpret_cast<std::uintptr_t>(regions_[left].BaseAddress);
    const auto right_base = reinterpret_cast<std::uintptr_t>(regions_[right].BaseAddress);
    return left_base < right_base || (left_base == right_base && left < right);
  });
  MEMORY_BASIC_INFORMATION fresh{};
  for (std::size_t position = 0; position < count_ && !failed_; ++position) {
    const auto i = order[position];
    const auto& initial = regions_[i];
    MEMORY_BASIC_INFORMATION current{};
    DWORD error = ERROR_SUCCESS;
    SIZE_T queried = sizeof(current);
    const auto base = reinterpret_cast<std::uintptr_t>(initial.BaseAddress);
    if (region_contains(fresh, base, 1)) {
      current = fresh;
      current.BaseAddress = initial.BaseAddress;
      current.RegionSize -= base - reinterpret_cast<std::uintptr_t>(fresh.BaseAddress);
    } else {
      queried = query_memory_uncached(initial.BaseAddress, current, &error);
    }
    const auto differs = [&](const char* field, std::uint64_t expected, std::uint64_t observed) {
      if (expected == observed)
        return false;
      failure_ = {"endpoint", field, reinterpret_cast<std::uintptr_t>(initial.BaseAddress), i, expected, observed, error};
      return true;
    };
    if (differs("query_size", sizeof(current), queried) ||
        differs("base_address", reinterpret_cast<std::uintptr_t>(initial.BaseAddress),
                reinterpret_cast<std::uintptr_t>(current.BaseAddress)) ||
        differs("allocation_base", reinterpret_cast<std::uintptr_t>(initial.AllocationBase),
                reinterpret_cast<std::uintptr_t>(current.AllocationBase)) ||
        differs("allocation_protect", initial.AllocationProtect, current.AllocationProtect) ||
        differs("region_size", initial.RegionSize, current.RegionSize) || differs("state", initial.State, current.State) ||
        differs("protect", initial.Protect, current.Protect) || differs("type", initial.Type, current.Type))
      failed_ = true;
    if (!failed_ && !region_contains(fresh, base, 1))
      fresh = current;
  }

  active_query_cache = previous_;
  active_ = false;
  return !failed_;
}

LocalMemoryReader::LocalMemoryReader(std::uint32_t limit) noexcept {
  reset_budget(limit);
}

void LocalMemoryReader::reset_budget(std::uint32_t limit) noexcept {
  limit_ = std::min(limit, kLocalObjectReadLimit);
  attempted_ = 0;
  failures_ = 0;
}

/*
 * Performs a bounded private-memory read after validating the range. ReadProcessMemory provides the final
 * access check instead of dereferencing a borrowed pointer directly.
 *
 * Region queries remain snapshots; RPM rechecks access instead of directly dereferencing an engine
 * allocation. Large fields use their own temporary.
 */
bool LocalMemoryReader::read(std::uint64_t address, void* destination, std::size_t size) noexcept {
  if (destination == nullptr || address == 0 || size == 0 || size > kLocalObjectFieldLimit ||
      address > std::numeric_limits<std::uintptr_t>::max() - size || size > limit_ - attempted_)
    return false;
  attempted_ += static_cast<std::uint32_t>(size);
  std::array<std::uint8_t, 64> bytes;
  const bool success = size > bytes.size() ? large_private_field(static_cast<std::uintptr_t>(address), destination, size)
                                           : private_bytes(static_cast<std::uintptr_t>(address), bytes.data(), size);
  if (!success) {
    ++failures_;
    return false;
  }
  if (size <= bytes.size())
    std::memcpy(destination, bytes.data(), size);
  return true;
}

LocalImageReader::LocalImageReader(HMODULE main_module, std::uint32_t image_size, LocalImageQueryMode mode) noexcept : mode_(mode) {
  const auto base = reinterpret_cast<std::uintptr_t>(main_module);
  if (main_module == nullptr || main_module != GetModuleHandleW(nullptr) || image_size == 0 || image_size > kMaximumImageSize ||
      image_size > std::numeric_limits<std::uintptr_t>::max() - base)
    return;
  module_ = main_module;
  base_ = base;
  limit_ = image_size;
}

/*
 * Returns verified image-region access for the requested address, using scoped page evidence only when that
 * transaction permits it.
 *
 * Standalone AA override reads retain no metadata between queries, writes or native calls. Allocation/type
 * is independent of page residency/COW.
 *
 * Unavailable/nonresident page metadata uses the original exact MBI guard.
 */
discovery::ReadWindow LocalImageReader::query(std::uint32_t rva, std::uint32_t maximum) noexcept {
  if (rva >= limit_ || maximum == 0)
    return {};
  maximum = std::min(maximum, limit_ - rva);
  const auto address = base_ + rva;
  if (mode_ == LocalImageQueryMode::pages && (!active_query_cache || active_query_cache->uses_private_pages())) {
    const auto page_bytes = page_size();
    if (!page_bytes)
      return {};
    const auto page = address - address % page_bytes;
    const auto size = static_cast<std::uint32_t>(std::min<std::size_t>(maximum, page_bytes - (address - page)));
    if (active_query_cache)
      return {size, active_query_cache->validate_image_range(address, size, base_)};
    WIN32_MEMORY_REGION_INFORMATION allocation{};
    DWORD error = ERROR_SUCCESS;
    if (query_allocation(reinterpret_cast<const void*>(address), allocation, error)) {
      if (!allocation_type(allocation, MEM_IMAGE) || allocation.AllocationBase != module_ ||
          !contains(base_, allocation.RegionSize, address, size))
        return {size, false};
      PSAPI_WORKING_SET_EX_INFORMATION info{};
      info.VirtualAddress = reinterpret_cast<void*>(page);
      if (query_pages(&info, 1, error) && info.VirtualAttributes.Valid)
        return {size, !info.VirtualAttributes.Bad && readable(static_cast<DWORD>(info.VirtualAttributes.Win32Protection))};
    }
    MEMORY_BASIC_INFORMATION region{};
    if (query_memory_uncached(reinterpret_cast<const void*>(address), region) != sizeof(region) || !region_contains(region, address, size))
      return {};
    return {size, region.State == MEM_COMMIT && region.Type == MEM_IMAGE && region.AllocationBase == module_ && readable(region.Protect)};
  }
  MEMORY_BASIC_INFORMATION region{};
  if (query_memory(reinterpret_cast<const void*>(address), region) != sizeof(region) || !region_contains(region, address, 1))
    return {};
  const auto available = region.RegionSize - (address - reinterpret_cast<std::uintptr_t>(region.BaseAddress));
  const auto size = static_cast<std::uint32_t>(std::min<std::size_t>(maximum, available));
  return {size, region.State == MEM_COMMIT && region.Type == MEM_IMAGE && region.AllocationBase == module_ && readable(region.Protect)};
}

/*
 * Reads the requested image bytes across validated regions and refuses any incomplete or inaccessible span.
 */
bool LocalImageReader::read(std::uint32_t rva, void* destination, std::size_t size) noexcept {
  if (destination == nullptr || size == 0 || rva >= limit_ || size > limit_ - rva)
    return false;
  auto* output = static_cast<std::uint8_t*>(destination);
  std::uint32_t offset = 0;
  while (offset < size) {
    const auto window = query(rva + offset, static_cast<std::uint32_t>(size - offset));
    if (!window.readable || window.size == 0 || window.size > size - offset)
      return false;
    SIZE_T copied = 0;
    if (!read_memory(reinterpret_cast<const void*>(base_ + rva + offset), output + offset, window.size, copied) || copied != window.size)
      return false;
    offset += window.size;
  }
  return true;
}

/*
 * Parses the live x64 PE header, sections and required directories with bounded reads. Only a coherent
 * executable inventory is returned to native contract resolvers.
 */
discovery::Inventory parse_verified_image_headers(discovery::ImageReader& reader) {
  discovery::Inventory result;
  HeaderReader source(reader, result);
  std::array<std::uint8_t, 64> dos{};
  if (!source.read(0, dos.data(), dos.size()))
    return result;
  const auto pe = u32(dos.data() + 60);
  if (u16(dos.data()) != 0x5a4d || pe < 64 || !within(pe, 24, kHeaderExtent)) {
    result.error = "The DOS signature or PE header offset is invalid.";
    return result;
  }
  std::array<std::uint8_t, 24> coff{};
  if (!source.read(pe, coff.data(), coff.size()))
    return result;
  result.machine = u16(coff.data() + 4);
  result.section_count = u16(coff.data() + 6);
  result.timestamp = u32(coff.data() + 8);
  const auto optional_size = u16(coff.data() + 20);
  if (u32(coff.data()) != 0x4550 || result.machine != 0x8664 || result.section_count == 0 || result.section_count > 96 ||
      optional_size < 112 || optional_size > 4096) {
    result.error = "The main PE header is not bounded AMD64 executable metadata.";
    return result;
  }
  std::array<std::uint8_t, 4096> optional{};
  if (!source.read(pe + 24, optional.data(), optional_size))
    return result;
  result.image_size = u32(optional.data() + 56);
  result.checksum = u32(optional.data() + 64);
  result.dll_characteristics = u16(optional.data() + 70);
  const auto headers_size = u32(optional.data() + 60);
  const auto directory_count = u32(optional.data() + 108);
  const auto section_table = pe + 24 + optional_size;
  const std::uint32_t section_bytes = result.section_count * 40;
  if (u16(optional.data()) != 0x20b || result.image_size == 0 || result.image_size > kMaximumImageSize || headers_size == 0 ||
      headers_size > result.image_size || headers_size > kHeaderExtent || !within(section_table, section_bytes, headers_size) ||
      directory_count > (optional_size - 112u) / 8u) {
    result.error = "The optional header, image extent or section-table bounds are invalid.";
    return result;
  }
  if (directory_count > 3) {
    result.exception_rva = u32(optional.data() + 112 + 3 * 8);
    result.exception_size = u32(optional.data() + 116 + 3 * 8);
  }
  if (directory_count > 10) {
    result.load_config_rva = u32(optional.data() + 112 + 10 * 8);
    result.load_config_size = u32(optional.data() + 116 + 10 * 8);
  }
  std::array<std::uint8_t, 96 * 40> table{};
  if (!source.read(section_table, table.data(), section_bytes))
    return result;
  for (std::uint32_t index = 0; index < result.section_count; ++index) {
    const auto* entry = table.data() + index * 40;
    std::string name;
    for (unsigned character = 0; character < 8 && entry[character] != 0; ++character)
      name.push_back(entry[character] >= 32 && entry[character] <= 126 ? static_cast<char>(entry[character]) : '?');
    const auto virtual_size = u32(entry + 8);
    const auto size = virtual_size != 0 ? virtual_size : u32(entry + 16);
    const auto rva = u32(entry + 12);
    if (!within(rva, size, result.image_size) || (size != 0 && rva < headers_size)) {
      result.error = "A declared loaded section overlaps headers or exceeds the image bounds.";
      return result;
    }
    result.sections.push_back({name, rva, size, u32(entry + 36)});
  }
  std::sort(result.sections.begin(), result.sections.end(), [](const auto& left, const auto& right) { return left.rva < right.rva; });
  if (std::none_of(result.sections.begin(), result.sections.end(), [](const auto& section) { return section.size != 0; })) {
    result.error = "The image has no nonempty loaded sections.";
    return result;
  }
  std::uint32_t previous_end = headers_size;
  for (const auto& section : result.sections) {
    if (section.size == 0)
      continue;
    if (section.rva < previous_end) {
      result.error = "The declared loaded sections overlap.";
      return result;
    }
    previous_end = section.rva + section.size;
  }
  if (!directory_valid(result, result.exception_rva, result.exception_size) ||
      !directory_valid(result, result.load_config_rva, result.load_config_size)) {
    result.error = "An exception or load-config directory is malformed or outside the declared sections.";
    return result;
  }
  result.valid_image = true;
  return result;
}

discovery::Inventory parse_verified_main_image() {
  LocalImageReader reader(GetModuleHandleW(nullptr), kHeaderExtent);
  return parse_verified_image_headers(reader);
}

}  /* namespace azp::native_camera */

namespace azp::native_camera {
/*
 * Reads the generation-checked aircraft scene transform and validates finite, orthonormal axes. It rereads
 * the observed matrix/identity before accepting a pose for attachment.
 */
AircraftScenePose inspect_aircraft_scene_pose(engine_camera::MemoryReader& reader,
                                              std::uint64_t verified_user,
                                              std::uint64_t image_base,
                                              const CameraImageLayout& layout) noexcept {
  AircraftScenePose result;
  if (!verified_user || (verified_user & 7) || !image_base || !camera_layout_detail::image_rva(layout.aircraft_controller_vtable, 8, 8) ||
      !camera_layout_detail::image_rva(layout.scene_node_vtable, 8, 8) ||
      !camera_layout_detail::image_rva(layout.scene_model_vtable, 8, 8) || image_base > UINT64_MAX - layout.aircraft_controller_vtable ||
      image_base > UINT64_MAX - layout.scene_node_vtable || image_base > UINT64_MAX - layout.scene_model_vtable)
    return result;
  struct Field {
    std::uint64_t address = 0;
    std::array<unsigned char, 24> value{};
    std::size_t bytes = 0;
  };
  std::array<Field, 16> trace{};
  unsigned count = 0;
  auto read = [&](std::uint64_t owner, std::uint64_t offset, auto& value) {
    constexpr auto size = sizeof(value);
    static_assert(size <= 24);
    if (!owner || owner > UINT64_MAX - offset || owner + offset > UINT64_MAX - size || count >= trace.size() ||
        result.read_bytes > 1024 - size)
      return false;
    auto& field = trace[count++];
    field.address = owner + offset;
    field.bytes = size;
    result.read_bytes += size;
    if (!reader.read(field.address, field.value.data(), size))
      return false;
    std::memcpy(&value, field.value.data(), size);
    return true;
  };
  std::uint64_t user_vptr = 0, node = 0, alias = 0, node_vptr = 0, attached = 0, attached_vptr = 0, matrix = 0;
  std::array<std::uint64_t, 2> handle{};
  std::uint32_t generation = 0;
  std::uint16_t type = 0;
  std::array<Vector3, 4> rows{};
  result.error = "scene_body_identity";
  if (!read(verified_user, 0, user_vptr) || user_vptr != image_base + layout.aircraft_controller_vtable ||
      !read(verified_user, 336, handle) || !read(handle[0], 28, generation) || generation != static_cast<std::uint32_t>(handle[1]) ||
      !read(handle[0], 0, node) || !node || (node & 7) || !read(verified_user, 464, alias) || alias != node || !read(node, 0, node_vptr) ||
      node_vptr != image_base + layout.scene_node_vtable || !read(node, 256, attached) || !attached || (attached & 7) ||
      !read(attached, 0, attached_vptr) || attached_vptr != image_base + layout.scene_model_vtable || !read(attached, 160, type) ||
      type != 5 || !read(node, 296, matrix) || !matrix || (matrix & 7))
    return result;
  result.error = "scene_body_matrix";
  for (unsigned row = 0; row < rows.size(); ++row)
    if (!read(matrix, row * 32, rows[row]))
      return result;
  const BodyPose candidate{rows[3], {-rows[0][0], -rows[0][1], -rows[0][2]}, rows[1], rows[2]};
  const auto radius = std::hypot(std::hypot(candidate.origin[0], candidate.origin[1]), candidate.origin[2]);
  if (!valid_body_pose(candidate) || radius < 6300000 || radius > 6500000)
    return result;
  result.error = "scene_body_changed";
  for (unsigned i = 0; i < count; ++i) {
    const auto& field = trace[i];
    std::array<unsigned char, 24> again{};
    if (result.read_bytes > 1024 - field.bytes)
      return result;
    result.read_bytes += static_cast<std::uint32_t>(field.bytes);
    if (!reader.read(field.address, again.data(), field.bytes) || std::memcmp(again.data(), field.value.data(), field.bytes))
      return result;
  }
  result.pose = candidate;
  result.node = node;
  result.complete = true;
  result.error = "";
  return result;
}

/*
 * Checks whether the rendered aircraft transform is compatible with public position and orientation within
 * caller-supplied tolerances. It validates identity plausibility without moving the camera.
 *
 * The provider enforces telemetry freshness and accounts for flight speed. Allow render/telemetry phase
 * differences, never accept a distant model or incompatible orientation. A failure keeps the owned render
 * gates closed.
 *
 * The minimum axis dot product is cos(15 degrees), allowing a bounded orientation difference between render
 * and telemetry phases.
 */
bool scene_body_matches_public(const BodyPose& scene, const BodyPose& public_pose, double maximum_distance) noexcept {
  if (!valid_body_pose(scene) || !valid_body_pose(public_pose) || !std::isfinite(maximum_distance) || maximum_distance < 0)
    return false;
  const Vector3 delta{scene.origin[0] - public_pose.origin[0], scene.origin[1] - public_pose.origin[1],
                      scene.origin[2] - public_pose.origin[2]};
  constexpr double minimum_dot = 0.9659258262890683;
  return mount_detail::dot(delta, delta) <= maximum_distance * maximum_distance &&
         mount_detail::dot(scene.right, public_pose.right) >= minimum_dot && mount_detail::dot(scene.up, public_pose.up) >= minimum_dot &&
         mount_detail::dot(scene.forward, public_pose.forward) >= minimum_dot;
}

}  /* namespace azp::native_camera */

namespace azp::native_camera {
/*
 * Validates the manager, control and renderer chain against the image/layout contract, then rereads identity
 * fields. Missing and changed objects produce refusal rather than reusable native-call permission.
 */
ManagerInspection inspect_manager_identity(discovery::ImageReader& image,
                                           engine_camera::MemoryReader& objects,
                                           std::uint64_t image_base,
                                           std::uint64_t current_manager,
                                           std::uint64_t owned_control,
                                           const CameraImageLayout& layout) {
  const auto unavailable = [](const char* detail) { return ManagerInspection{ManagerInspectionStatus::unavailable, detail}; };
  const auto refused = [](const char* detail) { return ManagerInspection{ManagerInspectionStatus::identity_refused, detail}; };
  if (!camera_layout_detail::image_rva(layout.manager_owner_global, 8, 8) ||
      !camera_layout_detail::image_rva(layout.renderer_global, 8, 8) || !camera_layout_detail::image_rva(layout.manager_vtable, 8, 8) ||
      layout.manager_owner_global == layout.renderer_global || !image_base || image_base > UINT64_MAX - layout.manager_vtable)
    return refused("Manager image layout is absent or outside the address bounds.");
  struct Handle {
    std::uint64_t control = 0;
    std::uint32_t generation = 0, extra = 0;
  };
  static_assert(sizeof(Handle) == 16);
  const auto word = [&](std::uint64_t address, auto& output) { return objects.read(address, &output, sizeof(output)); };
  std::uint64_t owner = 0, renderer = 0, cached = 0, payload = 0, vptr = 0;
  std::uint32_t generation = 0;
  Handle handle{};
  if (!image.read(layout.manager_owner_global, &owner, sizeof(owner)))
    return unavailable("Manager owner global could not be read.");
  if (!owner)
    return unavailable("Manager owner is temporarily absent.");
  if (!image.read(layout.renderer_global, &renderer, sizeof(renderer)))
    return unavailable("Manager renderer global could not be read.");
  if (!renderer)
    return unavailable("Manager renderer is temporarily absent.");
  if (owner > UINT64_MAX - 2504)
    return refused("Manager owner fields exceed the address bounds.");
  if (!word(owner + 2496, cached))
    return unavailable("Cached manager pointer could not be read.");
  if (cached != current_manager || !cached)
    return refused("Cached manager does not match the current update receiver.");
  if (!word(owner + 2480, handle))
    return unavailable("Manager weak handle could not be read.");
  if (!handle.control || handle.control > UINT64_MAX - 32)
    return refused("Manager weak control is absent or outside the address bounds.");
  if (!word(handle.control + 28, generation))
    return unavailable("Manager weak-control generation could not be read.");
  if (generation != handle.generation)
    return refused("Manager weak-handle generation does not match its control.");
  if (!word(handle.control, payload))
    return unavailable("Manager weak-control payload could not be read.");
  if (payload != cached)
    return refused("Manager weak-control payload does not match the cached manager.");
  if (!word(cached, vptr))
    return unavailable("Manager vtable pointer could not be read.");
  if (vptr != image_base + layout.manager_vtable)
    return refused("Manager vtable does not match the verified image.");

  std::uint64_t second = 0;
  Handle handle_again{};
  std::uint32_t generation_again = 0;
  if (!image.read(layout.manager_owner_global, &second, sizeof(second)))
    return unavailable("Manager owner global could not be reread.");
  if (second != owner)
    return unavailable("Manager owner changed during inspection.");
  if (!image.read(layout.renderer_global, &second, sizeof(second)))
    return unavailable("Manager renderer global could not be reread.");
  if (second != renderer)
    return unavailable("Manager renderer changed during inspection.");
  if (!word(owner + 2496, second))
    return unavailable("Cached manager pointer could not be reread.");
  if (second != cached)
    return unavailable("Cached manager changed during inspection.");
  if (!word(owner + 2480, handle_again))
    return unavailable("Manager weak handle could not be reread.");
  if (std::memcmp(&handle, &handle_again, sizeof(handle)) != 0)
    return unavailable("Manager weak handle changed during inspection.");
  if (!word(handle.control + 28, generation_again))
    return unavailable("Manager weak-control generation could not be reread.");
  if (generation_again != generation)
    return unavailable("Manager weak-control generation changed during inspection.");
  if (!word(handle.control, second))
    return unavailable("Manager weak-control payload could not be reread.");
  if (second != cached)
    return unavailable("Manager weak-control payload changed during inspection.");
  if (!word(cached, second))
    return unavailable("Manager vtable pointer could not be reread.");
  if (second != vptr)
    return unavailable("Manager vtable changed during inspection.");
  if (owned_control && owned_control != handle.control)
    return refused("Manager control does not match the retained camera owner.");
  return {ManagerInspectionStatus::ready,
          "Manager identity passed its complete field guard.",
          cached,
          renderer,
          handle.control,
          static_cast<std::uint64_t>(generation) + 1};
}

}  /* namespace azp::native_camera */
