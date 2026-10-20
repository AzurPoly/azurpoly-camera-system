/*
 * Resolves what zoom detail adjusts with the generated model (generated/zoom_detail_contract_model.hpp): the
 * model manager's per-piece LOD selection and its caller, whose primary-viewpoint call is redirected, and the
 * renderer's mesh fade (per-view copy of the fade factor, per-mesh evaluation, fade curve). Any failure leaves
 * zoom detail unavailable; the camera is unaffected.
 */

#include "zoom_detail_contract.hpp"
#include <algorithm>
#include <array>
#include <cstring>
#include "generated/zoom_detail_contract_model.hpp"
#include "relocatable_contract.hpp"

namespace azp::native_camera {
namespace {
/*
 * True when the PE function table describes [begin, begin + size) exactly, as one function whose unwind
 * fragments follow each other without gaps. The table is sorted by begin address.
 */
bool function_extent(discovery::ImageReader& reader, const discovery::Inventory& image, std::uint32_t begin, std::uint32_t size) {
  if (!image.exception_rva || image.exception_size % 12 || image.exception_size > 16 * 1024 * 1024)
    return false;
  const auto count = image.exception_size / 12;
  std::uint32_t low = 0, high = count;
  while (low < high) {
    const auto middle = low + (high - low) / 2;
    std::array<std::uint32_t, 3> record{};
    if (!reader.read(image.exception_rva + middle * 12, record.data(), sizeof(record)))
      return false;
    if (record[0] < begin)
      low = middle + 1;
    else
      high = middle;
  }
  auto next = begin;
  for (auto index = low; index < count && next < begin + size; ++index) {
    std::array<std::uint32_t, 3> record{};
    if (!reader.read(image.exception_rva + index * 12, record.data(), sizeof(record)) || record[0] != next || record[1] <= next)
      return false;
    next = record[1];
  }
  return next == begin + size;
}
}  /* namespace */

/*
 * Every byte of the five reviewed bodies is exact except their decoded address operands, which must agree
 * with each other and with their section kinds: both lod_select calls in lod_choose bind to lod_select, and
 * mesh_fade's call binds to fade_curve. Exact bytes fix the fade offsets: view_fade_scale reads the view
 * (context+0xc10) field 0xac and stores the quotient at context+0xbe4, which mesh_fade passes to fade_curve as
 * the distance factor. The bodies carry no relocation, and the PE function table must give each its exact
 * extent. The redirected call is read again: E8 to lod_select, inside a read-only executable section.
 */
ZoomDetailContract resolve_zoom_detail_contract(discovery::ImageReader& reader, const discovery::Inventory& image, std::uintptr_t base) {
  namespace generated = zoom_detail_contract_model;
  ZoomDetailContract result;
  const auto& model = generated::model();
  const auto found = relocatable::resolve_contract(reader, image, model, {}, base);
  if (!found.valid) {
    result.error = "Zoom detail unavailable: " + found.error;
    return result;
  }
  std::vector<CodeRange> ranges;
  for (const auto& code : model.code)
    ranges.push_back({found.symbols.at(code.symbol), static_cast<std::uint32_t>(code.bytes.size())});
  const auto proof = inspect_code_contract(reader, image, ranges);
  if (!proof.valid) {
    result.error = "Zoom detail unavailable: " + proof.error;
    return result;
  }
  for (const auto& code : model.code)
    if (!function_extent(reader, image, found.symbols.at(code.symbol), model.symbols.at(code.symbol).extent)) {
      result.error = "Zoom detail unavailable: function boundary refused for " + model.symbols.at(code.symbol).name + ".";
      return result;
    }
  result.lod_select = found.symbols.at(generated::symbol_index("lod_select"));
  result.primary_call = found.symbols.at(generated::symbol_index("lod_choose")) + generated::kPrimaryCallOffset;
  std::array<std::uint8_t, 5> call{};
  std::int32_t displacement = 0;
  if (reader.read(result.primary_call, call.data(), call.size()) && call[0] == 0xe8)
    std::memcpy(&displacement, call.data() + 1, sizeof(displacement));
  if (!displacement || static_cast<std::int64_t>(result.primary_call) + 5 + displacement != result.lod_select) {
    result.error = "Zoom detail unavailable: the primary LOD call does not reach the selection.";
    return result;
  }
  const auto section = std::find_if(image.sections.begin(), image.sections.end(), [&](const auto& item) {
    return (item.flags & 0x60000000u) == 0x60000000u && !(item.flags & 0x82000000u) && result.primary_call >= item.rva &&
           result.primary_call - item.rva < item.size && 5 <= item.size - (result.primary_call - item.rva);
  });
  if (section == image.sections.end()) {
    result.error = "Zoom detail unavailable: the primary LOD call is outside read-only code.";
    return result;
  }
  result.call_section_rva = section->rva;
  result.call_section_size = section->size;
  result.view_fade_scale = generated::kViewFadeScale;
  result.error = "Zoom detail contract verified.";
  result.valid = true;
  return result;
}

}  /* namespace azp::native_camera */
