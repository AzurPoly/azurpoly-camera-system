/*
 * Resolves and verifies the native node-parenting and camera-update operations used for rigid attachment. It
 * matches generated instruction evidence against the current executable and rejects ambiguity or changed
 * function boundaries.
 */

#include "camera_attachment_contract.hpp"
#include <algorithm>
#include "../memory.hpp"
#include "../rtti_vtables.hpp"
#include "code_contract.hpp"
#include "generated/camera_attachment_contract_model.hpp"

namespace azp::native_camera {
namespace {
/*
 * Checks the requested image span against a verified section and reads every byte through the bounded
 * reader. Partial reads cannot satisfy a code proof.
 */
bool exact(discovery::ImageReader& reader, std::uint32_t rva, void* output, std::uint32_t size) {
  std::uint32_t done = 0;
  while (done < size) {
    const auto window = reader.query(rva + done, std::min<std::uint32_t>(32768, size - done));
    if (!window.readable || !window.size || window.size > size - done ||
        !reader.read(rva + done, static_cast<unsigned char*>(output) + done, window.size))
      return false;
    done += window.size;
  }
  return true;
}
/*
 * Parses the executable's function extents and checks that resolved code symbols occupy the expected
 * reviewed boundaries.
 */
bool boundaries(discovery::ImageReader& reader, const discovery::Inventory& image, const std::vector<std::uint32_t>& symbols) {
  struct Function {
    std::uint32_t begin, end, unwind;
  };
  if (!image.exception_rva || !image.exception_size || image.exception_size > 8 * 1024 * 1024 || image.exception_size % sizeof(Function) ||
      image.exception_rva >= image.image_size || image.exception_size > image.image_size - image.exception_rva)
    return false;
  std::vector<Function> functions(image.exception_size / sizeof(Function));
  if (!exact(reader, image.exception_rva, functions.data(), image.exception_size))
    return false;
  std::uint32_t previous = 0;
  for (const auto& item : functions) {
    if (item.begin < previous || item.end <= item.begin || item.end > image.image_size || item.unwind >= image.image_size)
      return false;
    previous = item.end;
  }
  for (const auto& wanted : camera_attachment_contract_model::boundaries()) {
    if (wanted.symbol >= symbols.size())
      return false;
    const auto address = std::int64_t(symbols[wanted.symbol]) + wanted.begin_addend;
    if (address <= 0 || !wanted.bytes || address + wanted.bytes > image.image_size)
      return false;
    const auto begin = static_cast<std::uint32_t>(address);
    const auto found = std::lower_bound(functions.begin(), functions.end(), begin,
                                        [](const auto& entry, std::uint32_t value) { return entry.begin < value; });
    if (wanted.required_pdata) {
      if (found == functions.end() || found->begin != begin || found->end != begin + wanted.bytes)
        return false;
    } else {
      if (found != functions.begin() && (found - 1)->end > begin)
        return false;
      if (found != functions.end() && found->begin < begin + wanted.bytes && (found->begin != begin || found->end != begin + wanted.bytes))
        return false;
    }
  }
  return true;
}
/*
 * Compares the freshly parsed image layout with the supplied discovery inventory before applying a generated
 * model.
 */
bool same_image(const discovery::Inventory& a, const discovery::Inventory& b) {
  if (!a.valid_image || !b.valid_image || a.machine != b.machine || a.image_size != b.image_size || a.timestamp != b.timestamp ||
      a.checksum != b.checksum || a.exception_rva != b.exception_rva || a.exception_size != b.exception_size ||
      a.sections.size() != b.sections.size())
    return false;
  for (std::size_t i = 0; i < a.sections.size(); ++i)
    if (a.sections[i].rva != b.sections[i].rva || a.sections[i].size != b.sections[i].size || a.sections[i].flags != b.sections[i].flags)
      return false;
  return true;
}
}  /* namespace */

/*
 * Matches the attachment model to the current executable, verifies code extents and fingerprints, then binds
 * the node and camera operations. Any missing or ambiguous proof rejects the complete attachment contract.
 *
 * The observed Camera_Z slot is +216; resolve its actual RTTI/vtable identity.
 */
CameraAttachmentContract resolve_camera_attachment_contract(discovery::ImageReader& reader,
                                                            const discovery::Inventory& image,
                                                            std::uint64_t image_base,
                                                            const CameraContract& camera) {
  const auto refused = [](const std::string& reason) {
    CameraAttachmentContract out;
    out.error = "Camera attachment refused: " + reason;
    return out;
  };
  if (!image_base || !same_image(image, parse_verified_image_headers(reader)))
    return refused("image identity");
  const auto& model = camera_attachment_contract_model::model();
  const auto index = [](const char* name) { return camera_attachment_contract_model::symbol_index(name); };
  const std::vector<relocatable::SymbolBinding> roots{{index("set_position"), camera.functions.set_position},
                                                      {index("set_target"), camera.functions.set_target},
                                                      {index("set_up"), camera.functions.set_up}};
  const auto resolved = relocatable::resolve_contract(reader, image, model, roots, image_base);
  if (!resolved.valid)
    return refused(resolved.error);
  std::vector<CodeRange> ranges;
  for (const auto& code : model.code)
    ranges.push_back({resolved.symbols.at(code.symbol), static_cast<std::uint32_t>(code.bytes.size())});
  const auto proof = inspect_code_contract(reader, image, ranges);
  if (!proof.valid || !boundaries(reader, image, resolved.symbols))
    return refused("function boundaries or relocations");
  CameraAttachmentContract result;
  result.attach = resolved.symbols.at(index("attach"));
  result.detach = resolved.symbols.at(index("detach"));
  result.update = resolved.symbols.at(index("camera_update"));
  const auto tables = resolve_rtti_vtables(reader, image, image_base, {{"", 224, {{216, result.update}}, 0}});
  if (!tables.valid || tables.vtables.size() != 1)
    return refused("Camera_Z vtable: " + tables.error);
  result.vtable = tables.vtables.front();
  std::vector<relocatable::SymbolBinding> bindings;
  for (std::uint32_t i = 0; i < resolved.symbols.size(); ++i)
    bindings.push_back({i, resolved.symbols[i]});
  if (!same_image(image, parse_verified_image_headers(reader)) || !verify_code_contract(reader, image, proof.records).valid ||
      !relocatable::resolve_contract(reader, image, model, bindings, image_base).valid)
    return refused("contract changed during verification");
  result.valid = true;
  return result;
}
}  /* namespace azp::native_camera */
