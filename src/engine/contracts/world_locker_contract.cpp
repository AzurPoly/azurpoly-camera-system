/*
 * Resolves target-registration, terrain and vegetation operations used for scenery preloading. It combines
 * the generated model with exact code and function-boundary checks before exposing callback slots or native
 * calls.
 */

#include "world_locker_contract.hpp"
#include <algorithm>
#include <cstring>
#include "../memory.hpp"
#include "../rtti_vtables.hpp"
#include "generated/world_locker_contract_model.hpp"

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
  for (const auto& wanted : world_locker_contract_model::boundaries()) {
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
/*
 * Lets the engine object behind the preload gates grow. service_update, add_zone88 and add_zone78 each call
 * the same accessor (model symbol reference_222) and test a byte flag of the object it returns with
 * `cmp byte [reg+disp32], 0`: +0xD0B0 in both registrations, +0xD190 in the update. MSFS 1.9.12.0 and
 * 1.9.18.0 (SU7) moved all three by +0xE0 and changed nothing else in these bodies. The three displacements
 * form one shifted-field group, so they must move together; every other byte, operand and relation stays
 * exact. AZP never reads these flags itself.
 */
const relocatable::ContractModel& reviewed_preload_model() {
  static const auto value = [] {
    auto out = world_locker_contract_model::model();
    struct Gate {
      const char* name;
      std::uint32_t offset, displacement;
    };
    for (const auto& gate : {Gate{"service_update", 671, 0xd190}, Gate{"add_zone88", 78, 0xd0b0}, Gate{"add_zone78", 41, 0xd0b0}}) {
      const auto symbol = world_locker_contract_model::symbol_index(gate.name);
      const auto code = std::find_if(out.code.begin(), out.code.end(), [&](const auto& x) { return x.symbol == symbol; });
      std::uint32_t displacement = 0;
      if (code == out.code.end() || code->bytes.size() < gate.offset + 5 || code->bytes[gate.offset - 2] != 0x80 ||
          code->bytes[gate.offset - 1] != 0xba || code->bytes[gate.offset + 4] != 0)
        continue;
      std::memcpy(&displacement, code->bytes.data() + gate.offset, sizeof(displacement));
      if (displacement == gate.displacement)
        code->fields.push_back({gate.offset, 0});
    }
    return out;
  }();
  return value;
}
}  /* namespace */

/*
 * Resolves the complete preload model, checks function extents and exact code, then exposes the
 * terrain/model/vegetation operations as one verified contract.
 */
WorldLockerContract resolve_world_locker_contract(discovery::ImageReader& reader,
                                                  const discovery::Inventory& image,
                                                  std::uint64_t image_base) {
  const auto refused = [](const std::string& reason) {
    WorldLockerContract out;
    out.error = "World locker refuse: " + reason;
    return out;
  };
  if (!image_base || !same_image(image, parse_verified_image_headers(reader)))
    return refused("identite de l'image.");
  const auto& model = reviewed_preload_model();
  const auto resolved = relocatable::resolve_contract(reader, image, model, {}, image_base);
  if (!resolved.valid)
    return refused(resolved.error);
  std::vector<CodeRange> ranges;
  for (const auto& code : model.code)
    ranges.push_back({resolved.symbols.at(code.symbol), static_cast<std::uint32_t>(code.bytes.size())});
  const auto proof = inspect_code_contract(reader, image, ranges);
  if (!proof.valid || !boundaries(reader, image, resolved.symbols))
    return refused("bornes de fonctions ou relocations.");
  const auto symbol = [&](const char* name) { return resolved.symbols.at(world_locker_contract_model::symbol_index(name)); };
  WorldLockerContract result;
  result.update = symbol("service_update");
  result.add88 = symbol("add_zone88");
  result.add78 = symbol("add_zone78");
  result.remove = symbol("remove_owner");
  result.convert = symbol("convert_position");
  result.current_world = symbol("world_lookup");
  result.terrain_classify = symbol("terrain_classify");
  result.terrain_update = symbol("terrain_update");
  result.terrain_bounds = symbol("terrain_bounds");
  result.terrain_intersects = symbol("terrain_intersects");
  result.tile_submit = symbol("tile_submit");
  result.node_ready = symbol("node_ready");
  result.node_bounds = symbol("node_bounds");
  result.vegetation_update = symbol("vegetation_update");
  result.vegetation_level = symbol("vegetation_level");
  result.vegetation_quality = symbol("vegetation_quality");
  result.vegetation_density_low = symbol("vegetation_density_low");
  result.vegetation_density_high = symbol("vegetation_density_high");
  const auto world = resolve_rtti_vtables(reader, image, image_base, {{"", 0x1a8, {{0x1a0, result.tile_submit}}, 0}});
  if (!world.valid || world.vtables.size() != 1)
    return refused("identite RTTI du monde: " + world.error);
  result.world_vtable = world.vtables.front();
  const auto tables = resolve_rtti_vtables(reader, image, image_base, {{"", 0x80, {{0x78, result.update}}, 0}});
  if (!tables.valid || tables.vtables.size() != 1)
    return refused("identite RTTI du service: " + tables.error);
  result.vtable = tables.vtables.front();
  const auto terrain =
      resolve_rtti_vtables(reader, image, image_base, {{"", 0x28, {{0x08, result.terrain_update}, {0x20, result.terrain_classify}}, 0}});
  if (!terrain.valid || terrain.vtables.size() != 1)
    return refused("identite RTTI du critere terrain: " + terrain.error);
  result.terrain_vtable = terrain.vtables.front();
  std::vector<relocatable::SymbolBinding> bindings;
  for (std::uint32_t index = 0; index < resolved.symbols.size(); ++index)
    bindings.push_back({index, resolved.symbols[index]});
  if (!same_image(image, parse_verified_image_headers(reader)) || !verify_code_contract(reader, image, proof.records).valid ||
      !relocatable::resolve_contract(reader, image, model, bindings, image_base).valid)
    return refused("le contrat moteur a change pendant sa verification.");
  result.fingerprints = proof.records;
  result.valid = true;
  return result;
}
}  /* namespace azp::native_camera */
