/*
 * Resolves the reviewed native night-vision path and applies it only to AZP's owned view. A closed-gate flag
 * change enables post-processing, which gives the view's final image in TV and IR; in IR the verified callback
 * then writes the NV parameters before their GPU transfer. Failure keeps the camera from starting and never
 * changes the pilot's view.
 */

#include "night_vision.hpp"
#include <algorithm>
#include <bit>
#include <cmath>
#include "../shared/configuration.hpp"
#include "contracts/code_contract.hpp"
#include "contracts/generated/night_vision_contract_model.hpp"
#include "contracts/relocatable_contract.hpp"
#include "rtti_vtables.hpp"

namespace azp::native_camera {
namespace {
constexpr std::uint32_t kParameterOffset = 0x228;

bool valid_view(const engine_camera::OwnedViewSnapshot& view) noexcept {
  return view.complete && view.ready && view.mode == 2 && view.status == engine_camera::OwnedViewStatus::ready && !view.read_failures &&
         view.view_index >= 0 && view.view_index < 8 && view.view_address && !(view.view_address & 7) &&
         view.view_address <= UINTPTR_MAX - kParameterOffset - 8;
}

bool read_parameters(const engine_camera::OwnedViewSnapshot& view, std::array<std::uint32_t, 2>& words) noexcept {
  LocalMemoryReader reader(32);
  std::array<std::uint32_t, 2> again{};
  return valid_view(view) && reader.read(view.view_address + kParameterOffset, words.data(), sizeof(words)) &&
         reader.read(view.view_address + kParameterOffset, again.data(), sizeof(again)) && words == again &&
         std::isfinite(std::bit_cast<float>(words[0])) && std::bit_cast<float>(words[0]) >= 0 &&
         std::bit_cast<float>(words[0]) <= kNightVisionMaximumIntensity && words[1] <= 2;
}

/*
 * Adds the reviewed SU7 bodies while retaining the generated pre-SU7 model unchanged. In the render-transfer
 * phase the same global byte now uses a signed comparison (bit 7) instead of a bit-6 test; AZP's stricter
 * runtime guard still requires that whole override byte to be zero before writing parameters. The paired
 * post-stage selector changes from 0x1D5 to 0x1CD in both execution branches. All other bytes, decoded
 * operands, graph edges and extents remain exact, and each alternative must match as one complete body.
 */
const relocatable::ContractModel& reviewed_night_vision_model() {
  static const auto value = [] {
    auto out = night_vision_contract_model::model();
    const auto transfer_symbol = night_vision_contract_model::symbol_index("phase_render_transfer");
    const auto execution_symbol = night_vision_contract_model::symbol_index("post_stage_execution");
    const auto transfer = std::find_if(out.code.begin(), out.code.end(), [&](const auto& code) { return code.symbol == transfer_symbol; });
    if (transfer != out.code.end() && transfer->bytes.size() > 213 && transfer->bytes[199] == 0xf6 && transfer->bytes[200] == 0x05 &&
        transfer->bytes[205] == 0x40 && transfer->bytes[213] == 0x75) {
      auto su7 = transfer->bytes;
      su7[199] = 0x80;
      su7[200] = 0x3d;
      su7[205] = 0x00;
      su7[213] = 0x7c;
      transfer->alternatives.push_back(std::move(su7));
    }
    const auto execution =
        std::find_if(out.code.begin(), out.code.end(), [&](const auto& code) { return code.symbol == execution_symbol; });
    if (execution != out.code.end() && execution->bytes.size() > 5761 && execution->bytes[5113] == 0xd5 && execution->bytes[5761] == 0xd5) {
      auto su7 = execution->bytes;
      su7[5113] = 0xcd;
      su7[5761] = 0xcd;
      execution->alternatives.push_back(std::move(su7));
    }
    return out;
  }();
  return value;
}
} /* namespace */

/*
 * Finds and verifies the night-vision path with the generated model (generated/night_vision_contract_model.hpp):
 * the complete render function, which walks the renderer's view list, including mode2 views, and transfers
 * P+0x228 intensity -> output+0xBBC and P+0x22C integer mode -> output+0xBC0 float; its reset, cycle, world
 * callback and render-transfer phases, called in that order; the flight world's constructor, whose final vtable
 * assignment anchors the vtable actually used by the AZP view; the world render callback in that vtable's slot
 * 0xB8; and both post-processing gates, the stage-61 submission and its execution. Clearing bit 36 without both
 * gates would leave NV parameters unused. Only a complete proof yields a valid contract; without it the camera
 * does not start, and no old-build address is ever used instead.
 *
 * Every byte of each reviewed body is exact except its decoded address operands, which must agree with each
 * other, with the call and vtable relations and with their section kinds. Explicit complete alternatives
 * cover the reviewed SU7 semantic changes; no opcode, member offset or immediate is wildcarded. The bodies
 * carry no relocation, the PE function table must give the render function its exact extent, and the global
 * override is a writable data byte that the render function reads.
 *
 * The complete callback takes only world/context, and its caller ignores the return registers. It applies
 * scene cycles before returning. The hook must therefore call the original first, then override only the
 * owned view. PE unwind records split this 513-byte body into five adjacent fragments.
 *
 * Separately, the post-processing setup's luminance-pass selection is proved: bit 19 or 62 of the effective
 * view flags selects shader 0x548 (SID_VS_LUMINANCE_INIT), otherwise 0x549 (SID_VS_LUMINANCE_ADAPTATION).
 * Failure of this proof only leaves the final image on the native adaptation.
 */
NightVisionContract resolve_night_vision_contract(discovery::ImageReader& reader, const discovery::Inventory& image, std::uintptr_t base) {
  namespace generated = night_vision_contract_model;
  NightVisionContract result;
  const auto& model = reviewed_night_vision_model();
  const auto found = relocatable::resolve_contract(reader, image, model, {}, base);
  result.error = "NV unavailable: " + found.error;
  if (!found.valid)
    return result;
  const auto symbol = [&](const char* name) { return found.symbols.at(generated::symbol_index(name)); };
  const auto begin = symbol("render");
  const auto end = symbol("render_tail") + model.symbols.at(generated::symbol_index("render_tail")).extent;
  std::vector<CodeRange> ranges;
  for (const auto& code : model.code)
    for (std::uint32_t offset = 0; offset < code.bytes.size(); offset += 8192)
      ranges.push_back({found.symbols.at(code.symbol) + offset, std::min<std::uint32_t>(8192, code.bytes.size() - offset)});
  const auto proof = inspect_code_contract(reader, image, ranges);
  if (symbol("render_tail") != begin + generated::kRenderTailOffset || !proof.valid) {
    result.error = "NV unavailable: reviewed render contract refused: " + (proof.valid ? std::string("split render body.") : proof.error);
    return result;
  }
  if (!image.exception_rva || image.exception_size % 12 || image.exception_size > 16 * 1024 * 1024) {
    result.error = "NV unavailable: invalid function table.";
    return result;
  }
  std::uint32_t low = 0, high = image.exception_size / 12;
  bool boundary = false;
  while (low < high) {
    const auto middle = low + (high - low) / 2;
    std::array<std::uint32_t, 3> record{};
    if (!reader.read(image.exception_rva + middle * 12, record.data(), sizeof(record)))
      break;
    if (record[0] == begin) {
      boundary = record[1] == end;
      break;
    }
    if (record[0] < begin)
      low = middle + 1;
    else
      high = middle;
  }
  if (!boundary) {
    result.error = "NV unavailable: render function boundary refused.";
    return result;
  }
  result.override_rva = symbol("global_override");
  result.callback_return_rva = symbol("phase_world_callback") + 0x4c;
  result.world_vtable_rva = symbol("world_vtable");
  const auto rtti = resolve_rtti_vtables(reader, image, base, {{"", 0xc0, {}, result.world_vtable_rva}});
  if (!rtti.valid || rtti.vtables.size() != 1 || rtti.vtables[0] != result.world_vtable_rva) {
    result.error = "NV unavailable: world render callback identity/implementation refused.";
    return result;
  }
  const auto slot = result.world_vtable_rva + 0xb8;
  const auto section = std::find_if(image.sections.begin(), image.sections.end(), [&](const auto& item) {
    return (item.flags & 0x40000000u) && !(item.flags & 0xa2000000u) && slot >= item.rva && slot - item.rva <= item.size &&
           sizeof(void*) <= item.size - (slot - item.rva);
  });
  if (section == image.sections.end()) {
    result.error = "NV unavailable: callback pin/static slot proof refused.";
    return result;
  }
  result.callback_slot_rva = slot;
  result.callback_original_rva = symbol("world_render_callback");
  result.slot_section_rva = section->rva;
  result.slot_section_size = section->size;
  result.valid = true;

  relocatable::ContractModel luminance;
  luminance.symbols.push_back({"luminance_pass_selection", relocatable::SectionKind::code, 69});
  luminance.symbols.push_back({"luminance_pass_descriptor", relocatable::SectionKind::writable_data, 1});
  relocatable::CodeTemplate luminance_selection{
      0,
      {0x48, 0xb8, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x40, 0xc6, 0x44, 0x24, 0x50, 0x00, 0x4c, 0x8d, 0x4d,
       0x90, 0x4c, 0x8d, 0x45, 0xa8, 0x48, 0x8d, 0x55, 0xb0, 0x48, 0x8b, 0xce, 0x49, 0x85, 0x86, 0xd0, 0x0d, 0x00,
       0x00, 0x48, 0x8d, 0x05, 0x00, 0x00, 0x00, 0x00, 0x74, 0x09, 0x66, 0xc7, 0x44, 0x24, 0x48, 0x48, 0x05, 0xeb,
       0x07, 0x66, 0xc7, 0x44, 0x24, 0x48, 0x49, 0x05, 0x66, 0xc7, 0x44, 0x24, 0x40, 0xd5, 0x01},
      {{0x28, 4, 0x2c, relocatable::AddressKind::pc_relative, 1, 0}}};
  /*
   * SU7 uses the already reviewed post-stage selector 0x1CD in this selection body as well. Retain both
   * complete 69-byte bodies so bit 19 is set only when the current-frame luminance path is proved exactly.
   */
  auto su7_luminance_selection = luminance_selection.bytes;
  su7_luminance_selection[67] = 0xcd;
  luminance_selection.alternatives.push_back(std::move(su7_luminance_selection));
  luminance.code.push_back(std::move(luminance_selection));
  const auto selection = relocatable::resolve_contract(reader, image, luminance);
  result.per_frame_exposure = selection.valid && selection.symbols.size() == 2;
  result.error = result.per_frame_exposure ? "NV contract verified; per-frame exposure verified."
                                           : "NV contract verified; per-frame exposure unavailable: " + selection.error;
  return result;
}

/*
 * Pins the observer, then installs it on the verified world render-callback slot. Failure leaves the
 * controller unavailable.
 */
void NightVision::initialize(const NightVisionContract& contract,
                             std::uintptr_t base,
                             std::uint32_t image_size,
                             engine_hook::ViewParametersObserver observer) {
  if (available_)
    return;
  if (!contract.valid) {
    message_ = contract.error;
    return;
  }
  HMODULE pinned = nullptr;
  if (!observer || !GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN,
                                       reinterpret_cast<LPCWSTR>(observer), &pinned)) {
    message_ = "NV unavailable: callback pin/static slot proof refused.";
    return;
  }
  override_rva_ = contract.override_rva;
  per_frame_exposure_ = contract.per_frame_exposure;
  world_vtable_rva_ = contract.world_vtable_rva;
  callback_return_rva_ = contract.callback_return_rva;
  image_size_ = image_size;
  const engine_hook::ImageDataSlotProof slot_proof{reinterpret_cast<HMODULE>(base), image_size, contract.slot_section_rva,
                                                   contract.slot_section_size};
  const auto hook =
      engine_hook::install_view_parameters(reinterpret_cast<void**>(base + contract.callback_slot_rva),
                                           reinterpret_cast<void*>(base + contract.callback_original_rva), observer, &slot_proof);
  available_ = hook.pointer_changed && hook.protection_restored;
  message_ = available_ ? "Flight-world hook installed; waiting for an AZP post-cycle callback."
                        : std::string("NV unavailable: render callback hook refused: ") + engine_hook::status_name(hook.status);
}

std::uint64_t NightVision::post_processing_flags(std::uint64_t flags) const noexcept {
  flags &= ~kViewBypassPostProcessing;
  return per_frame_exposure_ ? flags | kViewPerFrameExposure : flags & ~kViewPerFrameExposure;
}

/*
 * Clears the owned view's reviewed bypass flag, and sets the verified per-frame exposure flag, only while its
 * render gate is closed and ownership/code/memory checks still agree. Atomic replacement preserves unrelated
 * flags and rejects concurrent change. A global override may not contradict the bypass flag, nor the
 * per-frame exposure flag when it is being set.
 *
 * One aligned compare/exchange preserves every other flag, including the closed render gate. apply_pose
 * refreshes the native context before reopening.
 */
bool NightVision::enable_post_processing(const engine_camera::OwnedViewSnapshot& view, const CameraImageLayout& layout) {
  if (!available_ || !valid_view(view) || !(view.flags[0] & 1u) || !writable_private_span(view.view_address + 48, 16)) {
    message_ = "NV post-processing refused: a verified closed owned view is required.";
    return false;
  }
  LocalImageReader image(GetModuleHandleW(nullptr), image_size_, LocalImageQueryMode::pages);
  std::array<std::uint64_t, 2> flags{}, overrides{}, again{};
  if (!read_local_flag_words(view.view_address + 48, flags) || flags != view.flags ||
      !image.read(layout.view_flag_clear_override, &overrides[0], 8) || !image.read(layout.view_flag_set_override, &overrides[1], 8)) {
    message_ = "NV post-processing refused: flags changed or global overrides are unreadable.";
    return false;
  }
  const auto desired = post_processing_flags(flags[0]);
  const auto owned = kViewBypassPostProcessing | (desired & kViewPerFrameExposure);
  if ((((desired | overrides[1]) & ~overrides[0]) & owned) != (desired & owned)) {
    message_ = "NV post-processing refused: a global override conflicts with the requested view setting.";
    return false;
  }
  const auto observed = InterlockedCompareExchange64(reinterpret_cast<volatile LONG64*>(view.view_address + 48),
                                                     std::bit_cast<LONG64>(desired), std::bit_cast<LONG64>(flags[0]));
  if (std::bit_cast<std::uint64_t>(observed) != flags[0]) {
    message_ = "NV post-processing refused: concurrent flag change.";
    return false;
  }
  flags[0] = desired;
  std::array<std::uint64_t, 2> confirmed{};
  if (!read_local_flag_words(view.view_address + 48, confirmed) || confirmed != flags ||
      !image.read(layout.view_flag_clear_override, &again[0], 8) || !image.read(layout.view_flag_set_override, &again[1], 8) ||
      again != overrides) {
    message_ = "NV post-processing flag readback failed; owned view must remain closed.";
    return false;
  }
  message_ = "Native post-processing permitted; waiting for the NV render callback.";
  return true;
}

/*
 * Matches the callback arguments to the freshly inspected AZP view before writing the aligned intensity/mode
 * pair. Other views are ignored; native fullscreen IR uses intensity 250 and mode 2 before the verified GPU
 * parameter transfer.
 *
 * The arguments come from the engine's current per-view callback. Match them to the freshly inspected AZP
 * view, its generation-checked world association and its inline rendering context. A reused address alone is
 * never authority.
 *
 * The same world also renders the pilot's view. Ignore that ordinary callback without replacing the last AZP
 * status or changing any of its fields.
 *
 * Validated fullscreen native mode: float intensity 250, enum 2.
 *
 * Eight aligned bytes hold float intensity + integer mode. One compare/exchange avoids partial parameter
 * pairs and refuses a concurrent engine replacement. This native callback precedes their transfer to the GPU
 * parameter block.
 */
void NightVision::apply(const engine_camera::OwnedViewSnapshot& view,
                        void* world,
                        void* context,
                        std::uintptr_t return_address,
                        float intensity) {
  if (!available_)
    return;
  if (!valid_view(view) || view.view_address > UINTPTR_MAX - 0x68a8) {
    message_ = "NV refused: request/owned-view inspection invalid.";
    return;
  }
  const auto world_address = reinterpret_cast<std::uintptr_t>(world);
  const auto context_address = reinterpret_cast<std::uintptr_t>(context);
  if (context_address != render_context(view.view_address))
    return;
  if (view.flags[0] & 1u) {
    message_ = "NV skipped: owned view render gate closed.";
    return;
  }
  LocalMemoryReader reader(256);
  std::array<std::uint64_t, 2> association{}, again{};
  std::uint64_t payload = 0, table = 0, destination = 0;
  std::uint32_t generation = 0;
  const auto base = reinterpret_cast<std::uintptr_t>(GetModuleHandleW(nullptr));
  if (return_address != base + callback_return_rva_ || !world_address || world_address > UINTPTR_MAX - 8 ||
      !reader.read(view.view_address + 0x48, association.data(), sizeof(association)) || !association[0] ||
      association[0] > UINTPTR_MAX - 32 || !reader.read(association[0], &payload, sizeof(payload)) || payload != world_address ||
      !reader.read(association[0] + 28, &generation, sizeof(generation)) || generation != std::uint32_t(association[1]) ||
      !reader.read(world_address, &table, sizeof(table)) || table != base + world_vtable_rva_ ||
      !reader.read(context_address + 0xc10, &destination, sizeof(destination)) || destination != view.view_address ||
      !reader.read(view.view_address + 0x48, again.data(), sizeof(again)) || association != again) {
    message_ = "NV refused: render caller/context/world identity changed.";
    return;
  }
  std::array<std::uint32_t, 2> current{};
  if (!read_parameters(view, current)) {
    message_ = "NV refused: post-reset parameters unreadable/changing/out of range.";
    return;
  }
  if (view.flags[0] & kViewBypassPostProcessing) {
    message_ = "NV waiting for the camera observer to enable native post-processing.";
    return;
  }
  unsigned char global_override = 0;
  LocalImageReader image(GetModuleHandleW(nullptr), image_size_, LocalImageQueryMode::pages);
  if (!image.read(override_rva_, &global_override, 1) || global_override ||
      !writable_private_span(view.view_address + kParameterOffset, 8)) {
    message_ = "NV refused: global override active/unreadable or parameters not writable.";
    return;
  }
  if (!std::isfinite(intensity) || intensity < kNightVisionMinimumIntensity || intensity > kNightVisionMaximumIntensity)
    intensity = kNightVisionDefaultIntensity;
  const std::array<std::uint32_t, 2> desired{std::bit_cast<std::uint32_t>(intensity), 2};
  const auto expected = std::bit_cast<LONG64>(current);
  const auto observed = InterlockedCompareExchange64(reinterpret_cast<volatile LONG64*>(view.view_address + kParameterOffset),
                                                     std::bit_cast<LONG64>(desired), expected);
  if (observed != expected) {
    message_ = "NV skipped: parameters changed during the render callback.";
    return;
  }
  std::array<std::uint32_t, 2> readback{};
  if (!read_parameters(view, readback) || readback != desired) {
    message_ = "NV render write replaced before readback; visual result unverified.";
    return;
  }
  message_ = "IR (native fullscreen night vision)";
}

} /* namespace azp::native_camera */
