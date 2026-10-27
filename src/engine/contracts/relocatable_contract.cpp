/*
 * Finds native symbols using instruction templates plus relationships between their operands. It scans
 * bounded executable sections, propagates constraints and accepts only a consistent unambiguous solution,
 * allowing relocation without accepting unrelated code.
 */

#include "relocatable_contract.hpp"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <limits>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace azp::native_camera::relocatable {
namespace {
std::mutex evidence_mutex;
std::vector<std::string> evidence_lines;

std::string hex_bytes(const std::uint8_t* bytes, std::size_t begin, std::size_t end) {
  std::string text;
  char item[4]{};
  for (auto at = begin; at < end; ++at) {
    std::snprintf(item, sizeof(item), "%02x", bytes[at]);
    text += item;
  }
  return text;
}
constexpr std::uint32_t Readable = 0x40000000, Writable = 0x80000000, Executable = 0x20000000, Discardable = 0x02000000;
constexpr std::uint32_t Chunk = 32768, Unknown = std::numeric_limits<std::uint32_t>::max();
/*
 * Length of the extra invariant runs that locate a missing template for the support report.
 */
constexpr std::uint32_t EvidenceRun = 24;
constexpr std::uint64_t CodeLimit = 192ull * 1024 * 1024, ReadLimit = 512ull * 1024 * 1024;
using Binding = std::pair<std::uint32_t, std::uint32_t>;
/*
 * Bindings pair a symbol with its RVA. Keys from symbols.size() on are shifted-field groups paired with their
 * delta (two's complement), so the symbol agreement rules also make every field of a group move together.
 */
struct Candidate {
  std::vector<Binding> bindings;
  std::uint32_t variant = 0;
};
struct TemplateState {
  std::vector<std::uint8_t> invariant, seedable;
  std::uint32_t seed_offset = 0, seed_size = 0;
  bool discoverable = false;
  std::vector<Candidate> candidates;
  std::unordered_set<std::uint32_t> attempted;
  /*
   * Support-report evidence only: up to two more invariant runs (offset, size) and the places where they
   * occurred, which locate a template whose seed itself differs. They never create a candidate.
   */
  std::vector<std::pair<std::uint32_t, std::uint32_t>> evidence_runs;
  std::unordered_set<std::uint32_t> evidence_places;
};

bool static_code(const discovery::ImageSection& section) {
  return (section.flags & (Readable | Executable | Writable | Discardable)) == (Readable | Executable);
}
bool supports(const discovery::ImageSection& section, SectionKind kind) {
  if (kind == SectionKind::code)
    return static_code(section);
  if (kind == SectionKind::read_only_data)
    return (section.flags & (Readable | Executable | Writable | Discardable)) == Readable;
  if (kind == SectionKind::writable_data)
    return (section.flags & (Readable | Executable | Writable | Discardable)) == (Readable | Writable);
  return false;
}
/*
 * Checks that resolved operand relationships agree with the candidate symbol locations rather than matching
 * instruction bytes in isolation.
 */
bool agrees(const Candidate& left, const Candidate& right) {
  std::size_t a = 0, b = 0;
  while (a < left.bindings.size() && b < right.bindings.size()) {
    if (left.bindings[a].first < right.bindings[b].first)
      ++a;
    else if (left.bindings[a].first > right.bindings[b].first)
      ++b;
    else {
      if (left.bindings[a].second != right.bindings[b].second)
        return false;
      ++a;
      ++b;
    }
  }
  return true;
}

class Resolver {
 public:
  Resolver(discovery::ImageReader& reader,
           const discovery::Inventory& image,
           const ContractModel& model,
           const std::vector<SymbolBinding>& roots,
           std::uint64_t loaded_image_base)
      : reader_(reader), image_(image), model_(model), roots_(roots), loaded_image_base_(loaded_image_base) {}

  /*
   * Validates the model, gathers symbol candidates, propagates relationships and accepts only a complete
   * consistent resolution. Missing or ambiguous candidates return a refusal.
   *
   * Reread every accepted range without adding, moving or adopting bindings.
   */
  ContractResolution run() {
    if (!prepare() || !scan() || !infer_short_templates() || !constrain())
      return result_;
    std::vector<std::uint32_t> resolved(model_.symbols.size() + groups_, Unknown);
    for (std::size_t i = 0; i < model_.symbols.size(); ++i)
      if (model_.symbols[i].kind == SectionKind::image_base)
        resolved[i] = 0;
    for (const auto& [symbol, rva] : fixed_.bindings)
      resolved[symbol] = rva;
    for (std::size_t i = 0; i < states_.size(); ++i) {
      const auto& state = states_[i];
      if (state.candidates.size() != 1) {
        result_.error = "ambiguous_template: " + model_.symbols[model_.code[i].symbol].name + " (" +
                        std::to_string(state.candidates.size()) + " candidates)";
        return result_;
      }
      for (const auto& [symbol, rva] : state.candidates.front().bindings) {
        if (resolved[symbol] != Unknown && resolved[symbol] != rva) {
          fail("conflicting_symbol");
          return result_;
        }
        resolved[symbol] = rva;
      }
    }
    if (std::find(resolved.begin(), resolved.end(), Unknown) != resolved.end()) {
      fail("unresolved_symbol");
      return result_;
    }
    for (std::size_t i = 0; i < states_.size(); ++i) {
      Candidate fresh;
      if (!match(i, resolved[model_.code[i].symbol], fresh) || fresh.bindings != states_[i].candidates.front().bindings ||
          fresh.variant != states_[i].candidates.front().variant) {
        fail("code_changed_during_resolution");
        return result_;
      }
    }
    for (const auto& constant : model_.constants)
      if (!check_constant(constant, resolved[constant.symbol])) {
        fail("constant_changed_during_resolution");
        return result_;
      }
    Candidate all;
    for (std::uint32_t i = 0; i < resolved.size(); ++i)
      all.bindings.emplace_back(i, resolved[i]);
    const auto expected = all.bindings;
    if (!follow_pointers(all) || all.bindings != expected) {
      fail("pointer_changed_during_resolution");
      return result_;
    }
    resolved.resize(model_.symbols.size());
    result_.symbols = std::move(resolved);
    result_.valid = true;
    return result_;
  }

 private:
  bool fail(const char* message) {
    if (result_.error.empty())
      result_.error = message;
    return false;
  }

  const discovery::ImageSection* containing(std::uint32_t rva, std::uint32_t extent, SectionKind kind) const {
    const auto end = std::uint64_t(rva) + extent;
    if (!extent || end > image_.image_size)
      return nullptr;
    for (const auto& section : image_.sections)
      if (supports(section, kind) && rva >= section.rva && end <= std::uint64_t(section.rva) + section.size)
        return &section;
    return nullptr;
  }

  bool symbol_valid(std::uint32_t index, std::uint32_t rva) const {
    const auto& symbol = model_.symbols[index];
    if (symbol.kind == SectionKind::image_base)
      return rva == 0;
    return containing(rva, symbol.extent, symbol.kind) != nullptr;
  }

  /*
   * Reads candidate code within verified section/page bounds while charging the resolver's bounded work
   * budget.
   */
  bool read(std::uint32_t rva, std::uint8_t* destination, std::size_t size) {
    if (!size || std::uint64_t(rva) + size > image_.image_size)
      return fail("read_bounds");
    while (size) {
      if (result_.read_calls >= 65536)
        return fail("read_call_limit");
      const auto maximum = static_cast<std::uint32_t>(std::min<std::size_t>(size, Chunk));
      const auto window = reader_.query(rva, maximum);
      if (!window.readable || !window.size || window.size > maximum)
        return fail("unreadable_or_invalid_window");
      if (result_.requested_bytes + window.size > ReadLimit)
        return fail("read_byte_limit");
      ++result_.read_calls;
      result_.requested_bytes += window.size;
      if (!reader_.read(rva, destination, window.size))
        return fail("read_failed");
      rva += window.size;
      destination += window.size;
      size -= window.size;
    }
    return true;
  }

  /*
   * Checks symbols, templates, operands and relations for internal consistency before any executable scan.
   * Invalid offsets, incompatible masks or impossible model references are rejected early.
   */
  bool prepare() {
    if (!image_.valid_image || image_.machine != 0x8664 || !image_.image_size || image_.sections.empty() || image_.sections.size() > 96 ||
        image_.section_count != image_.sections.size())
      return fail("invalid_image_metadata");
    std::vector<const discovery::ImageSection*> sections;
    for (const auto& section : image_.sections) {
      if (!section.rva || !section.size || std::uint64_t(section.rva) + section.size > image_.image_size)
        return fail("invalid_section_bounds");
      sections.push_back(&section);
      if (static_code(section))
        code_size_ += section.size;
    }
    std::sort(sections.begin(), sections.end(), [](const auto* a, const auto* b) { return a->rva < b->rva; });
    for (std::size_t i = 1; i < sections.size(); ++i)
      if (std::uint64_t(sections[i - 1]->rva) + sections[i - 1]->size > sections[i]->rva)
        return fail("overlapping_sections");
    if (!code_size_ || code_size_ > CodeLimit)
      return fail("code_scan_limit");
    if (model_.symbols.empty() || model_.symbols.size() > 4096 || model_.code.empty() || model_.code.size() > 64 ||
        model_.constants.size() > 4096 || model_.pointers.size() > 256)
      return fail("model_count_limit");
    std::unordered_set<std::string> names;
    for (const auto& symbol : model_.symbols) {
      if (symbol.name.empty() || symbol.name.size() > 128 || !names.insert(symbol.name).second ||
          (symbol.kind != SectionKind::code && symbol.kind != SectionKind::read_only_data && symbol.kind != SectionKind::writable_data &&
           symbol.kind != SectionKind::image_base) ||
          (symbol.kind != SectionKind::image_base && (!symbol.extent || symbol.extent > image_.image_size)) ||
          (symbol.kind == SectionKind::image_base && symbol.extent > 1))
        return fail("invalid_symbol");
    }
    states_.resize(model_.code.size());
    template_for_symbol_.assign(model_.symbols.size(), Unknown);
    std::uint64_t template_bytes = 0, operand_count = 0;
    bool has_seed = false;
    for (std::size_t i = 0; i < model_.code.size(); ++i) {
      const auto& code = model_.code[i];
      auto& state = states_[i];
      template_bytes += code.bytes.size();
      for (const auto& alternative : code.alternatives)
        template_bytes += alternative.size();
      operand_count += code.operands.size();
      if (code.symbol >= model_.symbols.size() || code.bytes.empty() || code.bytes.size() > 16384 || template_bytes > 262144 ||
          operand_count > 65536 || code.alternatives.size() > 8 || code.fields.size() > 16 ||
          (code.minimum_seed != 7 && code.minimum_seed != 8) || model_.symbols[code.symbol].kind != SectionKind::code ||
          code.bytes.size() > model_.symbols[code.symbol].extent || template_for_symbol_[code.symbol] != Unknown)
        return fail("invalid_code_template");
      for (std::size_t alternative = 0; alternative < code.alternatives.size(); ++alternative) {
        const auto& body = code.alternatives[alternative];
        if (body.size() != code.bytes.size() || body == code.bytes)
          return fail("invalid_code_alternative");
        for (std::size_t previous = 0; previous < alternative; ++previous)
          if (body == code.alternatives[previous])
            return fail("duplicate_code_alternative");
      }
      template_for_symbol_[code.symbol] = static_cast<std::uint32_t>(i);
      state.invariant.assign(code.bytes.size(), 1);
      state.seedable.assign(code.bytes.size(), 1);
      for (const auto& operand : code.operands) {
        if ((operand.width != 1 && operand.width != 4) || operand.target_symbol >= model_.symbols.size() ||
            std::uint64_t(operand.offset) + operand.width > code.bytes.size() ||
            (operand.kind != AddressKind::pc_relative && operand.kind != AddressKind::image_rva) ||
            (operand.kind == AddressKind::pc_relative &&
             (operand.pc_offset < std::uint64_t(operand.offset) + operand.width || operand.pc_offset > code.bytes.size())) ||
            (operand.kind == AddressKind::image_rva && operand.pc_offset != 0))
          return fail("invalid_address_operand");
        for (std::uint32_t at = operand.offset; at < operand.offset + operand.width; ++at) {
          if (!state.invariant[at])
            return fail("overlapping_address_operands");
          state.invariant[at] = 0;
          state.seedable[at] = 0;
        }
      }
      for (const auto& field : code.fields) {
        if (field.group >= 16 || std::uint64_t(field.offset) + 4 > code.bytes.size())
          return fail("invalid_shifted_field");
        for (std::uint32_t at = field.offset; at < field.offset + 4; ++at) {
          if (!state.invariant[at])
            return fail("overlapping_shifted_field");
          state.invariant[at] = 0;
          state.seedable[at] = 0;
        }
        groups_ = std::max(groups_, field.group + 1);
      }
      for (const auto& alternative : code.alternatives) {
        bool fixed_difference = false;
        for (std::size_t at = 0; at < code.bytes.size(); ++at)
          if (alternative[at] != code.bytes[at]) {
            state.seedable[at] = 0;
            fixed_difference |= state.invariant[at] != 0;
          }
        if (!fixed_difference)
          return fail("operand_only_code_alternative");
      }
      std::uint32_t length = 0;
      for (std::uint32_t at = 0; at < state.seedable.size(); ++at) {
        length = state.seedable[at] ? length + 1 : 0;
        if (length > state.seed_size) {
          state.seed_offset = at + 1 - length;
          state.seed_size = length;
        }
      }
      state.discoverable = state.seed_size >= code.minimum_seed;
      if (state.discoverable) {
        has_seed = true;
        maximum_seed_ = std::max(maximum_seed_, state.seed_size);
        seeds_[key(code.bytes.data() + state.seed_offset)].push_back({i, 0});
        std::vector<std::pair<std::uint32_t, std::uint32_t>> runs;
        for (std::uint32_t at = 0, length = 0; at <= state.seedable.size(); ++at) {
          if (at < state.seedable.size() && state.seedable[at]) {
            ++length;
            continue;
          }
          if (length >= EvidenceRun && at - length != state.seed_offset)
            runs.emplace_back(at - length, length);
          length = 0;
        }
        std::sort(runs.begin(), runs.end(), [](const auto& a, const auto& b) { return a.second > b.second; });
        for (std::size_t run = 0; run < runs.size() && run < 2; ++run) {
          state.evidence_runs.emplace_back(runs[run].first, EvidenceRun);
          seeds_[key(code.bytes.data() + runs[run].first)].push_back({i, static_cast<std::uint32_t>(run + 1)});
          maximum_seed_ = std::max(maximum_seed_, EvidenceRun);
        }
      }
    }
    if (!has_seed)
      return fail("no_discoverable_template");
    template_for_symbol_.resize(model_.symbols.size() + groups_, Unknown);
    std::uint64_t constant_bytes = 0;
    for (const auto& constant : model_.constants) {
      constant_bytes += constant.bytes.size();
      if (constant.symbol >= model_.symbols.size() || model_.symbols[constant.symbol].kind != SectionKind::read_only_data ||
          constant.bytes.empty() || constant.bytes.size() > 4096 || constant_bytes > 65536 ||
          std::uint64_t(constant.offset) + constant.bytes.size() > model_.symbols[constant.symbol].extent)
        return fail("invalid_data_constant");
    }
    if (!model_.pointers.empty() &&
        (!loaded_image_base_ || (loaded_image_base_ & 7) || loaded_image_base_ > UINT64_MAX - image_.image_size))
      return fail("invalid_loaded_image_base");
    std::vector<Binding> slots;
    for (const auto& pointer : model_.pointers) {
      if (pointer.owner_symbol >= model_.symbols.size() || pointer.target_symbol >= model_.symbols.size() ||
          model_.symbols[pointer.owner_symbol].kind != SectionKind::read_only_data ||
          model_.symbols[pointer.target_symbol].kind != SectionKind::code || template_for_symbol_[pointer.target_symbol] == Unknown ||
          (pointer.offset & 7) || std::uint64_t(pointer.offset) + 8 > model_.symbols[pointer.owner_symbol].extent)
        return fail("invalid_pointer_relation");
      const Binding slot{pointer.owner_symbol, pointer.offset};
      if (std::find(slots.begin(), slots.end(), slot) != slots.end())
        return fail("duplicate_pointer_relation");
      slots.push_back(slot);
    }
    if (roots_.size() > model_.symbols.size())
      return fail("root_count_limit");
    for (const auto& root : roots_) {
      if (root.symbol >= model_.symbols.size() || !symbol_valid(root.symbol, root.rva))
        return fail("invalid_root_binding");
      fixed_.bindings.emplace_back(root.symbol, root.rva);
    }
    std::sort(fixed_.bindings.begin(), fixed_.bindings.end());
    for (std::size_t i = 1; i < fixed_.bindings.size(); ++i)
      if (fixed_.bindings[i - 1].first == fixed_.bindings[i].first)
        return fail("duplicate_or_conflicting_root");
    if (!follow_pointers(fixed_))
      return fail("invalid_root_pointer_relation");
    for (const auto& constant : model_.constants)
      for (const auto& root : roots_)
        if (constant.symbol == root.symbol && !check_constant(constant, root.rva))
          return fail("root_constant_mismatch");
    return true;
  }

  static std::uint32_t key(const std::uint8_t* bytes) {
    return std::uint32_t(bytes[0]) | (std::uint32_t(bytes[1]) << 8) | (std::uint32_t(bytes[2]) << 16) | (std::uint32_t(bytes[3]) << 24);
  }

  bool check_constant(const DataConstant& constant, std::uint32_t rva) {
    const auto address = std::uint64_t(rva) + constant.offset;
    if (address > Unknown ||
        !containing(static_cast<std::uint32_t>(address), static_cast<std::uint32_t>(constant.bytes.size()), SectionKind::read_only_data))
      return false;
    std::vector<std::uint8_t> bytes(constant.bytes.size());
    return read(static_cast<std::uint32_t>(address), bytes.data(), bytes.size()) && bytes == constant.bytes;
  }

  static bool normalize(Candidate& candidate) {
    std::sort(candidate.bindings.begin(), candidate.bindings.end());
    for (std::size_t i = 1; i < candidate.bindings.size(); ++i)
      if (candidate.bindings[i - 1].first == candidate.bindings[i].first &&
          candidate.bindings[i - 1].second != candidate.bindings[i].second)
        return false;
    candidate.bindings.erase(std::unique(candidate.bindings.begin(), candidate.bindings.end()), candidate.bindings.end());
    return true;
  }

  /*
   * Follows only the explicitly modelled bounded indirections when resolving an operand relationship. A
   * pointer chain outside permitted sections cannot become a candidate.
   *
   * Owners are read-only data; targets are code, so these relations cannot recursively invent another
   * pointer owner. Keep lookup input immutable.
   */
  bool follow_pointers(Candidate& candidate) {
    const auto known = candidate.bindings;
    for (const auto& pointer : model_.pointers) {
      const auto owner = std::lower_bound(known.begin(), known.end(), Binding{pointer.owner_symbol, 0});
      if (owner == known.end() || owner->first != pointer.owner_symbol)
        continue;
      const auto address = std::uint64_t(owner->second) + pointer.offset;
      if ((address & 7) || address > Unknown || !containing(static_cast<std::uint32_t>(address), 8, SectionKind::read_only_data))
        return false;
      std::uint8_t bytes[8]{};
      if (!read(static_cast<std::uint32_t>(address), bytes, sizeof(bytes)))
        return false;
      const auto value = std::uint64_t(key(bytes)) | (std::uint64_t(key(bytes + 4)) << 32);
      if (value < loaded_image_base_ || value - loaded_image_base_ >= image_.image_size)
        return false;
      const auto target = static_cast<std::uint32_t>(value - loaded_image_base_);
      if (!symbol_valid(pointer.target_symbol, target))
        return false;
      candidate.bindings.emplace_back(pointer.target_symbol, target);
    }
    return normalize(candidate);
  }

  /*
   * Compares one candidate instruction sequence with its byte mask and decodes the modelled operands for
   * later relationship checks.
   */
  bool match(std::size_t index, std::uint32_t rva, Candidate& candidate) {
    const auto& code = model_.code[index];
    const auto& state = states_[index];
    if (!symbol_valid(code.symbol, rva))
      return false;
    std::vector<std::uint8_t> bytes(code.bytes.size());
    if (!read(rva, bytes.data(), bytes.size()))
      return false;
    const auto matches = [&](const std::vector<std::uint8_t>& body) {
      for (std::size_t at = 0; at < bytes.size(); ++at)
        if (state.invariant[at] && bytes[at] != body[at])
          return false;
      return true;
    };
    candidate.variant = Unknown;
    if (matches(code.bytes))
      candidate.variant = 0;
    for (std::size_t alternative = 0; alternative < code.alternatives.size(); ++alternative)
      if (matches(code.alternatives[alternative])) {
        if (candidate.variant != Unknown)
          return false;
        candidate.variant = static_cast<std::uint32_t>(alternative + 1);
      }
    if (candidate.variant == Unknown)
      return false;
    candidate.bindings.emplace_back(code.symbol, rva);
    const auto& body = candidate.variant ? code.alternatives[candidate.variant - 1] : code.bytes;
    for (const auto& field : code.fields) {
      const auto delta = std::int64_t(static_cast<std::int32_t>(key(bytes.data() + field.offset))) -
                         static_cast<std::int32_t>(key(body.data() + field.offset));
      if (delta % 8 || delta < -4096 || delta > 4096)
        return false;
      candidate.bindings.emplace_back(static_cast<std::uint32_t>(model_.symbols.size() + field.group), static_cast<std::uint32_t>(delta));
    }
    for (const auto& operand : code.operands) {
      const auto value = operand.width == 1 ? bytes[operand.offset] : key(bytes.data() + operand.offset);
      std::int64_t target = value;
      if (operand.kind == AddressKind::pc_relative) {
        const auto sign = operand.width == 1 ? 0x80u : 0x80000000u;
        const auto modulus = operand.width == 1 ? 256ll : 4294967296ll;
        const auto displacement = std::int64_t(value) - ((value & sign) ? modulus : 0);
        target = std::int64_t(rva) + operand.pc_offset + displacement;
      }
      if (target < 0 || target >= image_.image_size)
        return false;
      const auto base = target - operand.addend;
      if (base < 0 || base > Unknown || !symbol_valid(operand.target_symbol, static_cast<std::uint32_t>(base)))
        return false;
      candidate.bindings.emplace_back(operand.target_symbol, static_cast<std::uint32_t>(base));
    }
    if (!normalize(candidate) || !follow_pointers(candidate))
      return false;
    if (!agrees(candidate, fixed_))
      return false;
    for (const auto& constant : model_.constants) {
      const auto binding = std::lower_bound(candidate.bindings.begin(), candidate.bindings.end(), Binding{constant.symbol, 0});
      if (binding != candidate.bindings.end() && binding->first == constant.symbol && !check_constant(constant, binding->second))
        return false;
    }
    return true;
  }

  /*
   * Records a distinct bounded candidate without allowing duplicate matches or capacity overflow to
   * masquerade as unique resolution.
   */
  bool add_candidate(std::size_t index, std::uint32_t rva) {
    auto& state = states_[index];
    if (!state.attempted.insert(rva).second)
      return true;
    if (++attempts_ > 16384)
      return fail("candidate_attempt_limit");
    Candidate candidate;
    if (match(index, rva, candidate)) {
      if (state.candidates.size() >= 64)
        return fail("candidate_limit");
      state.candidates.push_back(std::move(candidate));
      ++result_.candidate_count;
    }
    return result_.error.empty();
  }

  /*
   * Searches eligible image sections for a symbol's verified template, collecting candidates within the
   * configured scan/read limits.
   */
  bool scan() {
    std::vector<std::uint8_t> bytes(Chunk + maximum_seed_ - 1);
    for (const auto& section : image_.sections) {
      if (!static_code(section))
        continue;
      for (std::uint32_t offset = 0; offset < section.size;) {
        const auto count = std::min(Chunk, section.size - offset);
        const auto size = static_cast<std::uint32_t>(std::min<std::uint64_t>(bytes.size(), section.size - offset));
        if (!read(section.rva + offset, bytes.data(), size))
          return false;
        result_.scanned_bytes += count;
        for (std::uint32_t at = 0; at < count && std::uint64_t(at) + 7 <= size; ++at) {
          const auto seeds = seeds_.find(key(bytes.data() + at));
          if (seeds == seeds_.end())
            continue;
          for (const auto& [index, run] : seeds->second) {
            auto& state = states_[index];
            const auto [run_offset, run_size] = run ? state.evidence_runs[run - 1] : std::pair{state.seed_offset, state.seed_size};
            if (std::uint64_t(at) + run_size > size ||
                std::memcmp(bytes.data() + at, model_.code[index].bytes.data() + run_offset, run_size))
              continue;
            const auto address = std::uint64_t(section.rva) + offset + at;
            if (address < run_offset)
              continue;
            if (run) {
              if (state.evidence_places.size() < 4)
                state.evidence_places.insert(static_cast<std::uint32_t>(address - run_offset));
            } else if (!add_candidate(index, static_cast<std::uint32_t>(address - run_offset))) {
              return false;
            }
          }
        }
        offset += count;
      }
    }
    std::string missing;
    for (std::size_t index = 0; index < states_.size(); ++index)
      if (states_[index].discoverable && states_[index].candidates.empty()) {
        missing += (missing.empty() ? "" : ", ") + model_.symbols[model_.code[index].symbol].name;
        record_evidence(index);
      }
    if (!missing.empty() && result_.error.empty())
      result_.error = "template_not_found: " + missing;
    return missing.empty();
  }

  /*
   * Records where a missing template's seed occurred, and where its other invariant runs occurred when the
   * seed itself differs. At the first two such places it lists the differing invariant bytes as spans (at
   * most eight, up to 16 bytes each, found then expected); when every invariant byte matches, an address
   * operand, its section or a relation refused that place.
   */
  void record_evidence(std::size_t index) {
    const auto& code = model_.code[index];
    const auto& state = states_[index];
    std::vector<std::uint32_t> places(state.attempted.begin(), state.attempted.end());
    std::sort(places.begin(), places.end());
    const auto seed_places = places.size();
    for (const auto place : state.evidence_places)
      if (!state.attempted.count(place))
        places.push_back(place);
    char text[128]{};
    std::snprintf(text, sizeof(text), " (model %s): %zu seed place(s), %zu other place(s)", model_.symbols.front().name.c_str(),
                  seed_places, places.size() - seed_places);
    auto line = model_.symbols[code.symbol].name + text;
    std::vector<std::uint8_t> bytes(code.bytes.size());
    for (std::size_t n = 0; n < places.size() && n < 2; ++n) {
      if (!read(places[n], bytes.data(), bytes.size()))
        continue;
      const std::vector<std::uint8_t>* expected = &code.bytes;
      auto closest = std::numeric_limits<std::size_t>::max();
      const auto consider = [&](const std::vector<std::uint8_t>& body) {
        std::size_t differences = 0;
        for (std::size_t at = 0; at < bytes.size(); ++at)
          differences += state.invariant[at] && bytes[at] != body[at];
        if (differences < closest) {
          closest = differences;
          expected = &body;
        }
      };
      consider(code.bytes);
      for (const auto& alternative : code.alternatives)
        consider(alternative);
      std::vector<std::pair<std::size_t, std::size_t>> spans;
      std::size_t differing = 0;
      for (std::size_t at = 0; at < bytes.size(); ++at) {
        if (!state.invariant[at] || bytes[at] == (*expected)[at])
          continue;
        ++differing;
        if (!spans.empty() && at < spans.back().second + 4)
          spans.back().second = at + 1;
        else
          spans.emplace_back(at, at + 1);
      }
      std::snprintf(text, sizeof(text), "; at RVA 0x%x: %zu differing byte(s) in %zu span(s)", places[n], differing, spans.size());
      line += text;
      if (!differing)
        line += ", so an address operand, its section, a shifted field or a relation refused this place";
      for (std::size_t span = 0; span < spans.size() && span < 8; ++span) {
        const auto begin = spans[span].first, end = std::min(spans[span].second, begin + 16);
        std::snprintf(text, sizeof(text), " | +%zu found ", begin);
        line += text + hex_bytes(bytes.data(), begin, end) + " expected " + hex_bytes(expected->data(), begin, end);
      }
    }
    const std::lock_guard lock(evidence_mutex);
    if (evidence_lines.size() < 64)
      evidence_lines.push_back(std::move(line));
  }

  /*
   * Uses relationships to constrain short templates that would be ambiguous in a standalone byte scan.
   *
   * At most one newly reached template layer per pass is needed. Candidate copies avoid references
   * invalidated when another short domain grows.
   */
  bool infer_short_templates() {
    for (const auto& [symbol, rva] : fixed_.bindings) {
      const auto target = template_for_symbol_[symbol];
      if (target != Unknown && !states_[target].discoverable && !add_candidate(target, rva))
        return false;
    }
    for (std::size_t pass = 0; pass <= states_.size(); ++pass) {
      const auto before = result_.candidate_count;
      for (std::size_t i = 0; i < states_.size(); ++i) {
        const auto candidates = states_[i].candidates;
        for (const auto& candidate : candidates)
          for (const auto& [symbol, rva] : candidate.bindings) {
            const auto target = template_for_symbol_[symbol];
            if (target != Unknown && !states_[target].discoverable && !add_candidate(target, rva))
              return false;
          }
      }
      if (result_.candidate_count == before) {
        for (const auto& state : states_)
          if (state.candidates.empty())
            return fail("unresolved_dependent_template");
        return true;
      }
    }
    return fail("inference_iteration_limit");
  }

  /*
   * Eliminates candidates whose symbol relationships cannot agree and repeats until stable. A remaining
   * ambiguous graph is not converted into a guessed address.
   *
   * Conservative arc consistency: remove candidates without support in every other domain. No branch
   * selection, search guesses or arbitrary tie-breaks.
   */
  bool constrain() {
    std::uint64_t comparisons = 0;
    for (std::uint32_t pass = 0; pass < 4096; ++pass) {
      bool changed = false;
      for (std::size_t i = 0; i < states_.size(); ++i) {
        auto& candidates = states_[i].candidates;
        for (std::size_t at = 0; at < candidates.size();) {
          bool supported = true;
          for (std::size_t other = 0; other < states_.size() && supported; ++other) {
            if (other == i)
              continue;
            bool found = false;
            for (const auto& candidate : states_[other].candidates) {
              if (++comparisons > 16777216)
                return fail("constraint_comparison_limit");
              if (agrees(candidates[at], candidate)) {
                found = true;
                break;
              }
            }
            supported = found;
          }
          if (!supported) {
            candidates.erase(candidates.begin() + at);
            changed = true;
          } else {
            ++at;
          }
        }
        if (candidates.empty())
          return fail("inconsistent_template_graph");
      }
      if (!changed)
        return true;
    }
    return fail("constraint_iteration_limit");
  }

  discovery::ImageReader& reader_;
  const discovery::Inventory& image_;
  const ContractModel& model_;
  const std::vector<SymbolBinding>& roots_;
  std::uint64_t loaded_image_base_;
  Candidate fixed_;
  ContractResolution result_;
  std::vector<TemplateState> states_;
  std::vector<std::uint32_t> template_for_symbol_;
  std::unordered_map<std::uint32_t, std::vector<std::pair<std::size_t, std::uint32_t>>> seeds_;
  std::uint64_t code_size_ = 0;
  std::uint32_t maximum_seed_ = 0, attempts_ = 0, groups_ = 0;
};
} /* namespace */

ContractResolution resolve_contract(discovery::ImageReader& reader,
                                    const discovery::Inventory& image,
                                    const ContractModel& model,
                                    const std::vector<SymbolBinding>& roots,
                                    std::uint64_t loaded_image_base) {
  return Resolver(reader, image, model, roots, loaded_image_base).run();
}

std::vector<std::string> failure_evidence() {
  const std::lock_guard lock(evidence_mutex);
  return evidence_lines;
}

} /* namespace azp::native_camera::relocatable */
