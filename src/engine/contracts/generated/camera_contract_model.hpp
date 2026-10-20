/*
 * Declares access to the generated camera model and the binding step that turns a resolved match into a
 * usable contract. Callers must complete the resolver's verification before using those native entry points.
 */

#pragma once
#include <string_view>
#include "../camera_contract.hpp"
#include "../relocatable_contract.hpp"

namespace azp::native_camera::camera_contract_model {

/*
 * Enclosing runtime-function observations, relative to a model symbol. More than one span may cover a
 * template. A negative addend denotes an interior window. required_pdata=false is reserved for reviewed
 * leaf/tail bodies that need no stack-unwind entry; it does not invent unwind metadata or waive the caller's
 * instruction/relocation checks.
 */
struct FunctionBoundary {
  std::uint32_t symbol = 0;
  std::int32_t begin_addend = 0;
  std::uint32_t bytes = 0;
  bool required_pdata = true;
};

const relocatable::ContractModel& model();
const std::vector<FunctionBoundary>& boundaries();

/*
 * Returns UINT32_MAX for an unknown name. Names are semantic roles, never build-specific RVAs. The
 * seven-byte controller template still requires an independently verified and rechecked static RTTI/vtable
 * slot relationship.
 */
std::uint32_t symbol_index(std::string_view name) noexcept;

/*
 * Copies an already successful resolver result. This is a projection, not a compatibility check. Unresolved
 * vtable roles remain zero for the caller's separate identity resolver; no observed-address fallback is
 * provided.
 */
bool bind(const std::vector<std::uint32_t>& symbols, CameraFunctions& functions, CameraImageLayout& layout) noexcept;

}  /* namespace azp::native_camera::camera_contract_model */
