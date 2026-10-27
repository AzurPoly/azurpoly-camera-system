/*
 * Implements the aircraft-path matching used by both EXE and DLL to recognize the Rafale.
 */

#include "configuration.hpp"

namespace azp {

/*
 * Matches the marker from the start of a path component, with case and slash normalization. Its last
 * component is a prefix, so every azurpoly_rafale* aircraft folder matches; the leading boundary keeps a
 * folder that merely contains the name (another vendor's xazurpoly_rafale) out.
 */
bool path_contains(std::string_view path, std::string_view marker) noexcept {
  const auto fold = [](char c) { return c == '\\' ? '/' : c >= 'A' && c <= 'Z' ? char(c + ('a' - 'A')) : c; };
  for (size_t i = 0; i + marker.size() <= path.size(); ++i) {
    if (i && fold(path[i - 1]) != '/')
      continue;
    size_t n = 0;
    while (n < marker.size() && fold(path[i + n]) == fold(marker[n]))
      ++n;
    if (n == marker.size())
      return true;
  }
  return false;
}

bool is_rafale_package(std::string_view aircraft_path) noexcept {
  return path_contains(aircraft_path, kRafalePackageMarker);
}

}  /* namespace azp */
