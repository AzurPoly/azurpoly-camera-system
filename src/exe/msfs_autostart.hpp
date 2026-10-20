/*
 * Makes MSFS 2024 start AZP Camera. At startup the simulator launches the programs listed in the exe.xml of
 * its settings folder (Steam: %APPDATA%\Microsoft Flight Simulator 2024; Microsoft Store:
 * %LOCALAPPDATA%\Packages\Microsoft.Limitless_8wekyb3d8bbwe\LocalCache). AZP Camera's entry starts this
 * AZP-Camera.exe with --silent. Only that entry is added, replaced or removed: the rest of the file, other
 * add-ons included, is written back unchanged. A change applies the next time MSFS starts.
 */

#pragma once
#include <string>
#include <vector>

namespace azp::app {
/*
 * One installation whose settings folder exists. `current`: its exe.xml starts `executable` (this EXE) with
 * --silent. `other_path`: the EXE an AZP Camera entry starts instead; empty when none. `error`: the file could
 * not be read or changed.
 */
struct SimulatorStartup {
  std::wstring edition, file;
  bool current = false;
  std::wstring other_path;
  std::string error;
};
std::vector<SimulatorStartup> simulator_startup(const std::wstring& executable);
/*
 * Adds (`enabled`) or removes AZP Camera's entry in the exe.xml of every installation, then returns their new
 * state. `ok` is false when an installation failed or none was found.
 */
std::vector<SimulatorStartup> set_simulator_startup(const std::wstring& executable, bool enabled, bool& ok);

}  /* namespace azp::app */
