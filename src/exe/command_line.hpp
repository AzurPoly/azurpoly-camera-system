/*
 * Declares the EXE command line. Camera settings are chosen in the window and saved (user_settings.hpp); the
 * command line only selects how a run starts: with the window, in the background (--silent, how MSFS starts
 * AZP Camera through its exe.xml), --stop, or --autostart for an installer.
 */

#pragma once
#include <string>

namespace azp::app {
/*
 * `silent`: no window; the run ends when the attached MSFS exits. `stop`: asks a running EXE to stop.
 * `autostart`: 1 adds, 0 removes the MSFS start entry, then the EXE exits; -1 when absent. `error` is set for
 * an invalid command line.
 */
struct Options {
  bool silent = false, stop = false, help = false;
  int autostart = -1;
  std::wstring error;
};
Options parse_command_line(int argc, wchar_t** argv);
std::wstring usage();

}  /* namespace azp::app */
