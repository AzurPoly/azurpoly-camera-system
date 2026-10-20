/*
 * Turns what the EXE knows (its settings, the attached simulator and the DLL status) into text: the values
 * the window shows and the line written to the log every ten seconds.
 */

#pragma once
#include "../shared/protocol.hpp"
#include <string>
#include <utility>
#include <vector>

namespace azp::app {
/*
 * One loop iteration, copied for the window and the log. `fresh`: the DLL status is less than five seconds
 * old. `simulator`: the attached MSFS process ID, 0 while waiting. `event`: the last attachment or loader
 * event. `log_path`: the current log file, empty if logging is unavailable.
 */
struct Snapshot {
  standalone::Settings settings;
  standalone::Status status{};
  bool fresh = false;
  DWORD simulator = 0;
  std::string event;
  std::wstring log_path;
};

/*
 * Video input rate (images per second), capture-to-input latency (an upper bound) and image age in
 * milliseconds, as plain numbers, or "-" when the DLL has no valid measurement. The rate counts completed
 * copies into the WASM input image, not final gauge draws.
 */
struct InputMetrics {
  std::string rate = "-", latency = "-", age = "-";
  explicit InputMetrics(const standalone::Status& status);
};
bool infrared(const standalone::Status& status);
std::string log_line(const Snapshot& snapshot);
/*
 * UTF-16 <-> UTF-8 for Windows text written to UTF-8 logs and JSON, and back.
 */
std::string narrow(const std::wstring& text);
std::wstring widen(const std::string& text);

}  /* namespace azp::app */
