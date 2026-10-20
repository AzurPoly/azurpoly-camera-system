/*
 * Parses the EXE command line into validated options.
 */

#include "command_line.hpp"
#include <algorithm>
#include <cwctype>

namespace azp::app {
namespace {
std::wstring lowercase(std::wstring text) {
  std::transform(text.begin(), text.end(), text.begin(), [](wchar_t c) { return static_cast<wchar_t>(std::towlower(c)); });
  return text;
}
}  /* namespace */

/*
 * Accepts `--autostart value` and `--autostart=value`; option names and on/off values ignore case. The first
 * problem found is reported in `error`; later options are still read so that `--silent` is known even when
 * another option is wrong.
 */
Options parse_command_line(int argc, wchar_t** argv) {
  Options options;
  const auto fail = [&](const std::wstring& message) {
    if (options.error.empty())
      options.error = message;
  };
  for (int index = 1; index < argc; ++index) {
    std::wstring name = argv[index], value;
    const auto equals = name.find(L'=');
    const bool inline_value = equals != std::wstring::npos;
    if (inline_value) {
      value = name.substr(equals + 1);
      name.resize(equals);
    }
    name = lowercase(name);
    if (name == L"--silent" || name == L"--stop" || name == L"--help" || name == L"/?" || name == L"-h") {
      if (inline_value)
        fail(L"Option " + name + L" takes no value.");
      options.silent = options.silent || name == L"--silent";
      options.stop = options.stop || name == L"--stop";
      options.help = options.help || (name != L"--silent" && name != L"--stop");
      continue;
    }
    if (name != L"--autostart") {
      fail(L"Unknown option: " + std::wstring(argv[index]));
      continue;
    }
    if (!inline_value) {
      if (index + 1 >= argc) {
        fail(L"--autostart needs on or off.");
        continue;
      }
      value = argv[++index];
    }
    const auto choice = lowercase(value);
    if (choice == L"on" || choice == L"1")
      options.autostart = 1;
    else if (choice == L"off" || choice == L"0")
      options.autostart = 0;
    else
      fail(L"--autostart must be on or off.");
  }
  return options;
}

std::wstring usage() {
  return L"AZP-Camera.exe [--silent]\n"
         L"AZP-Camera.exe --stop\n"
         L"AZP-Camera.exe --autostart on|off\n\n"
         L"No option: opens the window, or the window of the copy already running.\n"
         L"--silent: no window. Runs in the background and exits when MSFS closes; this is how MSFS starts it.\n"
         L"--stop: stops the running copy.\n"
         L"--autostart on|off: makes MSFS 2024 start this EXE (exe.xml), or stops it doing so, then exits. Exit code 0 on "
         L"success.\n\n"
         L"Camera settings are chosen in the window and kept for every run.";
}

}  /* namespace azp::app */
