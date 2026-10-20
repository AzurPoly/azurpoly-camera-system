/*
 * Defines the launch results, module inventory and retry policy shared by the EXE and loader. Only safe
 * discovery failures are retryable; a remote operation that may already have started must not be repeated
 * automatically.
 */

#pragma once
#include <windows.h>
#include <tlhelp32.h>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>
#include <atomic>
#include <cstdio>

namespace azp::standalone {
struct Module {
  std::wstring name, path;
  std::uintptr_t base{};
  DWORD bytes{};
};
struct ModuleInventory {
  std::vector<Module> entries;
  DWORD error = ERROR_SUCCESS;
  unsigned attempts = 0;
};
struct WindowsModuleReader {
  HANDLE snapshot(DWORD pid) { return CreateToolhelp32Snapshot(TH32CS_SNAPMODULE | TH32CS_SNAPMODULE32, pid); }
  bool first(HANDLE snapshot, MODULEENTRY32W& entry) { return Module32FirstW(snapshot, &entry) != FALSE; }
  bool next(HANDLE snapshot, MODULEENTRY32W& entry) { return Module32NextW(snapshot, &entry) != FALSE; }
  DWORD error() { return GetLastError(); }
  void close(HANDLE snapshot) { CloseHandle(snapshot); }
  void pause() { Sleep(10); }
};
/*
 * Reads a complete module snapshot with bounded retries for the documented snapshot race. Failed or partial
 * enumeration is never returned as proof that the bridge DLL is absent.
 *
 * A module snapshot can race Windows loading/unloading another DLL. Retry the documented ERROR_BAD_LENGTH
 * only; never return a failed or partial inventory as evidence that a particular module is absent. No target
 * operation is retried.
 * https://learn.microsoft.com/windows/win32/api/tlhelp32/nf-tlhelp32-createtoolhelp32snapshot
 */
template <class Reader>
ModuleInventory module_inventory(DWORD pid, Reader& reader) {
  ModuleInventory result;
  constexpr unsigned MaximumAttempts = 32;
  constexpr std::size_t MaximumModules = 4096;
  for (unsigned attempt = 0; attempt < MaximumAttempts; ++attempt) {
    ++result.attempts;
    result.entries.clear();
    const auto snapshot = reader.snapshot(pid);
    if (snapshot == INVALID_HANDLE_VALUE) {
      result.error = reader.error();
      if (!result.error)
        result.error = ERROR_GEN_FAILURE;
    } else {
      MODULEENTRY32W entry{};
      entry.dwSize = sizeof(entry);
      bool available = reader.first(snapshot, entry);
      result.error = available ? ERROR_SUCCESS : reader.error();
      if (!available && !result.error)
        result.error = ERROR_GEN_FAILURE;
      while (available) {
        if (result.entries.size() == MaximumModules) {
          result.error = ERROR_BUFFER_OVERFLOW;
          break;
        }
        result.entries.push_back({entry.szModule, entry.szExePath, reinterpret_cast<std::uintptr_t>(entry.modBaseAddr), entry.modBaseSize});
        available = reader.next(snapshot, entry);
        if (!available) {
          result.error = reader.error();
          if (!result.error)
            result.error = ERROR_GEN_FAILURE;
        }
      }
      reader.close(snapshot);
      if (result.error == ERROR_NO_MORE_FILES)
        result.error = ERROR_SUCCESS;
    }
    if (result.error == ERROR_SUCCESS)
      return result;
    result.entries.clear();
    if (result.error != ERROR_BAD_LENGTH || attempt + 1 == MaximumAttempts)
      return result;
    reader.pause();
  }
  return result;
}
std::vector<Module> modules(DWORD pid, DWORD* error = nullptr, unsigned* attempts = nullptr);
}  /* namespace azp::standalone */

namespace azp::standalone {
bool same_path(const std::wstring& a, const std::wstring& b);
bool same_user(HANDLE process);
DWORD find_simulator(const std::wstring& expected);
bool read_exact(HANDLE process, std::uintptr_t address, void* data, SIZE_T count);
bool amd64_image(HANDLE process, const Module& module, DWORD* timestamp = nullptr);
std::uintptr_t remote_export(HANDLE process, DWORD pid, FARPROC function);
struct LaunchResult {
  bool ok{};
  DWORD error{};
  std::wstring message;
  /*
   * Only a preflight read can be retried. Once a remote load/start may have begun, every result remains
   * terminal for this companion session.
   */
  bool retry_before_load = false;
};
struct LaunchDiagnostics {
  const wchar_t* phase = L"preflight";
  DWORD module_error = 0, loader_thread_result = 0, loader_result_error = 0;
  unsigned module_attempts = 0;
  bool load_started = false;
};
class LaunchRetry {
 public:
  bool ready(std::uint64_t now) const noexcept { return now >= next_; }
  bool schedule(const LaunchResult& result, std::uint64_t now) noexcept;

 private:
  unsigned retries_{};
  std::uint64_t next_{};
};
DWORD wait_for_loader(HANDLE thread, const std::atomic<bool>* running);
LaunchResult load_bridge(DWORD pid,
                         const std::wstring& expected_exe,
                         const std::wstring& dll,
                         const std::atomic<bool>* running = nullptr,
                         LaunchDiagnostics* diagnostics = nullptr);
}  /* namespace azp::standalone */

namespace azp::standalone {
void log_launch(const std::wstring& directory, DWORD pid, const LaunchResult& result, const LaunchDiagnostics& trace) noexcept;
}  /* namespace azp::standalone */
