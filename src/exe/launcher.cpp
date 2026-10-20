/*
 * Finds the simulator belonging to the current Windows user and loads the matching bridge DLL into it.
 * Process, executable and module identities are checked before starting the remote loader; uncertain results
 * are reported instead of repeating an injection.
 */

#include "launcher.hpp"

namespace azp::standalone {
std::vector<Module> modules(DWORD pid, DWORD* error, unsigned* attempts) {
  WindowsModuleReader reader;
  auto result = module_inventory(pid, reader);
  if (error)
    *error = result.error;
  if (attempts)
    *attempts = result.attempts;
  return std::move(result.entries);
}

/*
 * Compares executable/module paths, resolving differently named paths by file identity when necessary. Path
 * aliases are accepted only when Windows proves they reference the same file.
 *
 * XboxGames and WindowsApps can expose the same MSFS executable under different paths. Accept aliases only
 * when both handles identify one file.
 * https://learn.microsoft.com/windows/win32/api/winbase/ns-winbase-file_id_info
 */
bool same_path(const std::wstring& a, const std::wstring& b) {
  if (a.empty() || b.empty())
    return false;
  if (_wcsicmp(a.c_str(), b.c_str()) == 0)
    return true;
  constexpr DWORD sharing = FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE;
  HANDLE first = CreateFileW(a.c_str(), FILE_READ_ATTRIBUTES, sharing, nullptr, OPEN_EXISTING, 0, nullptr);
  if (first == INVALID_HANDLE_VALUE)
    return false;
  HANDLE second = CreateFileW(b.c_str(), FILE_READ_ATTRIBUTES, sharing, nullptr, OPEN_EXISTING, 0, nullptr);
  FILE_ID_INFO first_id{}, second_id{};
  const bool same =
      second != INVALID_HANDLE_VALUE && GetFileInformationByHandleEx(first, FileIdInfo, &first_id, sizeof(first_id)) &&
      GetFileInformationByHandleEx(second, FileIdInfo, &second_id, sizeof(second_id)) &&
      first_id.VolumeSerialNumber == second_id.VolumeSerialNumber &&
      std::equal(std::begin(first_id.FileId.Identifier), std::end(first_id.FileId.Identifier), std::begin(second_id.FileId.Identifier));
  if (second != INVALID_HANDLE_VALUE)
    CloseHandle(second);
  CloseHandle(first);
  return same;
}

/*
 * Compares the target and current process user SIDs. Opening a process is not sufficient evidence that its
 * simulator belongs to this user.
 */
bool same_user(HANDLE process) {
  HANDLE own{}, other{};
  if (!OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &own))
    return false;
  if (!OpenProcessToken(process, TOKEN_QUERY, &other)) {
    CloseHandle(own);
    return false;
  }
  DWORD na{}, nb{};
  GetTokenInformation(own, TokenUser, nullptr, 0, &na);
  GetTokenInformation(other, TokenUser, nullptr, 0, &nb);
  std::vector<unsigned char> a(na), b(nb);
  const bool ok = na && nb && GetTokenInformation(own, TokenUser, a.data(), na, &na) &&
                  GetTokenInformation(other, TokenUser, b.data(), nb, &nb) &&
                  EqualSid(reinterpret_cast<TOKEN_USER*>(a.data())->User.Sid, reinterpret_cast<TOKEN_USER*>(b.data())->User.Sid);
  CloseHandle(own);
  CloseHandle(other);
  return ok;
}

/*
 * Enumerates processes in the current Windows session and selects the expected MSFS executable owned by this
 * user. An optional requested path must identify the same executable.
 */
DWORD find_simulator(const std::wstring& expected) {
  HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
  if (snapshot == INVALID_HANDLE_VALUE)
    return 0;
  DWORD found = 0, session{};
  ProcessIdToSessionId(GetCurrentProcessId(), &session);
  PROCESSENTRY32W entry{};
  entry.dwSize = sizeof(entry);
  if (Process32FirstW(snapshot, &entry))
    do {
      DWORD other_session{};
      if (_wcsicmp(entry.szExeFile, L"FlightSimulator2024.exe") || !ProcessIdToSessionId(entry.th32ProcessID, &other_session) ||
          session != other_session)
        continue;
      HANDLE process = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, entry.th32ProcessID);
      if (!process)
        continue;
      wchar_t path[32768];
      DWORD length = 32768;
      const bool match =
          QueryFullProcessImageNameW(process, 0, path, &length) && same_user(process) && (expected.empty() || same_path(expected, path));
      CloseHandle(process);
      if (match) {
        if (found) {
          found = 0;
          break;
        }
        found = entry.th32ProcessID;
      }
    } while (Process32NextW(snapshot, &entry));
  CloseHandle(snapshot);
  return found;
}

bool read_exact(HANDLE process, std::uintptr_t address, void* data, SIZE_T count) {
  SIZE_T read{};
  return address && address <= UINTPTR_MAX - count &&
         ReadProcessMemory(process, reinterpret_cast<const void*>(address), data, count, &read) && read == count;
}

/*
 * Checks the remote PE headers before interpreting the module as an x64 image. Failed bounded reads or a
 * different machine format reject it.
 */
bool amd64_image(HANDLE process, const Module& module, DWORD* timestamp) {
  IMAGE_DOS_HEADER dos{};
  IMAGE_NT_HEADERS64 nt{};
  if (!read_exact(process, module.base, &dos, sizeof(dos)) || dos.e_magic != IMAGE_DOS_SIGNATURE ||
      dos.e_lfanew < static_cast<LONG>(sizeof(dos)) || dos.e_lfanew > 1024 * 1024 ||
      !read_exact(process, module.base + dos.e_lfanew, &nt, sizeof(nt)))
    return false;
  if (timestamp)
    *timestamp = nt.FileHeader.TimeDateStamp;
  return nt.Signature == IMAGE_NT_SIGNATURE && nt.FileHeader.Machine == IMAGE_FILE_MACHINE_AMD64 &&
         nt.OptionalHeader.Magic == IMAGE_NT_OPTIONAL_HDR64_MAGIC && nt.OptionalHeader.SizeOfImage == module.bytes;
}

/*
 * Locates the remote module corresponding to a local exported function and applies its relative offset only
 * after module identity and architecture agree.
 */
std::uintptr_t remote_export(HANDLE process, DWORD pid, FARPROC function) {
  HMODULE owner{};
  if (!function || !GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                                       reinterpret_cast<LPCWSTR>(function), &owner))
    return 0;
  wchar_t path[32768]{};
  if (!GetModuleFileNameW(owner, path, 32768))
    return 0;
  const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(owner);
  const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS64*>(reinterpret_cast<const unsigned char*>(owner) + dos->e_lfanew);
  const auto rva = reinterpret_cast<std::uintptr_t>(function) - reinterpret_cast<std::uintptr_t>(owner);
  for (const auto& m : modules(pid)) {
    DWORD timestamp{};
    if (same_path(m.path, path) && amd64_image(process, m, &timestamp) && timestamp == nt->FileHeader.TimeDateStamp && rva < m.bytes)
      return m.base + rva;
  }
  return 0;
}

bool LaunchRetry::schedule(const LaunchResult& result, std::uint64_t now) noexcept {
  if (result.ok || !result.retry_before_load || retries_ >= 59 || now > UINT64_MAX - 1000)
    return false;
  ++retries_;
  next_ = now + 1000;
  return true;
}

DWORD wait_for_loader(HANDLE thread, const std::atomic<bool>* running) {
  const auto deadline = GetTickCount64() + 30000;
  while ((!running || running->load()) && GetTickCount64() < deadline) {
    const auto result = WaitForSingleObject(thread, 100);
    if (result != WAIT_TIMEOUT)
      return result;
  }
  return WAIT_TIMEOUT;
}

/*
 * Checks the simulator and existing module inventory, resolves its loader, then loads and starts the
 * requested DLL. Every uncertain remote operation ends this attempt; its arguments and module lifetime must
 * remain valid even if waiting times out.
 *
 * The loader may still own this argument on timeout. Keep it until process exit.
 *
 * LoadLibraryW returns an HMODULE; its thread exit code retains only the low 32 bits. Record it as a
 * diagnostic, never use it as a module address or a remote GetLastError value. Full module identity remains
 * the authority.
 */
LaunchResult load_bridge(DWORD pid,
                         const std::wstring& expected_exe,
                         const std::wstring& dll,
                         const std::atomic<bool>* running,
                         LaunchDiagnostics* diagnostics) {
  LaunchDiagnostics local_diagnostics;
  auto& trace = diagnostics ? *diagnostics : local_diagnostics;
  trace = {};
  constexpr DWORD rights =
      PROCESS_CREATE_THREAD | PROCESS_QUERY_INFORMATION | PROCESS_VM_OPERATION | PROCESS_VM_WRITE | PROCESS_VM_READ | SYNCHRONIZE;
  HANDLE process = OpenProcess(rights, FALSE, pid);
  if (!process)
    return {false, GetLastError(), L"Cannot open MSFS in this Windows session."};
  struct Close {
    HANDLE p;
    ~Close() { CloseHandle(p); }
  } close{process};
  if (!same_user(process))
    return {false, ERROR_ACCESS_DENIED, L"MSFS must run under the same Windows account."};
  wchar_t path[32768]{};
  DWORD n = 32768;
  if (!QueryFullProcessImageNameW(process, 0, path, &n) || (!expected_exe.empty() && !same_path(expected_exe, path)))
    return {false, ERROR_BAD_ENVIRONMENT, L"Simulator executable path changed; attach refused."};
  auto inventory = modules(pid, &trace.module_error, &trace.module_attempts);
  if (trace.module_error)
    return {false, trace.module_error, L"Cannot read the simulator module list during startup.",
            trace.module_error == ERROR_BAD_LENGTH || trace.module_error == ERROR_PARTIAL_COPY};
  Module main{}, bridge{};
  for (const auto& m : inventory) {
    if (!_wcsicmp(m.name.c_str(), L"FlightSimulator2024.exe") && same_path(m.path, path))
      main = m;
    if (!_wcsicmp(m.name.c_str(), L"azp-camera.dll") && !same_path(m.path, dll))
      return {false, ERROR_ALREADY_EXISTS,
              L"Another camera DLL is already loaded from: " + m.path + L"\nExit MSFS completely before changing the folder of AZP-Camera.exe."};
    if (same_path(m.path, dll))
      bridge = m;
  }
  if (!main.base || !amd64_image(process, main))
    return {false, ERROR_BAD_EXE_FORMAT, L"Waiting for readable, valid MSFS executable headers.", true};
  if (!bridge.base) {
    const auto loader = remote_export(process, pid, GetProcAddress(GetModuleHandleW(L"kernel32.dll"), "LoadLibraryW"));
    if (!loader)
      return {false, ERROR_INVALID_ADDRESS, L"Waiting for the verified Windows DLL loader in MSFS.", true};
    const SIZE_T bytes = (dll.size() + 1) * sizeof(wchar_t);
    void* memory = VirtualAllocEx(process, nullptr, bytes, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
    if (!memory)
      return {false, GetLastError(), L"Could not allocate the bridge path."};
    SIZE_T wrote{};
    if (!WriteProcessMemory(process, memory, dll.c_str(), bytes, &wrote) || wrote != bytes) {
      const DWORD e = GetLastError();
      VirtualFreeEx(process, memory, 0, MEM_RELEASE);
      return {false, e, L"Could not write the bridge path."};
    }
    HANDLE thread = CreateRemoteThread(process, nullptr, 0, reinterpret_cast<LPTHREAD_START_ROUTINE>(loader), memory, 0, nullptr);
    if (!thread) {
      const DWORD e = GetLastError();
      VirtualFreeEx(process, memory, 0, MEM_RELEASE);
      return {false, e, L"Windows refused to load the camera bridge."};
    }
    trace.load_started = true;
    trace.phase = L"windows_loader";
    const DWORD wait = wait_for_loader(thread, running);
    if (wait == WAIT_OBJECT_0 && !GetExitCodeThread(thread, &trace.loader_thread_result))
      trace.loader_result_error = GetLastError();
    CloseHandle(thread);
    if (wait != WAIT_OBJECT_0)
      return {false, WAIT_TIMEOUT, L"Bridge loading is still pending; no second load will be attempted this session."};
    VirtualFreeEx(process, memory, 0, MEM_RELEASE);
    trace.phase = L"after_load_module_scan";
    inventory = modules(pid, &trace.module_error, &trace.module_attempts);
    if (trace.module_error)
      return {false, trace.module_error, L"Windows finished the load attempt, but the bridge module check failed. See launcher.log."};
    for (const auto& m : inventory)
      if (same_path(m.path, dll))
        bridge = m;
    if (!bridge.base)
      return {false, ERROR_MOD_NOT_FOUND, L"The bridge was not found after Windows loading completed. See launcher.log for details."};
  }
  trace.phase = L"bridge_export";
  HMODULE local = LoadLibraryExW(dll.c_str(), nullptr, DONT_RESOLVE_DLL_REFERENCES);
  if (!local)
    return {false, GetLastError(), L"Cannot read the camera bridge's entry point."};
  const auto start = GetProcAddress(local, "AzpCameraStart");
  const auto offset = start ? reinterpret_cast<std::uintptr_t>(start) - reinterpret_cast<std::uintptr_t>(local) : UINTPTR_MAX;
  FreeLibrary(local);
  if (offset >= bridge.bytes || !amd64_image(process, bridge))
    return {false, ERROR_INVALID_ADDRESS, L"Invalid camera bridge entry point."};
  HANDLE thread =
      CreateRemoteThread(process, nullptr, 0, reinterpret_cast<LPTHREAD_START_ROUTINE>(bridge.base + offset), nullptr, 0, nullptr);
  if (!thread)
    return {false, GetLastError(), L"Cannot start the loaded camera bridge."};
  trace.phase = L"bridge_start";
  const DWORD wait = wait_for_loader(thread, running);
  DWORD result = ERROR_GEN_FAILURE;
  if (wait == WAIT_OBJECT_0)
    GetExitCodeThread(thread, &result);
  CloseHandle(thread);
  if (wait != WAIT_OBJECT_0)
    return {false, WAIT_TIMEOUT, L"Camera bridge startup is still pending."};
  if (result)
    return {false, result, L"The camera bridge refused to start."};
  trace.phase = L"complete";
  return {true, 0, L"Camera bridge loaded. Waiting for graphics and the aircraft."};
}

/*
 * Writes the launch outcome to a bounded launcher.log in `directory`, the EXE's dated log folder. File or
 * formatting failures are contained so reporting cannot alter the loader result.
 *
 * Diagnostics must never change the result or disrupt startup.
 */
void log_launch(const std::wstring& directory, DWORD pid, const LaunchResult& result, const LaunchDiagnostics& trace) noexcept {
  try {
    if (directory.empty())
      return;
    SYSTEMTIME utc{};
    GetSystemTime(&utc);
    wchar_t prefix[512]{};
    std::swprintf(prefix, 512,
                  L"%04u-%02u-%02uT%02u:%02u:%02uZ pid=%lu ok=%u error=%lu phase=%ls load_started=%u module_error=%lu "
                  L"module_attempts=%u loader_thread_low32=0x%08lx loader_result_error=%lu retry_before_load=%u | ",
                  utc.wYear, utc.wMonth, utc.wDay, utc.wHour, utc.wMinute, utc.wSecond, pid, unsigned(result.ok), result.error, trace.phase,
                  unsigned(trace.load_started), trace.module_error, trace.module_attempts, trace.loader_thread_result,
                  trace.loader_result_error, unsigned(result.retry_before_load));
    const auto line = std::wstring(prefix) + result.message + L"\r\n";
    const int size =
        WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, line.data(), static_cast<int>(line.size()), nullptr, 0, nullptr, nullptr);
    if (size <= 0 || size > 8192)
      return;
    std::string bytes(size, '\0');
    if (WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, line.data(), static_cast<int>(line.size()), bytes.data(), size, nullptr,
                            nullptr) != size)
      return;
    const auto path = directory + L"\\launcher.log";
    HANDLE file = CreateFileW(path.c_str(), FILE_APPEND_DATA | FILE_READ_ATTRIBUTES, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                              nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file == INVALID_HANDLE_VALUE)
      return;
    LARGE_INTEGER length{};
    DWORD written = 0;
    if (GetFileSizeEx(file, &length) && length.QuadPart >= 0 && length.QuadPart < 4 * 1024 * 1024)
      WriteFile(file, bytes.data(), static_cast<DWORD>(bytes.size()), &written, nullptr);
    CloseHandle(file);
  } catch (...) {
  }
}

}  /* namespace azp::standalone */
