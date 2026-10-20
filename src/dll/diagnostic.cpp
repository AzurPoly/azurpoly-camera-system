/*
 * Implements the DLL's log files and the support report (diagnostic.hpp). The report is plain text in UTF-8.
 */

#include "diagnostic.hpp"
#include <windows.h>
#include <dxgi.h>
#include <tlhelp32.h>
#include <winver.h>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <vector>
#include "../engine/contracts/relocatable_contract.hpp"
#include "../shared/configuration.hpp"
#include "../shared/protocol.hpp"

namespace azp::diagnostic {
namespace {
constexpr std::uint64_t kMaximumLogBytes = 16ull << 20;
constexpr std::size_t kMaximumFileBytes = 64 * 1024;

struct Files {
  std::wstring folder, stamp;
};

std::string narrow(const std::wstring& text) {
  if (text.empty())
    return {};
  const int size = WideCharToMultiByte(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), nullptr, 0, nullptr, nullptr);
  std::string bytes(size > 0 ? size : 0, '\0');
  if (size > 0)
    WideCharToMultiByte(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), bytes.data(), size, nullptr, nullptr);
  return bytes;
}

std::wstring environment(const wchar_t* name) {
  wchar_t value[32768]{};
  const DWORD length = GetEnvironmentVariableW(name, value, 32768);
  return length && length < 32768 ? std::wstring(value, length) : std::wstring();
}

bool make_directory(const std::wstring& path) {
  return CreateDirectoryW(path.c_str(), nullptr) || GetLastError() == ERROR_ALREADY_EXISTS;
}

template <typename... Values>
std::string text(const char* format, Values... values) {
  char line[1024]{};
  std::snprintf(line, sizeof(line), format, values...);
  return line;
}

/*
 * Chooses the folder and the time stamp once, at the first use in this DLL load: next to azp-camera.dll like
 * the EXE's log, else under %LOCALAPPDATA%. The folder is empty when neither can be created.
 */
const Files& files() {
  static const Files chosen = [] {
    Files result;
    SYSTEMTIME now{};
    GetLocalTime(&now);
    wchar_t date[16]{}, stamp[16]{};
    std::swprintf(date, 16, L"%04u-%02u-%02u", now.wYear, now.wMonth, now.wDay);
    std::swprintf(stamp, 16, L"%02u%02u%02u", now.wHour, now.wMinute, now.wSecond);
    result.stamp = stamp;
    std::wstring roots[2];
    HMODULE self = nullptr;
    if (GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                           reinterpret_cast<LPCWSTR>(&append_bridge_log), &self)) {
      wchar_t path[32768]{};
      const DWORD length = GetModuleFileNameW(self, path, 32768);
      const std::wstring module(path, length < 32768 ? length : 0);
      if (const auto slash = module.find_last_of(L"\\/"); slash != std::wstring::npos)
        roots[0] = module.substr(0, slash) + L"\\log";
    }
    if (const auto local = environment(L"LOCALAPPDATA"); !local.empty()) {
      const auto vendor = local + L"\\AzurPoly";
      const auto product = vendor + L"\\CameraSystem";
      if (make_directory(vendor) && make_directory(product))
        roots[1] = product + L"\\log";
    }
    for (const auto& root : roots)
      if (!root.empty() && make_directory(root) && make_directory(root + L"\\" + date)) {
        result.folder = root + L"\\" + date;
        break;
      }
    return result;
  }();
  return chosen;
}

bool write_file(const std::wstring& path, const void* data, std::size_t size) {
  HANDLE file = CreateFileW(path.c_str(), GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE)
    return false;
  DWORD written = 0;
  const bool ok = WriteFile(file, data, static_cast<DWORD>(size), &written, nullptr) && written == size;
  CloseHandle(file);
  return ok;
}

/*
 * Reads at most kMaximumFileBytes of a small text file; empty when it is missing.
 */
std::string read_file(const std::wstring& path) {
  HANDLE file = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr, OPEN_EXISTING,
                            FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE)
    return {};
  std::string bytes(kMaximumFileBytes, '\0');
  DWORD read = 0;
  if (!ReadFile(file, bytes.data(), static_cast<DWORD>(bytes.size()), &read, nullptr))
    read = 0;
  CloseHandle(file);
  bytes.resize(read);
  return bytes;
}

/*
 * "a.b.c.d" from a file's version resource, or "no version resource".
 */
std::string file_version(const std::wstring& path) {
  DWORD ignored = 0;
  const DWORD size = GetFileVersionInfoSizeW(path.c_str(), &ignored);
  std::vector<std::uint8_t> data(size);
  VS_FIXEDFILEINFO* info = nullptr;
  UINT length = 0;
  if (!size || !GetFileVersionInfoW(path.c_str(), 0, size, data.data()) ||
      !VerQueryValueW(data.data(), L"\\", reinterpret_cast<void**>(&info), &length) || !info || length < sizeof(VS_FIXEDFILEINFO))
    return "no version resource";
  return text("%u.%u.%u.%u", unsigned{HIWORD(info->dwFileVersionMS)}, unsigned{LOWORD(info->dwFileVersionMS)},
              unsigned{HIWORD(info->dwFileVersionLS)}, unsigned{LOWORD(info->dwFileVersionLS)});
}

std::string windows_version() {
  using RtlGetVersion = LONG(WINAPI*)(OSVERSIONINFOW*);
  OSVERSIONINFOW info{};
  info.dwOSVersionInfoSize = sizeof(info);
  const auto get = reinterpret_cast<RtlGetVersion>(GetProcAddress(GetModuleHandleW(L"ntdll.dll"), "RtlGetVersion"));
  if (!get || get(&info) != 0)
    return "unknown";
  return text("%lu.%lu.%lu", info.dwMajorVersion, info.dwMinorVersion, info.dwBuildNumber);
}

std::string modules() {
  std::vector<std::pair<std::uintptr_t, std::string>> lines;
  HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE, GetCurrentProcessId());
  if (snapshot == INVALID_HANDLE_VALUE)
    return "  module list unavailable\n";
  MODULEENTRY32W entry{};
  entry.dwSize = sizeof(entry);
  for (bool more = Module32FirstW(snapshot, &entry); more; more = Module32NextW(snapshot, &entry)) {
    const auto base = reinterpret_cast<std::uintptr_t>(entry.modBaseAddr);
    lines.emplace_back(base, text("  0x%016llx %10lu  %-16s %s\n", static_cast<unsigned long long>(base), entry.modBaseSize,
                                  file_version(entry.szExePath).c_str(), narrow(entry.szExePath).c_str()));
  }
  CloseHandle(snapshot);
  std::sort(lines.begin(), lines.end());
  std::string result = text("  %zu modules (base, size, file version, path)\n", lines.size());
  for (const auto& line : lines)
    result += line.second;
  return result;
}

std::string adapters() {
  IDXGIFactory1* factory = nullptr;
  if (FAILED(CreateDXGIFactory1(__uuidof(IDXGIFactory1), reinterpret_cast<void**>(&factory))) || !factory)
    return "  DXGI unavailable\n";
  std::string result;
  IDXGIAdapter1* adapter = nullptr;
  for (UINT index = 0; factory->EnumAdapters1(index, &adapter) != DXGI_ERROR_NOT_FOUND; ++index) {
    DXGI_ADAPTER_DESC1 description{};
    LARGE_INTEGER driver{};
    if (SUCCEEDED(adapter->GetDesc1(&description))) {
      const bool known = SUCCEEDED(adapter->CheckInterfaceSupport(__uuidof(IDXGIDevice), &driver));
      result += text("  %s | vendor 0x%04x device 0x%04x subsystem 0x%08x revision %u | %llu MB | driver %s%s\n",
                     narrow(description.Description).c_str(), description.VendorId, description.DeviceId, description.SubSysId,
                     description.Revision, static_cast<unsigned long long>(description.DedicatedVideoMemory >> 20),
                     known ? text("%u.%u.%u.%u", unsigned{HIWORD(driver.HighPart)}, unsigned{LOWORD(driver.HighPart)},
                                  unsigned{HIWORD(driver.LowPart)}, unsigned{LOWORD(driver.LowPart)})
                                 .c_str()
                           : "unknown",
                     (description.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) ? " | software" : "");
    }
    adapter->Release();
  }
  factory->Release();
  return result.empty() ? "  no adapter\n" : result;
}

/*
 * The simulator's exe.xml and UserCfg.opt for each distribution folder that exists, and the Community folder
 * named by UserCfg.opt's InstalledPackagesPath.
 */
std::string simulator_files() {
  const auto roaming = environment(L"APPDATA"), local = environment(L"LOCALAPPDATA");
  const std::pair<const char*, std::wstring> folders[]{
      {"Steam", roaming.empty() ? std::wstring() : roaming + kSteamSettingsFolder},
      {"Microsoft Store", local.empty() ? std::wstring() : local + kStoreSettingsFolder}};
  std::string result;
  for (const auto& [name, folder] : folders) {
    if (folder.empty() || GetFileAttributesW(folder.c_str()) == INVALID_FILE_ATTRIBUTES)
      continue;
    for (const auto* file : {L"\\exe.xml", L"\\UserCfg.opt"}) {
      const auto content = read_file(folder + file);
      result += "---- " + std::string(name) + ": " + narrow(folder + file) + (content.empty() ? " (missing or empty)" : "") + "\n" + content;
      if (!content.empty() && content.back() != '\n')
        result += '\n';
      if (std::wcscmp(file, L"\\UserCfg.opt") || content.empty())
        continue;
      const auto key = content.find("InstalledPackagesPath");
      const auto open = key == std::string::npos ? key : content.find('"', key);
      const auto close = open == std::string::npos ? open : content.find('"', open + 1);
      if (close == std::string::npos)
        continue;
      const auto community = content.substr(open + 1, close - open - 1) + "\\Community";
      result += "---- Community folder " + community + "\n";
      const int size = MultiByteToWideChar(CP_UTF8, 0, community.data(), static_cast<int>(community.size()), nullptr, 0);
      std::wstring path(size > 0 ? size : 0, L'\0');
      if (size > 0)
        MultiByteToWideChar(CP_UTF8, 0, community.data(), static_cast<int>(community.size()), path.data(), size);
      WIN32_FIND_DATAW found{};
      HANDLE search = FindFirstFileW((path + L"\\*").c_str(), &found);
      for (int count = 0; search != INVALID_HANDLE_VALUE && count < 1000;) {
        if (std::wcscmp(found.cFileName, L".") && std::wcscmp(found.cFileName, L"..")) {
          result += "  " + narrow(found.cFileName) + ((found.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) ? "\n" : " (file)\n");
          ++count;
        }
        if (!FindNextFileW(search, &found))
          break;
      }
      if (search != INVALID_HANDLE_VALUE)
        FindClose(search);
    }
  }
  return result.empty() ? "  no simulator settings folder found\n" : result;
}
}  /* namespace */

void append_bridge_log(const std::string& line) {
  static std::mutex mutex;
  static std::uint64_t bytes = 0;
  const auto& chosen = files();
  if (chosen.folder.empty())
    return;
  const std::lock_guard lock(mutex);
  if (bytes >= kMaximumLogBytes)
    return;
  auto entry = line + "\r\n";
  const bool last = bytes + entry.size() > kMaximumLogBytes;
  if (last)
    entry = "Log size limit reached; no more lines in this file.\r\n";
  HANDLE file = CreateFileW((chosen.folder + L"\\bridge-" + chosen.stamp + L".log").c_str(), FILE_APPEND_DATA,
                            FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE)
    return;
  DWORD written = 0;
  WriteFile(file, entry.data(), static_cast<DWORD>(entry.size()), &written, nullptr);
  CloseHandle(file);
  bytes = last ? kMaximumLogBytes : bytes + written;
}

void write_report(const native_camera::EngineApi& api) {
  static std::atomic<bool> done{false};
  const auto& chosen = files();
  if (done.exchange(true) || chosen.folder.empty())
    return;
  SYSTEMTIME now{};
  GetLocalTime(&now);
  wchar_t simulator[32768]{}, self[32768]{};
  const DWORD simulator_length = GetModuleFileNameW(nullptr, simulator, 32768);
  const std::wstring simulator_path(simulator, simulator_length < 32768 ? simulator_length : 0);
  HMODULE module = nullptr;
  GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                     reinterpret_cast<LPCWSTR>(&write_report), &module);
  const DWORD self_length = module ? GetModuleFileNameW(module, self, 32768) : 0;
  const auto path = narrow(simulator_path);
  auto lowered = path;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  const char* distribution = lowered.find("\\steamapps\\") != std::string::npos                                               ? "Steam"
                             : lowered.find("\\windowsapps\\") != std::string::npos || lowered.find("xboxgames") != std::string::npos ? "Microsoft Store"
                                                                                                                                      : "unknown";

  std::string report = "AZP Camera diagnostic report\n";
  report += text("Local time: %04u-%02u-%02u %02u:%02u:%02u\n", now.wYear, now.wMonth, now.wDay, now.wHour, now.wMinute, now.wSecond);
  report += text("AZP Camera %s, EXE/DLL protocol %u\n", AZP_CAMERA_VERSION, standalone::ProtocolVersion);
  report += "DLL: " + narrow(std::wstring(self, self_length < 32768 ? self_length : 0)) + "\n";
  report += "Windows: " + windows_version() + "\n\n";

  report += "Simulator executable\n";
  report += "  path: " + path + "\n";
  report += std::string("  distribution: ") + distribution + "\n";
  report += "  file version: " + file_version(simulator_path) + "\n";
  report += text("  loaded at 0x%llx; PE timestamp 0x%08x, image size 0x%x, checksum 0x%08x, %u sections\n",
                 static_cast<unsigned long long>(api.base), api.image.timestamp, api.image.image_size, api.image.checksum,
                 static_cast<unsigned>(api.image.sections.size()));
  for (const auto& section : api.image.sections)
    report += text("  section %-8s RVA 0x%08x size 0x%08x flags 0x%08x\n", section.name.c_str(), section.rva, section.size, section.flags);

  report += "\nEngine contracts\n";
  for (const auto& part : api.parts)
    report += std::string("  ") + part.name + ": " + (part.valid ? "ok" : part.required ? "FAILED" : "unavailable") +
              (part.detail.empty() ? "" : " (" + part.detail + ")") + "\n";
  report += "  night vision detail: " + api.night_vision.error + "\n";
  const auto evidence = native_camera::relocatable::failure_evidence();
  report += text("\nTemplate evidence (%zu)\n", evidence.size());
  for (const auto& line : evidence)
    report += "  " + line + "\n";

  report += "\nModules loaded in MSFS\n" + modules();
  report += "\nGraphics adapters\n" + adapters();
  report += "\nSimulator files\n" + simulator_files();
  write_file(chosen.folder + L"\\diagnostic-" + chosen.stamp + L".txt", report.data(), report.size());
}

}  /* namespace azp::diagnostic */
