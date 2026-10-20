/*
 * Locates the installed WebView2 Runtime and creates its environment. EdgeUpdate records the runtime's
 * version folder under ClientState\{stable channel}\EBWebView, machine-wide or per user; the environment
 * comes from CreateWebViewEnvironmentWithOptionsInternal in that folder's EBWebView\x64\
 * EmbeddedBrowserWebView.dll, the entry point WebView2Loader.dll itself calls for an installed runtime.
 */

#include "webview2_runtime.hpp"
#include <cwchar>

namespace azp::app {
namespace {
constexpr const wchar_t* kClientState = L"SOFTWARE\\Microsoft\\EdgeUpdate\\ClientState\\{F3017226-FE2A-4295-8BDF-00C3A9A7E4C5}";
/*
 * `installed` selects the runtime installed with Windows rather than one shipped beside the application.
 */
constexpr int kInstalledRuntime = 0;
using CreateEnvironment = HRESULT(STDMETHODCALLTYPE*)(bool, int, PCWSTR, IUnknown*,
                                                       ICoreWebView2CreateCoreWebView2EnvironmentCompletedHandler*);

/*
 * The installed runtime's version folder: the machine-wide installation first, then a per-user one. EdgeUpdate
 * is a 32-bit program, so machine-wide keys are read from the 32-bit registry view.
 */
std::wstring runtime_folder() {
  for (const HKEY root : {HKEY_LOCAL_MACHINE, HKEY_CURRENT_USER}) {
    HKEY key{};
    if (RegOpenKeyExW(root, kClientState, 0, KEY_QUERY_VALUE | KEY_WOW64_32KEY, &key) != ERROR_SUCCESS)
      continue;
    wchar_t folder[1024]{};
    DWORD bytes = sizeof(folder) - sizeof(wchar_t);
    const auto result = RegGetValueW(key, nullptr, L"EBWebView", RRF_RT_REG_SZ, nullptr, folder, &bytes);
    RegCloseKey(key);
    if (result == ERROR_SUCCESS && *folder)
      return folder;
  }
  return {};
}
}  /* namespace */

bool create_webview_environment(const std::wstring& user_data,
                                ICoreWebView2CreateCoreWebView2EnvironmentCompletedHandler* completed,
                                std::wstring& error) {
  const auto folder = runtime_folder();
  if (folder.empty()) {
    error = L"The Microsoft Edge WebView2 Runtime is not installed.";
    return false;
  }
  const auto library = folder + L"\\EBWebView\\x64\\EmbeddedBrowserWebView.dll";
  /*
   * The runtime stays loaded for the life of the EXE: its windows and threads outlive this call.
   */
  const HMODULE module = LoadLibraryExW(library.c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
  if (!module) {
    error = L"The WebView2 Runtime could not be loaded (Windows error " + std::to_wstring(GetLastError()) + L"): " + library;
    return false;
  }
  const auto create =
      reinterpret_cast<CreateEnvironment>(reinterpret_cast<void*>(GetProcAddress(module, "CreateWebViewEnvironmentWithOptionsInternal")));
  if (!create) {
    error = L"This WebView2 Runtime version cannot be started: " + library;
    return false;
  }
  const HRESULT result = create(true, kInstalledRuntime, user_data.c_str(), nullptr, completed);
  if (FAILED(result)) {
    wchar_t code[16]{};
    std::swprintf(code, 16, L"0x%08lX", static_cast<unsigned long>(result));
    error = std::wstring(L"The WebView2 Runtime refused to start (") + code + L").";
    return false;
  }
  return true;
}

}  /* namespace azp::app */
