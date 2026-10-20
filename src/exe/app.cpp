/*
 * AZP-Camera.exe. Every run keeps the companion loop (companion.hpp) on a worker thread and a Win32 message
 * loop on the main thread, where the notification-area icon and the window (desktop_window.hpp) live.
 *
 * Started without options, the EXE opens its window. With --silent, which is how MSFS starts it through
 * exe.xml (msfs_autostart.hpp), there is no initial window and AZP Camera exits when the attached MSFS exits.
 * Its notification-area icon shows that the run is active: selecting it opens the window, closing the window
 * hides it again, and its Open/Quit menu provides the explicit lifecycle controls. One AZP Camera runs at a
 * time; starting the EXE again opens the running instance. `--stop` sets a named event that ends either kind
 * of run. At exit the video demand is withdrawn; the DLL stays loaded until MSFS exits.
 */

#include "application_controller.hpp"
#include "command_line.hpp"
#include "companion.hpp"
#include "desktop_window.hpp"
#include "msfs_autostart.hpp"
#include "resources.hpp"
#include "run_log.hpp"
#include "user_settings.hpp"
#include <windows.h>
#include <windowsx.h>
#include <objbase.h>
#include <shellapi.h>
#include <algorithm>
#include <memory>
#include <thread>

namespace {
namespace app = azp::app;

constexpr const wchar_t* kProductName = L"" AZP_CAMERA_PRODUCT_NAME;
constexpr const wchar_t* kInstanceMutex = L"Local\\AZPCamera.Companion";
constexpr const wchar_t* kStopEvent = L"Local\\AZPCamera.Stop";
/*
 * The hidden top-level window through which the notification icon and a second EXE start control the run.
 * It is a tool window so Windows never gives this control surface its own taskbar button.
 */
constexpr const wchar_t* kControlClass = L"AZPCamera.Control";
constexpr UINT kShowWindow = WM_APP + 1, kWindowClosed = WM_APP + 2, kNotificationIconMessage = WM_APP + 3;
constexpr UINT kNotificationIconId = 1;
constexpr UINT_PTR kOpenCommand = 1, kQuitCommand = 2;
constexpr UINT_PTR kFinishedTimer = 1;

struct NotificationMenuPlacement {
  POINT point{};
  UINT alignment = TPM_RIGHTALIGN | TPM_BOTTOMALIGN;
};

/*
 * %LOCALAPPDATA%\AzurPoly\CameraSystem: settings.json, the WebView2 profile, and the log folder when the one beside
 * AZP-Camera.exe cannot be created. Empty when unavailable.
 */
std::wstring local_directory() {
  wchar_t local[32768]{};
  const DWORD length = GetEnvironmentVariableW(L"LOCALAPPDATA", local, 32768);
  if (!length || length >= 32700)
    return {};
  const auto vendor = std::wstring(local) + L"\\AzurPoly";
  if (!CreateDirectoryW(vendor.c_str(), nullptr) && GetLastError() != ERROR_ALREADY_EXISTS)
    return {};
  const auto directory = vendor + L"\\CameraSystem";
  if (!CreateDirectoryW(directory.c_str(), nullptr) && GetLastError() != ERROR_ALREADY_EXISTS)
    return {};
  return directory;
}

std::wstring executable_path() {
  wchar_t path[32768]{};
  const DWORD length = GetModuleFileNameW(nullptr, path, 32768);
  return length && length < 32768 ? std::wstring(path, length) : std::wstring();
}

/*
 * The main thread's part of a run: the control window, notification-area icon, the AZP Camera window while it
 * is open, and the controller they share.
 */
class Application {
 public:
  Application(HINSTANCE instance,
              app::Companion& companion,
              app::RunLog& log,
              const std::wstring& local,
              const app::UserSettings& settings,
              const std::wstring& executable,
              bool background)
      : instance_(instance),
        companion_(companion),
        background_(background),
        controller_(companion,
                    log,
                    local.empty() ? std::wstring() : local + L"\\settings.json",
                    settings,
                    executable,
                    [this] { quit(); }),
        user_data_(local.empty() ? std::wstring() : local + L"\\WebView2") {
    WNDCLASSEXW control{};
    control.cbSize = sizeof(control);
    control.lpfnWndProc = procedure;
    control.hInstance = instance;
    control.lpszClassName = kControlClass;
    RegisterClassExW(&control);
    taskbar_created_ = RegisterWindowMessageW(L"TaskbarCreated");
    control_ = CreateWindowExW(WS_EX_TOOLWINDOW, kControlClass, L"", WS_POPUP, 0, 0, 0, 0, nullptr, nullptr, instance, this);
    if (control_) {
      SetTimer(control_, kFinishedTimer, 250, nullptr);
      add_notification_icon();
    }
  }
  ~Application() {
    window_.reset();
    if (control_) {
      remove_notification_icon();
      DestroyWindow(control_);
    }
  }
  Application(const Application&) = delete;
  Application& operator=(const Application&) = delete;

  /*
   * Runs the message loop until quit() or the end of the companion loop (--stop, or the attached MSFS exiting
   * for a --silent run).
   */
  int run() {
    if (!control_)
      return 1;
    if (!background_)
      show_window();
    MSG message{};
    while (GetMessageW(&message, nullptr, 0, 0) > 0) {
      TranslateMessage(&message);
      DispatchMessageW(&message);
    }
    window_.reset();
    return 0;
  }

 private:
  /*
   * Adds the application icon without a balloon or notification. Version 4 lets both mouse selection and
   * keyboard selection reach the control window through kNotificationIconMessage.
   */
  void add_notification_icon() {
    notification_icon_ = {};
    notification_icon_.cbSize = sizeof(notification_icon_);
    notification_icon_.hWnd = control_;
    notification_icon_.uID = kNotificationIconId;
    notification_icon_.uFlags = NIF_MESSAGE | NIF_ICON | NIF_TIP | NIF_SHOWTIP;
    notification_icon_.uCallbackMessage = kNotificationIconMessage;
    notification_icon_.hIcon = static_cast<HICON>(LoadImageW(instance_, MAKEINTRESOURCEW(AZP_CAMERA_ICON), IMAGE_ICON,
                                                              GetSystemMetrics(SM_CXSMICON), GetSystemMetrics(SM_CYSMICON), LR_SHARED));
    lstrcpynW(notification_icon_.szTip, kProductName, ARRAYSIZE(notification_icon_.szTip));
    notification_icon_added_ = notification_icon_.hIcon && Shell_NotifyIconW(NIM_ADD, &notification_icon_);
    if (notification_icon_added_) {
      notification_icon_.uVersion = NOTIFYICON_VERSION_4;
      Shell_NotifyIconW(NIM_SETVERSION, &notification_icon_);
    }
  }
  void remove_notification_icon() {
    if (notification_icon_added_)
      Shell_NotifyIconW(NIM_DELETE, &notification_icon_);
    notification_icon_added_ = false;
  }
  /*
   * Anchors the menu to the icon but keeps it inside the monitor's work area. The nearest monitor edge tells
   * which side holds the taskbar: a bottom taskbar needs bottom alignment, for example, so the menu grows up
   * instead of being covered by the taskbar.
   */
  NotificationMenuPlacement notification_menu_placement(WPARAM anchor, bool use_cursor_position) const {
    POINT fallback{GET_X_LPARAM(anchor), GET_Y_LPARAM(anchor)};
    if (use_cursor_position || (fallback.x == -1 && fallback.y == -1))
      GetCursorPos(&fallback);
    RECT icon{fallback.x, fallback.y, fallback.x, fallback.y};
    NOTIFYICONIDENTIFIER identifier{};
    identifier.cbSize = sizeof(identifier);
    identifier.hWnd = control_;
    identifier.uID = kNotificationIconId;
    Shell_NotifyIconGetRect(&identifier, &icon);

    NotificationMenuPlacement placement{fallback};
    MONITORINFO monitor{};
    monitor.cbSize = sizeof(monitor);
    if (!GetMonitorInfoW(MonitorFromRect(&icon, MONITOR_DEFAULTTONEAREST), &monitor))
      return placement;
    const LONG center_x = icon.left + (icon.right - icon.left) / 2;
    const LONG center_y = icon.top + (icon.bottom - icon.top) / 2;
    enum class Edge { left, right, top, bottom };
    Edge edge = Edge::left;
    LONG distance = center_x - monitor.rcMonitor.left;
    if (monitor.rcMonitor.right - center_x < distance) {
      edge = Edge::right;
      distance = monitor.rcMonitor.right - center_x;
    }
    if (center_y - monitor.rcMonitor.top < distance) {
      edge = Edge::top;
      distance = center_y - monitor.rcMonitor.top;
    }
    if (monitor.rcMonitor.bottom - center_y < distance)
      edge = Edge::bottom;

    if (edge == Edge::top) {
      placement.point = {std::clamp(icon.right, monitor.rcWork.left, monitor.rcWork.right), monitor.rcWork.top};
      placement.alignment = TPM_RIGHTALIGN | TPM_TOPALIGN;
    } else if (edge == Edge::left) {
      placement.point = {monitor.rcWork.left, std::clamp(icon.bottom, monitor.rcWork.top, monitor.rcWork.bottom)};
      placement.alignment = TPM_LEFTALIGN | TPM_BOTTOMALIGN;
    } else if (edge == Edge::right) {
      placement.point = {monitor.rcWork.right, std::clamp(icon.bottom, monitor.rcWork.top, monitor.rcWork.bottom)};
      placement.alignment = TPM_RIGHTALIGN | TPM_BOTTOMALIGN;
    } else {
      placement.point = {std::clamp(icon.right, monitor.rcWork.left, monitor.rcWork.right), monitor.rcWork.bottom};
      placement.alignment = TPM_RIGHTALIGN | TPM_BOTTOMALIGN;
    }
    return placement;
  }
  /*
   * Shows the icon's two explicit lifecycle commands at the Shell-provided anchor point. The hidden control
   * window owns the menu and is made foreground so clicking elsewhere dismisses it correctly.
   */
  void show_notification_menu(WPARAM anchor, bool use_cursor_position) {
    HMENU menu = CreatePopupMenu();
    if (!menu)
      return;
    if (!AppendMenuW(menu, MF_STRING, kOpenCommand, L"Open") || !AppendMenuW(menu, MF_STRING, kQuitCommand, L"Quit")) {
      DestroyMenu(menu);
      return;
    }
    SetMenuDefaultItem(menu, kOpenCommand, FALSE);
    const auto placement = notification_menu_placement(anchor, use_cursor_position);
    SetForegroundWindow(control_);
    const UINT command = TrackPopupMenu(menu, placement.alignment | TPM_RIGHTBUTTON | TPM_RETURNCMD | TPM_NONOTIFY,
                                        placement.point.x, placement.point.y, 0, control_, nullptr);
    DestroyMenu(menu);
    PostMessageW(control_, WM_NULL, 0, 0);
    Shell_NotifyIconW(NIM_SETFOCUS, &notification_icon_);
    if (command == kOpenCommand)
      show_window();
    else if (command == kQuitCommand)
      quit();
  }
  void show_window() {
    if (window_) {
      window_->bring_forward();
      return;
    }
    try {
      window_ = std::make_unique<app::DesktopWindow>(instance_, controller_, user_data_, [this] { PostMessageW(control_, kWindowClosed, 0, 0); });
    } catch (const std::exception&) {
      MessageBoxW(nullptr, L"The window could not be created.", kProductName, MB_OK | MB_ICONERROR);
      if (!background_)
        quit();
    }
  }
  /*
   * Closing either kind of window hides it. Quitting is explicit through the notification menu or --stop;
   * a --silent run also ends when its attached MSFS process exits.
   */
  void window_closed() {
    window_.reset();
  }
  void quit() {
    companion_.stop();
    PostQuitMessage(0);
  }

  static LRESULT CALLBACK procedure(HWND window, UINT message, WPARAM wparam, LPARAM lparam) {
    auto* self = reinterpret_cast<Application*>(GetWindowLongPtrW(window, GWLP_USERDATA));
    if (message == WM_NCCREATE) {
      self = static_cast<Application*>(reinterpret_cast<const CREATESTRUCTW*>(lparam)->lpCreateParams);
      SetWindowLongPtrW(window, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(self));
    }
    if (self && message == kShowWindow) {
      self->show_window();
      return 0;
    }
    if (self && message == kWindowClosed) {
      self->window_closed();
      return 0;
    }
    if (self && message == kNotificationIconMessage) {
      const UINT event = LOWORD(lparam);
      if (event == NIN_SELECT || event == NIN_KEYSELECT || event == WM_LBUTTONUP || event == WM_LBUTTONDBLCLK)
        self->show_window();
      else if (event == WM_CONTEXTMENU)
        self->show_notification_menu(wparam, false);
      else if (event == WM_RBUTTONUP)
        self->show_notification_menu(wparam, true);
      return 0;
    }
    if (self && self->taskbar_created_ && message == self->taskbar_created_) {
      self->add_notification_icon();
      return 0;
    }
    if (self && message == WM_TIMER && wparam == kFinishedTimer) {
      if (self->companion_.finished())
        PostQuitMessage(0);
      return 0;
    }
    return DefWindowProcW(window, message, wparam, lparam);
  }

  HINSTANCE instance_;
  app::Companion& companion_;
  bool background_;
  app::ApplicationController controller_;
  std::wstring user_data_;
  HWND control_ = nullptr;
  UINT taskbar_created_ = 0;
  NOTIFYICONDATAW notification_icon_{};
  bool notification_icon_added_ = false;
  std::unique_ptr<app::DesktopWindow> window_;
};

/*
 * `--stop`: signals the running EXE, if any. Returns 0 when one was signalled.
 */
int stop_running_instance() {
  HANDLE event = OpenEventW(EVENT_MODIFY_STATE, FALSE, kStopEvent);
  if (!event)
    return 1;
  const bool signalled = SetEvent(event) != FALSE;
  CloseHandle(event);
  return signalled ? 0 : 1;
}

/*
 * Asks the running EXE to open its window, allowing it to take the foreground. Its control window may still
 * be starting: wait up to two seconds for it.
 */
bool open_running_instance() {
  for (int attempt = 0; attempt < 20; ++attempt) {
    if (const HWND control = FindWindowW(kControlClass, nullptr)) {
      DWORD pid = 0;
      GetWindowThreadProcessId(control, &pid);
      AllowSetForegroundWindow(pid);
      return PostMessageW(control, kShowWindow, 0, 0) != FALSE;
    }
    Sleep(100);
  }
  return false;
}
}  /* namespace */

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int) {
  SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
  int argc = 0;
  wchar_t** argv = CommandLineToArgvW(GetCommandLineW(), &argc);
  const auto options = argv ? app::parse_command_line(argc, argv) : app::Options{};
  if (argv)
    LocalFree(argv);
  if (options.stop && options.error.empty())
    return stop_running_instance();

  const auto executable = executable_path();
  const auto slash = executable.find_last_of(L"\\/");
  const auto directory = slash == std::wstring::npos ? std::wstring() : executable.substr(0, slash);
  const auto local = local_directory();
  app::RunLog log(directory.empty() ? std::wstring() : directory + L"\\log", local.empty() ? std::wstring() : local + L"\\log");
  const bool quiet = options.silent || options.autostart >= 0;
  const auto fail = [&](const std::wstring& text) {
    log.write(app::narrow(text));
    if (!quiet)
      MessageBoxW(nullptr, text.c_str(), kProductName, MB_OK | MB_ICONERROR);
    return 1;
  };
  if (!options.error.empty())
    return fail(options.error + L"\n\n" + app::usage());
  if (options.help) {
    MessageBoxW(nullptr, app::usage().c_str(), kProductName, MB_OK | MB_ICONINFORMATION);
    return 0;
  }
  if (directory.empty())
    return fail(L"The folder of AZP-Camera.exe could not be determined.");
  if (options.autostart >= 0) {
    bool ok = false;
    std::string detail;
    for (const auto& startup : app::set_simulator_startup(executable, options.autostart == 1, ok))
      if (!startup.error.empty())
        detail += " " + app::narrow(startup.edition) + ": " + startup.error;
    log.write(std::string("--autostart ") + (options.autostart ? "on" : "off") + (ok ? ": done." : ": failed." + detail));
    return ok ? 0 : 1;
  }

  HANDLE instance_mutex = CreateMutexW(nullptr, FALSE, kInstanceMutex);
  const DWORD mutex_error = GetLastError();
  if (!instance_mutex)
    return fail(std::wstring(kProductName) + L" could not start (Windows error " + std::to_wstring(mutex_error) + L").");
  if (mutex_error == ERROR_ALREADY_EXISTS) {
    CloseHandle(instance_mutex);
    /*
     * A second --silent start (MSFS starting AZP Camera while it already runs) has nothing to add.
     */
    if (options.silent)
      return 0;
    return open_running_instance() ? 0 : fail(std::wstring(kProductName) + L" is already running.");
  }
  std::string note;
  const auto settings = local.empty() ? app::UserSettings{} : app::load_user_settings(local + L"\\settings.json", note);
  HANDLE stop_event = CreateEventW(nullptr, TRUE, FALSE, kStopEvent);
  if (stop_event)
    ResetEvent(stop_event);
  log.write(AZP_CAMERA_PRODUCT_NAME " " + app::narrow(azp::standalone::Version) + " started (" + (options.silent ? "background, started with MSFS" : "window") +
            "): " + app::describe(settings) + "; TV filter on.");
  if (!note.empty())
    log.write(note);

  const HRESULT com = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
  app::Companion companion(directory, settings, log);
  std::thread worker([&] { companion.run(stop_event, options.silent); });
  int result = 0;
  {
    Application application(instance, companion, log, local, settings, executable, options.silent);
    result = application.run();
  }
  companion.stop();
  worker.join();
  log.write(AZP_CAMERA_PRODUCT_NAME " stopped.");
  if (SUCCEEDED(com))
    CoUninitialize();
  if (stop_event)
    CloseHandle(stop_event);
  CloseHandle(instance_mutex);
  return result;
}
