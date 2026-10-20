/*
 * Implements the window. The interface is the single HTML file Vite builds from ui/ (scripts, styles and
 * images inlined), embedded in the EXE (resources.rc) and loaded with NavigateToString. A build with
 * -UiDevServer loads the Vite development server instead and enables DevTools, so interface changes appear
 * without rebuilding the EXE.
 */

#include "desktop_window.hpp"
#include "resources.hpp"
#include "webview2_runtime.hpp"
#include <dwmapi.h>
#include <cwchar>
#include <stdexcept>

namespace azp::app {
namespace {
using Microsoft::WRL::ComPtr;
constexpr const wchar_t* kWindowClass = L"AZPCamera.Window";
constexpr UINT_PTR kUpdateTimer = 1;
constexpr UINT kUpdateIntervalMs = 250;
/*
 * Client area and its minimum, in pixels at 96 DPI; scaled to the monitor. A small resizable window without
 * maximize button: the page scrolls when it does not fit.
 */
constexpr int kWidth = 420, kHeight = 580, kMinimumWidth = 360, kMinimumHeight = 320;
constexpr DWORD kStyle = WS_OVERLAPPEDWINDOW & ~WS_MAXIMIZEBOX;
/*
 * The page background (--bg in ui/src/styles/_base.scss), shown while WebView2 starts.
 */
constexpr BYTE kBackground[3] = {27, 36, 51};
/*
 * DwmSetWindowAttribute attributes: DWMWA_USE_IMMERSIVE_DARK_MODE (Windows 10 20H1 and later) for a dark title
 * bar, then DWMWA_BORDER_COLOR, DWMWA_CAPTION_COLOR and DWMWA_TEXT_COLOR (Windows 11; earlier versions ignore
 * them), so both windows look alike.
 */
constexpr DWORD kDarkTitleBar = 20, kBorderColor = 34, kCaptionColor = 35, kTextColor = 36;
constexpr COLORREF kFrameBorder = RGB(51, 68, 92), kFrameCaption = RGB(21, 28, 41), kFrameText = RGB(196, 208, 224);
#ifdef AZP_CAMERA_UI_DEV_SERVER
constexpr BOOL kDevelopment = TRUE;
#else
constexpr BOOL kDevelopment = FALSE;
#endif

/*
 * The interface's HTML as UTF-16, for NavigateToString.
 */
std::wstring interface_html(HINSTANCE instance) {
  const HRSRC resource = FindResourceW(instance, MAKEINTRESOURCEW(AZP_CAMERA_INTERFACE), MAKEINTRESOURCEW(10));
  const HGLOBAL loaded = resource ? LoadResource(instance, resource) : nullptr;
  const auto* bytes = loaded ? static_cast<const char*>(LockResource(loaded)) : nullptr;
  return bytes ? widen(std::string(bytes, SizeofResource(instance, resource))) : std::wstring();
}

std::wstring hex(HRESULT result) {
  wchar_t code[16]{};
  std::swprintf(code, 16, L"0x%08lX", static_cast<unsigned long>(result));
  return code;
}

/*
 * The window size whose client area is `width` x `height` pixels at 96 DPI, at `dpi`.
 */
SIZE window_size(int width, int height, UINT dpi) {
  RECT bounds{0, 0, MulDiv(width, static_cast<int>(dpi), 96), MulDiv(height, static_cast<int>(dpi), 96)};
  AdjustWindowRectExForDpi(&bounds, kStyle, FALSE, 0, dpi);
  return {bounds.right - bounds.left, bounds.bottom - bounds.top};
}
}  /* namespace */

class DesktopWindow::Impl {
 public:
  Impl(HINSTANCE instance, ApplicationController& controller, std::wstring user_data, std::function<void()> closed)
      : instance_(instance), controller_(controller), user_data_(std::move(user_data)), closed_(std::move(closed)) {
    register_class();
    create_window();
    create_webview();
  }

  ~Impl() {
    *self_ = nullptr;
    if (webview_controller_)
      webview_controller_->Close();
    if (window_) {
      KillTimer(window_, kUpdateTimer);
      SetWindowLongPtrW(window_, GWLP_USERDATA, 0);
      DestroyWindow(window_);
    }
  }

  void bring_forward() {
    if (IsIconic(window_))
      ShowWindow(window_, SW_RESTORE);
    SetForegroundWindow(window_);
  }

 private:
  /*
   * A class with the page's background colour; everything inside the window belongs to the page.
   */
  void register_class() const {
    static const ATOM registered = [this] {
      WNDCLASSEXW window_class{};
      window_class.cbSize = sizeof(window_class);
      window_class.style = CS_HREDRAW | CS_VREDRAW;
      window_class.lpfnWndProc = procedure;
      window_class.hInstance = instance_;
      window_class.hCursor = LoadCursorW(nullptr, MAKEINTRESOURCEW(32512));
      window_class.hIcon = static_cast<HICON>(LoadImageW(instance_, MAKEINTRESOURCEW(AZP_CAMERA_ICON), IMAGE_ICON,
                                                         GetSystemMetrics(SM_CXICON), GetSystemMetrics(SM_CYICON), 0));
      window_class.hIconSm = static_cast<HICON>(LoadImageW(instance_, MAKEINTRESOURCEW(AZP_CAMERA_ICON), IMAGE_ICON,
                                                           GetSystemMetrics(SM_CXSMICON), GetSystemMetrics(SM_CYSMICON), 0));
      window_class.hbrBackground = CreateSolidBrush(RGB(kBackground[0], kBackground[1], kBackground[2]));
      window_class.lpszClassName = kWindowClass;
      return RegisterClassExW(&window_class);
    }();
    if (!registered)
      throw std::runtime_error("unable to register the window class");
  }

  void create_window() {
    const auto size = window_size(kWidth, kHeight, GetDpiForSystem());
    window_ = CreateWindowExW(0, kWindowClass, L"" AZP_CAMERA_PRODUCT_NAME, kStyle, CW_USEDEFAULT, CW_USEDEFAULT, size.cx, size.cy, nullptr, nullptr,
                              instance_, this);
    if (!window_)
      throw std::runtime_error("unable to create the window");
    const BOOL dark = TRUE;
    DwmSetWindowAttribute(window_, kDarkTitleBar, &dark, sizeof(dark));
    DwmSetWindowAttribute(window_, kBorderColor, &kFrameBorder, sizeof(kFrameBorder));
    DwmSetWindowAttribute(window_, kCaptionColor, &kFrameCaption, sizeof(kFrameCaption));
    DwmSetWindowAttribute(window_, kTextColor, &kFrameText, sizeof(kFrameText));
    ShowWindow(window_, SW_SHOW);
    UpdateWindow(window_);
    SetTimer(window_, kUpdateTimer, kUpdateIntervalMs, nullptr);
  }

  /*
   * WebView2 calls back on this thread once the environment, then the view, exist. The callbacks hold `self_`,
   * which this object clears when destroyed first.
   */
  void create_webview() {
    const auto self = self_;
    const auto completed = make_handler<ICoreWebView2CreateCoreWebView2EnvironmentCompletedHandler>(
        [self](HRESULT result, ICoreWebView2Environment* environment) -> HRESULT {
          if (!*self)
            return S_OK;
          if (FAILED(result) || !environment) {
            (*self)->fail(L"WebView2 could not start (" + hex(result) + L").");
            return S_OK;
          }
          const auto created = make_handler<ICoreWebView2CreateCoreWebView2ControllerCompletedHandler>(
              [self](HRESULT result, ICoreWebView2Controller* controller) -> HRESULT {
                if (!*self)
                  return S_OK;
                if (FAILED(result) || !controller)
                  (*self)->fail(L"The WebView2 view could not be created (" + hex(result) + L").");
                else
                  (*self)->finish(controller);
                return S_OK;
              });
          const HRESULT started = environment->CreateCoreWebView2Controller((*self)->window_, created.Get());
          if (FAILED(started))
            (*self)->fail(L"The WebView2 view could not be created (" + hex(started) + L").");
          return S_OK;
        });
    std::wstring error;
    if (!create_webview_environment(user_data_, completed.Get(), error))
      fail(error);
  }

  /*
   * Configures the new view: no browser menus, shortcuts, zoom or new windows outside development builds,
   * the JSON channel to the controller, then the interface.
   */
  void finish(ICoreWebView2Controller* controller) {
    webview_controller_ = controller;
    if (FAILED(controller->get_CoreWebView2(&webview_)) || !webview_) {
      fail(L"The WebView2 view could not be created.");
      return;
    }
    ComPtr<ICoreWebView2Controller2> with_background;
    if (SUCCEEDED(webview_controller_.As(&with_background)))
      with_background->put_DefaultBackgroundColor({255, kBackground[0], kBackground[1], kBackground[2]});
    resize();
    ComPtr<ICoreWebView2Settings> settings;
    if (SUCCEEDED(webview_->get_Settings(&settings)) && settings) {
      settings->put_IsStatusBarEnabled(FALSE);
      settings->put_IsZoomControlEnabled(FALSE);
      settings->put_AreDefaultContextMenusEnabled(kDevelopment);
      settings->put_AreDevToolsEnabled(kDevelopment);
      ComPtr<ICoreWebView2Settings3> with_keys;
      if (SUCCEEDED(settings.As(&with_keys)))
        with_keys->put_AreBrowserAcceleratorKeysEnabled(kDevelopment);
    }
    EventRegistrationToken token{};
    webview_->add_NewWindowRequested(make_handler<ICoreWebView2NewWindowRequestedEventHandler>(
                                         [](ICoreWebView2*, ICoreWebView2NewWindowRequestedEventArgs* arguments) -> HRESULT {
                                           return arguments->put_Handled(TRUE);
                                         })
                                         .Get(),
                                     &token);
    const auto self = self_;
    webview_->add_WebMessageReceived(make_handler<ICoreWebView2WebMessageReceivedEventHandler>(
                                         [self](ICoreWebView2*, ICoreWebView2WebMessageReceivedEventArgs* arguments) -> HRESULT {
                                           LPWSTR json = nullptr;
                                           if (!*self || FAILED(arguments->get_WebMessageAsJson(&json)) || !json)
                                             return S_OK;
                                           const auto command = narrow(json);
                                           CoTaskMemFree(json);
                                           (*self)->controller_.handle_command(command);
                                           (*self)->post_state();
                                           return S_OK;
                                         })
                                         .Get(),
                                     &token);
#ifdef AZP_CAMERA_UI_DEV_SERVER
    webview_->Navigate(L"http://127.0.0.1:5173/");
#else
    webview_->NavigateToString(interface_html(instance_).c_str());
#endif
  }

  void post_state() {
    if (webview_)
      webview_->PostWebMessageAsJson(widen(controller_.state_json()).c_str());
  }

  void resize() {
    if (!webview_controller_)
      return;
    const bool visible = !IsIconic(window_);
    webview_controller_->put_IsVisible(visible);
    RECT bounds{};
    GetClientRect(window_, &bounds);
    if (visible)
      webview_controller_->put_Bounds(bounds);
  }

  void fail(const std::wstring& reason) {
    MessageBoxW(window_,
                (reason + L"\n\nThis window needs the Microsoft Edge WebView2 Runtime, which Windows 11 includes. The camera "
                          L"itself does not: it keeps running while MSFS runs if MSFS started it.")
                    .c_str(),
                L"" AZP_CAMERA_PRODUCT_NAME, MB_OK | MB_ICONERROR);
    closed_();
  }

  static LRESULT CALLBACK procedure(HWND window, UINT message, WPARAM wparam, LPARAM lparam) {
    auto* self = reinterpret_cast<Impl*>(GetWindowLongPtrW(window, GWLP_USERDATA));
    if (message == WM_NCCREATE) {
      self = static_cast<Impl*>(reinterpret_cast<const CREATESTRUCTW*>(lparam)->lpCreateParams);
      SetWindowLongPtrW(window, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(self));
    }
    if (!self)
      return DefWindowProcW(window, message, wparam, lparam);
    switch (message) {
      case WM_GETMINMAXINFO: {
        auto* limits = reinterpret_cast<MINMAXINFO*>(lparam);
        const auto minimum = window_size(kMinimumWidth, kMinimumHeight, GetDpiForWindow(window));
        limits->ptMinTrackSize.x = minimum.cx;
        limits->ptMinTrackSize.y = minimum.cy;
        return 0;
      }
      case WM_DPICHANGED: {
        const auto* suggested = reinterpret_cast<const RECT*>(lparam);
        SetWindowPos(window, nullptr, suggested->left, suggested->top, suggested->right - suggested->left, suggested->bottom - suggested->top,
                     SWP_NOZORDER | SWP_NOACTIVATE);
        return 0;
      }
      case WM_SIZE:
        self->resize();
        return 0;
      case WM_MOVE:
        if (self->webview_controller_)
          self->webview_controller_->NotifyParentWindowPositionChanged();
        return 0;
      case WM_TIMER:
        if (wparam == kUpdateTimer) {
          self->controller_.update();
          self->post_state();
        }
        return 0;
      case WM_CLOSE:
        self->closed_();
        return 0;
      default:
        return DefWindowProcW(window, message, wparam, lparam);
    }
  }

  HINSTANCE instance_;
  ApplicationController& controller_;
  std::wstring user_data_;
  std::function<void()> closed_;
  std::shared_ptr<Impl*> self_ = std::make_shared<Impl*>(this);
  HWND window_ = nullptr;
  ComPtr<ICoreWebView2Controller> webview_controller_;
  ComPtr<ICoreWebView2> webview_;
};

DesktopWindow::DesktopWindow(HINSTANCE instance, ApplicationController& controller, std::wstring user_data, std::function<void()> closed)
    : impl_(std::make_unique<Impl>(instance, controller, std::move(user_data), std::move(closed))) {}
DesktopWindow::~DesktopWindow() = default;
void DesktopWindow::bring_forward() {
  impl_->bring_forward();
}

}  /* namespace azp::app */
