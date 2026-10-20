/*
 * Starts Microsoft Edge WebView2 for the EXE window. The delivery is AZP-Camera.exe and azp-camera.dll only,
 * so there is no WebView2Loader.dll: like that loader, this finds the WebView2 Runtime installed with Windows
 * (Evergreen, stable channel), whose EmbeddedBrowserWebView.dll creates the environment. The SDK's
 * WebView2.h provides the interfaces; this header adds the interface identifiers the EXE needs, which that
 * header declares for MSVC only, and the handler objects WebView2 calls back.
 */

#pragma once
#include <windows.h>
#include <wrl/client.h>
#include <WebView2.h>
#include <atomic>
#include <functional>
#include <string>
#include <utility>

__CRT_UUID_DECL(ICoreWebView2CreateCoreWebView2EnvironmentCompletedHandler, 0x4e8a3389, 0xc9d8, 0x4bd2, 0xb6, 0xb5, 0x12, 0x4f, 0xee, 0x6c,
                0xc1, 0x4d)
__CRT_UUID_DECL(ICoreWebView2CreateCoreWebView2ControllerCompletedHandler, 0x6c4819f3, 0xc9b7, 0x4260, 0x81, 0x27, 0xc9, 0xf5, 0xbd, 0xe7,
                0xf6, 0x8c)
__CRT_UUID_DECL(ICoreWebView2WebMessageReceivedEventHandler, 0x57213f19, 0x00e6, 0x49fa, 0x8e, 0x07, 0x89, 0x8e, 0xa0, 0x1e, 0xcb, 0xd2)
__CRT_UUID_DECL(ICoreWebView2NewWindowRequestedEventHandler, 0xd4c185fe, 0xc81c, 0x4989, 0x97, 0xaf, 0x2d, 0x3f, 0xa7, 0xab, 0x56, 0x51)
__CRT_UUID_DECL(ICoreWebView2Controller2, 0xc979903e, 0xd4ca, 0x4228, 0x92, 0xeb, 0x47, 0xee, 0x3f, 0xa9, 0x6e, 0xab)
__CRT_UUID_DECL(ICoreWebView2Settings3, 0xfdb5ab74, 0xaf33, 0x4854, 0x84, 0xf0, 0x0a, 0x63, 0x1d, 0xeb, 0x5e, 0xba)

namespace azp::app {
/*
 * A WebView2 completion or event handler calling `function`. It is reference counted like any COM object:
 * WebView2 keeps it while it may still call it.
 */
template <class Interface, class... Arguments>
class Handler final : public Interface {
 public:
  explicit Handler(std::function<HRESULT(Arguments...)> function) : function_(std::move(function)) {}
  ULONG STDMETHODCALLTYPE AddRef() override { return ++references_; }
  ULONG STDMETHODCALLTYPE Release() override {
    const ULONG left = --references_;
    if (!left)
      delete this;
    return left;
  }
  HRESULT STDMETHODCALLTYPE QueryInterface(REFIID id, void** object) override {
    if (!object)
      return E_POINTER;
    if (IsEqualIID(id, __uuidof(IUnknown)) || IsEqualIID(id, __uuidof(Interface))) {
      *object = static_cast<Interface*>(this);
      AddRef();
      return S_OK;
    }
    *object = nullptr;
    return E_NOINTERFACE;
  }
  HRESULT STDMETHODCALLTYPE Invoke(Arguments... arguments) override { return function_(arguments...); }

 private:
  std::atomic<ULONG> references_{1};
  std::function<HRESULT(Arguments...)> function_;
};

template <class Method>
struct HandlerFor;
template <class Interface, class... Arguments>
struct HandlerFor<HRESULT (Interface::*)(Arguments...)> {
  using type = Handler<Interface, Arguments...>;
};

/*
 * A handler for `Interface`, whose Invoke arguments `function` receives.
 */
template <class Interface, class Function>
Microsoft::WRL::ComPtr<Interface> make_handler(Function&& function) {
  Microsoft::WRL::ComPtr<Interface> handler;
  handler.Attach(new typename HandlerFor<decltype(&Interface::Invoke)>::type(std::forward<Function>(function)));
  return handler;
}

/*
 * Starts creating the WebView2 environment, with its browser profile in `user_data`; `completed` then runs on
 * this thread's message loop. Returns false, with a reason for the user, when no WebView2 Runtime is installed
 * or it refuses to start.
 */
bool create_webview_environment(const std::wstring& user_data,
                                ICoreWebView2CreateCoreWebView2EnvironmentCompletedHandler* completed,
                                std::wstring& error);

}  /* namespace azp::app */
