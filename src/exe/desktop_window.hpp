/*
 * The AZP Camera window: a Win32 window hosting the React interface (ui/) in WebView2. WebView2 holds only the
 * presentation. The page's JSON commands go to ApplicationController, and the controller's state returns to
 * the page four times per second, so the companion loop keeps running whatever happens to the page.
 */

#pragma once
#include "application_controller.hpp"
#include <windows.h>
#include <functional>
#include <memory>

namespace azp::app {
class DesktopWindow {
 public:
  /*
   * Creates and shows the window, then starts WebView2 with its profile in `user_data`. `closed` runs when the
   * user closes the window or WebView2 cannot start; the owner then destroys this object, outside the window
   * procedure.
   */
  DesktopWindow(HINSTANCE instance, ApplicationController& controller, std::wstring user_data, std::function<void()> closed);
  ~DesktopWindow();
  DesktopWindow(const DesktopWindow&) = delete;
  DesktopWindow& operator=(const DesktopWindow&) = delete;
  /*
   * Restores the window if minimized and brings it to the foreground.
   */
  void bring_forward();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  /* namespace azp::app */
