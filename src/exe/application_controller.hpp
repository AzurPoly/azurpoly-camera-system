/*
 * Connects the window's interface to the rest of the EXE. React never touches the companion loop, the
 * settings file or exe.xml: it sends short JSON commands, which this controller validates and executes, and it
 * displays the state this controller assembles. Every value the window shows is decided here.
 */

#pragma once
#include "companion.hpp"
#include "msfs_autostart.hpp"
#include "run_log.hpp"
#include "user_settings.hpp"
#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace azp::app {
class ApplicationController {
 public:
  /*
   * `settings_path`: settings.json, empty when it cannot be saved. `executable`: this AZP-Camera.exe, which
   * the MSFS start entry names. `quit`: the application's complete-stop action, shared with the notification
   * icon's Quit command.
   */
  ApplicationController(Companion& companion,
                        RunLog& log,
                        std::wstring settings_path,
                        const UserSettings& settings,
                        std::wstring executable,
                        std::function<void()> quit);
  /*
   * Re-reads the MSFS start entries once they are five seconds old: MSFS, an installer or the user can change
   * exe.xml at any time.
   */
  void update();
  /*
   * Executes one command from the window's page. An invalid or failed command leaves its reason in the
   * state's `error`.
   */
  void handle_command(std::string_view json);
  std::string state_json() const;

 private:
  void change_settings(UserSettings settings);
  void change_startup(bool enabled);
  void open_log_folder();

  Companion& companion_;
  RunLog& log_;
  std::wstring settings_path_, executable_;
  UserSettings settings_;
  std::function<void()> quit_;
  /*
   * The window's ON/OFF switch. OFF pauses the camera; every run starts ON.
   */
  bool enabled_ = true;
  std::vector<SimulatorStartup> startup_;
  std::uint64_t startup_read_ = 0;
  std::string error_;
};

}  /* namespace azp::app */
