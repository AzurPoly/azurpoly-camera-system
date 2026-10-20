/*
 * The companion loop behind every AZP Camera run, with or without window. It waits for MSFS, loads
 * azp-camera.dll into it, then keeps the DLL supplied with the settings and a presence counter through shared
 * memory (see shared/protocol.hpp). Every 100 ms the loop:
 *
 *   1. advances the presence counter once per second (the DLL relays it to the WASM);
 *   2. follows MSFS: detects start/exit and loads the DLL once per simulator process;
 *   3. applies the window's ON/OFF switch and last settings, reads the status the DLL publishes and publishes
 *      the settings;
 *   4. writes a status line to the log every ten seconds (run_log.hpp).
 *
 * The loop runs on its own thread; the window reads copies of its state (snapshot()).
 */

#pragma once
#include "launcher.hpp"
#include "run_log.hpp"
#include "status_text.hpp"
#include "user_settings.hpp"
#include <atomic>
#include <mutex>
#include <optional>
#include <string>

namespace azp::app {
/*
 * The simulator this EXE is attached to, its shared mailbox and the DLL load attempt. The DLL is loaded at
 * most once per simulator process; only failures that happened before any remote load started are retried,
 * once per second. Attachment and loader events go to the log and to the window's last event.
 */
class Simulator {
 public:
  Simulator(std::wstring directory, RunLog& log, const std::atomic<bool>& running)
      : directory_(std::move(directory)), log_(log), running_(running) {}

  bool follow(const standalone::Settings& settings);
  DWORD pid() const noexcept { return pid_; }
  const std::string& event() const noexcept { return event_; }
  void note(const std::string& text);
  bool read(standalone::Status& status);
  void publish(const standalone::Settings& settings);
  void withdraw();

 private:
  void load(const standalone::Settings& settings);
  void write(const standalone::Settings& settings);

  std::wstring directory_;
  RunLog& log_;
  const std::atomic<bool>& running_;
  standalone::Mailbox mailbox_;
  standalone::LaunchRetry retry_;
  DWORD pid_{};
  bool attempted_ = false;
  std::string event_;
};

class Companion {
 public:
  /*
   * `directory` holds AZP-Camera.exe and azp-camera.dll. launcher.log goes to the folder of `log`'s current
   * file, beside the EXE's and the DLL's logs.
   */
  Companion(const std::wstring& directory, const UserSettings& settings, RunLog& log);
  /*
   * Runs until stop(), the stop event, or, with `exit_with_simulator`, the exit of an attached MSFS. At the end
   * the video demand and presence are withdrawn.
   */
  void run(HANDLE stop_event, bool exit_with_simulator);
  void stop() noexcept { running_ = false; }
  bool finished() const noexcept { return finished_.load(); }
  Snapshot snapshot() const;
  /*
   * Settings chosen in the window. The loop applies them at its next iteration; the DLL takes the resolution
   * only for a camera it has not created yet.
   */
  void apply(const UserSettings& settings);
  /*
   * ON/OFF. OFF sends `enabled = 0`: the DLL stops rendering, capture and preload and no longer relays the
   * presence counter, so the aircraft sees AZP Camera as absent. The EXE keeps running.
   */
  void set_enabled(bool enabled) noexcept { enabled_ = enabled; }

 private:
  std::atomic<bool> running_{true}, finished_{false}, enabled_{true};
  standalone::Settings settings_;
  RunLog& log_;
  Simulator simulator_;
  mutable std::mutex mutex_;
  Snapshot snapshot_;
  std::optional<UserSettings> requested_;
};

}  /* namespace azp::app */
