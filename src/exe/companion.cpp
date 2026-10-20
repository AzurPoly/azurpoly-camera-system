/*
 * Implements the companion loop and the simulator attachment it drives.
 */

#include "companion.hpp"

namespace azp::app {
namespace win = azp::standalone;

/*
 * Follows MSFS start/exit and performs the pending load attempt. Returns true when the simulator changed, so
 * the caller forgets the previous session.
 */
bool Simulator::follow(const win::Settings& settings) {
  const DWORD pid = win::find_simulator({});
  const bool changed = pid != pid_;
  if (changed) {
    mailbox_.close();
    pid_ = pid;
    attempted_ = false;
    retry_ = {};
    note(pid ? "MSFS 2024 found. Loading the camera bridge..." : "MSFS closed. Waiting for MSFS 2024...");
  }
  if (pid_ && !attempted_ && retry_.ready(GetTickCount64()))
    load(settings);
  return changed;
}

void Simulator::note(const std::string& text) {
  event_ = text;
  log_.write(text);
}

/*
 * Copies the DLL's latest status. Returns false when no mailbox is open.
 */
bool Simulator::read(win::Status& status) {
  if (!mailbox_.data() || !mailbox_.lock(100))
    return false;
  status = mailbox_.data()->status;
  mailbox_.unlock();
  return true;
}

void Simulator::publish(const win::Settings& settings) {
  if (!mailbox_.data() || !mailbox_.lock(100))
    return;
  write(settings);
  mailbox_.unlock();
}

/*
 * Withdraws video demand and presence at exit; the DLL stays loaded until MSFS exits.
 */
void Simulator::withdraw() {
  if (!mailbox_.data() || !mailbox_.lock(100))
    return;
  mailbox_.data()->settings.enabled = 0;
  mailbox_.data()->owner_heartbeat = 0;
  mailbox_.unlock();
}

void Simulator::load(const win::Settings& settings) {
  attempted_ = true;
  if (!mailbox_.open(pid_, true) || !mailbox_.lock(100)) {
    note("Control channel unavailable. Restart AZP-Camera.exe to retry.");
    return;
  }
  write(settings);
  mailbox_.unlock();
  win::LaunchDiagnostics trace;
  const auto loaded = win::load_bridge(pid_, {}, directory_ + L"\\azp-camera.dll", &running_, &trace);
  const auto file = log_.path();
  const auto slash = file.find_last_of(L"\\/");
  win::log_launch(slash == std::wstring::npos ? std::wstring() : file.substr(0, slash), pid_, loaded, trace);
  attempted_ = !retry_.schedule(loaded, GetTickCount64());
  note(narrow(loaded.message) + (loaded.error ? " (Windows error " + std::to_string(loaded.error) + ")" : std::string()));
  if (!loaded.ok)
    mailbox_.close();
}

/*
 * Mailbox lock held by the caller.
 */
void Simulator::write(const win::Settings& settings) {
  mailbox_.data()->owner_pid = GetCurrentProcessId();
  mailbox_.data()->owner_heartbeat = GetTickCount64();
  mailbox_.data()->settings = settings;
}

Companion::Companion(const std::wstring& directory, const UserSettings& settings, RunLog& log)
    : settings_(dll_settings(settings)), log_(log), simulator_(directory, log, running_) {}

void Companion::run(HANDLE stop_event, bool exit_with_simulator) {
  win::Status status{};
  bool attached = false;
  std::uint64_t last_counter = GetTickCount64(), last_log = 0;
  while (running_.load()) {
    if (stop_event && WaitForSingleObject(stop_event, 0) == WAIT_OBJECT_0) {
      log_.write("Stop requested (--stop).");
      break;
    }
    if (GetTickCount64() - last_counter >= 1000) {
      ++settings_.heartbeat_counter;
      last_counter = GetTickCount64();
    }
    std::optional<UserSettings> requested;
    {
      const std::lock_guard lock(mutex_);
      requested.swap(requested_);
    }
    if (requested) {
      auto applied = dll_settings(*requested);
      applied.enabled = settings_.enabled;
      applied.heartbeat_counter = settings_.heartbeat_counter;
      applied.aircraft_session_epoch = settings_.aircraft_session_epoch;
      settings_ = applied;
      log_.write("Settings changed: " + describe(*requested) + ".");
    }
    if (const std::uint32_t enabled = enabled_.load() ? 1 : 0; enabled != settings_.enabled) {
      settings_.enabled = enabled;
      log_.write(enabled ? "Camera switched ON." : "Camera switched OFF: the DLL pauses it and the aircraft sees the camera as absent.");
    }
    if (simulator_.follow(settings_)) {
      status = {};
      settings_.aircraft_session_epoch = 0;
      if (simulator_.pid()) {
        attached = true;
      } else if (attached && exit_with_simulator) {
        log_.write("MSFS closed; the background run ends.");
        break;
      }
    }
    if (simulator_.read(status) && settings_.aircraft_session_epoch != status.aircraft_session_epoch) {
      simulator_.note(settings_.aircraft_session_epoch ? "Flight or aircraft changed; video resumes automatically."
                                                       : "Aircraft session detected; video follows the IMA page.");
      settings_.aircraft_session_epoch = status.aircraft_session_epoch;
    }
    simulator_.publish(settings_);
    const auto now = GetTickCount64();
    Snapshot current;
    current.settings = settings_;
    current.status = status;
    current.fresh = status.heartbeat && now >= status.heartbeat && now - status.heartbeat < 5000;
    current.simulator = simulator_.pid();
    current.event = simulator_.event();
    current.log_path = log_.path();
    if (!last_log || now - last_log >= 10000) {
      log_.write(log_line(current));
      last_log = now;
    }
    {
      const std::lock_guard lock(mutex_);
      snapshot_ = std::move(current);
    }
    if (stop_event)
      WaitForSingleObject(stop_event, 100);
    else
      Sleep(100);
  }
  simulator_.withdraw();
  finished_ = true;
}

Snapshot Companion::snapshot() const {
  const std::lock_guard lock(mutex_);
  return snapshot_;
}

void Companion::apply(const UserSettings& settings) {
  const std::lock_guard lock(mutex_);
  requested_ = settings;
}

}  /* namespace azp::app */
