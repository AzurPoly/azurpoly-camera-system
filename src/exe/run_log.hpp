/*
 * The EXE's own log: log\<YYYY-MM-DD>\AZP-Camera-<HHMMSS>.log next to AZP-Camera.exe (or under
 * %LOCALAPPDATA%\AzurPoly\CameraSystem when that folder is not writable). One file per run and per day; each line
 * starts with the local time. The same dated folder receives launcher.log and the DLL's bridge log and
 * diagnostic report (src/dll/diagnostic.hpp).
 */

#pragma once
#include <windows.h>
#include <cstdint>
#include <mutex>
#include <string>

namespace azp::app {
class RunLog {
 public:
  /*
   * `root` is the preferred log folder, `fallback` the one used when `root` cannot be created.
   */
  RunLog(std::wstring root, std::wstring fallback);
  ~RunLog();
  RunLog(const RunLog&) = delete;
  RunLog& operator=(const RunLog&) = delete;
  /*
   * Appends one UTF-8 line, opening the day's file first. Failures are silent: logging never stops the EXE.
   */
  void write(const std::string& line) noexcept;
  std::wstring path() const;

 private:
  bool open(const SYSTEMTIME& now) noexcept;

  mutable std::mutex mutex_;
  std::wstring root_, fallback_, path_;
  HANDLE file_ = INVALID_HANDLE_VALUE;
  WORD year_ = 0, month_ = 0, day_ = 0;
  std::uint64_t bytes_ = 0;
  bool full_ = false;
};

}  /* namespace azp::app */
