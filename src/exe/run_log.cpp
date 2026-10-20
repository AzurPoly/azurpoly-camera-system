/*
 * Implements the EXE log files. A file is capped at 16 MB; past that, one last line says so and the rest of
 * the day is dropped rather than filling the disk.
 */

#include "run_log.hpp"
#include <cstdio>

namespace azp::app {
namespace {
constexpr std::uint64_t kMaximumBytes = 16ull * 1024 * 1024;

bool make_directory(const std::wstring& path) {
  return CreateDirectoryW(path.c_str(), nullptr) || GetLastError() == ERROR_ALREADY_EXISTS;
}
}  /* namespace */

RunLog::RunLog(std::wstring root, std::wstring fallback) : root_(std::move(root)), fallback_(std::move(fallback)) {}

RunLog::~RunLog() {
  if (file_ != INVALID_HANDLE_VALUE)
    CloseHandle(file_);
}

std::wstring RunLog::path() const {
  const std::lock_guard lock(mutex_);
  return path_;
}

/*
 * Creates <root>\<date>\ and a new file named after the current time, trying the fallback root when the
 * preferred one cannot be created. The date is local, like the timestamps.
 */
bool RunLog::open(const SYSTEMTIME& now) noexcept {
  if (file_ != INVALID_HANDLE_VALUE)
    CloseHandle(file_);
  file_ = INVALID_HANDLE_VALUE;
  path_.clear();
  bytes_ = 0;
  full_ = false;
  year_ = now.wYear;
  month_ = now.wMonth;
  day_ = now.wDay;
  wchar_t date[16]{}, name[40]{};
  std::swprintf(date, 16, L"%04u-%02u-%02u", now.wYear, now.wMonth, now.wDay);
  std::swprintf(name, 40, L"AZP-Camera-%02u%02u%02u.log", now.wHour, now.wMinute, now.wSecond);
  for (const auto* root : {&root_, &fallback_}) {
    if (root->empty() || !make_directory(*root) || !make_directory(*root + L"\\" + date))
      continue;
    const auto path = *root + L"\\" + date + L"\\" + name;
    file_ = CreateFileW(path.c_str(), FILE_APPEND_DATA, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr, OPEN_ALWAYS,
                        FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file_ != INVALID_HANDLE_VALUE) {
      path_ = path;
      return true;
    }
  }
  return false;
}

void RunLog::write(const std::string& line) noexcept {
  try {
    const std::lock_guard lock(mutex_);
    SYSTEMTIME now{};
    GetLocalTime(&now);
    if ((now.wYear != year_ || now.wMonth != month_ || now.wDay != day_) && !open(now))
      return;
    if (file_ == INVALID_HANDLE_VALUE || full_)
      return;
    char stamp[16]{};
    std::snprintf(stamp, sizeof(stamp), "%02u:%02u:%02u ", now.wHour, now.wMinute, now.wSecond);
    std::string text = stamp + line + "\r\n";
    if (bytes_ + text.size() > kMaximumBytes) {
      text = std::string(stamp) + "Log size limit reached; no more lines today in this file.\r\n";
      full_ = true;
    }
    DWORD written = 0;
    if (WriteFile(file_, text.data(), static_cast<DWORD>(text.size()), &written, nullptr))
      bytes_ += written;
  } catch (...) {
  }
}

}  /* namespace azp::app */
